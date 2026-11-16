import pytest
from swarm_execution_agent.execution import Execution
from swarm_execution_agent.routes import wait_seconds


def prepared(tmp_path, holding=False):
    agent = Execution("A", tmp_path)
    location = {
        "frame_id": "m",
        "pos": [2.0, 2.0, 0.0],
        "orientation": [0.0, 0.0, 0.0, 1.0],
    }
    tasks = [
        {
            "id": f"t{i}",
            "location": location,
            "tasks": [{"method": "wait", "params": {"duration": 1.2}}],
        }
        for i in range(2)
    ]
    route = {
        "task_ids": ["t0", "t1"],
        "checkpoints": [
            {"type": "task", "task_ids": [t["id"]], "location": location} for t in tasks
        ],
    }
    if holding:
        route = {
            "task_ids": [],
            "checkpoints": [
                {
                    "type": "holding",
                    "task_ids": [],
                    "location": location,
                    "duration": 2.2,
                }
            ],
        }
    agent.prepare(
        {
            "session_id": agent.session,
            "round_epoch": 1,
            "revision": 1,
            "route_name": "r1",
            "plan_digest": "d",
            "route": route,
            "mission": {"tasks": tasks},
        }
    )
    request = agent.request_permit(0)
    assert agent.grant(request, 1)
    assert agent.begin_start(1)
    return agent


def snapshot(**fields):
    return {
        "protocol_version": 1,
        "route_name": "r1",
        "execution_id": "e",
        "snapshot_sequence": 1,
        "route_state": "COMPLETED",
        "finalized": True,
        "reached_waypoint_ids": [1],
        "skipped_waypoints": [],
        "pending_waypoint_ids": [],
        **fields,
    }


def test_same_position_mapping_and_start_callback_snapshot(tmp_path):
    agent = prepared(tmp_path)
    assert agent.current["mapping"] == {"1": ["t0", "t1"]}
    assert not agent.observe(snapshot())
    assert not agent.outbox
    assert agent.started(agent.generation, True) == "started"
    assert agent.outbox["result"]["result"]["reached"] == ["t0", "t1"]


@pytest.mark.parametrize(
    "bad",
    [
        {"reached_waypoint_ids": []},
        {"pending_waypoint_ids": [1]},
        {"execution_id": ""},
        {"snapshot_sequence": True},
        {"route_state": "FAILED", "finalized": False},
        {"skipped_waypoints": [{"waypoint_id": 1, "reason": "blocked"}]},
    ],
)
def test_incomplete_conflicting_snapshot_rejected(tmp_path, bad):
    agent = prepared(tmp_path)
    agent.started(agent.generation, True)
    with pytest.raises(ValueError):
        agent.observe(snapshot(**bad))
    assert not agent.outbox


def test_skip_then_navigation_retry_success(tmp_path):
    agent = prepared(tmp_path)
    agent.started(agent.generation, True)
    agent.observe(
        snapshot(
            route_state="RUNNING",
            finalized=False,
            reached_waypoint_ids=[],
            skipped_waypoints=[{"waypoint_id": 1, "reason": "failed"}],
        )
    )
    agent.observe(snapshot(snapshot_sequence=2))
    assert agent.outbox["result"]["result"]["skipped"] == {}
    assert agent.outbox["result"]["result"]["reached"] == ["t0", "t1"]


def test_holding_is_real_route_but_not_fake_task(tmp_path):
    agent = prepared(tmp_path, True)
    agent.started(agent.generation, True)
    agent.observe(snapshot())
    result = agent.outbox["result"]["result"]
    assert result["finalized"] and not result["reached"]
    assert "duration: 3" in open(agent.current["yaml_path"]).read()


def test_unknown_is_sealed_and_old_ack_cannot_clear_new_result(tmp_path):
    agent = prepared(tmp_path)
    agent.started(agent.generation, True)
    agent.observe(
        snapshot(
            route_state="RUNNING",
            finalized=False,
            reached_waypoint_ids=[],
            pending_waypoint_ids=[1],
        )
    )
    old = agent.outbox["result"]["event_id"]
    agent.unknown("no sealed result")
    agent.ack(old)
    assert agent.outbox["result"]["result"]["state"] == "UNKNOWN"
    with pytest.raises(ValueError):
        agent.observe(snapshot())


@pytest.mark.parametrize(
    "value", [True, -1, float("nan"), float("inf"), "2", 4294967295]
)
def test_invalid_wait_rejected(value):
    with pytest.raises(ValueError):
        wait_seconds(value)


def test_process_restart_preserves_result_outbox_but_never_restores_start_authority(
    tmp_path,
):
    old = prepared(tmp_path)
    old.started(old.generation, True)
    old.observe(snapshot())
    event = old.outbox["result"]
    restarted = Execution("A", tmp_path)
    assert restarted.session != old.session
    assert restarted.outbox["result"] == event
    assert restarted.current is None and not restarted.owned
    assert restarted.request_permit(10) is None
    restarted.ack(event["event_id"])
    assert not restarted.outbox


def test_cancel_snapshot_after_pause_is_collected_before_route_replacement(tmp_path):
    agent = prepared(tmp_path)
    agent.started(agent.generation, True)
    agent.observe(
        snapshot(
            route_state="PAUSED",
            finalized=False,
            reached_waypoint_ids=[],
            pending_waypoint_ids=[1],
        )
    )
    agent.stopped()
    assert agent.observe(
        snapshot(
            snapshot_sequence=2,
            route_state="CANCELLED",
            finalized=True,
            reached_waypoint_ids=[],
            pending_waypoint_ids=[1],
        )
    )
    assert agent.latest_result["state"] == "CANCELLED"
    assert agent.latest_result["pending"] == ["t0", "t1"]
