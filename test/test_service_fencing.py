"""A timed-out service must drain before any compensation call is sent."""

from types import SimpleNamespace

from rclpy.task import Future
from swarm_execution_agent.node import AgentNode


def test_hanging_mutation_is_fenced_once_and_never_replaced(monkeypatch):
    monkeypatch.setattr("swarm_execution_agent.node.time.monotonic", lambda: 20.0)
    future = Future()
    errors = []
    agent = SimpleNamespace(
        expired=False,
        generation=0,
        current={"route_name": "route"},
        terminal_since=None,
        outbox={},
        queue=lambda kind, value: errors.append((kind, value)),
        needs_pause=lambda now: False,
    )
    calls = []
    node = SimpleNamespace(
        agent=agent,
        mutating=("start_route", future, 0.0),
        mutation_warned=None,
        start_wanted=True,
        stop_wanted=None,
        stop_cause="",
        ready=True,
        stop_confirmation=None,
        last_cancel=0.0,
        last_outbox=20.0,
        last_hello=20.0,
        last_lease=20.0,
        get_parameter=lambda key: SimpleNamespace(value=15.0),
        get_logger=lambda: SimpleNamespace(error=lambda text: None),
        reconcile_stop=lambda now: None,
        query_status=lambda now: None,
        safe_controller=lambda: True,
        hub_clients={
            "cancel_route": SimpleNamespace(
                service_is_ready=lambda: True,
                call_async=lambda request: calls.append(request),
            )
        },
        on_cancel=lambda response: None,
    )
    node.call_mutation = lambda *args: AgentNode.call_mutation(node, *args)
    AgentNode.tick(node)
    AgentNode.tick(node)
    assert node.mutating[1] is future and not future.done()
    assert agent.expired and agent.generation == 1
    assert not node.start_wanted and node.stop_wanted == "CANCEL"
    assert len(errors) == 1 and errors[0][0] == "error"
    assert not calls
