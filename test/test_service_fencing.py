"""A timed-out service must drain before any compensation call is sent."""

from types import SimpleNamespace

from rclpy.task import Future
from swarm_execution_agent.node import AgentNode
from swarm_execution_agent.execution import RouteControl


def test_hanging_mutation_is_fenced_once_and_never_replaced(monkeypatch):
    monkeypatch.setattr("swarm_execution_agent.node.time.monotonic", lambda: 20.0)
    future = Future()
    errors = []
    agent = SimpleNamespace(
        control=RouteControl(mutating=("start_route", future, 0.0), start_wanted=True, ready=True),
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
    assert agent.control.mutating[1] is future and not future.done()
    assert agent.expired and agent.generation == 1
    assert not agent.control.start_wanted and agent.control.stop_wanted == "CANCEL"
    assert len(errors) == 1 and errors[0][0] == "error"
    assert not calls
