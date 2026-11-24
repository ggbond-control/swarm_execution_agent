"""Local execution record and request-anchored permit, independent of ROS IO."""

from dataclasses import dataclass

import copy
import json
import uuid

from swarm_protocol.storage import atomic_json

from .routes import convert


@dataclass
class RouteControl:
    """Volatile service and stop state for the locally owned route."""

    ready: bool = False
    mutating: tuple | None = None
    start_wanted: bool = False
    stop_wanted: str | None = None
    stop_confirmation: dict | None = None
    stop_cause: str = ""
    stop_before: int = 0
    mutation_warned: object = None
    last_cancel: float = 0.0

    def request_stop(self, command, cause=""):
        self.stop_cause = self.stop_cause or cause
        self.start_wanted = False
        self.stop_wanted = command


class Execution:
    def __init__(self, robot, root, *, permit_seconds=10.0):
        self.control = RouteControl()
        self.robot, self.root = robot, root
        self.session = uuid.uuid4().hex
        self.generation = 0
        self.current = None
        self.owned = False
        self.inflight = False
        self.expired = False
        self.permit_until = 0.0
        self.permit_seconds = permit_seconds
        self.requests = {}
        path = self.root / "outbox.json"
        self.outbox = json.loads(path.read_text()) if path.exists() else {}
        if any(e.get("robot_id") != self.robot for e in self.outbox.values()):
            raise ValueError("outbox belongs to another robot")
        self.latest_result = None
        self.sequence = 0
        self.snapshot = None
        self.pending_snapshot = None
        self.terminal_since = None
        self.sealed_unknown = False

    def prepare(self, value):
        if value["session_id"] != self.session:
            raise ValueError("prepare belongs to another boot")
        if self.current and self.current["route_name"] == value["route_name"]:
            if self.current["plan_digest"] != value["plan_digest"]:
                raise ValueError("same route has conflicting plan")
            return False
        if self.owned or self.inflight:
            raise ValueError("old route must be confirmed stopped before replacement")
        text, mapping = convert(value["route"], value["mission"], value["route_name"])
        self.root.mkdir(parents=True, exist_ok=True)
        path = self.root / (value["route_name"] + ".yaml")
        # YAML is derived data; execution record is the durable start fence.
        path.write_text(text, encoding="utf-8")
        candidate = copy.deepcopy(value)
        candidate.update(mapping=mapping, yaml_path=str(path), start_phase="prepared")
        atomic_json(self.root / "execution.json", candidate)
        atomic_json(
            self.root / "route_archive" / (value["route_name"] + ".json"), candidate
        )
        self.current = candidate
        self.generation += 1
        self.snapshot, self.pending_snapshot, self.terminal_since = None, None, None
        self.latest_result = None
        self.sealed_unknown = False
        self.expired, self.permit_until = False, 0.0
        self.requests.clear()
        # A new committed plan supersedes the previous revision's local outbox.
        atomic_json(self.root / "outbox.json", {})
        self.outbox = {}
        return True

    def request_permit(self, now):
        if (
            not self.current
            or self.expired
            or self.sealed_unknown
            or (self.snapshot and self.snapshot["finalized"])
        ):
            return None
        request = uuid.uuid4().hex
        self.requests = {
            k: v for k, v in self.requests.items() if now - v < self.permit_seconds
        }
        self.requests[request] = now
        return {**self.identity(), "request_id": request}

    def grant(self, value, now):
        if (
            not self.current
            or self.expired
            or any(value.get(k) != v for k, v in self.identity().items())
        ):
            return False
        started = self.requests.pop(value.get("request_id"), None)
        if started is None or now >= started + self.permit_seconds:
            return False
        self.permit_until = max(self.permit_until, started + self.permit_seconds)
        return True

    def identity(self):
        return {
            "robot_id": self.robot,
            "session_id": self.session,
            "round_epoch": self.current["round_epoch"],
            "revision": self.current["revision"],
            "route_name": self.current["route_name"],
        }

    def begin_start(self, now):
        if not self.current or self.expired or now >= self.permit_until:
            raise ValueError("no live quorum-backed execution permit")
        if self.current["start_phase"] != "prepared":
            return False
        self.current["start_phase"] = "starting"
        atomic_json(self.root / "execution.json", self.current)
        self.inflight = True
        return True

    def started(self, generation, success):
        self.inflight = False
        if generation != self.generation or self.expired:
            return "cancel" if success else "stale"
        if not success:
            self.current["start_phase"] = "failed"
            atomic_json(self.root / "execution.json", self.current)
            return "failed"
        self.owned = True
        self.current["start_phase"] = "started"
        atomic_json(self.root / "execution.json", self.current)
        if self.pending_snapshot:
            self.observe(self.pending_snapshot)
            self.pending_snapshot = None
        return "started"

    def needs_pause(self, now):
        if (
            (self.owned or self.inflight)
            and not self.expired
            and now >= self.permit_until
        ):
            self.expired = True
            self.generation += 1
            return True
        return False

    def observe(self, value):
        if not self.current or value.get("route_name") != self.current["route_name"]:
            return False
        if value.get("protocol_version") != 1:
            raise ValueError("unknown TaskHub snapshot protocol")
        if self.sealed_unknown:
            raise ValueError("unknown result already sealed")
        if self.snapshot and self.snapshot["finalized"]:
            if self.snapshot == value:
                return False
            raise ValueError("sealed snapshot cannot change")
        if not self.owned and self.current["start_phase"] != "stopped":
            if self.inflight:
                self.pending_snapshot = copy.deepcopy(value)
            return False
        mapping = self.current["mapping"]
        reached, skipped, pending = (
            value.get("reached_waypoint_ids"),
            value.get("skipped_waypoints"),
            value.get("pending_waypoint_ids"),
        )
        if not all(isinstance(x, list) for x in (reached, skipped, pending)):
            raise ValueError("snapshot partitions missing")
        if any(type(x) is not int for x in reached + pending):
            raise ValueError("invalid waypoint id")
        if any(
            not isinstance(x, dict)
            or type(x.get("waypoint_id")) is not int
            or not x.get("reason")
            for x in skipped
        ):
            raise ValueError("skipped waypoints require final reason")
        skipped_ids = [x["waypoint_id"] for x in skipped]
        all_ids = reached + skipped_ids + pending
        if len(all_ids) != len(set(all_ids)) or {str(x) for x in all_ids} != set(
            mapping
        ):
            raise ValueError("snapshot must partition all generated waypoints")
        state, final = value.get("route_state"), value.get("finalized")
        if (
            state not in {"RUNNING", "PAUSED", "COMPLETED", "FAILED", "CANCELLED"}
            or type(final) is not bool
            or final != (state in {"COMPLETED", "FAILED", "CANCELLED"})
            or (state == "COMPLETED" and pending)
        ):
            raise ValueError("invalid route finalization")
        if (
            not value.get("execution_id")
            or type(value.get("snapshot_sequence")) is not int
            or value["snapshot_sequence"] <= 0
        ):
            raise ValueError("snapshot execution identity required")
        old = self.snapshot
        if old:
            if old == value:
                return False
            if old["finalized"]:
                raise ValueError("sealed snapshot cannot change")
            if (
                old["execution_id"] != value["execution_id"]
                or old["snapshot_sequence"] >= value["snapshot_sequence"]
                or not set(old["reached_waypoint_ids"]).issubset(reached)
            ):
                raise ValueError("old instance, sequence or regressed arrival")
        result = {
            "execution_id": value["execution_id"],
            "sequence": value["snapshot_sequence"],
            "state": state,
            "finalized": final,
            "reached": sorted({t for n in reached for t in mapping[str(n)]}),
            "skipped": {
                t: x["reason"] for x in skipped for t in mapping[str(x["waypoint_id"])]
            },
            "pending": sorted({t for n in pending for t in mapping[str(n)]}),
        }
        self.queue("result", {"result": result})
        self.latest_result = copy.deepcopy(result)
        self.snapshot = copy.deepcopy(value)
        if final:
            self.owned = False
            self.current["start_phase"] = "finalized"
            atomic_json(self.root / "execution.json", self.current)
        return True

    def unknown(self, reason):
        if (
            not self.current
            or self.sealed_unknown
            or (self.snapshot and self.snapshot["finalized"])
        ):
            return
        reached = {
            t
            for n in (self.snapshot or {}).get("reached_waypoint_ids", [])
            for t in self.current["mapping"][str(n)]
        }
        all_tasks = set(self.current["route"]["task_ids"])
        result = {
            "execution_id": (self.snapshot or {}).get(
                "execution_id", "unknown-" + self.session
            ),
            "sequence": (self.snapshot or {}).get("snapshot_sequence", 0) + 1,
            "state": "UNKNOWN",
            "finalized": True,
            "reached": sorted(reached),
            "skipped": {},
            "pending": sorted(all_tasks - reached),
            "reason": reason,
        }
        self.queue("result", {"result": result})
        self.latest_result = copy.deepcopy(result)
        self.sealed_unknown = True

    def queue(self, kind, fields):
        self.sequence += 1
        value = {
            **self.identity(),
            "kind": kind,
            "event_id": f"{self.session}-{self.sequence}",
            **fields,
        }
        # Keep the latest cumulative result, but never erase it before committed ACK.
        candidate = {**self.outbox, kind: value}
        atomic_json(self.root / "outbox.json", candidate)
        self.outbox = candidate
        return value

    def ack(self, event_id):
        candidate = {k: v for k, v in self.outbox.items() if v["event_id"] != event_id}
        atomic_json(self.root / "outbox.json", candidate)
        self.outbox = candidate

    def stopped(self):
        self.owned = False
        self.expired = True
        self.generation += 1
        if self.current:
            self.current["start_phase"] = "stopped"
            atomic_json(self.root / "execution.json", self.current)
