"""Independent TaskHub YAML conversion with exact task/waypoint mapping."""

import copy
import math

import yaml
from swarm_protocol.contracts import finite


def wait_seconds(value):
    seconds = finite(value, "wait duration")
    if seconds < 0 or seconds > 4294967294:
        raise ValueError("wait duration is outside uint32 range")
    return math.ceil(seconds)


def convert(route, mission, route_name):
    tasks = {t["id"]: t for t in mission["tasks"]}
    waypoints, mapping = [], {}
    for point in route["checkpoints"]:
        kind = point["type"]
        if kind not in {"task", "turn", "holding"}:
            raise ValueError("unknown checkpoint type")
        location = point["location"]
        operations = []
        for task_id in point["task_ids"]:
            if task_id not in tasks:
                raise ValueError("unknown original task in checkpoint")
            for operation in tasks[task_id]["tasks"]:
                fields = copy.deepcopy(operation.get("params", {}))
                if "id" in fields or "type" in fields:
                    raise ValueError("operation.params must not override id/type")
                if operation["method"] == "wait" and "duration" in fields:
                    fields["duration"] = wait_seconds(fields["duration"])
                for nested in ("data", "parameters"):
                    if (
                        operation["method"] == "wait"
                        and isinstance(fields.get(nested), dict)
                        and "duration" in fields[nested]
                    ):
                        fields[nested]["duration"] = wait_seconds(
                            fields[nested]["duration"]
                        )
                operations.append({"type": operation["method"], **fields})
        if kind == "holding":
            operations.append(
                {
                    "type": "wait",
                    "duration": wait_seconds(point["duration"]),
                    "timing": "after_arrival",
                }
            )
        # Merge adjacent colocated tasks only; preserve every original task ID.
        merge = (
            waypoints
            and waypoints[-1]["frame_id"] == location["frame_id"]
            and waypoints[-1]["position"] == location["pos"]
            and kind == "task"
        )
        if merge:
            waypoint = waypoints[-1]
            waypoint["tasks"].extend(operations)
            mapping[str(waypoint["id"])].extend(point["task_ids"])
        else:
            number = len(waypoints) + 1
            waypoint = {
                "id": number,
                "name": "swarm-waypoint-" + str(number),
                "frame_id": location["frame_id"],
                "position": location["pos"],
                "orientation": location.get("orientation", [0.0, 0.0, 0.0, 1.0]),
                "max_retry": 1,
                "failure_handling": "skip",
                "tasks": operations,
            }
            waypoints.append(waypoint)
            mapping[str(number)] = list(point["task_ids"])
    for waypoint in waypoints:
        if len(waypoint["tasks"]) >= 100:
            raise ValueError("at most 99 operations per merged waypoint")
        for index, operation in enumerate(waypoint["tasks"], 1):
            operation["id"] = waypoint["id"] * 100 + index
    return yaml.safe_dump(
        {"route": {"name": route_name, "waypoints": waypoints}},
        allow_unicode=True,
        sort_keys=False,
    ), mapping
