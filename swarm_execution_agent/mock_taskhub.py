"""Local validation fixture only. Publishes odometry and never opens motion IO."""

import time
import uuid

import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger
from swarm_protocol.contracts import decode
from swarm_protocol.ros import main_node, publish

from inspection_interfaces.srv import GetStatus, StartRoute


class MockTaskHub(Node):
    def __init__(self):
        super().__init__("swarm_mock_taskhub")
        for k, v in [
            ("prefix", "/swarm/mock"),
            ("x", 2.0),
            ("y", 2.0),
            ("frame_id", "map_000"),
            ("auto_complete", False),
        ]:
            self.declare_parameter(k, v)
        prefix = self.get_parameter("prefix").value.rstrip("/")
        if not prefix or not prefix.startswith("/swarm/"):
            raise ValueError("mock services must stay below /swarm/")
        self.output = self.create_publisher(
            String,
            prefix + "/navigation_results",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.pose = self.create_publisher(Odometry, "/odometry_multi_maps", 10)
        self.status = self.create_publisher(String, prefix + "/audit", 10)
        self.create_service(StartRoute, prefix + "/start_route", self.start)
        self.create_service(GetStatus, prefix + "/get_status", self.get_status)
        self.create_service(Trigger, prefix + "/pause_route", self.pause)
        self.create_service(Trigger, prefix + "/cancel_route", self.cancel)
        self.create_subscription(String, prefix + "/inject", self.inject, 10)
        self.current = None
        self.starts = 0
        self.ids = []
        self.started_at = 0.0
        self.create_timer(0.2, self.tick)

    def start(self, request, response):
        if self.current and self.current["route_state"] in {"RUNNING", "PAUSED"}:
            response.success, response.message = False, "old mock route still active"
            return response
        with open(request.route_config_path) as stream:
            route = yaml.safe_load(stream)["route"]
        self.ids = [w["id"] for w in route["waypoints"]]
        self.current = {
            "protocol_version": 1,
            "route_name": route["name"],
            "execution_id": uuid.uuid4().hex,
            "snapshot_sequence": 1,
            "route_state": "RUNNING",
            "finalized": False,
            "reached_waypoint_ids": [],
            "skipped_waypoints": [],
            "pending_waypoint_ids": self.ids.copy(),
        }
        self.starts += 1
        self.started_at = time.monotonic()
        response.success, response.message = True, "mock route started"
        return response

    def get_status(self, _request, response):
        states = {
            "IDLE": 0,
            "RUNNING": 1,
            "PAUSED": 2,
            "COMPLETED": 3,
            "FAILED": 4,
            "CANCELLED": 5,
        }
        response.route_status.route_name = (self.current or {}).get("route_name", "")
        response.route_status.state = states[
            (self.current or {}).get("route_state", "IDLE")
        ]
        response.route_status.total_waypoints = len(self.ids)
        response.route_status.current_waypoint_index = len(
            (self.current or {}).get("reached_waypoint_ids", [])
        )
        return response

    def pause(self, _request, response):
        if self.current and not self.current["finalized"]:
            self.current["route_state"] = "PAUSED"
            self.current["snapshot_sequence"] += 1
        response.success, response.message = True, "mock active goals canceled; paused"
        return response

    def cancel(self, _request, response):
        if self.current and not self.current["finalized"]:
            self.current.update(route_state="CANCELLED", finalized=True)
            self.current["snapshot_sequence"] += 1
        response.success, response.message = True, "mock canceled"
        return response

    def inject(self, message):
        if not self.current or self.current["finalized"]:
            return
        value = decode(message.data)
        reached = value.get("reached", [])
        skipped = value.get("skipped", [])
        self.current.update(
            reached_waypoint_ids=reached,
            skipped_waypoints=[
                {
                    "waypoint_id": n,
                    "reason": "injected NAVIGATION_ABORTED",
                    "reason_code": "NAVIGATION_FAILED",
                }
                for n in skipped
            ],
            pending_waypoint_ids=[
                n for n in self.ids if n not in set(reached + skipped)
            ],
            route_state=value.get("state", "COMPLETED"),
            finalized=value.get("finalized", True),
        )
        self.current["snapshot_sequence"] += 1

    def tick(self):
        pose = Odometry()
        pose.header.frame_id = self.get_parameter("frame_id").value
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.pose.position.x = self.get_parameter("x").value
        pose.pose.pose.position.y = self.get_parameter("y").value
        pose.pose.pose.orientation.w = 1.0
        self.pose.publish(pose)
        if self.current:
            if (
                self.get_parameter("auto_complete").value
                and self.current["route_state"] == "RUNNING"
                and time.monotonic() - self.started_at > 2.0
            ):
                self.current.update(
                    route_state="COMPLETED",
                    finalized=True,
                    reached_waypoint_ids=self.ids.copy(),
                    pending_waypoint_ids=[],
                )
                self.current["snapshot_sequence"] += 1
            publish(self.output, self.current)
        publish(self.status, {"starts": self.starts, "route": self.current})


def main(args=None):
    main_node(MockTaskHub, args)
