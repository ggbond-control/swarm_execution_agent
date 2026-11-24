"""Session-fenced TaskHub adapter controlled by committed execution permits."""

import time

from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from swarm_protocol.audit import Audit
from swarm_protocol.contracts import decode, finite
from swarm_protocol.ros import main_node, publish, runtime_path
from swarm_protocol.storage import ProcessLock

from inspection_interfaces.srv import GetStatus, StartRoute

from .execution import Execution


class AgentNode(Node):
    def __init__(self):
        super().__init__("swarm_execution_agent")
        for key, value in [
            ("robot_id", ""),
            ("task_hub_prefix", ""),
            ("capabilities", ["device"]),
            ("nominal_speed", 1.0),
            ("odometry_topic", "/odometry_multi_maps"),
            ("permit_seconds", 10.0),
            ("odometry_max_age", 3.0),
            ("task_hub_call_timeout", 15.0),
        ]:
            self.declare_parameter(key, value)
        root = runtime_path(self, "agent")
        self.lock = ProcessLock(root / "process.lock")
        self.agent = Execution(
            self.get_parameter("robot_id").value,
            root,
            permit_seconds=self.get_parameter("permit_seconds").value,
        )
        self.audit = Audit(root, self.agent.robot, self.get_logger())
        self.output = self.create_publisher(String, "/swarm/agent/events", 100)
        self.hello = self.create_publisher(String, "/swarm/agent/state", 10)
        self.lease = self.create_publisher(String, "/swarm/agent/permit_request", 100)
        prefix = self.get_parameter("task_hub_prefix").value.rstrip("/")
        self.hub_clients = {
            name: self.create_client(cls, prefix + "/" + name)
            for name, cls in [
                ("start_route", StartRoute),
                ("pause_route", Trigger),
                ("cancel_route", Trigger),
                ("get_status", GetStatus),
            ]
        }
        self.latest_pose, self.pose_at = None, 0.0
        self.status_query, self.query_at, self.last_status_at = None, 0.0, 0.0
        self.last_lease = self.last_hello = self.last_outbox = 0.0
        self.status_state = 0
        self.create_subscription(
            Odometry,
            self.get_parameter("odometry_topic").value,
            self.odometry,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.create_subscription(String, "/swarm/agent/decision", self.decision, 100)
        self.create_subscription(String, "/swarm/agent/permit", self.permit, 100)
        self.create_subscription(String, "/swarm/agent/ack", self.ack, 100)
        nav_topic = (
            prefix + "/navigation_results" if prefix else "/task_hub/navigation_results"
        )
        self.create_subscription(
            String,
            nav_topic,
            self.navigation,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.create_service(Trigger, "/swarm/agent/local_stop", self.local_stop)
        self.create_timer(0.1, self.tick)
        self.get_logger().info(
            f"Agent={self.agent.robot} session={self.agent.session} "
            f"TaskHub={prefix or '/'}"
        )

    def odometry(self, message):
        try:
            if not message.header.frame_id:
                return
            p = message.pose.pose.position
            self.latest_pose = {
                "frame_id": message.header.frame_id,
                "pos": [
                    finite(p.x, "pose.x"),
                    finite(p.y, "pose.y"),
                    finite(p.z, "pose.z"),
                ],
            }
            self.pose_at = time.monotonic()
        except ValueError:
            self.latest_pose = None

    def record(self, kind, value):
        current = self.agent.current or {}
        route = value.get("route_name") or current.get("route_name", "")
        self.audit.record(kind, value, epoch=current.get("round_epoch", 0),
                          revision=current.get("revision", 0),
                          request_id=route, agent_session=self.agent.session)

    def decision(self, message):
        control = self.agent.control
        try:
            value = decode(message.data)
            if value["robot_id"] != self.agent.robot:
                return
            command = value["command"]
            self.record("decision", value)
            if command == "PREPARE":
                if not control.ready or control.mutating or control.stop_confirmation:
                    return
                if (
                    self.agent.current
                    and value["route_name"] != self.agent.current["route_name"]
                    and (self.agent.snapshot or {}).get("route_state") == "PAUSED"
                ):
                    control.stop_wanted = "CANCEL"
                    return
                if self.agent.prepare(value):
                    control.stop_cause = ""
                    self.agent.queue("prepared", {"plan_digest": value["plan_digest"]})
            elif command == "START":
                if (
                    value["route_name"] == (self.agent.current or {}).get("route_name")
                    and self.agent.current.get("start_phase") == "prepared"
                ):
                    control.start_wanted = True
            elif command in {"PAUSE", "CANCEL"}:
                if self.agent.current and (
                    (self.agent.snapshot or {}).get("finalized")
                    or (
                        command == "PAUSE"
                        and self.agent.current["start_phase"] == "stopped"
                    )
                ):
                    return
                if control.stop_confirmation:
                    return
                self.agent.expired = True
                self.agent.generation += 1
                control.stop_wanted = command
                control.start_wanted = False
            else:
                raise ValueError("unknown local decision")
        except Exception as error:
            self.get_logger().warning(f"Agent拒绝决定：{error}")

    def permit(self, message):
        self.agent.grant(decode(message.data), time.monotonic())

    def ack(self, message):
        value = decode(message.data)
        for event in self.agent.outbox.values():
            if (event["session_id"] == value.get("session_id")
                    and event["event_id"] == value.get("event_id")):
                self.record("result_ack", {**event, "ack": value})
                self.agent.ack(value["event_id"])
                break

    def navigation(self, message):
        try:
            if self.agent.sealed_unknown:
                return
            value = decode(message.data)
            self.agent.observe(value)
            self.record("navigation", value)
        except Exception as error:
            self.get_logger().warning(f"累计导航证据拒绝：{error}")

    def safe_controller(self):
        control = self.agent.control
        prefix = self.get_parameter("task_hub_prefix").value.rstrip("/")
        conflicts = {"mission_execution_agent", "mission_dispatch_server"} & set(
            self.get_node_names()
        )
        if conflicts:
            if control.ready:
                self.get_logger().error(
                    "发现其他协同控制节点，禁止继续控制：" + ",".join(sorted(conflicts))
                )
            control.ready = False
            return False
        return all(
            self.count_services(prefix + "/" + name) == 1
            for name in ("start_route", "pause_route", "cancel_route", "get_status")
        )

    def count_services(self, name):
        return sum(
            name
            in {
                service
                for service, _ in self.get_service_names_and_types_by_node(n, ns)
            }
            for n, ns in self.get_node_names_and_namespaces()
        )

    def call_mutation(self, name, request, callback):
        control = self.agent.control
        if not self.safe_controller():
            return False
        if control.mutating is not None or not self.hub_clients[name].service_is_ready():
            return False
        if name in {"pause_route", "cancel_route"}:
            control.stop_before = (self.agent.snapshot or {}).get("snapshot_sequence", 0)
        future = self.hub_clients[name].call_async(request)
        control.mutating = (name, future, time.monotonic())

        def done(result):
            control.mutating = None
            control.mutation_warned = None
            try:
                callback(result.result())
            except Exception as error:
                self.get_logger().error(f"TaskHub {name} 回调失败：{error}")
                control.stop_wanted = "CANCEL"

        future.add_done_callback(done)
        return True

    def confirm_stop(self, response, expected):
        control = self.agent.control
        if not self.agent.current:
            control.ready, control.stop_wanted = True, None
            return
        # Drain a cumulative snapshot after the stop service response before
        # allowing a replacement plan. A service ACK alone contains no arrivals.
        control.ready, control.stop_wanted = False, None
        self.agent.expired = True
        self.agent.generation += 1
        control.stop_confirmation = {
            "minimum": control.stop_before + 1,
            "state": expected,
            "deadline": time.monotonic() + 10.0,
            "message": response.message,
        }

    def on_cancel(self, response):
        if not response.success:
            self.get_logger().error("取消未确认：" + response.message)
            return
        self.confirm_stop(response, "CANCELLED")

    def on_pause(self, response):
        control = self.agent.control
        if response.success:
            self.confirm_stop(response, "PAUSED")
        else:
            control.stop_wanted = "CANCEL"
            self.get_logger().error("暂停未确认，尝试取消：" + response.message)

    def reconcile_stop(self, now):
        control = self.agent.control
        item = control.stop_confirmation
        if not item:
            return
        snapshot = self.agent.snapshot or {}
        confirmed = snapshot.get("finalized") or (
            snapshot.get("route_state") == item["state"]
            and snapshot.get("snapshot_sequence", 0) >= item["minimum"]
        )
        if not confirmed and now < item["deadline"]:
            return
        if not confirmed:
            self.agent.unknown("停止已确认，但10秒内未取得停止后的累计导航证据")
        self.agent.stopped()
        self.agent.queue(
            "stopped",
            {
                "success": True,
                "message": item["message"],
                "cause": control.stop_cause,
                "navigation_unknown": not confirmed,
                "navigation_result": self.agent.latest_result,
            },
        )
        control.stop_confirmation = None
        control.stop_wanted = None
        control.ready = True

    def start(self, now):
        control = self.agent.control
        if control.mutating or not self.hub_clients["start_route"].service_is_ready():
            return
        if not self.agent.begin_start(now):
            control.start_wanted = False
            return
        request = StartRoute.Request()
        request.route_config_path = self.agent.current["yaml_path"]
        generation = self.agent.generation
        control.start_wanted = False

        def callback(response):
            action = self.agent.started(generation, response.success)
            if action == "cancel":
                control.stop_wanted = "CANCEL"
            elif action == "started":
                self.agent.queue("started", {"success": True})
            elif action == "failed":
                self.agent.queue(
                    "error", {"message": response.message or "TaskHub start failed"}
                )

        self.call_mutation("start_route", request, callback)

    def query_status(self, now):
        if self.status_query and now - self.query_at > 2.0:
            self.status_query.cancel()
            self.status_query = None
        if (
            self.status_query
            or now - self.last_status_at < 1.0
            or not self.hub_clients["get_status"].service_is_ready()
        ):
            return
        self.last_status_at, self.query_at = now, now
        future = self.hub_clients["get_status"].call_async(GetStatus.Request())
        self.status_query = future
        generation = self.agent.generation

        def done(result):
            if self.status_query is not result:
                return
            self.status_query = None
            try:
                status = result.result().route_status
                self.status_state = status.state
                if generation != self.agent.generation or not self.agent.owned:
                    return
                if self.agent.snapshot and self.agent.snapshot["finalized"]:
                    return
                if status.state in {3, 4, 5}:
                    self.agent.terminal_since = (
                        self.agent.terminal_since or time.monotonic()
                    )
                elif status.state in {1, 2}:
                    self.agent.terminal_since = None
            except Exception as error:
                self.get_logger().warning(f"TaskHub状态查询失败：{error}")

        future.add_done_callback(done)

    def local_stop(self, _request, response):
        control = self.agent.control
        control.request_stop("CANCEL", "local_stop")
        self.agent.expired = True
        self.agent.generation += 1
        response.success, response.message = True, "本机停止已请求，不代表全局取消完成"
        return response

    def tick(self):
        control = self.agent.control
        now = time.monotonic()
        try:
            if (
                control.mutating
                and now - control.mutating[2]
                > self.get_parameter("task_hub_call_timeout").value
                and control.mutation_warned is not control.mutating[1]
            ):
                control.mutation_warned = control.mutating[1]
                self.agent.expired = True
                control.stop_cause = control.stop_cause or "task_hub_timeout"
                self.agent.generation += 1
                detail = (
                    "TaskHub "
                    + control.mutating[0]
                    + " 响应超时；物理执行状态未知，等待旧调用排空后补偿取消"
                )
                self.get_logger().error(detail)
                if self.agent.current:
                    self.agent.queue("error", {"message": detail})
                control.start_wanted = False
                control.stop_wanted = "CANCEL"
            self.reconcile_stop(now)
            if not control.ready and not control.stop_confirmation:
                if now - control.last_cancel > 1.0:
                    control.last_cancel = now
                    self.call_mutation(
                        "cancel_route", Trigger.Request(), self.on_cancel
                    )
            if self.agent.needs_pause(now):
                control.request_stop("PAUSE", "permit_expired")
                self.get_logger().warning("执行授权到期，暂停旧路线；未经确认不会恢复")
            if (
                control.stop_wanted
                and not control.stop_confirmation
                and now - control.last_cancel > 1.0
            ):
                control.last_cancel = now
                name = "pause_route" if control.stop_wanted == "PAUSE" else "cancel_route"
                self.call_mutation(
                    name,
                    Trigger.Request(),
                    self.on_pause if name == "pause_route" else self.on_cancel,
                )
            if (
                self.agent.current
                and not self.agent.expired
                and now - self.last_lease >= 1.0
            ):
                value = self.agent.request_permit(now)
                if value:
                    publish(self.lease, value)
                self.last_lease = now
            if control.start_wanted and now < self.agent.permit_until:
                self.start(now)
            self.query_status(now)
            if self.agent.terminal_since and now - self.agent.terminal_since > 10.0:
                self.agent.unknown("路线结束后10秒未取得有效封存快照")
                self.agent.terminal_since = None
            if now - self.last_outbox >= 1.0:
                for value in self.agent.outbox.values():
                    publish(self.output, value)
                    self.record("agent_event", value)
                self.last_outbox = now
            if now - self.last_hello >= 1.0:
                fresh = (
                    self.latest_pose is not None
                    and now - self.pose_at
                    <= self.get_parameter("odometry_max_age").value
                )
                publish(
                    self.hello,
                    {
                        "robot_id": self.agent.robot,
                        "session_id": self.agent.session,
                        "ready": control.ready and not control.mutating,
                        "pose": self.latest_pose if fresh else None,
                        "capabilities": self.get_parameter("capabilities").value,
                        "speed": self.get_parameter("nominal_speed").value,
                        "route_name": (self.agent.current or {}).get("route_name", ""),
                        "start_phase": (self.agent.current or {}).get(
                            "start_phase", ""
                        ),
                        "expired": self.agent.expired,
                        "task_hub_state": self.status_state,
                        "services_ready": all(
                            x.service_is_ready() for x in self.hub_clients.values()
                        ),
                    },
                )
                self.last_hello = now
        except Exception as error:
            self.get_logger().error(f"Agent保护性停止：{error}")
            control.request_stop("CANCEL", "local_error")

    def destroy_node(self):
        self.audit.close()
        self.lock.close()
        return super().destroy_node()


def main(args=None):
    main_node(AgentNode, args)
