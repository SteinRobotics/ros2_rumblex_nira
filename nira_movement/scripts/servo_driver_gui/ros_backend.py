"""Connect the servo command center to Nira's movement node."""

import sys

import rclpy
from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication
from nira_interfaces.srv import ServoDashboard
from rclpy.executors import SingleThreadedExecutor
from rclpy.utilities import remove_ros_args

from gui import Window
from servo_bus import BusWorker


class DashboardClient:
    def __init__(self, node, executor):
        self.executor = executor
        self.client = node.create_client(ServoDashboard, "servo_dashboard")
        self.names = {}

    def request(self, operation, **fields):
        if not self.client.wait_for_service(timeout_sec=0.5):
            raise RuntimeError(f"Movement service unavailable: {self.client.service_name}")
        future = self.client.call_async(ServoDashboard.Request(operation=operation, **fields))
        self.executor.spin_until_future_complete(future, timeout_sec=2.0)
        if not future.done():
            self.client.remove_pending_request(future)
            raise TimeoutError("No service reply. Check the servo state before retrying.")
        reply = future.result()
        if reply.outcome != ServoDashboard.Response.VERIFIED:
            raise RuntimeError(reply.detail or "The movement node could not verify the operation")
        return reply

    def refresh(self):
        reply = self.request(ServoDashboard.Request.LIST)
        self.names = dict(zip(reply.ids, reply.names, strict=True))

    def name_for(self, sid):
        if sid not in self.names:
            raise ValueError(f"ID {sid} is not configured on the movement node")
        return self.names[sid]

    def snapshot(self, sid):
        state = self.request(ServoDashboard.Request.STATE, name=self.name_for(sid)).state
        if state.id != sid:
            raise RuntimeError("The device ID changed. Scan for servos again.")
        return {key: getattr(state, key) for key in (
            "minimum", "maximum", "speed", "acc", "goal", "position",
            "velocity", "voltage", "temperature", "status",
        )}

    settings = snapshot

    def close(self):
        self.names.clear()


class DashboardWorker(BusWorker):
    # Keep the same queue, feedback polling and Qt signals as the serial worker.
    operations = {
        "move": (ServoDashboard.Request.MOVE, ("goal", "speed", "acc")),
        "profile": (ServoDashboard.Request.PROFILE, ("speed", "acc")),
        "hold": (ServoDashboard.Request.HOLD, ()),
        "limits": (ServoDashboard.Request.LIMITS, ("minimum", "maximum")),
        "id": (ServoDashboard.Request.ID, ("new_id",)),
    }

    def __init__(self, dashboard):
        super().__init__()
        self.dashboard = dashboard

    def _execute(self, command, args):
        if command == "connect" and not args[2]:
            self.dashboard.refresh()
            self.bus = self.dashboard
            self.connected.emit(True, self.dashboard.client.service_name)
            return
        if self.bus is not self.dashboard or command in ("select", "disconnect"):
            super()._execute(command, args)
            return
        if command == "scan":
            self.sid = None
            self.dashboard.refresh()
            first, last = args
            for sid in sorted(self.dashboard.names):
                if first <= sid <= last and not self.cancel_scan:
                    self.found.emit(sid)
            self.scan_progress.emit(1, 1)
            self.scan_done.emit()
            return

        sid, *values = args
        if sid != self.sid:
            raise RuntimeError("Select the servo again before sending a command")
        operation, fields = self.operations[command]
        reply = self.dashboard.request(operation, name=self.dashboard.name_for(sid),
                                       **dict(zip(fields, values, strict=True)))
        if command == "id":
            self.sid = reply.state.id
            self.dashboard.names[self.sid] = self.dashboard.names.pop(sid)
            self.changed_id.emit(sid, self.sid)
        if command == "limits":
            self.selected.emit(sid, {key: getattr(reply.state, key)
                                     for key in ("minimum", "maximum", "speed", "acc")})
        self.activity.emit(reply.detail)
        self.completed.emit(command)


def main():
    rclpy.init()
    node = rclpy.create_node("servo_driver_gui")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        dashboard = DashboardClient(node, executor)
        qt_args = remove_ros_args(sys.argv)
        app = QApplication(qt_args)
        app.setStyle("Fusion")
        window = Window(worker=DashboardWorker(dashboard), ros_service=dashboard.client.service_name,
                        demo="--demo" in qt_args)
        timer = QTimer(window)
        timer.timeout.connect(lambda: None if rclpy.ok() else window.close())
        timer.start(250)
        window.show()
        return app.exec()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
