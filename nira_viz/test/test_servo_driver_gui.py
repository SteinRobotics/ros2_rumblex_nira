"""Check the installed ROS dashboard without connecting to a servo bus."""

import runpy
from pathlib import Path
from unittest.mock import Mock

import pytest
from ament_index_python.packages import get_package_prefix
from nira_interfaces.srv import ServoDashboard


@pytest.fixture
def backend(monkeypatch):
    lib = Path(get_package_prefix("nira_viz")) / "lib" / "nira_viz"
    gui = lib / "servo_gui"
    for asset in ("chevron-up.svg", "chevron-down.svg"):
        assert (gui / "assets" / asset).is_file()
    assert (lib / "servo_driver_gui_node").is_file()
    monkeypatch.syspath_prepend(str(gui))
    return runpy.run_path(str(gui / "ros_backend.py"))


def test_connect_scan_and_select_only_read_state(backend):
    client = backend["DashboardClient"](Mock(), Mock())
    state = ServoDashboard.Response()
    state.state.id = 10
    client.request = Mock(side_effect=[ServoDashboard.Response(ids=[10], names=["head"]),
                                       ServoDashboard.Response(ids=[10], names=["head"]), state])
    client.client.service_name = "/robot/servo_dashboard"
    worker = backend["DashboardWorker"](client)
    worker._execute("connect", (client.client.service_name, 0, False))
    worker._execute("scan", (0, 253))
    worker._execute("select", (10,))

    request = ServoDashboard.Request
    assert [call.args[0] for call in client.request.call_args_list] == [
        request.LIST, request.LIST, request.STATE,
    ]
    assert client.request.call_args.kwargs == {"name": "head"}
    assert worker.sid == 10


def test_move_routes_to_selected_ros_name(backend):
    client = Mock()
    client.name_for.return_value = "head"
    client.request.return_value = ServoDashboard.Response(detail="Verified")
    worker = backend["DashboardWorker"](client)
    worker.bus = client
    worker.sid = 10
    with pytest.raises(RuntimeError, match="Select the servo again"):
        worker._execute("move", (11, 2000, 400, 20))
    client.request.assert_not_called()
    worker._execute("move", (10, 2000, 400, 20))
    client.name_for.assert_called_once_with(10)
    client.request.assert_called_once_with(ServoDashboard.Request.MOVE, name="head",
                                           goal=2000, speed=400, acc=20)
