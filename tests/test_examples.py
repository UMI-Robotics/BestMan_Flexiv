"""Exercise bounded force examples through their real CLI/lifecycle, no hardware."""

import sys

import pytest

from Examples import force_control


def test_force_example_stops_without_automatic_home_or_zero(arm, backend, monkeypatch):
    monkeypatch.setattr(force_control, "Bestman_Real_Flexiv", lambda *a, **kw: arm)
    monkeypatch.setattr(sys, "argv", ["force_control", "Rizon4-123456", "--duration", "0.001"])
    force_control.main()
    names = [e[0] for e in backend.events]
    assert "SendCartesianMotionForce" in names
    assert "ExecutePlan" not in names and "ExecutePrimitive" not in names
    assert backend.events[-1] == ("Stop",)


def test_force_example_measured_limit_aborts_before_command(arm, backend, monkeypatch):
    monkeypatch.setattr(force_control, "Bestman_Real_Flexiv", lambda *a, **kw: arm)
    monkeypatch.setattr(sys, "argv", ["force_control", "Rizon4-123456", "--duration", "0.001"])
    backend.state.ext_wrench_in_world = [0, 0, 20, 0, 0, 0]
    with pytest.raises(RuntimeError, match="exceeded"):
        force_control.main()
    assert "SendCartesianMotionForce" not in [e[0] for e in backend.events]
    assert backend.events[-1] == ("Stop",)


def test_invalid_force_cli_does_not_connect(monkeypatch):
    def unexpected(*a, **kw):
        raise AssertionError("Should not connect")

    monkeypatch.setattr(force_control, "Bestman_Real_Flexiv", unexpected)
    monkeypatch.setattr(
        sys, "argv", ["force_control", "Rizon4-123456", "--wrench", "3", "0", "0", "0", "0", "0"]
    )
    with pytest.raises(SystemExit) as error:
        force_control.main()
    assert error.value.code == 2


def test_pure_motion_clears_passive_even_when_all_axes_were_motion(arm, backend):
    arm.configure_force_control([False] * 6, passive=True)
    arm.move_eef_to_goal_pose(arm.get_current_eef_pose())
    assert arm._passive_force is False
    assert ("SetPassiveForceControl", False) in backend.events
