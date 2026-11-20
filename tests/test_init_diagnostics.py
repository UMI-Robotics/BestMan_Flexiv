"""Joint diagnostics never mutate controller state; timeout reports follow Stop."""

import json
import math

import pytest
import test_stay_with_me_init as joint_tests

from Examples import move_stay_with_me_init as example

run = joint_tests.run
setup = joint_tests.setup


def report_line(output, status):
    return json.loads(
        next(line for line in output.splitlines() if line.startswith('{"status": "' + status + '"'))
    )


def test_read_joints_even_in_faulted_active_mode_without_mutation(setup, capsys):
    c = setup.controller
    c.current_mode = setup.arm.mode.NRT_PLAN_EXECUTION
    c.plan_busy, c.has_fault, c.is_ready = True, True, False
    c.state.q[3] = math.radians(-107.037)
    c.information.q_min[3] = math.radians(-106.998)
    assert run("--read-state") == 0
    assert c.events == []
    report = report_line(capsys.readouterr().out, "INIT_DIAGNOSTIC")
    assert report["motion_commands_sent"] is False
    assert report["actual_joint_rad"] == c.state.q
    assert report["target_joint_rad"] == pytest.approx(example.joint_target("Left"))
    assert report["max_joint_error_deg"] > 220
    assert report["near_joint_limits"][0]["joint"] == 4
    assert report["near_joint_limits"][0]["nearest_margin_deg"] == pytest.approx(-0.039)


def test_timeout_reports_joints_after_stop(setup, capsys):
    setup.follow = False
    assert run("--run", "--timeout", ".2") == 2
    assert setup.controller.events[-1] == ("Stop",)
    output = capsys.readouterr()
    report = report_line(output.out, "INIT_TIMEOUT")
    assert report["controller_status"]["mode"] == str(setup.arm.mode.IDLE)
    assert report["actual_joint_rad"] == setup.controller.state.q
    assert len(report["joints"]) == 7
    assert "INIT_REACHED" not in output.out


@pytest.mark.parametrize("flag", ["--enable", "--run"])
def test_read_state_rejects_mutation_flags_before_connection(setup, flag):
    assert run("--read-state", flag) == 2
    assert setup.created == []
    assert setup.controller.events == []
