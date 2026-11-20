"""Exercise actual BestMan joint APIs with a command recorder, never hardware."""

import fcntl
import hashlib
import json
import math
import signal
from types import SimpleNamespace

import pytest

from Examples import move_stay_with_me_init as example
from Robotics_API import Bestman_Real_Flexiv

SERIAL = "Rizon4-OFFLINE_L"


@pytest.fixture
def setup(backend, monkeypatch, tmp_path):
    backend.information.serial_num = SERIAL
    backend.information.q_min = list(map(math.radians, [-160, -130, -170, -107, -170, -80, -170]))
    backend.information.q_max = list(map(math.radians, [160, 130, 170, 154, 170, 260, 170]))
    arm = Bestman_Real_Flexiv(SERIAL, robot=backend)
    state = SimpleNamespace(
        now=0.0,
        created=[],
        sends=[],
        follow=True,
        lag=0.0,
        on_send=None,
        arm=arm,
        controller=backend,
        tmp_path=tmp_path,
    )
    send, enable = backend.SendJointPosition, backend.Enable

    def dispatch(*args):
        send(*args)
        state.sends.append((state.now, args))
        if state.follow:
            backend.state.q = list(args[0])
        state.now += state.lag
        if state.on_send:
            state.on_send(backend)

    def enable_robot():
        enable()
        backend.is_ready = True

    def factory(serial, frequency=None, **kwargs):
        state.created.append(serial)
        return arm

    monkeypatch.setattr(backend, "SendJointPosition", dispatch)
    monkeypatch.setattr(backend, "Enable", enable_robot)
    monkeypatch.setattr(example, "Bestman_Real_Flexiv", factory)
    monkeypatch.setattr(example.time, "monotonic", lambda: state.now)
    monkeypatch.setattr(example.time, "sleep", lambda dt: setattr(state, "now", state.now + dt))
    monkeypatch.setattr(example.tempfile, "gettempdir", lambda: str(tmp_path))
    return state


def run(*args):
    return example.main(["--left-sn", SERIAL, *args])


def test_preview_uses_source_joint_degrees_without_connection(setup, capsys):
    assert run() == 0
    report = json.loads(capsys.readouterr().out)
    assert setup.created == []
    assert report["control_space"] == "joint"
    assert report["joint_order"] == "J1..J7"
    assert report["targets_deg"]["Left"] == pytest.approx(
        [
            23.839993772894275,
            103.11853942611782,
            -63.325904744874336,
            116.76360790640103,
            82.66197167425484,
            -22.298794228275693,
            55.77233573427813,
        ]
    )
    assert report["targets_rad"]["Left"] == pytest.approx(
        [math.radians(v) for v in report["targets_deg"]["Left"]]
    )
    assert report["max_joint_vel_deg_s"] == 5.0
    assert report["max_joint_acc_deg_s2"] == 10.0


def test_only_joint_commands_with_radian_speed_limits_and_stop(setup, capsys):
    assert run("--run", "--max-joint-vel-deg-s", "3", "--max-joint-acc-deg-s2", "6") == 0
    assert setup.created == [SERIAL]
    assert len(setup.sends) >= 10
    for _, (q, dq, vmax, amax) in setup.sends:
        assert q == pytest.approx(example.joint_target("Left"))
        assert dq == [0.0] * 7
        assert vmax == pytest.approx([math.radians(3)] * 7)
        assert amax == pytest.approx([math.radians(6)] * 7)
    events = setup.controller.events
    assert {e[0] for e in events} == {"SwitchMode", "SendJointPosition", "Stop"}
    assert [e[1] for e in events if e[0] == "SwitchMode"] == [setup.arm.mode.NRT_JOINT_POSITION]
    assert events[-1] == ("Stop",)
    assert '"status": "INIT_REACHED"' in capsys.readouterr().out


@pytest.mark.parametrize(
    "args",
    [
        ("--frequency", "nan"),
        ("--frequency", "101"),
        ("--timeout", "inf"),
        ("--max-joint-vel-deg-s", "0"),
        ("--max-joint-acc-deg-s2", "nan"),
        ("--left-sn", "Rizon4R-123456"),
        ("--left-sn", "Rizon4-"),
        ("--arm", "Right", "--right-sn", "Rizon4-OFFLINE_R"),
        ("--arm", "both", "--right-sn", "Rizon4-OFFLINE_R"),
    ],
)
def test_invalid_or_incompatible_targets_fail_before_connection(setup, args):
    assert run("--run", *args) == 2
    assert setup.created == []
    assert setup.controller.events == []


@pytest.mark.parametrize("failure", ["target_limit", "speed_limit", "model", "serial", "busy"])
def test_preflight_rejects_bad_controller_before_mutation(setup, failure):
    controller = setup.controller
    if failure == "target_limit":
        controller.information.q_max[3] = 2.0
    elif failure == "speed_limit":
        controller.information.dq_max[0] = math.radians(1)
    elif failure == "model":
        controller.information.model_name = "Rizon4R"
    elif failure == "serial":
        setup.arm.get_arm_id = lambda: "Rizon4-OTHER"
    else:
        controller.current_mode = setup.arm.mode.NRT_PLAN_EXECUTION
        controller.plan_busy = True
    assert run("--run", "--enable") == 2
    assert controller.events == []


def test_enable_is_explicit_without_clearing_existing_fault(setup):
    controller = setup.controller
    controller.is_ready = False
    assert run("--run") == 2
    assert controller.events == []
    controller.has_fault = True
    assert run("--run", "--enable") == 2
    assert controller.events == []
    controller.has_fault = False
    assert run("--run", "--enable") == 0
    assert ("Enable",) in controller.events
    assert not any(e[0] == "ClearFault" for e in controller.events)


@pytest.mark.parametrize(
    "failure", ["unreached", "one_joint", "moving", "disconnect", "mode", "fault", "send"]
)
def test_failed_motion_stops_without_false_success(setup, capsys, failure):
    if failure == "unreached":
        setup.follow = False
    elif failure == "one_joint":
        setup.on_send = lambda c: c.state.q.__setitem__(6, c.state.q[6] + math.radians(1.0))
    elif failure == "moving":
        setup.controller.state.dq[6] = math.radians(1.0)
    elif failure == "disconnect":
        setup.on_send = lambda c: setattr(c, "connected", lambda: False)
    elif failure == "mode":
        setup.on_send = lambda c: setattr(c, "current_mode", setup.arm.mode.IDLE)
    elif failure == "fault":
        setup.on_send = lambda c: setattr(c, "has_fault", True)
    else:

        def fail(c):
            raise RuntimeError("injected send failure")

        setup.on_send = fail
    assert run("--run", "--timeout", "0.8") == 2
    assert setup.controller.events[-1] == ("Stop",)
    assert "INIT_REACHED" not in capsys.readouterr().out


def test_blocking_status_read_cannot_send_after_deadline(setup, monkeypatch):
    original = setup.arm.get_status
    calls = 0

    def read():
        nonlocal calls
        calls += 1
        if calls == 4:  # main preflight, main enable check, helper preflight, loop
            setup.now += 2.0
        return original()

    monkeypatch.setattr(setup.arm, "get_status", read)
    assert run("--run", "--timeout", "1") == 2
    assert setup.sends == []
    assert setup.controller.events[-1] == ("Stop",)


def test_stop_failure_cannot_report_success(setup, monkeypatch, capsys):
    stop = setup.controller.Stop

    def fail():
        moving = setup.controller.current_mode == setup.arm.mode.NRT_JOINT_POSITION
        stop()
        if moving:
            raise RuntimeError("Stop failed")

    monkeypatch.setattr(setup.controller, "Stop", fail)
    assert run("--run") == 2
    output = capsys.readouterr()
    assert "INIT_REACHED" not in output.out
    assert "Stop failed" in output.err


@pytest.mark.parametrize("kind", ["sigterm", "keyboard"])
def test_interruption_stops_and_restores_signal(setup, kind):
    previous = signal.getsignal(signal.SIGTERM)

    def interrupt(c):
        if kind == "sigterm":
            signal.getsignal(signal.SIGTERM)(signal.SIGTERM, None)
        else:
            raise KeyboardInterrupt

    setup.on_send = interrupt
    assert run("--run") == 130
    assert setup.controller.events[-1] == ("Stop",)
    assert signal.getsignal(signal.SIGTERM) == previous


def test_slow_loop_skips_missed_ticks(setup):
    setup.lag = 0.08
    assert run("--run") == 0
    times = [t for t, _ in setup.sends]
    assert all(b - a >= 0.08 for a, b in zip(times, times[1:]))


def test_serial_lock_prevents_connection(setup):
    key = hashlib.sha256(SERIAL.encode()).hexdigest()[:24]
    with (setup.tmp_path / ("dj-real-" + key + ".lock")).open("a+") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        assert run("--run") == 2
    assert setup.created == []
