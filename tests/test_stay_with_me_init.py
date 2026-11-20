"""Check the standalone INIT example through real BestMan APIs and offline SDK recorders."""

import fcntl
import hashlib
import json
import signal
from types import SimpleNamespace

import pytest

from Examples import move_stay_with_me_init as example
from Robotics_API import Bestman_Real_Flexiv

SERIALS = {"Left": "Rizon4-OFFLINE_L", "Right": "Rizon4-OFFLINE_R"}


@pytest.fixture
def setup(backend, monkeypatch, tmp_path):
    state = SimpleNamespace(now=0.0, created=[], sends=[], order=[], follow=True, lag=0.0, on_send=None)
    state.controllers, state.arms, state.tools = {}, {}, {}
    for side, serial in SERIALS.items():
        controller = type(backend)()
        controller.information.serial_num = serial
        arm = Bestman_Real_Flexiv(serial, robot=controller)
        tool = SimpleNamespace(
            name="ModifiedTip",
            tcp_location=[0.01, 0.02, 0.19, 1, 0, 0, 0],
            mass=0.7,
            CoM=[0, 0, 0.06],
            inertia=[0.001] * 6,
        )
        state.controllers[side], state.arms[side], state.tools[side] = controller, arm, tool
        monkeypatch.setattr(arm, "get_tool", lambda tool=tool: (tool.name, tool))
        send, stop, enable = controller.SendCartesianMotionForce, controller.Stop, controller.Enable

        def dispatch(*args, side=side, controller=controller, send=send):
            send(*args)
            state.sends.append((side, list(args[0]), state.now, args[1:]))
            state.order.append((side, "send"))
            if state.follow:
                controller.state.tcp_pose = list(args[0])
            state.now += state.lag
            if state.on_send:
                state.on_send(side, controller)

        def stop_robot(side=side, stop=stop):
            state.order.append((side, "stop"))
            stop()

        def enable_robot(controller=controller, enable=enable):
            enable()
            controller.is_ready = True

        monkeypatch.setattr(controller, "SendCartesianMotionForce", dispatch)
        monkeypatch.setattr(controller, "Stop", stop_robot)
        monkeypatch.setattr(controller, "Enable", enable_robot)

    def factory(serial, frequency=None, **kwargs):
        state.created.append(serial)
        return state.arms[next(side for side in SERIALS if SERIALS[side] == serial)]

    def sleep(seconds):
        state.now += seconds

    monkeypatch.setattr(example, "Bestman_Real_Flexiv", factory)
    monkeypatch.setattr(example.time, "monotonic", lambda: state.now)
    monkeypatch.setattr(example.time, "sleep", sleep)
    monkeypatch.setattr(example.tempfile, "gettempdir", lambda: str(tmp_path))
    state.tmp_path = tmp_path
    return state


def run(*extra):
    return example.main(["--left-sn", SERIALS["Left"], "--right-sn", SERIALS["Right"], *extra])


def test_preview_never_connects_and_preserves_wxyz_targets(setup, capsys):
    assert run() == 0
    assert setup.created == []
    report = json.loads(capsys.readouterr().out)
    assert report["quaternion_order"] == "wxyz"
    assert report["frame"] == "WORLD"
    assert report["targets"] == {side: example.ROBOTS[side]["init_tcp"] for side in SERIALS}


def test_both_arms_send_exact_tcp_targets_sequentially_through_bestman(setup, capsys):
    assert run("--run") == 0
    assert setup.created == list(SERIALS.values())
    for side in SERIALS:
        sent = [event for event in setup.sends if event[0] == side]
        assert len(sent) >= 10
        assert all(event[1] == example.ROBOTS[side]["init_tcp"] for event in sent)
        assert all(event[3] == ([0.0] * 6, [0.0] * 6, 0.03, 0.2, 0.05, 0.3) for event in sent)
        assert setup.controllers[side].events[-1] == ("Stop",)
    first_right = setup.order.index(("Right", "send"))
    assert all(side == "Right" for side, action in setup.order[first_right:])
    assert setup.order[first_right - 2] == ("Left", "stop")
    assert capsys.readouterr().out.count('"status": "INIT_REACHED"') == 2


@pytest.mark.parametrize(
    "args",
    [
        ("--frequency", "nan"),
        ("--max-linear-vel", "-1"),
        ("--timeout", "inf"),
        ("--left-sn", "Rizon4R-123456"),
        ("--right-sn", SERIALS["Left"]),
    ],
)
def test_invalid_settings_or_models_fail_before_connecting(setup, args):
    assert run("--run", *args) == 2
    assert setup.created == []


def test_both_preflight_before_any_commands(setup):
    setup.tools["Right"].name = "Flange"
    assert run("--run") == 2
    assert setup.sends == []
    assert all(controller.events == [] for controller in setup.controllers.values())


def test_active_controller_is_not_interrupted(setup):
    controller = setup.controllers["Left"]
    controller.current_mode = setup.arms["Left"].mode.NRT_PLAN_EXECUTION
    controller.plan_busy = True
    assert run("--arm", "Left", "--run") == 2
    assert controller.events == []


def test_enable_is_explicit_and_does_not_clear_existing_faults(setup):
    controller = setup.controllers["Left"]
    controller.is_ready = False
    assert run("--arm", "Left", "--run") == 2
    assert controller.events == []
    controller.has_fault = True
    assert run("--arm", "Left", "--run", "--enable") == 2
    assert controller.events == []
    controller.has_fault = False
    assert run("--arm", "Left", "--run", "--enable") == 0
    assert ("Enable",) in controller.events
    assert not any(event[0] == "ClearFault" for event in controller.events)


@pytest.mark.parametrize("failure", ["unreached", "moving", "disconnect", "mode", "tool"])
def test_motion_failures_stop_without_reporting_success(setup, capsys, failure):
    if failure == "unreached":
        setup.follow = False
    elif failure == "moving":
        setup.controllers["Left"].state.tcp_vel = [0.01, 0, 0, 0, 0, 0]
    elif failure == "disconnect":
        setup.on_send = lambda side, controller: setattr(controller, "connected", lambda: False)
    elif failure == "mode":
        setup.on_send = lambda side, controller: setattr(
            controller, "current_mode", setup.arms[side].mode.IDLE
        )
    else:
        setup.on_send = lambda side, controller: setattr(setup.tools[side], "name", "ChangedTool")
    assert run("--arm", "Left", "--run", "--timeout", "0.8") == 2
    assert setup.controllers["Left"].events[-1] == ("Stop",)
    assert "INIT_REACHED" not in capsys.readouterr().out


def test_blocking_tool_read_cannot_dispatch_after_timeout(setup, monkeypatch):
    arm = setup.arms["Left"]
    read_tool = arm.get_tool
    calls = 0

    def delayed_read():
        nonlocal calls
        calls += 1
        if calls == 3:  # First loop's tool check, after main/helper preflight.
            setup.now += 2.0
        return read_tool()

    monkeypatch.setattr(arm, "get_tool", delayed_read)
    assert run("--arm", "Left", "--run", "--timeout", "1") == 2
    assert setup.sends == []
    assert setup.controllers["Left"].events[-1] == ("Stop",)


def test_stop_failure_cannot_report_init_reached(setup, monkeypatch, capsys):
    controller = setup.controllers["Left"]
    stop = controller.Stop

    def failing_stop():
        was_moving = controller.current_mode == setup.arms["Left"].mode.NRT_CARTESIAN_MOTION_FORCE
        stop()
        if was_moving:
            raise RuntimeError("Stop failed")

    monkeypatch.setattr(controller, "Stop", failing_stop)
    assert run("--arm", "Left", "--run") == 2
    output = capsys.readouterr()
    assert "INIT_REACHED" not in output.out
    assert "Stop failed" in output.err


def test_sigterm_stops_and_restores_original_handler(setup):
    previous = signal.getsignal(signal.SIGTERM)
    setup.on_send = lambda side, controller: signal.getsignal(signal.SIGTERM)(signal.SIGTERM, None)
    assert run("--arm", "Left", "--run") == 130
    assert setup.controllers["Left"].events[-1] == ("Stop",)
    assert signal.getsignal(signal.SIGTERM) == previous


def test_slow_commands_skip_missed_ticks_without_bursts(setup):
    setup.lag = 0.08
    assert run("--arm", "Left", "--run") == 0
    times = [event[2] for event in setup.sends]
    assert all(b - a >= 0.08 for a, b in zip(times, times[1:]))


def test_local_serial_lock_is_respected_before_connection(setup):
    key = hashlib.sha256(SERIALS["Left"].encode()).hexdigest()[:24]
    with (setup.tmp_path / ("dj-real-" + key + ".lock")).open("a+") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        assert run("--arm", "Left", "--run") == 2
    assert setup.created == []


def test_arrival_accepts_equivalent_quaternion_sign():
    target = example.ROBOTS["Left"]["init_tcp"]
    distance, angle = example.pose_error(target[:3] + [-x for x in target[3:]], target)
    assert distance == 0
    assert angle < 1e-7
