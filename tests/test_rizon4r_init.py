"""Rizon4R native joint control and mixed-arm INIT, with injected SDK recorders."""

import math
from types import SimpleNamespace

import pytest

from Examples import move_stay_with_me_init as example
from Robotics_API import Bestman_Real_Flexiv
from Robotics_API.config import RobotConfig

SERIALS = {"Left": "Rizon4-OFFLINE_L", "Right": "Rizon4R-OFFLINE_R"}


@pytest.fixture
def pair(backend, monkeypatch, tmp_path):
    state = SimpleNamespace(now=0.0, created=[], order=[], arms={}, controllers={})
    for side, serial in SERIALS.items():
        controller = type(backend)()
        controller.information.serial_num = serial
        controller.information.model_name = "Rizon 4R" if side == "Right" else "Rizon 4"
        controller.information.q_min = [-3.0] * 7
        controller.information.q_max = [3.0] * 7
        state.controllers[side] = controller
        state.arms[side] = Bestman_Real_Flexiv(serial, robot=controller)
        send, stop = controller.SendJointPosition, controller.Stop

        def dispatch(q, dq, vmax, amax, side=side, controller=controller, send=send):
            send(q, dq, vmax, amax)
            state.order.append((side, "send"))
            controller.state.q = list(q)

        def stop_robot(side=side, stop=stop):
            state.order.append((side, "stop"))
            stop()

        monkeypatch.setattr(controller, "SendJointPosition", dispatch)
        monkeypatch.setattr(controller, "Stop", stop_robot)

    def factory(serial, *args, **kwargs):
        state.created.append(serial)
        return state.arms[next(s for s, sn in SERIALS.items() if sn == serial)]

    monkeypatch.setattr(example, "Bestman_Real_Flexiv", factory)
    monkeypatch.setattr(example.time, "monotonic", lambda: state.now)
    monkeypatch.setattr(example.time, "sleep", lambda dt: setattr(state, "now", state.now + dt))
    monkeypatch.setattr(example.tempfile, "gettempdir", lambda: str(tmp_path))
    return state


def run(*args):
    return example.main(["--left-sn", SERIALS["Left"], "--right-sn", SERIALS["Right"], *args])


def test_right_uses_rizon4r_native_joints_without_kinematics(pair):
    assert run("--arm", "Right", "--run") == 0
    assert pair.created == [SERIALS["Right"]]
    arm = pair.arms["Right"]
    assert arm.config.model == "Rizon4R"
    assert arm._robot_chain is None
    events = pair.controllers["Right"].events
    sends = [e for e in events if e[0] == "SendJointPosition"]
    assert len(sends) >= 10
    for _, q, dq, vmax, amax in sends:
        assert q == pytest.approx(example.joint_target("Right"))
        assert dq == [0.0] * 7
        assert vmax == pytest.approx([math.radians(5)] * 7)
        assert amax == pytest.approx([math.radians(10)] * 7)
    assert events[-1] == ("Stop",)
    assert {e[0] for e in events} == {"SendJointPosition", "SwitchMode", "Stop"}


def test_both_check_then_move_left_stop_then_right(pair):
    assert run("--arm", "both", "--run") == 0
    assert pair.created == list(SERIALS.values())
    first_right = pair.order.index(("Right", "send"))
    assert pair.order[first_right - 1] == ("Left", "stop")
    assert all(s == "Left" for s, _ in pair.order[:first_right])
    assert all(s == "Right" for s, _ in pair.order[first_right:])
    assert pair.order[-1] == ("Right", "stop")


@pytest.mark.parametrize("bad", ["model", "limit", "busy"])
def test_right_preflight_failure_does_not_move_left(pair, bad):
    right = pair.controllers["Right"]
    if bad == "model":
        right.information.model_name = "Rizon4"
    elif bad == "limit":
        right.information.q_max[3] = 2.0
    else:
        right.plan_busy = True
    assert run("--arm", "both", "--run") == 2
    assert all(c.events == [] for c in pair.controllers.values())


def test_right_read_only_accepts_rizon4r(pair):
    assert run("--arm", "Right", "--read-state") == 0
    assert pair.created == [SERIALS["Right"]]
    assert pair.controllers["Right"].events == []


@pytest.mark.parametrize("serial", ["Rizon4R-", "Rizon4-123456", "Rizon4s-123456"])
def test_wrong_right_serial_rejected_before_connection(pair, serial):
    assert run("--arm", "Right", "--right-sn", serial, "--run") == 2
    assert pair.created == []


def test_explicit_matching_config_and_mismatched_config(backend, tmp_path):
    backend.information.model_name = "Rizon4R"
    backend.information.serial_num = SERIALS["Right"]
    config = tmp_path / "rizon4r.json"
    config.write_text('{"model":"Rizon4R", "joint_velocity_scale":0.05}')
    arm = Bestman_Real_Flexiv(SERIALS["Right"], robot=backend, config=config)
    assert arm.config.model == "Rizon4R"
    assert arm.config.joint_velocity_scale == 0.05
    with pytest.raises(ValueError, match="Configuration model"):
        Bestman_Real_Flexiv(SERIALS["Right"], robot=backend, config=RobotConfig())
    assert backend.events == []


@pytest.mark.parametrize("model,dof,external", [("Rizon4", 7, 0), ("Rizon4R", 6, 0), ("Rizon4R", 7, 1)])
def test_right_native_controller_must_match_model_and_dof(backend, model, dof, external):
    backend.information.model_name = model
    backend.information.DoF, backend.information.DoF_e = dof, external
    with pytest.raises(ValueError, match="Expected Rizon4R"):
        Bestman_Real_Flexiv(SERIALS["Right"], robot=backend)
    assert backend.events == []


def test_spaced_right_serial_uses_the_same_identity_and_lock(pair):
    assert run("--arm", "Right", "--right-sn", "Rizon 4R-OFFLINE_R", "--read-state") == 0
    assert pair.created == [SERIALS["Right"]]
    assert pair.controllers["Right"].events == []
