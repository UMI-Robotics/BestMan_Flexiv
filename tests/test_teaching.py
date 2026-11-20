"""Exercise button-controlled teaching with real pipe input and an offline SDK recorder."""

import json
import os
import select
import sys
import time
from types import SimpleNamespace

import pytest

from Examples import move_eef_follow_trajectory, teach_by_demonstration


@pytest.fixture
def teaching(arm, backend, monkeypatch, tmp_path):
    output = tmp_path / "poses.json"
    read_fd, write_fd = os.pipe()
    stream = os.fdopen(read_fd)
    state = SimpleNamespace(button=False, now=0.0, name="Hold", steps=[], tick=0, acknowledge=True)
    monkeypatch.setattr(sys, "stdin", stream)
    monkeypatch.setattr(sys, "argv", ["teach", "Rizon4-123456", "--output", str(output)])
    monkeypatch.setattr(teach_by_demonstration, "Bestman_Real_Flexiv", lambda *a, **kw: arm)
    # Also let the old blocking-input implementation exit, to expose the wrong API path.
    monkeypatch.setattr("builtins.input", lambda *a: "q")
    monkeypatch.setattr(backend, "enabling_button_pressed", lambda: state.button)
    monkeypatch.setattr(time, "monotonic", lambda: state.now)
    monkeypatch.setattr(
        backend, "primitive_states", lambda: {"primitiveName": state.name, "terminated": backend.terminated}
    )
    execute = backend.ExecutePrimitive
    stop = backend.Stop

    def execute_primitive(name, parameters, block_until_started=True):
        execute(name, parameters, block_until_started)
        backend.plan_busy = True
        if state.acknowledge:
            state.name = name
            backend.terminated = False

    def stop_robot():
        stop()
        backend.plan_busy = False
        backend.terminated = True

    def poll(readers, writers, errors, timeout):
        assert state.tick < len(state.steps), "Teaching did not terminate within the scripted observations"
        step = state.steps[state.tick]
        state.tick += 1
        state.now += step.get("elapsed", 0.02)
        if "pressed" in step:
            state.button = step["pressed"]
        if "mutate" in step:
            step["mutate"]()
        if step.get("interrupt"):
            raise KeyboardInterrupt
        if "command" in step:
            os.write(write_fd, step["command"].encode())
            return readers, [], []
        if step.get("eof"):
            os.close(write_fd)
            return readers, [], []
        return [], [], []

    monkeypatch.setattr(backend, "ExecutePrimitive", execute_primitive)
    monkeypatch.setattr(backend, "Stop", stop_robot)
    monkeypatch.setattr(select, "select", poll)
    state.output = output
    yield state
    stream.close()
    try:
        os.close(write_fd)
    except OSError:
        pass


def test_teaching_uses_documented_primitive_and_stops_on_release(teaching, backend):
    teaching.steps = [
        {"pressed": False},
        {"pressed": True},
        {},
        {},
        {"pressed": False},
        {"command": "q\n"},
    ]
    teach_by_demonstration.main()
    calls = [event for event in backend.events if event[0] == "ExecutePrimitive"]
    assert calls == [("ExecutePrimitive", "FloatingCartesian", {}, False)]
    assert not any(event[0] == "ExecutePlan" for event in backend.events)
    assert backend.events[-1] == ("Stop",)
    # Stop on button release must precede the separate final cleanup Stop.
    assert sum(event == ("Stop",) for event in backend.events[backend.events.index(calls[0]) + 1 :]) == 2


def test_teaching_requires_release_before_first_activation(teaching, backend):
    def still_not_started():
        assert not any(event[0] == "ExecutePrimitive" for event in backend.events)

    teaching.steps = [
        {"pressed": True},
        {"pressed": True, "mutate": still_not_started},
        {"pressed": False},
        {"pressed": True},
        {"command": "q\n"},
    ]
    teach_by_demonstration.main()
    assert sum(event[0] == "ExecutePrimitive" for event in backend.events) == 1


def test_teaching_records_only_after_release_and_replays_wxyz(teaching, backend, arm, monkeypatch, capsys):
    values = [0.4, 0.1, 0.5, 0.8, 0.2, 0.4, 0.4]
    backend.state.tcp_pose = values.copy()
    teaching.steps = [
        {"pressed": False},
        {"pressed": True},
        {"command": "r\n"},
        {"pressed": False, "command": "r\n"},
        {"command": "q\n"},
    ]
    teach_by_demonstration.main()
    assert json.loads(teaching.output.read_text()) == [values]
    assert "Release the enabling button before recording" in capsys.readouterr().out
    backend.events.clear()
    monkeypatch.setattr(move_eef_follow_trajectory, "Bestman_Real_Flexiv", lambda *a, **kw: arm)
    monkeypatch.setattr(sys, "argv", ["replay", "Rizon4-123456", "--trajectory", str(teaching.output)])
    move_eef_follow_trajectory.main()
    sent = [event for event in backend.events if event[0] == "SendCartesianMotionForce"]
    assert len(sent) == 1 and sent[0][1] == values
    assert backend.events[-1] == ("Stop",)


@pytest.mark.parametrize("failure", ["fault", "disconnect", "mode_changed", "primitive_ended"])
def test_teaching_stops_and_saves_existing_records_on_controller_failure(teaching, backend, arm, failure):
    def fail():
        if failure == "fault":
            backend.has_fault = True
        elif failure == "disconnect":
            backend.connected = lambda: False
        elif failure == "mode_changed":
            backend.current_mode = arm.mode.NRT_PLAN_EXECUTION
        else:
            backend.terminated = True

    teaching.steps = [
        {"pressed": False, "command": "r\n"},
        {"pressed": True},
        {},
        {"mutate": fail, "command": "r\n"},
    ]
    with pytest.raises(RuntimeError):
        teach_by_demonstration.main()
    assert json.loads(teaching.output.read_text()) == [backend.state.tcp_pose]
    assert backend.events[-1] == ("Stop",)


def test_teaching_rechecks_button_after_mode_switch(teaching, backend):
    switch = backend.SwitchMode

    def release_during_switch(mode):
        switch(mode)
        teaching.button = False

    backend.SwitchMode = release_during_switch
    teaching.steps = [{"pressed": False}, {"pressed": True}, {"command": "q\n"}]
    teach_by_demonstration.main()
    assert not any(event[0] == "ExecutePrimitive" for event in backend.events)


def test_teaching_stops_if_primitive_does_not_start(teaching, backend):
    teaching.acknowledge = False
    teaching.steps = [{"pressed": False}, {"pressed": True}] + [{} for _ in range(110)]
    with pytest.raises(RuntimeError, match="did not start"):
        teach_by_demonstration.main()
    assert backend.events[-1] == ("Stop",)


def test_teaching_stops_after_polling_stall(teaching, backend):
    teaching.steps = [{"pressed": False}, {"pressed": True}, {"elapsed": 0.5}]
    with pytest.raises(RuntimeError, match="monitoring was delayed"):
        teach_by_demonstration.main()
    assert backend.events[-1] == ("Stop",)


@pytest.mark.parametrize("exit_step", [{"eof": True}, {"interrupt": True}])
def test_teaching_stops_and_saves_on_input_exit(teaching, backend, exit_step):
    teaching.steps = [{"pressed": False, "command": "r\n"}, {"pressed": True}, exit_step]
    teach_by_demonstration.main()
    assert json.loads(teaching.output.read_text()) == [backend.state.tcp_pose]
    assert backend.events[-1] == ("Stop",)


def test_teaching_reports_actual_primitive_without_reissuing_it(teaching, backend, capsys):
    teaching.steps = [{"pressed": False}, {"pressed": True}, {"command": "s\n"}, {"command": "q\n"}]
    teach_by_demonstration.main()
    text = capsys.readouterr().out
    assert "primitive=FloatingCartesian" in text
    assert "enabling_button=PRESSED" in text
    assert sum(event[0] == "ExecutePrimitive" for event in backend.events) == 1


def test_teaching_joint_primitive_option(teaching, backend, monkeypatch):
    monkeypatch.setattr(sys, "argv", sys.argv + ["--primitive", "FloatingJoint"])
    teaching.steps = [{"pressed": False}, {"pressed": True}, {"command": "q\n"}]
    teach_by_demonstration.main()
    assert ("ExecutePrimitive", "FloatingJoint", {}, False) in backend.events


def test_teaching_partial_input_does_not_block_button_monitoring(teaching, backend):
    teaching.steps = [
        {"pressed": False},
        {"pressed": True, "command": "s"},
        {"pressed": False},
        {"command": "\nq\n"},
        {},
    ]
    teach_by_demonstration.main()
    assert backend.events[-2:] == [("Stop",), ("Stop",)]


def test_teaching_release_stops_a_primitive_still_loading(teaching, backend):
    teaching.acknowledge = False
    teaching.steps = [{"pressed": False}, {"pressed": True}, {"pressed": False}, {"command": "q\n"}]
    teach_by_demonstration.main()
    assert backend.events[-2:] == [("Stop",), ("Stop",)]


def test_teaching_command_failure_stops_and_preserves_records(teaching, backend):
    def reject(*args):
        raise RuntimeError("Primitive license missing")

    backend.ExecutePrimitive = reject
    teaching.steps = [{"pressed": False, "command": "r\n"}, {"pressed": True}]
    with pytest.raises(RuntimeError, match="license missing"):
        teach_by_demonstration.main()
    assert json.loads(teaching.output.read_text()) == [backend.state.tcp_pose]
    assert backend.events[-1] == ("Stop",)


def test_teaching_does_not_record_if_stop_reveals_a_fault(teaching, backend):
    stop = backend.Stop

    def fault_after_stop():
        stop()
        backend.has_fault = True

    teaching.steps = [
        {"pressed": False},
        {"pressed": True},
        {"pressed": False, "command": "r\n", "mutate": lambda: setattr(backend, "Stop", fault_after_stop)},
    ]
    with pytest.raises(RuntimeError, match="unavailable before recording"):
        teach_by_demonstration.main()
    assert json.loads(teaching.output.read_text()) == []
