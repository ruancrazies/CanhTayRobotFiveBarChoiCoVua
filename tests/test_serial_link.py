"""Transports, line framing, reconnect handling and the in-flight limit (SPEC 5 and 5.3)."""

from __future__ import annotations

import threading
import time
from typing import Any, Callable, Optional

import pytest

from desktop_app.hardware.protocol import Alarm, Command, DataLine, Done, ErrorResponse, Ok
from config.protocol_commands import AlarmCode, ErrorCode
from desktop_app.hardware.serial_link import (
    BadLine,
    Connected,
    ConnectionLost,
    Disconnected,
    LineFramer,
    LinkDownError,
    LinkError,
    LinkMessage,
    LinkSettings,
    LinkState,
    LoopbackTransport,
    PySerialTransport,
    ReconnectFailed,
    ReconnectPolicy,
    Reply,
    SendTimeoutError,
    SerialLink,
    TransportError,
    Unsolicited,
    loopback_pair,
)

WAIT_S = 2.0  # generous upper bound for "eventually"; the happy paths take milliseconds


def wait_for(predicate: Callable[[], bool], timeout_s: float = WAIT_S) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.001)
    return predicate()


PING = Command("PING", {})
POS = Command("POS", {})
ESTOP = Command("ESTOP", {})
WAIT = Command("WAIT", {})


def move(a: float = 1.0) -> Command:
    return Command("MOVE", {"A": a, "B": 2.0, "F": 3.0})


# --------------------------------------------------------------------------- #
# LineFramer
# --------------------------------------------------------------------------- #


def lines(frames: list[Any]) -> list[Optional[str]]:
    return [f.text for f in frames]


def test_framer_splits_lines_and_tolerates_crlf() -> None:
    framer = LineFramer(64)
    assert lines(framer.feed(b"ok\ndone\r\nerror:3\n")) == ["ok", "done", "error:3"]


def test_framer_keeps_a_partial_line_until_its_newline() -> None:
    framer = LineFramer(64)
    assert framer.feed(b"!alarm:ES") == []
    assert framer.feed(b"TOP") == []
    assert lines(framer.feed(b"\nok\n")) == ["!alarm:ESTOP", "ok"]


def test_framer_works_one_byte_at_a_time() -> None:
    framer = LineFramer(64)
    out: list[Optional[str]] = []
    for byte in b"POS A=1 B=2\nok\n":
        out += lines(framer.feed(bytes([byte])))
    assert out == ["POS A=1 B=2", "ok"]


def test_framer_drops_empty_lines() -> None:
    assert lines(LineFramer(64).feed(b"\n\r\n\nok\n\n")) == ["ok"]


def test_framer_reports_a_line_that_is_too_long_and_recovers_at_the_next_newline() -> None:
    framer = LineFramer(8)
    frames = framer.feed(b"x" * 20 + b"\nok\n")
    assert [f.error for f in frames][0] == "line longer than 8 bytes"
    assert lines(frames)[1:] == ["ok"]
    # The overflow is reported once, not once per extra byte.
    assert sum(f.error is not None for f in LineFramer(8).feed(b"y" * 500 + b"\n")) == 1


def test_framer_overflow_across_several_feeds() -> None:
    framer = LineFramer(4)
    errors = []
    for _ in range(5):
        errors += [f.error for f in framer.feed(b"zzz") if f.error]
    assert len(errors) == 1
    assert lines(framer.feed(b"\nok\n")) == ["ok"]


def test_framer_reports_non_ascii_and_control_characters() -> None:
    frames = LineFramer(64).feed(b"ok\xff\n" + b"o\x01k\n" + b"ok\n")
    assert [f.error for f in frames] == ["not ASCII", "control characters in the line", None]
    assert frames[2].text == "ok"


def test_framer_reset_forgets_a_partial_line() -> None:
    framer = LineFramer(64)
    framer.feed(b"half a li")
    framer.reset()
    assert lines(framer.feed(b"ok\n")) == ["ok"]


def test_framer_rejects_a_zero_limit() -> None:
    with pytest.raises(ValueError):
        LineFramer(0)


# --------------------------------------------------------------------------- #
# Loopback transport
# --------------------------------------------------------------------------- #


def open_pair(chunk_size: Optional[int] = None) -> Any:
    pair = loopback_pair(chunk_size)
    pair.pc.open()
    pair.device.open()
    return pair


def test_loopback_carries_bytes_both_ways() -> None:
    pair = open_pair()
    pair.pc.write(b"PING\n")
    assert pair.device.read(0.5) == b"PING\n"
    pair.device.write(b"ok\n")
    assert pair.pc.read(0.5) == b"ok\n"


def test_loopback_read_times_out_with_empty_bytes() -> None:
    pair = open_pair()
    started = time.monotonic()
    assert pair.pc.read(0.05) == b""
    assert 0.04 <= time.monotonic() - started < 1.0


def test_loopback_read_wakes_when_data_arrives() -> None:
    pair = open_pair()
    threading.Timer(0.02, pair.device.write, args=(b"ok\n",)).start()
    started = time.monotonic()
    assert pair.pc.read(1.0) == b"ok\n"
    assert time.monotonic() - started < 0.5


def test_loopback_chunking_delivers_small_pieces() -> None:
    pair = open_pair(chunk_size=2)
    pair.pc.write(b"abcde")
    assert [pair.device.read(0.1) for _ in range(3)] == [b"ab", b"cd", b"e"]


def test_loopback_closed_end_cannot_be_used() -> None:
    pair = loopback_pair()
    with pytest.raises(ConnectionLost):
        pair.pc.write(b"x")
    with pytest.raises(ConnectionLost):
        pair.pc.read(0.01)
    pair.pc.open()
    pair.pc.close()
    pair.pc.close()  # twice is fine
    assert not pair.pc.is_open
    with pytest.raises(ConnectionLost):
        pair.pc.read(0.01)


def test_cutting_the_cable_breaks_both_ends_and_loses_bytes_in_transit() -> None:
    pair = open_pair()
    pair.device.write(b"ok\n")
    pair.cut()
    assert not pair.connected
    for end in (pair.pc, pair.device):
        with pytest.raises(ConnectionLost, match="unplugged"):
            end.write(b"x")
        with pytest.raises(ConnectionLost, match="unplugged"):
            end.read(0.01)
    with pytest.raises(ConnectionLost):
        pair.pc.open()


def test_cutting_the_cable_wakes_a_blocked_reader() -> None:
    pair = open_pair()
    result: list[object] = []

    def reader() -> None:
        try:
            result.append(pair.pc.read(5.0))
        except ConnectionLost as exc:
            result.append(exc)

    thread = threading.Thread(target=reader)
    thread.start()
    time.sleep(0.02)
    pair.cut()
    thread.join(timeout=2.0)
    assert not thread.is_alive()
    assert isinstance(result[0], ConnectionLost)


def test_closing_wakes_a_blocked_reader() -> None:
    pair = open_pair()
    result: list[object] = []
    thread = threading.Thread(target=lambda: result.append(_read_or_error(pair.pc, 5.0)))
    thread.start()
    time.sleep(0.02)
    pair.pc.close()
    thread.join(timeout=2.0)
    assert not thread.is_alive() and isinstance(result[0], ConnectionLost)


def _read_or_error(end: LoopbackTransport, timeout_s: float) -> object:
    try:
        return end.read(timeout_s)
    except ConnectionLost as exc:
        return exc


def test_restoring_the_cable_requires_reopening_both_ends_and_drops_old_bytes() -> None:
    pair = open_pair()
    pair.pc.write(b"old\n")
    pair.cut()
    pair.restore()
    assert pair.connected
    with pytest.raises(ConnectionLost):  # a replugged port has to be reopened
        pair.pc.write(b"x")
    pair.pc.open()
    pair.device.open()
    assert pair.device.read(0.01) == b""
    pair.pc.write(b"new\n")
    assert pair.device.read(0.5) == b"new\n"


def test_inject_to_pc_delivers_raw_noise() -> None:
    pair = open_pair()
    pair.inject_to_pc(b"\xff\xfe garbage")
    assert pair.pc.read(0.5) == b"\xff\xfe garbage"


def test_loopback_pair_rejects_a_bad_chunk_size() -> None:
    with pytest.raises(ValueError):
        loopback_pair(0)


# --------------------------------------------------------------------------- #
# PySerialTransport
# --------------------------------------------------------------------------- #


def test_pyserial_transport_needs_a_baud_rate_and_has_no_default() -> None:
    with pytest.raises(TypeError):
        PySerialTransport("loop://", write_timeout_s=1.0)  # type: ignore[call-arg]
    for bad in (0, -9600, 1.5, True, None):
        with pytest.raises(ValueError):
            PySerialTransport("loop://", bad, write_timeout_s=1.0)  # type: ignore[arg-type]
    with pytest.raises(ValueError):
        PySerialTransport("loop://", 9600, write_timeout_s=0)


def test_pyserial_transport_round_trip_over_pyserials_in_process_loopback() -> None:
    """``loop://`` is pyserial's own in-process port: no operating system port, no virtual COM pair."""
    transport = PySerialTransport("loop://", 9600, write_timeout_s=1.0)
    transport.open()
    assert transport.is_open
    transport.write(b"PING\n")
    received = b""
    deadline = time.monotonic() + WAIT_S
    while not received.endswith(b"\n") and time.monotonic() < deadline:
        received += transport.read(0.05)
    assert received == b"PING\n"
    assert transport.read(0.01) == b""  # nothing more: a timeout, not an error
    transport.close()
    transport.close()
    assert not transport.is_open
    with pytest.raises(ConnectionLost):
        transport.write(b"x")
    with pytest.raises(ConnectionLost):
        transport.read(0.01)


def test_pyserial_transport_reopens_after_close() -> None:
    transport = PySerialTransport("loop://", 9600, write_timeout_s=1.0)
    transport.open()
    transport.close()
    transport.open()
    transport.write(b"a")
    assert transport.read(0.5) == b"a"
    transport.close()


def test_pyserial_transport_open_failure_is_connection_lost() -> None:
    with pytest.raises(ConnectionLost, match="cannot open"):
        PySerialTransport("/dev/definitely-not-a-port", 9600, write_timeout_s=1.0).open()
    with pytest.raises(ConnectionLost, match="cannot open"):
        PySerialTransport("nonsense://", 9600, write_timeout_s=1.0).open()


class FakePort:
    """A stand-in for a pyserial port that records how it was set up."""

    instances: list["FakePort"] = []

    def __init__(self, port: str, baudrate: int, do_not_open: bool) -> None:
        self.args = (port, baudrate, do_not_open)
        self.opened_with: dict[str, Any] = {}
        self.timeout: Optional[float] = None
        self.write_timeout: Optional[float] = None
        self.dtr: Optional[bool] = None
        self.rts: Optional[bool] = None
        self.in_waiting = 0
        self.fail_write = False
        self.fail_read = False
        self.short_write = False
        self.closed = False
        self.written = b""
        FakePort.instances.append(self)

    def open(self) -> None:
        self.opened_with = {"dtr": self.dtr, "rts": self.rts}

    def write(self, data: bytes) -> int:
        if self.fail_write:
            raise OSError("device disconnected")
        self.written += data
        return len(data) - 1 if self.short_write else len(data)

    def flush(self) -> None:
        pass

    def read(self, count: int) -> bytes:
        if self.fail_read:
            raise OSError("device disconnected")
        return b""

    def cancel_read(self) -> None:
        pass

    def close(self) -> None:
        self.closed = True


def fake_transport() -> tuple[PySerialTransport, FakePort]:
    FakePort.instances = []
    transport = PySerialTransport(
        "COM-FAKE", 9600, write_timeout_s=0.5,
        serial_factory=lambda port, baudrate, do_not_open: FakePort(port, baudrate, do_not_open),
    )
    transport.open()
    return transport, FakePort.instances[0]


def test_opening_holds_dtr_and_rts_low_so_the_esp32_is_not_reset() -> None:
    transport, port = fake_transport()
    assert port.args == ("COM-FAKE", 9600, True)
    assert port.opened_with == {"dtr": False, "rts": False}
    assert port.write_timeout == 0.5
    transport.close()


def test_pyserial_write_failure_closes_the_transport_and_reports_connection_lost() -> None:
    transport, port = fake_transport()
    port.fail_write = True
    with pytest.raises(ConnectionLost, match="write"):
        transport.write(b"x")
    assert port.closed and not transport.is_open


def test_pyserial_short_write_is_treated_as_a_lost_connection() -> None:
    transport, port = fake_transport()
    port.short_write = True
    with pytest.raises(ConnectionLost, match="short write"):
        transport.write(b"abc")
    assert not transport.is_open


def test_pyserial_read_failure_closes_the_transport_and_reports_connection_lost() -> None:
    transport, port = fake_transport()
    port.fail_read = True
    with pytest.raises(ConnectionLost, match="read"):
        transport.read(0.01)
    assert port.closed and not transport.is_open


# --------------------------------------------------------------------------- #
# SerialLink
# --------------------------------------------------------------------------- #


class Rig:
    """A link on a loopback cable, with the device end driven by hand."""

    def __init__(self, settings: Optional[LinkSettings] = None, chunk_size: Optional[int] = None) -> None:
        self.pair = loopback_pair(chunk_size)
        self.link = SerialLink(self.pair.pc, settings or LinkSettings(max_in_flight=2, read_poll_s=0.01))
        self.messages: list[LinkMessage] = []
        self.link.add_listener(self.messages.append)
        self.pair.device.open()
        self.link.start()

    def device_lines(self, count: int = 1) -> list[str]:
        out: list[str] = []
        buffer = b""
        deadline = time.monotonic() + WAIT_S
        while len(out) < count and time.monotonic() < deadline:
            buffer += self.pair.device.read(0.02)
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                out.append(line.decode())
        assert len(out) >= count, f"device saw {out}"
        return out

    def device_says(self, *lines: str) -> None:
        self.pair.device.write(("\n".join(lines) + "\n").encode())

    def kinds(self) -> list[str]:
        return [type(m).__name__ for m in self.messages]

    def wait_for(self, predicate: Callable[[], bool]) -> bool:
        return wait_for(predicate)

    def close(self) -> None:
        self.link.stop()


@pytest.fixture
def rig() -> Any:
    made: list[Rig] = []

    def make(settings: Optional[LinkSettings] = None, chunk_size: Optional[int] = None) -> Rig:
        made.append(Rig(settings, chunk_size))
        return made[-1]

    yield make
    for item in made:
        item.close()


def test_link_settings_are_validated() -> None:
    assert LinkSettings().max_in_flight >= 1
    for bad in ({"max_in_flight": 0}, {"read_poll_s": 0}, {"max_line_bytes": 0}):
        with pytest.raises(ValueError):
            LinkSettings(**bad)
    with pytest.raises(ValueError):
        ReconnectPolicy(interval_s=0)
    with pytest.raises(ValueError):
        ReconnectPolicy(interval_s=1, max_attempts=0)


def test_start_announces_the_connection_and_stop_ends_it(rig: Any) -> None:
    r = rig()
    assert r.link.state is LinkState.CONNECTED
    assert r.messages == [Connected(reconnected=False)]
    r.link.stop()
    assert r.link.state is LinkState.STOPPED
    with pytest.raises(LinkDownError):
        r.link.send(PING, 0.1)


def test_starting_twice_is_an_error(rig: Any) -> None:
    with pytest.raises(LinkError, match="already started"):
        rig().link.start()


def test_start_fails_when_the_cable_is_out() -> None:
    pair = loopback_pair()
    pair.cut()
    link = SerialLink(pair.pc)
    with pytest.raises(ConnectionLost):
        link.start()
    assert link.state is LinkState.STOPPED


def test_a_command_goes_out_as_one_terminated_line(rig: Any) -> None:
    r = rig()
    r.link.send(move(135.4), 1.0)
    assert r.device_lines() == ["MOVE A=135.4 B=2 F=3"]


def test_an_invalid_command_is_refused_before_it_takes_a_slot(rig: Any) -> None:
    r = rig()
    with pytest.raises(ValueError):
        r.link.send(Command("MOVE", {"A": 1.0}), 0.1)
    assert r.link.in_flight == 0


def test_ok_acknowledges_the_oldest_command_and_frees_its_slot(rig: Any) -> None:
    r = rig()
    first, second = move(1), move(2)
    r.link.send(first, 1.0)
    r.link.send(second, 1.0)
    assert r.link.in_flight == 2
    r.device_lines(2)
    r.device_says("ok")
    assert r.wait_for(lambda: r.link.in_flight == 1)
    assert Reply(Ok(), first, answered=True) in r.messages
    r.device_says("error:3")
    assert r.wait_for(lambda: r.link.in_flight == 0)
    assert Reply(ErrorResponse(ErrorCode.QUEUE_FULL), second, answered=True) in r.messages


def test_the_limit_on_unacknowledged_commands_blocks_the_next_send(rig: Any) -> None:
    r = rig(LinkSettings(max_in_flight=2, read_poll_s=0.01))
    r.link.send(move(1), 1.0)
    r.link.send(move(2), 1.0)
    with pytest.raises(SendTimeoutError):
        r.link.send(move(3), 0.05)
    assert r.device_lines(2) and r.link.in_flight == 2
    assert r.pair.device.read(0.02) == b""  # the third command was never written


def test_a_blocked_send_proceeds_as_soon_as_a_reply_frees_a_slot(rig: Any) -> None:
    r = rig(LinkSettings(max_in_flight=1, read_poll_s=0.01))
    r.link.send(move(1), 1.0)
    done = threading.Event()

    def second() -> None:
        r.link.send(move(2), 2.0)
        done.set()

    threading.Thread(target=second, daemon=True).start()
    time.sleep(0.05)
    assert not done.is_set()
    r.device_lines(1)
    r.device_says("ok")
    assert done.wait(WAIT_S)
    assert r.device_lines(1) == ["MOVE A=2 B=2 F=3"]


def test_the_window_never_exceeds_the_limit_under_concurrent_senders(rig: Any) -> None:
    limit, total = 3, 60
    r = rig(LinkSettings(max_in_flight=limit, read_poll_s=0.01))
    peak = 0
    stop = threading.Event()

    def device() -> None:  # a tiny ESP32: answers ok to everything it receives
        nonlocal peak
        buffer = b""
        answered = 0
        while answered < total and not stop.is_set():
            peak = max(peak, r.link.in_flight)
            buffer += r.pair.device.read(0.005)
            while b"\n" in buffer:
                _, buffer = buffer.split(b"\n", 1)
                r.pair.device.write(b"ok\n")
                answered += 1

    server = threading.Thread(target=device, daemon=True)
    server.start()
    senders = [threading.Thread(target=lambda: [r.link.send(move(), 5.0) for _ in range(total // 3)]) for _ in range(3)]
    for thread in senders:
        thread.start()
    for thread in senders:
        thread.join(timeout=10)
    assert r.wait_for(lambda: r.link.in_flight == 0)
    stop.set()
    server.join(timeout=2)
    assert peak <= limit
    assert sum(isinstance(m, Reply) for m in r.messages) == total


def test_replies_are_matched_to_commands_in_order(rig: Any) -> None:
    r = rig(LinkSettings(max_in_flight=3, read_poll_s=0.01))
    commands = [move(1), PING, move(3)]
    for command in commands:
        r.link.send(command, 1.0)
    r.device_says("ok", "error:5", "ok")
    assert r.wait_for(lambda: r.link.in_flight == 0)
    replies = [m for m in r.messages if isinstance(m, Reply)]
    assert [m.command for m in replies] == commands
    assert [type(m.response) for m in replies] == [Ok, ErrorResponse, Ok]


def test_done_is_asynchronous_and_never_frees_a_slot(rig: Any) -> None:
    r = rig()
    r.link.send(WAIT, 1.0)
    r.device_says("done")
    assert r.wait_for(lambda: Unsolicited(Done()) in r.messages)
    assert r.link.in_flight == 1
    r.device_says("ok")
    assert r.wait_for(lambda: r.link.in_flight == 0)


def test_a_data_line_belongs_to_its_command_and_ok_still_follows(rig: Any) -> None:
    r = rig()
    r.link.send(POS, 1.0)
    r.device_says("POS A=1 B=2 SA=1 SB=2 NA=10 NB=20 Z=TOP")
    assert r.wait_for(lambda: any(isinstance(m, Reply) and isinstance(m.response, DataLine) for m in r.messages))
    assert r.link.in_flight == 1  # not acknowledged until ok
    reply = next(m for m in r.messages if isinstance(m, Reply))
    assert reply.command is POS and reply.answered is False and reply.response.fields["Z"] == "TOP"
    r.device_says("ok")
    assert r.wait_for(lambda: r.link.in_flight == 0)


def test_a_data_line_for_the_wrong_command_is_a_bad_line(rig: Any) -> None:
    r = rig()
    r.link.send(PING, 1.0)
    r.device_says("INFO FW=x PROTO=1")
    assert r.wait_for(lambda: any(isinstance(m, BadLine) for m in r.messages))
    assert r.link.in_flight == 1


def test_estop_is_acknowledged_by_its_alarm(rig: Any) -> None:
    r = rig()
    r.link.send(ESTOP, 1.0)
    r.device_says("!alarm:ESTOP")
    assert r.wait_for(lambda: r.link.in_flight == 0)
    assert Reply(Alarm(AlarmCode.ESTOP), ESTOP, answered=True) in r.messages
    assert not any(isinstance(m, Unsolicited) for m in r.messages)


@pytest.mark.parametrize("code", list(AlarmCode), ids=lambda c: c.value)
def test_an_alarm_with_nothing_in_flight_is_unsolicited(rig: Any, code: AlarmCode) -> None:
    r = rig()
    r.device_says(f"!alarm:{code.value}")
    assert r.wait_for(lambda: Unsolicited(Alarm(code)) in r.messages)


def test_an_alarm_never_acknowledges_an_ordinary_command(rig: Any) -> None:
    r = rig()
    r.link.send(move(), 1.0)
    r.device_says("!alarm:LIMIT_Z")
    assert r.wait_for(lambda: Unsolicited(Alarm(AlarmCode.LIMIT_Z)) in r.messages)
    assert r.link.in_flight == 1
    # An ESTOP alarm does not acknowledge a MOVE either; only a pending ESTOP command owns it.
    r.device_says("!alarm:ESTOP")
    assert r.wait_for(lambda: Unsolicited(Alarm(AlarmCode.ESTOP)) in r.messages)
    assert r.link.in_flight == 1


def test_an_undeclared_alarm_is_reported_not_dropped(rig: Any) -> None:
    r = rig()
    r.device_says("!alarm:OVERHEAT")
    assert r.wait_for(lambda: any(isinstance(m, BadLine) for m in r.messages))
    bad = next(m for m in r.messages if isinstance(m, BadLine))
    assert bad.alarm_code == "OVERHEAT"


@pytest.mark.parametrize("noise", [b"hello\n", b"\xff\xfe\n", b"error:9\n", b"POS A=1\n", b"ok\n"])
def test_garbage_and_unexpected_replies_become_bad_lines_and_change_nothing(rig: Any, noise: bytes) -> None:
    r = rig()
    r.pair.inject_to_pc(noise)
    assert r.wait_for(lambda: any(isinstance(m, BadLine) for m in r.messages))
    assert r.link.in_flight == 0 and r.link.bad_lines == 1
    r.link.send(PING, 1.0)  # the link still works
    r.device_says("ok")
    assert r.wait_for(lambda: r.link.in_flight == 0)


def test_replies_are_assembled_from_one_byte_chunks(rig: Any) -> None:
    r = rig(chunk_size=1)
    r.link.send(PING, 1.0)
    r.device_says("ok")
    assert r.wait_for(lambda: r.link.in_flight == 0)


def test_a_line_split_across_reads_is_reassembled(rig: Any) -> None:
    r = rig()
    r.link.send(move(), 1.0)
    r.pair.device.write(b"o")
    time.sleep(0.03)
    assert r.link.in_flight == 1
    r.pair.device.write(b"k\n")
    assert r.wait_for(lambda: r.link.in_flight == 0)


def test_a_failing_listener_does_not_stop_delivery(rig: Any) -> None:
    r = rig()

    def broken(_message: LinkMessage) -> None:
        raise RuntimeError("listener bug")

    r.link.add_listener(broken)
    r.link.send(PING, 1.0)
    r.device_says("ok")
    assert r.wait_for(lambda: r.link.in_flight == 0)
    r.link.remove_listener(broken)
    r.link.remove_listener(broken)  # removing twice is fine


# --------------------------------------------------------------------------- #
# Lost connection and reconnect
# --------------------------------------------------------------------------- #


def test_a_lost_cable_reports_the_commands_that_were_in_flight(rig: Any) -> None:
    r = rig(LinkSettings(max_in_flight=2, read_poll_s=0.01))
    a, b = move(1), move(2)
    r.link.send(a, 1.0)
    r.link.send(b, 1.0)
    r.pair.cut()
    assert r.wait_for(lambda: any(isinstance(m, Disconnected) for m in r.messages))
    event = next(m for m in r.messages if isinstance(m, Disconnected))
    assert event.lost == (a, b) and event.will_retry is False
    assert r.link.state is LinkState.DISCONNECTED
    assert r.link.in_flight == 0
    with pytest.raises(LinkDownError):
        r.link.send(PING, 0.1)


def test_a_blocked_sender_is_released_when_the_link_goes_down(rig: Any) -> None:
    r = rig(LinkSettings(max_in_flight=1, read_poll_s=0.01))
    r.link.send(move(1), 1.0)
    outcome: list[object] = []

    def second() -> None:
        try:
            r.link.send(move(2), 5.0)
        except LinkError as exc:
            outcome.append(exc)

    thread = threading.Thread(target=second)
    thread.start()
    time.sleep(0.05)
    r.pair.cut()
    thread.join(timeout=3.0)
    assert not thread.is_alive()
    assert isinstance(outcome[0], LinkDownError)


def test_a_write_on_a_dead_cable_reports_the_loss_and_raises(rig: Any) -> None:
    r = rig(LinkSettings(max_in_flight=2, read_poll_s=10.0))  # a reader that is slow to notice
    r.pair.cut()
    with pytest.raises(LinkDownError):
        r.link.send(PING, 0.5)
    assert r.wait_for(lambda: any(isinstance(m, Disconnected) for m in r.messages))
    assert r.link.in_flight == 0


def test_reconnect_resumes_without_resending_anything(rig: Any) -> None:
    policy = ReconnectPolicy(interval_s=0.01)
    r = rig(LinkSettings(max_in_flight=2, read_poll_s=0.01, reconnect=policy))
    pending = move(1)
    r.link.send(pending, 1.0)
    r.device_lines(1)
    r.pair.cut()
    assert r.wait_for(lambda: r.link.state is LinkState.RECONNECTING)
    assert r.wait_for(lambda: any(isinstance(m, Disconnected) for m in r.messages))  # delivered after the state change
    event = next(m for m in r.messages if isinstance(m, Disconnected))
    assert event.lost == (pending,) and event.will_retry is True
    time.sleep(0.05)
    assert r.link.state is LinkState.RECONNECTING  # still unplugged
    with pytest.raises(LinkDownError):
        r.link.send(PING, 0.1)

    r.pair.restore()
    r.pair.device.open()
    assert r.wait_for(lambda: r.link.state is LinkState.CONNECTED)
    assert Connected(reconnected=True) in r.messages
    assert r.link.in_flight == 0
    assert r.pair.device.read(0.05) == b""  # the lost MOVE was NOT sent again
    r.link.send(PING, 1.0)
    assert r.device_lines(1) == ["PING"]
    r.device_says("ok")
    assert r.wait_for(lambda: r.link.in_flight == 0)


def test_bytes_from_before_the_loss_never_reach_the_new_connection(rig: Any) -> None:
    policy = ReconnectPolicy(interval_s=0.01)
    r = rig(LinkSettings(max_in_flight=2, read_poll_s=0.01, reconnect=policy))
    r.pair.device.write(b"ok")  # half a line, then the cable goes
    r.pair.cut()
    assert r.wait_for(lambda: r.link.state is LinkState.RECONNECTING)
    r.pair.restore()
    r.pair.device.open()
    assert r.wait_for(lambda: r.link.state is LinkState.CONNECTED)
    r.link.send(PING, 1.0)
    r.device_says("ok")
    assert r.wait_for(lambda: r.link.in_flight == 0)
    assert r.link.bad_lines == 0  # "ok" + "ok" was not glued into "okok"


def test_reconnect_gives_up_after_the_configured_attempts(rig: Any) -> None:
    policy = ReconnectPolicy(interval_s=0.01, max_attempts=3)
    r = rig(LinkSettings(max_in_flight=2, read_poll_s=0.01, reconnect=policy))
    r.pair.cut()
    assert r.wait_for(lambda: r.link.state is LinkState.DISCONNECTED)
    assert ReconnectFailed(3) in r.messages


def test_stop_during_reconnect_returns_promptly(rig: Any) -> None:
    policy = ReconnectPolicy(interval_s=30.0)
    r = rig(LinkSettings(max_in_flight=2, read_poll_s=0.01, reconnect=policy))
    r.pair.cut()
    assert r.wait_for(lambda: r.link.state is LinkState.RECONNECTING)
    started = time.monotonic()
    r.link.stop()
    assert time.monotonic() - started < 3.0
    assert r.link.state is LinkState.STOPPED


def test_wait_connected(rig: Any) -> None:
    policy = ReconnectPolicy(interval_s=0.01)
    r = rig(LinkSettings(max_in_flight=2, read_poll_s=0.01, reconnect=policy))
    assert r.link.wait_connected(0.1)
    r.pair.cut()
    assert r.wait_for(lambda: r.link.state is LinkState.RECONNECTING)
    assert not r.link.wait_connected(0.05)
    r.pair.restore()
    r.pair.device.open()
    assert r.link.wait_connected(WAIT_S)


def test_the_link_works_over_pyserials_loopback_port() -> None:
    """The real transport class under the real link: send a command, see it come back, parse a reply."""
    transport = PySerialTransport("loop://", 9600, write_timeout_s=1.0)
    link = SerialLink(transport, LinkSettings(max_in_flight=2, read_poll_s=0.01))
    messages: list[LinkMessage] = []
    link.add_listener(messages.append)
    link.start()
    try:
        link.send(PING, 1.0)
        # loop:// echoes our own bytes back, so the "reply" is the line "PING": not a response.
        assert wait_for(lambda: any(isinstance(m, BadLine) and m.raw == "PING" for m in messages))
        assert link.in_flight == 1
    finally:
        link.stop()


# --------------------------------------------------------------------------- #
# Urgent sends and the simulated-transport flag (used by the Robot Commander)
# --------------------------------------------------------------------------- #


def test_an_urgent_command_does_not_wait_for_a_free_slot(rig: Any) -> None:
    r = rig(LinkSettings(max_in_flight=1, read_poll_s=0.01))
    r.link.send(move(1), 1.0)
    with pytest.raises(SendTimeoutError):
        r.link.send(PING, 0.05)  # an ordinary command waits for the slot...
    r.link.send(PING, 0.05, urgent=True)  # ...the heartbeat and ESTOP do not
    assert r.device_lines(2) == ["MOVE A=1 B=2 F=3", "PING"]
    assert r.link.in_flight == 2
    r.device_says("ok", "ok")  # replies are still matched in order
    assert r.wait_for(lambda: r.link.in_flight == 0)
    replies = [m for m in r.messages if isinstance(m, Reply)]
    assert [m.command.name for m in replies] == ["MOVE", "PING"]


def test_an_urgent_command_on_a_down_link_still_fails(rig: Any) -> None:
    r = rig()
    r.pair.cut()
    assert r.wait_for(lambda: r.link.state is LinkState.DISCONNECTED)
    with pytest.raises(LinkDownError):
        r.link.send(ESTOP, 0.1, urgent=True)


def test_only_the_loopback_transport_counts_as_simulated() -> None:
    pair = loopback_pair()
    assert pair.pc.is_simulated and pair.device.is_simulated
    assert not PySerialTransport("loop://", 9600, write_timeout_s=1.0).is_simulated  # pyserial's loop:// is still the real class

    class Custom(LoopbackTransport.__mro__[1]):  # type: ignore[misc]
        def open(self) -> None: ...
        def close(self) -> None: ...
        @property
        def is_open(self) -> bool:
            return True
        def write(self, data: bytes) -> None: ...
        def read(self, timeout_s: float) -> bytes:
            return b""

    assert not Custom().is_simulated  # fail-safe: an unknown transport is treated as a real port


def test_the_link_exposes_its_transport(rig: Any) -> None:
    r = rig()
    assert r.link.transport is r.pair.pc
