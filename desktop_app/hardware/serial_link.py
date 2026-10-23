"""Serial link: transports, line framing, reconnect handling and the in-flight limit (SPEC 5 and 5.3).

Layers, bottom to top:

* :class:`Transport`: bytes in, bytes out. :class:`PySerialTransport` drives a real port;
  :class:`LoopbackTransport` is an in-process cable for tests and for the ESP32 simulator. Neither
  needs a virtual COM port. A transport raises :class:`ConnectionLost` when the cable is gone.
* :class:`LineFramer`: bytes to lines (``\\n`` terminated, ``\\r\\n`` tolerated), with a maximum
  line length so noise cannot grow a buffer without bound.
* :class:`SerialLink`: one reader thread, ordered delivery of :class:`LinkMessage` objects to
  listeners, and the limit on unacknowledged commands (SPEC 5.3: "keep only a few unacknowledged
  commands in flight"). A command is acknowledged by the ``ok`` or ``error:<n>`` that answers it
  (``!alarm:ESTOP`` answers ESTOP). Replies arrive in the order the commands were sent, so the
  oldest command in flight is the one a reply belongs to. ``done`` is asynchronous and never
  frees a slot.

Reconnect: when the cable is lost the link reports :class:`Disconnected` with the commands that
were in flight, then (if a :class:`ReconnectPolicy` is set) keeps trying to reopen the transport.
It NEVER resends a command after a reconnect: a MOVE may have been half-applied, and the ESP32 will
have stopped on its heartbeat timeout and need homing. What a lost link means for the game is
decided by the Robot Commander, not here.

No Qt, no hardware constant. The baud rate is [TO CONFIRM] (SPEC 5) and therefore has no default.
"""

from __future__ import annotations

import logging
import threading
import time
from abc import ABC, abstractmethod
from collections import deque
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Callable, Deque, Optional, Union

from config.protocol_commands import Ack, AlarmCode, COMMANDS
from desktop_app.hardware.protocol import (
    Alarm,
    Command,
    DataLine,
    Done,
    ErrorResponse,
    Ok,
    ParseError,
    Response,
    UnknownAlarmError,
    frame,
    parse_response,
)

logger = logging.getLogger(__name__)

_JOIN_TIMEOUT_S = 3.0  # how long stop() waits for a worker thread
_NEWLINE = b"\n"[0]  # the byte that ends a line
_RAW_PREVIEW_BYTES = 24  # how much of an over-long line is kept for the log


# --------------------------------------------------------------------------- #
# Errors
# --------------------------------------------------------------------------- #


class TransportError(OSError):
    """A transport could not do what was asked."""


class ConnectionLost(TransportError):
    """The cable is gone (unplugged, port closed, device reset). The transport is unusable until reopened."""


class LinkError(RuntimeError):
    """Base class of the link's own errors."""


class LinkDownError(LinkError):
    """The link is not connected, so nothing can be sent."""


class SendTimeoutError(LinkError):
    """No in-flight slot became free in time."""


# --------------------------------------------------------------------------- #
# Transports
# --------------------------------------------------------------------------- #


class Transport(ABC):
    """A byte pipe to the ESP32. Thread safety: one reader thread and any number of writers."""

    @abstractmethod
    def open(self) -> None:
        """Open (or reopen) the pipe. Raises :class:`ConnectionLost` when it cannot."""

    @abstractmethod
    def close(self) -> None:
        """Close the pipe. Safe to call twice, and safe while another thread is blocked in ``read``."""

    @property
    @abstractmethod
    def is_open(self) -> bool: ...

    @property
    def is_simulated(self) -> bool:
        """True only for an in-process transport with no hardware behind it.

        Fail-safe default: anything that does not say so is treated as a real port. The Robot
        Commander's placeholder guard (SPEC 7.2) relies on this.
        """
        return False

    @abstractmethod
    def write(self, data: bytes) -> None:
        """Send ``data``. Raises :class:`ConnectionLost` when the cable is gone."""

    @abstractmethod
    def read(self, timeout_s: float) -> bytes:
        """Wait up to ``timeout_s`` for bytes; return what arrived, or ``b""`` on timeout.

        Raises :class:`ConnectionLost` when the cable is gone or the transport was closed.
        """


class PySerialTransport(Transport):
    """A real serial port through pyserial.

    ``baud`` has no default: the baud rate is [TO CONFIRM] in the SPEC. ``port`` is anything
    ``serial.serial_for_url`` accepts ("COM3", "/dev/ttyUSB0", "loop://" for pyserial's own
    in-process loopback). DTR and RTS are held low while opening: on most ESP32 boards they are
    wired to EN and GPIO0, so toggling them would RESET the board every time the PC connects and
    lose its homing. (Verify on the real board in phase 4.)
    """

    def __init__(
        self,
        port: str,
        baud: int,
        *,
        write_timeout_s: float,
        serial_factory: Optional[Callable[..., Any]] = None,
    ) -> None:
        if isinstance(baud, bool) or not isinstance(baud, int) or baud <= 0:
            raise ValueError(f"baud must be a positive integer, got {baud!r}")
        if not write_timeout_s > 0:
            raise ValueError(f"write_timeout_s must be > 0, got {write_timeout_s!r}")
        self._port = port
        self._baud = baud
        self._write_timeout_s = write_timeout_s
        self._factory = serial_factory
        self._serial: Any = None
        self._lock = threading.Lock()

    def _make(self) -> Any:
        if self._factory is not None:
            return self._factory(self._port, baudrate=self._baud, do_not_open=True)
        try:
            import serial
        except ImportError as exc:  # pragma: no cover - pyserial is in requirements.txt
            raise TransportError("pyserial is not installed (pip install pyserial)") from exc
        return serial.serial_for_url(self._port, baudrate=self._baud, do_not_open=True)

    def open(self) -> None:
        with self._lock:
            if self._serial is not None:
                return
            try:
                port = self._make()
                port.timeout = 0
                port.write_timeout = self._write_timeout_s
                port.dtr = False
                port.rts = False
                port.open()
            except (OSError, ValueError) as exc:
                raise ConnectionLost(f"cannot open {self._port}: {exc}") from exc
            except Exception as exc:  # serial.SerialException is not an OSError on every pyserial version
                raise ConnectionLost(f"cannot open {self._port}: {exc}") from exc
            self._serial = port

    def close(self) -> None:
        with self._lock:
            port, self._serial = self._serial, None
        if port is not None:
            try:
                port.cancel_read()
            except Exception:  # not every port type supports it
                logger.debug("cancel_read failed", exc_info=True)
            try:
                port.close()
            except Exception:
                logger.debug("closing the serial port failed", exc_info=True)

    @property
    def is_open(self) -> bool:
        return self._serial is not None

    def _port_or_raise(self) -> Any:
        port = self._serial
        if port is None:
            raise ConnectionLost(f"{self._port} is closed")
        return port

    def write(self, data: bytes) -> None:
        port = self._port_or_raise()
        try:
            written = port.write(data)
            port.flush()
        except Exception as exc:
            self.close()
            raise ConnectionLost(f"write to {self._port} failed: {exc}") from exc
        if written is not None and written != len(data):
            self.close()
            raise ConnectionLost(f"short write to {self._port}: {written} of {len(data)} bytes")

    def read(self, timeout_s: float) -> bytes:
        port = self._port_or_raise()
        try:
            port.timeout = timeout_s
            first = port.read(1)
            if not first:
                return b""
            waiting = port.in_waiting
            return first + (port.read(waiting) if waiting else b"")
        except Exception as exc:
            if self._serial is None:  # closed by another thread while we were blocked
                raise ConnectionLost(f"{self._port} was closed") from exc
            self.close()
            raise ConnectionLost(f"read from {self._port} failed: {exc}") from exc


class _Cable:
    """Shared state of a loopback pair."""

    def __init__(self, chunk_size: Optional[int]) -> None:
        self.cond = threading.Condition()
        self.connected = True
        self.chunk_size = chunk_size
        self.queues: dict[str, Deque[bytes]] = {"pc": deque(), "device": deque()}


class LoopbackTransport(Transport):
    """One end of an in-process cable. Create both ends with :func:`loopback_pair`."""

    def __init__(self, cable: _Cable, side: str) -> None:
        self._cable = cable
        self._side = side
        self._peer = "device" if side == "pc" else "pc"
        self._open = False
        self.bytes_written = 0

    def open(self) -> None:
        with self._cable.cond:
            if not self._cable.connected:
                raise ConnectionLost("the cable is unplugged")
            self._open = True
            self._cable.queues[self._side].clear()  # a fresh open never sees old bytes
            self._cable.cond.notify_all()

    def close(self) -> None:
        with self._cable.cond:
            self._open = False
            self._cable.cond.notify_all()

    @property
    def is_open(self) -> bool:
        return self._open

    @property
    def is_simulated(self) -> bool:
        return True

    def _check(self) -> None:
        if not self._open:
            raise ConnectionLost(f"the {self._side} end is closed")
        if not self._cable.connected:
            raise ConnectionLost("the cable is unplugged")

    def write(self, data: bytes) -> None:
        with self._cable.cond:
            self._check()
            self.bytes_written += len(data)
            size = self._cable.chunk_size
            chunks = [data[i : i + size] for i in range(0, len(data), size)] if size else [data]
            self._cable.queues[self._peer].extend(chunk for chunk in chunks if chunk)
            self._cable.cond.notify_all()

    def read(self, timeout_s: float) -> bytes:
        deadline = time.monotonic() + timeout_s
        with self._cable.cond:
            while True:
                self._check()
                queue = self._cable.queues[self._side]
                if queue:
                    if self._cable.chunk_size:
                        return queue.popleft()
                    data = b"".join(queue)
                    queue.clear()
                    return data
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return b""
                self._cable.cond.wait(remaining)


class LoopbackPair:
    """Both ends of an in-process cable, with the fault a real cable can have: being unplugged."""

    def __init__(self, chunk_size: Optional[int] = None) -> None:
        if chunk_size is not None and chunk_size < 1:
            raise ValueError("chunk_size must be >= 1")
        self._cable = _Cable(chunk_size)
        self.pc = LoopbackTransport(self._cable, "pc")
        self.device = LoopbackTransport(self._cable, "device")

    @property
    def connected(self) -> bool:
        return self._cable.connected

    def cut(self) -> None:
        """Unplug the cable: bytes in transit are lost and both ends raise ConnectionLost."""
        with self._cable.cond:
            self._cable.connected = False
            for queue in self._cable.queues.values():
                queue.clear()
            self._cable.cond.notify_all()

    def restore(self) -> None:
        """Plug the cable back in. Each end must call ``open()`` again, like a real port."""
        with self._cable.cond:
            self._cable.connected = True
            for end in (self.pc, self.device):
                end._open = False  # a replugged port has to be reopened
            self._cable.cond.notify_all()

    def inject_to_pc(self, data: bytes) -> None:
        """Put raw bytes on the wire towards the PC (noise, partial lines), as if the ESP32 sent them."""
        with self._cable.cond:
            self._cable.queues["pc"].append(data)
            self._cable.cond.notify_all()


def loopback_pair(chunk_size: Optional[int] = None) -> LoopbackPair:
    """A connected cable. ``chunk_size`` delivers at most that many bytes per read (tests framing)."""
    return LoopbackPair(chunk_size)


# --------------------------------------------------------------------------- #
# Line framing
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class Frame:
    """One framed line: ``text`` is set for a good line, ``error`` for one that cannot be used."""

    text: Optional[str] = None
    error: Optional[str] = None
    raw: bytes = b""


class LineFramer:
    """Splits a byte stream into lines. Empty lines are dropped; ``\\r\\n`` and ``\\n`` both end a line."""

    def __init__(self, max_line_bytes: int) -> None:
        if max_line_bytes < 1:
            raise ValueError("max_line_bytes must be >= 1")
        self._max = max_line_bytes
        self._buffer = bytearray()
        self._discarding = False

    def reset(self) -> None:
        self._buffer.clear()
        self._discarding = False

    def feed(self, data: bytes) -> list[Frame]:
        frames: list[Frame] = []
        for byte in data:
            if byte == _NEWLINE:
                if self._discarding:
                    self._discarding = False
                else:
                    raw = bytes(self._buffer).rstrip(b"\r")
                    if raw:  # a blank line (including a bare CRLF) is dropped, not reported
                        frames.append(self._finish(raw))
                self._buffer.clear()
            elif self._discarding:
                continue
            else:
                self._buffer.append(byte)
                if len(self._buffer) > self._max:
                    frames.append(Frame(error=f"line longer than {self._max} bytes", raw=bytes(self._buffer[:_RAW_PREVIEW_BYTES])))
                    self._buffer.clear()
                    self._discarding = True
        return frames

    @staticmethod
    def _finish(raw: bytes) -> Frame:
        try:
            text = raw.decode("ascii")
        except UnicodeDecodeError:
            return Frame(error="not ASCII", raw=raw)
        if not text.isprintable():
            return Frame(error="control characters in the line", raw=raw)
        return Frame(text=text, raw=raw)


# --------------------------------------------------------------------------- #
# Messages delivered to listeners
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class Reply:
    """A response that answers ``command`` (``ok``, ``error:<n>``, or the data line before ``ok``)."""

    response: Union[Ok, ErrorResponse, DataLine, Alarm]
    command: Command
    answered: bool  # True when it freed the in-flight slot (False for a data line, which ok follows)


@dataclass(frozen=True)
class Unsolicited:
    """A response that answers no command: ``done`` or an asynchronous ``!alarm``."""

    response: Union[Done, Alarm]


@dataclass(frozen=True)
class BadLine:
    """A line that could not be used. ``alarm_code`` is set for an undeclared ``!alarm`` (a safety message)."""

    raw: str
    reason: str
    alarm_code: Optional[str] = None


@dataclass(frozen=True)
class Connected:
    reconnected: bool


@dataclass(frozen=True)
class Disconnected:
    reason: str
    lost: tuple[Command, ...]  # commands that were in flight: their outcome is unknown
    will_retry: bool


@dataclass(frozen=True)
class ReconnectFailed:
    attempts: int


LinkMessage = Union[Reply, Unsolicited, BadLine, Connected, Disconnected, ReconnectFailed]
Listener = Callable[[LinkMessage], None]


class LinkState(str, Enum):
    STOPPED = "stopped"
    CONNECTED = "connected"
    RECONNECTING = "reconnecting"
    DISCONNECTED = "disconnected"  # lost, and not retrying


@dataclass(frozen=True)
class ReconnectPolicy:
    """Retry opening the transport every ``interval_s``; ``max_attempts=None`` retries forever."""

    interval_s: float
    max_attempts: Optional[int] = None

    def __post_init__(self) -> None:
        if not self.interval_s > 0:
            raise ValueError("interval_s must be > 0")
        if self.max_attempts is not None and self.max_attempts < 1:
            raise ValueError("max_attempts must be >= 1 or None")


@dataclass(frozen=True)
class LinkSettings:
    """Software settings of the link (none of them is a hardware value)."""

    max_in_flight: int = 3  # SPEC 5.3: "only a few"; must not exceed the firmware motion queue depth
    read_poll_s: float = 0.05  # how often the reader thread looks at the stop flag
    max_line_bytes: int = 256
    reconnect: Optional[ReconnectPolicy] = None

    def __post_init__(self) -> None:
        if self.max_in_flight < 1:
            raise ValueError("max_in_flight must be >= 1")
        if not self.read_poll_s > 0:
            raise ValueError("read_poll_s must be > 0")
        if self.max_line_bytes < 1:
            raise ValueError("max_line_bytes must be >= 1")


# --------------------------------------------------------------------------- #
# The link
# --------------------------------------------------------------------------- #


class SerialLink:
    """Thread-safe link: ``send`` from any thread, listeners are called from the reader thread."""

    def __init__(self, transport: Transport, settings: Optional[LinkSettings] = None) -> None:
        self._transport = transport
        self._settings = settings or LinkSettings()
        self._framer = LineFramer(self._settings.max_line_bytes)
        self._cond = threading.Condition()
        self._state = LinkState.STOPPED
        self._in_flight: Deque[Command] = deque()
        self._listeners: list[Listener] = []
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self.bad_lines = 0

    # ------------------------------------------------------------------ #
    # Read-only view
    # ------------------------------------------------------------------ #

    @property
    def state(self) -> LinkState:
        return self._state

    @property
    def transport(self) -> Transport:
        return self._transport

    @property
    def settings(self) -> LinkSettings:
        return self._settings

    @property
    def max_in_flight(self) -> int:
        return self._settings.max_in_flight

    @property
    def in_flight(self) -> int:
        with self._cond:
            return len(self._in_flight)

    @property
    def in_flight_commands(self) -> tuple[Command, ...]:
        with self._cond:
            return tuple(self._in_flight)

    def add_listener(self, listener: Listener) -> None:
        with self._cond:
            self._listeners.append(listener)

    def remove_listener(self, listener: Listener) -> None:
        with self._cond:
            if listener in self._listeners:
                self._listeners.remove(listener)

    # ------------------------------------------------------------------ #
    # Lifecycle
    # ------------------------------------------------------------------ #

    def start(self) -> None:
        """Open the transport and start reading. Raises :class:`ConnectionLost` if it cannot open."""
        with self._cond:
            if self._state is not LinkState.STOPPED:
                raise LinkError(f"the link is already started ({self._state.value})")
        self._transport.open()
        self._framer.reset()
        self._stop.clear()
        with self._cond:
            self._in_flight.clear()
            self._state = LinkState.CONNECTED
            self._cond.notify_all()
        self._thread = threading.Thread(target=self._run, name="serial-link-reader", daemon=True)
        self._thread.start()
        self._emit(Connected(reconnected=False))

    def stop(self) -> None:
        """Close the transport and stop the reader. In-flight commands are forgotten."""
        self._stop.set()
        self._transport.close()
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=_JOIN_TIMEOUT_S)
        self._thread = None
        with self._cond:
            self._state = LinkState.STOPPED
            self._in_flight.clear()
            self._cond.notify_all()

    def wait_connected(self, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        with self._cond:
            while self._state is not LinkState.CONNECTED:
                remaining = deadline - time.monotonic()
                if remaining <= 0 or self._state in (LinkState.STOPPED, LinkState.DISCONNECTED):
                    return self._state is LinkState.CONNECTED
                self._cond.wait(remaining)
            return True

    # ------------------------------------------------------------------ #
    # Sending
    # ------------------------------------------------------------------ #

    def send(self, command: Command, timeout_s: float, *, urgent: bool = False) -> None:
        """Send ``command``, waiting up to ``timeout_s`` for a free in-flight slot.

        ``urgent`` skips the wait for a slot (the command still counts as in flight, so replies stay
        matched). It exists for the heartbeat and the emergency stop, which must never queue behind a
        saturated stream of MOVE commands.

        Raises:
            SendTimeoutError: no slot became free in time (the command was not sent).
            LinkDownError: the link is not connected (the command was not sent).
        """
        line = frame(command.encode())  # validates the command before it takes a slot
        deadline = time.monotonic() + timeout_s
        with self._cond:
            while True:
                if self._state is not LinkState.CONNECTED:
                    raise LinkDownError(f"the link is {self._state.value}; {command.name} was not sent")
                if urgent or len(self._in_flight) < self._settings.max_in_flight:
                    break
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise SendTimeoutError(
                        f"{len(self._in_flight)} command(s) unacknowledged for {timeout_s:g} s; {command.name} was not sent"
                    )
                self._cond.wait(remaining)
            self._in_flight.append(command)  # before the write, so a fast reply cannot beat it
            try:
                self._transport.write(line.encode("ascii"))
            except ConnectionLost as exc:
                self._in_flight.pop()
                reason = str(exc)
            else:
                logger.debug("-> %s", line.strip())
                return
        self._connection_lost(reason)
        raise LinkDownError(f"the link went down while sending {command.name}: {reason}")

    # ------------------------------------------------------------------ #
    # Reader thread
    # ------------------------------------------------------------------ #

    def _run(self) -> None:
        while not self._stop.is_set():
            if self._state is LinkState.CONNECTED:
                try:
                    data = self._transport.read(self._settings.read_poll_s)
                except ConnectionLost as exc:
                    if not self._stop.is_set():
                        self._connection_lost(str(exc))
                    continue
                for item in self._framer.feed(data):
                    self._dispatch(item)
            elif self._state is LinkState.RECONNECTING:
                self._try_reconnect()
            else:
                self._stop.wait(self._settings.read_poll_s)

    def _try_reconnect(self) -> None:
        policy = self._settings.reconnect
        assert policy is not None
        attempts = 0
        while not self._stop.is_set():
            attempts += 1
            try:
                self._transport.open()
            except ConnectionLost as exc:
                logger.info("reconnect attempt %d failed: %s", attempts, exc)
                if policy.max_attempts is not None and attempts >= policy.max_attempts:
                    with self._cond:
                        self._state = LinkState.DISCONNECTED
                        self._cond.notify_all()
                    self._emit(ReconnectFailed(attempts))
                    return
                self._stop.wait(policy.interval_s)
                continue
            self._framer.reset()
            with self._cond:
                self._in_flight.clear()
                self._state = LinkState.CONNECTED
                self._cond.notify_all()
            logger.info("reconnected after %d attempt(s)", attempts)
            self._emit(Connected(reconnected=True))
            return

    def _connection_lost(self, reason: str) -> None:
        with self._cond:
            if self._state is not LinkState.CONNECTED:
                return
            lost = tuple(self._in_flight)
            self._in_flight.clear()
            retry = self._settings.reconnect is not None
            self._state = LinkState.RECONNECTING if retry else LinkState.DISCONNECTED
            self._cond.notify_all()
        self._transport.close()
        self._framer.reset()
        logger.warning("serial link lost: %s (%d command(s) in flight)", reason, len(lost))
        self._emit(Disconnected(reason=reason, lost=lost, will_retry=retry))

    # ------------------------------------------------------------------ #
    # Dispatch
    # ------------------------------------------------------------------ #

    def _bad(self, raw: str, reason: str, alarm_code: Optional[str] = None) -> None:
        self.bad_lines += 1
        logger.warning("bad line from the ESP32 (%s): %r", reason, raw)
        self._emit(BadLine(raw, reason, alarm_code))

    def _dispatch(self, item: Frame) -> None:
        if item.error is not None:
            self._bad(item.raw.decode("ascii", "replace"), item.error)
            return
        assert item.text is not None
        logger.debug("<- %s", item.text)
        try:
            response: Response = parse_response(item.text)
        except UnknownAlarmError as exc:
            self._bad(item.text, str(exc), alarm_code=exc.raw_code)
            return
        except ParseError as exc:
            self._bad(item.text, str(exc))
            return

        if isinstance(response, Done):
            self._emit(Unsolicited(response))
            return
        with self._cond:
            head = self._in_flight[0] if self._in_flight else None
        if isinstance(response, Alarm):
            if (
                head is not None
                and COMMANDS[head.name].ack is Ack.ESTOP_ALARM
                and response.code is AlarmCode.ESTOP
            ):
                self._release(head)
                self._emit(Reply(response, head, answered=True))
            else:
                self._emit(Unsolicited(response))
            return
        if head is None:
            self._bad(item.text, "a reply arrived but no command is in flight")
            return
        if isinstance(response, DataLine):
            if COMMANDS[head.name].data_line != response.name:
                self._bad(item.text, f"data line {response.name} does not belong to {head.name}")
                return
            self._emit(Reply(response, head, answered=False))
            return
        self._release(head)
        self._emit(Reply(response, head, answered=True))

    def _release(self, command: Command) -> None:
        with self._cond:
            if self._in_flight and self._in_flight[0] is command:
                self._in_flight.popleft()
            self._cond.notify_all()

    def _emit(self, message: LinkMessage) -> None:
        with self._cond:
            listeners = list(self._listeners)
        for listener in listeners:
            try:
                listener(message)
            except Exception:
                logger.exception("a link listener failed on %s", type(message).__name__)
