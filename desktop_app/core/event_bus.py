"""Event Bus built on Qt signals (docs/SPEC.md sections 2 and 11.1).

The bus is the only channel between ``core/`` and everything else:

* ``Notification`` events flow out of the core (state changed, game updated,
  alarm raised...). The GUI listens to them.
* ``Request`` events flow into the core (start, pause, confirm, resync...).
  The GUI publishes them; the FSM decides whether to accept each one.

Delivery rules
--------------
* ``subscribe(..., receiver=None)``: the handler runs synchronously in the
  thread that calls ``publish``.
* ``subscribe(..., receiver=obj)``: the handler runs in the thread that owns
  ``obj`` (a queued connection when that differs from the publisher's thread).
  GUI code passes itself as the receiver so camera, engine and robot threads
  never touch widgets directly. The receiver's thread must run an event loop.
* A handler subscribed to a base class (for example ``Event``) receives every
  subclass event as well.

This module imports only ``PySide6.QtCore``; it knows nothing about the GUI.
"""

from __future__ import annotations

import logging
import threading
from dataclasses import dataclass
from typing import Callable, TypeVar

from PySide6.QtCore import QObject, Qt, Signal, Slot

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class Event:
    """Base class of every message on the bus. Subclasses must be frozen dataclasses."""


@dataclass(frozen=True)
class Notification(Event):
    """Core to the outside world: something happened."""


@dataclass(frozen=True)
class Request(Event):
    """Outside world (GUI) to the core: please do something. The FSM may refuse."""


E = TypeVar("E", bound=Event)


class _Channel(QObject):
    """One signal per event type; publishing emits it."""

    emitted = Signal(object)


class _Relay(QObject):
    """Lives in the receiver's thread and calls the handler there."""

    def __init__(self, handler: Callable[[object], None]) -> None:
        super().__init__()
        self._handler = handler

    @Slot(object)
    def deliver(self, event: object) -> None:
        self._handler(event)


class Subscription:
    """Handle returned by ``EventBus.subscribe``; call ``cancel`` to stop delivery."""

    def __init__(
        self,
        channel: _Channel,
        target: Callable[[object], None],
        relay: _Relay | None,
        on_cancel: Callable[[Subscription], None],
    ) -> None:
        self._channel = channel
        self._target = target
        self._relay = relay
        self._on_cancel = on_cancel
        self._lock = threading.Lock()
        self._active = True

    @property
    def active(self) -> bool:
        return self._active

    def cancel(self) -> None:
        """Stop delivery. Safe to call from any thread and more than once."""
        with self._lock:
            if not self._active:
                return
            self._active = False
        try:
            self._channel.emitted.disconnect(self._target)
        except (RuntimeError, TypeError):
            logger.debug("Subscription target was already disconnected")
        if self._relay is not None:
            try:
                self._relay.deleteLater()
            except RuntimeError:
                logger.debug("Subscription relay was already deleted")
        self._on_cancel(self)

    def _receiver_destroyed(self, *_args: object) -> None:
        self.cancel()


class EventBus(QObject):
    """Typed publish/subscribe hub. Create one per application and pass it around."""

    def __init__(self, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self._lock = threading.Lock()
        self._channels: dict[type[Event], _Channel] = {}
        # Qt holds bound methods only weakly, so the bus must keep every live
        # subscription (and its relay) alive itself, even if the caller drops
        # the returned handle.
        self._subscriptions: set[Subscription] = set()

    def subscribe(
        self,
        event_type: type[E],
        handler: Callable[[E], None],
        *,
        receiver: QObject | None = None,
    ) -> Subscription:
        """Call ``handler`` for every published event that is an instance of ``event_type``.

        With ``receiver`` the handler runs in the receiver's thread and the
        subscription ends automatically when the receiver is destroyed.
        """
        if not (isinstance(event_type, type) and issubclass(event_type, Event)):
            raise TypeError(f"event_type must be an Event subclass, got {event_type!r}")
        channel = self._channel_for(event_type)

        if receiver is None:
            channel.emitted.connect(handler, Qt.ConnectionType.DirectConnection)
            return self._register(Subscription(channel, handler, None, self._forget))

        relay = _Relay(handler)  # type: ignore[arg-type]
        relay.moveToThread(receiver.thread())
        channel.emitted.connect(relay.deliver, Qt.ConnectionType.AutoConnection)
        subscription = self._register(Subscription(channel, relay.deliver, relay, self._forget))
        receiver.destroyed.connect(subscription._receiver_destroyed, Qt.ConnectionType.DirectConnection)
        return subscription

    def publish(self, event: Event) -> None:
        """Deliver ``event`` to every subscriber of its type or of a base class. Thread-safe."""
        if not isinstance(event, Event):
            raise TypeError(f"only Event instances can be published, got {event!r}")
        with self._lock:
            channels = [self._channels[t] for t in type(event).__mro__ if t in self._channels]
        logger.debug("Publishing %s to %d channel(s)", event, len(channels))
        for channel in channels:
            channel.emitted.emit(event)

    def _register(self, subscription: Subscription) -> Subscription:
        with self._lock:
            self._subscriptions.add(subscription)
        return subscription

    def _forget(self, subscription: Subscription) -> None:
        with self._lock:
            self._subscriptions.discard(subscription)

    def _channel_for(self, event_type: type[Event]) -> _Channel:
        with self._lock:
            channel = self._channels.get(event_type)
            if channel is None:
                channel = _Channel()
                channel.moveToThread(self.thread())
                self._channels[event_type] = channel
            return channel
