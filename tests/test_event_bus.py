"""Tests for the Qt-signal Event Bus."""

from __future__ import annotations

import gc
import threading
from dataclasses import dataclass

import pytest
import shiboken6
from PySide6.QtCore import QObject, Qt, QThread
from pytestqt.qtbot import QtBot

from desktop_app.core.event_bus import Event, EventBus, Notification, Request


@dataclass(frozen=True)
class Ping(Notification):
    n: int = 0


@dataclass(frozen=True)
class Other(Notification):
    pass


@dataclass(frozen=True)
class Go(Request):
    pass


@pytest.fixture
def bus() -> EventBus:
    return EventBus()


# --------------------------------------------------------------------------- #
# Routing
# --------------------------------------------------------------------------- #


def test_handler_receives_published_event_synchronously(bus: EventBus) -> None:
    seen: list[Event] = []
    bus.subscribe(Ping, lambda e: seen.append(e))
    bus.publish(Ping(1))
    assert seen == [Ping(1)]


def test_handler_only_receives_its_own_event_type(bus: EventBus) -> None:
    seen: list[Event] = []
    bus.subscribe(Ping, lambda e: seen.append(e))
    bus.publish(Other())
    bus.publish(Go())
    assert seen == []


def test_base_class_subscription_receives_subclasses(bus: EventBus) -> None:
    everything: list[Event] = []
    notifications: list[Event] = []
    requests: list[Event] = []
    bus.subscribe(Event, lambda e: everything.append(e))
    bus.subscribe(Notification, lambda e: notifications.append(e))
    bus.subscribe(Request, lambda e: requests.append(e))

    bus.publish(Ping(1))
    bus.publish(Go())

    assert everything == [Ping(1), Go()]
    assert notifications == [Ping(1)]
    assert requests == [Go()]


def test_every_subscriber_of_a_type_is_called(bus: EventBus) -> None:
    first: list[Event] = []
    second: list[Event] = []
    bus.subscribe(Ping, lambda e: first.append(e))
    bus.subscribe(Ping, lambda e: second.append(e))
    bus.publish(Ping(5))
    assert first == second == [Ping(5)]


def test_publish_without_subscribers_is_a_noop(bus: EventBus) -> None:
    bus.publish(Ping())


def test_only_events_can_be_published_or_subscribed(bus: EventBus) -> None:
    with pytest.raises(TypeError):
        bus.publish("not an event")  # type: ignore[arg-type]
    with pytest.raises(TypeError):
        bus.subscribe(str, lambda e: None)  # type: ignore[type-var]


# --------------------------------------------------------------------------- #
# Cancelling
# --------------------------------------------------------------------------- #


def test_cancelled_subscription_stops_delivery(bus: EventBus) -> None:
    seen: list[Event] = []
    subscription = bus.subscribe(Ping, lambda e: seen.append(e))
    bus.publish(Ping(1))
    subscription.cancel()
    bus.publish(Ping(2))
    assert seen == [Ping(1)]
    assert not subscription.active


def test_cancel_twice_is_safe(bus: EventBus) -> None:
    subscription = bus.subscribe(Ping, lambda e: None)
    subscription.cancel()
    subscription.cancel()


def test_subscription_survives_when_the_caller_drops_the_handle(bus: EventBus) -> None:
    receiver = QObject()
    seen: list[Event] = []
    bus.subscribe(Ping, lambda e: seen.append(e), receiver=receiver)  # handle discarded on purpose
    gc.collect()
    bus.publish(Ping(1))
    assert seen == [Ping(1)]


def test_cancel_releases_the_subscription_held_by_the_bus(bus: EventBus) -> None:
    receiver = QObject()
    subscription = bus.subscribe(Ping, lambda e: None, receiver=receiver)
    assert subscription in bus._subscriptions
    subscription.cancel()
    assert subscription not in bus._subscriptions


def test_cancelled_receiver_subscription_stops_delivery(bus: EventBus) -> None:
    receiver = QObject()
    seen: list[Event] = []
    subscription = bus.subscribe(Ping, lambda e: seen.append(e), receiver=receiver)
    bus.publish(Ping(1))
    assert seen == [Ping(1)]
    subscription.cancel()
    bus.publish(Ping(2))
    assert seen == [Ping(1)]


def test_destroying_the_receiver_ends_the_subscription(bus: EventBus, qtbot: QtBot) -> None:
    receiver = QObject()
    seen: list[Event] = []
    subscription = bus.subscribe(Ping, lambda e: seen.append(e), receiver=receiver)
    bus.publish(Ping(1))

    shiboken6.delete(receiver)

    assert not subscription.active
    bus.publish(Ping(2))
    qtbot.wait(20)
    assert seen == [Ping(1)]


# --------------------------------------------------------------------------- #
# Threading
# --------------------------------------------------------------------------- #


def test_plain_handler_runs_in_the_publishing_thread(bus: EventBus) -> None:
    handler_threads: list[int] = []
    bus.subscribe(Ping, lambda e: handler_threads.append(threading.get_ident()))

    publisher_ident: list[int] = []

    def publish() -> None:
        publisher_ident.append(threading.get_ident())
        bus.publish(Ping())

    thread = threading.Thread(target=publish)
    thread.start()
    thread.join()

    assert handler_threads == publisher_ident
    assert handler_threads[0] != threading.get_ident()


def test_receiver_handler_runs_in_the_receiver_thread_not_the_publisher(bus: EventBus, qtbot: QtBot) -> None:
    receiver = QObject()  # lives in the main (test) thread
    handler_threads: list[int] = []
    bus.subscribe(Ping, lambda e: handler_threads.append(threading.get_ident()), receiver=receiver)

    thread = threading.Thread(target=bus.publish, args=(Ping(7),))
    thread.start()
    thread.join()

    qtbot.waitUntil(lambda: len(handler_threads) == 1, timeout=2000)
    assert handler_threads == [threading.get_ident()]


def test_receiver_living_in_a_worker_thread(bus: EventBus, qtbot: QtBot) -> None:
    worker = QThread()
    worker_ident: list[int] = []
    worker.started.connect(lambda: worker_ident.append(threading.get_ident()), Qt.ConnectionType.DirectConnection)

    receiver = QObject()
    receiver.moveToThread(worker)
    handler_threads: list[int] = []
    bus.subscribe(Ping, lambda e: handler_threads.append(threading.get_ident()), receiver=receiver)

    worker.start()
    qtbot.waitUntil(lambda: len(worker_ident) == 1, timeout=2000)

    bus.publish(Ping())  # published from the main thread

    qtbot.waitUntil(lambda: len(handler_threads) == 1, timeout=2000)
    assert handler_threads == worker_ident
    assert handler_threads[0] != threading.get_ident()

    receiver.deleteLater()
    worker.quit()
    assert worker.wait(2000)
