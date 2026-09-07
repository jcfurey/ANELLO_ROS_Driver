"""Exercise persistent recovery, GGA freshness, and cancellation off the executor."""

import threading
import time

from ntrip_client.worker import NTRIPWorker


class FakeClient:

    def __init__(self):
        self.connected = False
        self.reconnect_attempt_wait_seconds = 0.01
        self.rtcm_timeout_seconds = 4
        self.attempts = 0
        self.sent = []
        self.reads = 0
        self.stopped = False
        self._logwarn = lambda _: None

    def connect(self):
        self.attempts += 1
        self.connected = self.attempts >= 3
        return self.connected

    def disconnect(self):
        self.connected = False

    def shutdown(self):
        self.stopped = True
        self.disconnect()

    def normalize_nmea(self, sentence):
        return sentence if sentence.startswith('$GPGGA') else None

    def send_nmea(self, sentence):
        self.sent.append(sentence)
        return True

    def recv_rtcm(self):
        self.reads += 1
        if self.reads == 2:
            self.connected = False
        return [b'correction'] if self.connected else []


def until(predicate, timeout=2):
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        time.sleep(0.01)
    assert predicate()


def test_startup_retries_and_resends_latest_gga_after_reconnect():
    client = FakeClient()
    worker = NTRIPWorker(client, nmea_interval=10)
    worker.submit_nmea('$GPGGA,old')
    worker.submit_nmea('$GPGGA,latest')
    worker.start()
    try:
        until(lambda: len(client.sent) >= 2)
        assert client.attempts >= 4
        assert client.sent == ['$GPGGA,latest', '$GPGGA,latest']
        assert worker.next_packet() == b'correction'
    finally:
        worker.stop()
    assert client.stopped


def test_stale_or_invalid_gga_is_not_resent():
    client = FakeClient()
    worker = NTRIPWorker(client, nmea_max_age=0.01)
    worker.submit_nmea('$GPGGA,old')
    worker.submit_nmea('invalid')
    time.sleep(0.02)
    worker.start()
    try:
        until(lambda: client.reads >= 1)
        assert client.sent == []
    finally:
        worker.stop()


def test_stop_interrupts_blocked_connection():
    client = FakeClient()
    blocked = threading.Event()
    client.connect = lambda: blocked.wait(5)
    client.shutdown = lambda: blocked.set()
    worker = NTRIPWorker(client)
    worker.start()
    before = time.monotonic()
    worker.stop()
    assert time.monotonic() - before < 1


def test_packet_queue_is_bounded_and_reports_overflow():
    client = FakeClient()
    worker = NTRIPWorker(client)
    client.recv_rtcm = lambda: [b'x'] * 64
    worker.start()
    try:
        until(lambda: worker.dropped_packets > 0)
        assert worker.packets.qsize() == 32
    finally:
        worker.stop()


def test_delayed_executor_cannot_publish_expired_corrections():
    client = FakeClient()
    worker = NTRIPWorker(client)
    worker.packets.put((b'expired', time.monotonic() - 5))
    worker.packets.put((b'fresh', time.monotonic()))
    assert worker.next_packet() == b'fresh'
    assert worker.expired_packets == 1
