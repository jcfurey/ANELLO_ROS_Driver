"""Own network I/O outside the ROS executor, with bounded queues and shutdown."""

import queue
import threading
import time


class NTRIPWorker:

    def __init__(self, client, nmea_interval=10.0, nmea_max_age=30.0):
        self.client = client
        self.nmea_interval = nmea_interval
        self.nmea_max_age = nmea_max_age
        self.packets = queue.Queue(maxsize=32)
        self.dropped_packets = 0
        self.expired_packets = 0
        self._lock = threading.Lock()
        self._latest_nmea = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def submit_nmea(self, sentence):
        normalized = self.client.normalize_nmea(sentence)
        if normalized is not None:
            with self._lock:
                self._latest_nmea = (normalized, time.monotonic())

    def next_packet(self):
        while True:
            packet, received = self.packets.get_nowait()
            if time.monotonic() - received <= self.client.rtcm_timeout_seconds:
                return packet
            self.expired_packets += 1

    def stop(self):
        self._stop.set()
        self.client.shutdown()  # interrupts connect, recv and TLS handshake
        if self._thread.ident is not None:
            self._thread.join(timeout=1.0)
        if self._thread.is_alive():
            raise RuntimeError('NTRIP network worker failed to stop')

    def _run(self):
        next_attempt = 0.0
        last_nmea_send = float('-inf')
        try:
            while not self._stop.is_set():
                now = time.monotonic()
                if not self.client.connected:
                    if now < next_attempt:
                        self._stop.wait(min(0.1, next_attempt - now))
                        continue
                    try:
                        connected = self.client.connect()
                    except Exception as exc:
                        self.client._logwarn('NTRIP connect failed: {}'.format(exc))
                        self.client.disconnect()
                        connected = False
                    next_attempt = time.monotonic() + max(
                        0.1, self.client.reconnect_attempt_wait_seconds)
                    if not connected:
                        continue
                    last_nmea_send = float('-inf')
                with self._lock:
                    latest = self._latest_nmea
                now = time.monotonic()
                if (latest is not None and now - latest[1] <= self.nmea_max_age
                        and now - last_nmea_send >= self.nmea_interval):
                    if self.client.send_nmea(latest[0]):
                        last_nmea_send = now
                try:
                    packets = self.client.recv_rtcm()
                except Exception as exc:
                    self.client._logwarn('NTRIP read failed: {}'.format(exc))
                    self.client.disconnect()
                    packets = []
                for packet in packets:
                    try:
                        self.packets.put_nowait((packet, time.monotonic()))
                    except queue.Full:
                        # Keep the newest corrections during executor congestion.
                        try:
                            self.packets.get_nowait()
                            self.dropped_packets += 1
                        except queue.Empty:
                            pass
                        self.packets.put_nowait((packet, time.monotonic()))
                self._stop.wait(0.02)
        finally:
            self.client.disconnect()
