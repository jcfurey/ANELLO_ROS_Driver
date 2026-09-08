"""Adversarial cancellation and resource boundaries, using local sockets only."""

import fcntl
import socket
import threading
import time
from unittest.mock import Mock

import pytest

from ntrip_client.ntrip_client import NTRIPClient
from ntrip_client.rtcm_parser import RTCMParser
from ntrip_client.worker import NTRIPWorker


def client():
    return NTRIPClient('127.0.0.1', 2101, 'MOUNT', None, None, None)


@pytest.mark.parametrize('error', [UnicodeError, OverflowError])
def test_resolver_exceptions_do_not_strand_future_attempts(monkeypatch, error):
    connection = client()
    addresses = [(socket.AF_INET, socket.SOCK_STREAM, 6, '', ('127.0.0.1', 2101))]
    resolve = Mock(side_effect=[error('invalid resolver input'), addresses])
    monkeypatch.setattr(socket, 'getaddrinfo', resolve)
    with pytest.raises(error):
        connection._resolve(time.monotonic() + 0.5)
    assert connection._resolve(time.monotonic() + 0.5) == addresses
    assert resolve.call_count == 2


def test_corrections_on_socket_above_select_descriptor_limit():
    connection = client()
    # Duplicate directly into the high range without opening 1024 spare fds.
    incoming, outgoing = socket.socketpair()
    try:
        high_fd = fcntl.fcntl(incoming.fileno(), fcntl.F_DUPFD_CLOEXEC, 1024)
        connection._server_socket = socket.socket(fileno=high_fd)
        connection._connected = True
        connection._recv_rtcm_last_packet_timestamp = time.monotonic()
        body = b'\xd3\x00\x13\x3e\xd0' + bytes(17)
        correction = body + RTCMParser()._checksum(body).to_bytes(3, 'big')
        outgoing.sendall(correction)
        assert connection.recv_rtcm() == [correction]
        assert connection.connected
        outgoing.close()
        assert connection.recv_rtcm() == []
        assert not connection.connected
    finally:
        connection.shutdown()
        incoming.close()
        outgoing.close()


def test_cancel_between_socket_open_and_request(monkeypatch):
    connection = client()

    def open_then_cancel():
        connection.shutdown()
        return True

    monkeypatch.setattr(connection, '_open_socket', open_then_cancel)
    assert connection.connect() is False
    assert not connection.connected


def test_cancel_during_response_cannot_resurrect_connection(monkeypatch):
    connection = client()

    def open_socket():
        connection._server_socket = Mock()
        return True

    def read_then_cancel():
        connection.shutdown()
        return 'HTTP/1.1 200 OK\r\n\r\n'

    monkeypatch.setattr(connection, '_open_socket', open_socket)
    monkeypatch.setattr(connection, '_read_response_headers', read_then_cancel)
    assert connection.connect() is False
    assert not connection.connected


def test_cancel_during_tls_wrap_does_not_start_handshake(monkeypatch):
    connection = client()
    connection.ssl = True
    wrapped = []

    def wrap_socket(raw, **kwargs):
        # SSL wrapping transfers fd ownership out of the original socket.
        # Cancel after that transfer, before the new socket is registered.
        replacement = Mock(wraps=socket.socket(fileno=raw.detach()))
        replacement.do_handshake = Mock()
        wrapped.append(replacement)
        connection.shutdown()
        return replacement

    context = Mock()
    context.wrap_socket.side_effect = wrap_socket
    monkeypatch.setattr('ntrip_client.ntrip_client.ssl.create_default_context', lambda: context)
    with socket.socket() as listener:
        listener.bind(('127.0.0.1', 0))
        listener.listen(1)
        connection._port = listener.getsockname()[1]
        try:
            assert not connection._open_socket()
            assert len(wrapped) == 1
            wrapped[0].do_handshake.assert_not_called()
            assert wrapped[0].fileno() == -1
        finally:
            connection.shutdown()
            for sock in wrapped:
                sock.close()


def test_tls_buffered_plaintext_does_not_need_fd_readiness():
    connection = client()
    connection.ssl = True
    connection._server_socket = Mock()
    connection._server_socket.pending.return_value = 1
    assert connection._data_available()
    connection._server_socket.fileno.assert_not_called()


def test_success_status_is_not_a_completed_connection():
    connection = client()
    assert connection._classify_response('HTTP/1.1 200 OK\r\n')
    assert not connection.connected


def test_resolver_thread_creation_failure_is_retryable(monkeypatch):
    connection = client()
    with monkeypatch.context() as patch:
        patch.setattr(threading.Thread, 'start', Mock(side_effect=RuntimeError('thread limit')))
        with pytest.raises(RuntimeError, match='thread limit'):
            connection._resolve(time.monotonic() + 0.5)
    addresses = [(socket.AF_INET, socket.SOCK_STREAM, 6, '', ('127.0.0.1', 2101))]
    monkeypatch.setattr(socket, 'getaddrinfo', lambda *args, **kwargs: addresses)
    assert connection._resolve(time.monotonic() + 0.5) == addresses


def test_disconnect_detaches_even_if_socket_cleanup_fails():
    connection = client()
    sock = Mock()
    sock.shutdown.side_effect = OSError('already disconnected')
    sock.close.side_effect = OSError('close failed')
    connection._server_socket = sock
    connection._connected = True
    connection.shutdown()
    connection.disconnect()
    sock.shutdown.assert_called_once()
    sock.close.assert_called_once()
    assert not connection.connected
    assert connection._server_socket is None


def test_worker_stop_interrupts_incomplete_http_response():
    connection = client()
    with socket.socket() as listener:
        listener.bind(('127.0.0.1', 0))
        listener.listen(1)
        listener.settimeout(2)
        connection._port = listener.getsockname()[1]
        worker = NTRIPWorker(connection)
        worker.start()
        try:
            incoming, _ = listener.accept()
            with incoming:
                incoming.settimeout(2)
                assert incoming.recv(4096).startswith(b'GET /MOUNT ')
                incoming.sendall(b'HTTP/1.1 200 OK\r\nUnfinished: ')
                before = time.monotonic()
                worker.stop()
                assert time.monotonic() - before < 1
                assert incoming.recv(4096) == b''
        finally:
            worker.stop()
    assert not connection.connected
