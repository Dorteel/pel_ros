"""Rate-limited SPARQL-over-HTTP client with a persistent response cache.

Every (endpoint, query) pair is fetched at most once and stored in SQLite, so
experiments are restartable, polite to the public endpoints, and repeatable
against the recorded responses even after the live graphs have changed.
"""
from __future__ import annotations

import hashlib
import json
import sqlite3
import threading
import time
from pathlib import Path

import requests

CONNECTION_BACKOFF_SECONDS = (3, 10, 30, 60, 120, 300, 600)
USER_AGENT = "pel-reproduction-research/0.1 (academic paper reproduction; python-requests)"


class SparqlError(RuntimeError):
    """The endpoint failed permanently for a query (timeout, HTTP error)."""


class CachedSparqlClient:
    def __init__(self, cache_path: str | Path, offline: bool = False,
                 timeout_seconds: float = 40.0, max_attempts: int = 8):
        self.offline = offline
        self.timeout_seconds = timeout_seconds
        self.max_attempts = max_attempts
        Path(cache_path).parent.mkdir(parents=True, exist_ok=True)
        self._database = sqlite3.connect(str(cache_path), check_same_thread=False, timeout=120)
        self._database.execute(
            "CREATE TABLE IF NOT EXISTS responses (key TEXT PRIMARY KEY, endpoint TEXT,"
            " query TEXT, status TEXT, rows TEXT, fetched_at REAL)")
        self._database_lock = threading.Lock()
        self._throttle_locks: dict[str, threading.Lock] = {}
        self._last_request: dict[str, float] = {}

    @staticmethod
    def _key(endpoint: str, query: str) -> str:
        return hashlib.sha256(f"{endpoint}\n{query}".encode()).hexdigest()

    def _throttle(self, endpoint: str, min_interval: float) -> None:
        lock = self._throttle_locks.setdefault(endpoint, threading.Lock())
        with lock:
            wait = self._last_request.get(endpoint, 0.0) + min_interval - time.monotonic()
            if wait > 0:
                time.sleep(wait)
            self._last_request[endpoint] = time.monotonic()

    def select(self, endpoint: str, query: str, min_interval: float = 0.0) -> list[dict]:
        """Run a SELECT query; return rows as {variable: value} dictionaries.

        Raises SparqlError if the endpoint cannot answer.  Failures are cached
        too (status 'error'), so that a timing-out query is a recorded,
        reproducible outcome rather than a source of run-to-run variation.
        """
        key = self._key(endpoint, query)
        with self._database_lock:
            cached = self._database.execute(
                "SELECT status, rows FROM responses WHERE key = ?", (key,)).fetchone()
        if cached is not None:
            if cached[0] != "ok":
                raise SparqlError(cached[1])
            return json.loads(cached[1])
        if self.offline:
            raise SparqlError("offline and query not in cache")

        # Only a (repeated) *query timeout* is treated as a property of
        # (graph, query) and is recorded; every other failure (network, overload, rate limit)
        # is transient and raised without being cached, so a later run retries.
        status, payload, deterministic = "error", "unknown failure", False
        for attempt in range(self.max_attempts):
            self._throttle(endpoint, min_interval)
            try:
                arguments = {"timeout": self.timeout_seconds,
                             "headers": {"User-Agent": USER_AGENT,
                                         "Accept": "application/sparql-results+json"}}
                if len(query) < 1500:      # short queries: GET (cacheable, always allowed)
                    response = requests.get(endpoint, params={"query": query}, **arguments)
                else:
                    response = requests.post(endpoint, data={"query": query}, **arguments)
            except requests.Timeout:
                # A stalled connection also ends here, so try twice before
                # accepting that this query really exceeds the time limit.
                payload = "query timeout (client side)"
                if attempt >= 1:
                    deterministic = True
                    break
                continue
            except requests.RequestException as error:
                # Refused/reset connections are how public endpoints shed load:
                # retry soon, then back off for minutes rather than insisting.
                payload = f"request failed: {type(error).__name__}"
                time.sleep(CONNECTION_BACKOFF_SECONDS[min(attempt, len(CONNECTION_BACKOFF_SECONDS) - 1)])
                continue
            if response.status_code == 200:
                try:
                    bindings = response.json()["results"]["bindings"]
                except (ValueError, KeyError):
                    payload = "malformed response"
                    time.sleep(2.0)
                    continue
                rows = [{name: cell["value"] for name, cell in row.items()}
                        for row in bindings]
                status, payload = "ok", json.dumps(rows)
                break
            body = response.text[:2000].lower()
            if response.status_code == 504 or "timeout" in body or "timed out" in body \
                    or "time limit" in body:
                payload, deterministic = f"query timeout (HTTP {response.status_code})", True
                break
            payload = f"HTTP {response.status_code}"
            retry_after = response.headers.get("Retry-After", "")
            time.sleep(min(float(retry_after) if retry_after.isdigit() else 5.0 * (attempt + 1),
                           120.0))

        if status == "error" and not deterministic:
            raise SparqlError(payload)
        with self._database_lock:
            self._database.execute(
                "INSERT OR REPLACE INTO responses VALUES (?, ?, ?, ?, ?, ?)",
                (key, endpoint, query, status, payload, time.time()))
            self._database.commit()
        if status != "ok":
            raise SparqlError(payload)
        return json.loads(payload)
