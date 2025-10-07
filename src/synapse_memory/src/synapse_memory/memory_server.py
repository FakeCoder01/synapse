#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Project Synapse — Memory Server
#
# Provides the Cognitive Core services over ROS 2:
# - AddPerson
# - GetPersonByFace
# - AddConversationMemory
# - RetrieveRelevantMemories
#
# Backends:
# - SQLite (persons table: id, name, face_encoding, first_seen_date, notes)
# - Vector database (ChromaDB persistent client) for conversational memories
#
# Embeddings:
# - For conversational memories and queries, uses sentence-transformers (configurable)
# - For face encodings, the caller provides a 128-d vector (e.g., dlib/face_recognition)
#
# Environment/Parameters:
# - Base data dir: ~/.synapse (created if missing)
# - SQLite path: ~/.synapse/memory.db (override via param: sqlite_path)
# - Chroma path: ~/.synapse/chroma (override via param: chroma_path)
# - Chroma collection: synapse_memories (override via param: chroma_collection)
# - Embedding model (SentenceTransformers): all-MiniLM-L6-v2 (override via param: embed_model_name)
#
# Notes:
# - If ChromaDB is unavailable, falls back to an in-memory list store (non-persistent).
# - If SentenceTransformers is unavailable, AddConversationMemory requires an embedding in the request.
#   RetrieveRelevantMemories will require query_embedding if query_text cannot be embedded.
#
# License: Apache-2.0

from __future__ import annotations

import json
import math
import os
import sqlite3
import threading
import time
import uuid
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time as RosTime

from synapse_interfaces.srv import (
    AddPerson,
    GetPersonByFace,
    AddConversationMemory,
    RetrieveRelevantMemories,
)

# Optional dependencies (ChromaDB and SentenceTransformers)
_HAVE_CHROMA = True
_HAVE_SENTENCE_TRANSFORMERS = True
_CHROMA_IMPORT_ERROR = None
_SENTENCE_TRANSFORMERS_IMPORT_ERROR = None

try:
    import chromadb  # type: ignore
    from chromadb.config import Settings as ChromaSettings  # type: ignore
except Exception as _exc:
    _HAVE_CHROMA = False
    _CHROMA_IMPORT_ERROR = _exc

try:
    from sentence_transformers import SentenceTransformer  # type: ignore
    import numpy as _np  # for vector ops if needed
except Exception as _exc:
    _HAVE_SENTENCE_TRANSFORMERS = False
    _SENTENCE_TRANSFORMERS_IMPORT_ERROR = _exc
    SentenceTransformer = None  # type: ignore
    _np = None  # type: ignore


# ------------------------------
# Utility helpers
# ------------------------------


def ensure_dir(path: str) -> None:
    if not os.path.isdir(path):
        os.makedirs(path, exist_ok=True)


def ros_time_to_unix_seconds(t: RosTime) -> float:
    # builtin_interfaces/Time: sec (int32), nanosec (uint32)
    try:
        return float(t.sec) + (float(t.nanosec) * 1e-9)
    except Exception:
        return 0.0


def unix_seconds_to_iso(ts: float) -> str:
    try:
        return (
            time.strftime("%Y-%m-%dT%H:%M:%S", time.gmtime(ts))
            + f".{int((ts % 1) * 1000):03d}Z"
        )
    except Exception:
        return ""


def normalize_score_from_distance(distance: float, max_distance: float) -> float:
    """
    Map a distance (smaller is better) to a normalized score in [0,1],
    where 'max_distance' acts as the threshold for minimum acceptable match.
    """
    if max_distance <= 0.0:
        return 0.0
    score = 1.0 - (distance / max_distance)
    if score < 0.0:
        return 0.0
    if score > 1.0:
        return 1.0
    return score


def l2_distance(a: List[float], b: List[float]) -> float:
    if len(a) != len(b):
        return float("inf")
    s = 0.0
    for x, y in zip(a, b):
        d = x - y
        s += d * d
    return math.sqrt(s)


# ------------------------------
# In-memory fallback vector store (if Chroma unavailable)
# ------------------------------


@dataclass
class MemoryEntry:
    memory_id: str
    person_id: str
    conversation_id: str
    role: str
    text: str
    embedding: List[float]
    timestamp: float
    tags: List[str]
    channel: str


class InMemoryVectorStore:
    """
    Minimal vector-memory store for fallback when ChromaDB is not installed.
    Non-persistent; for development only.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._items: List[MemoryEntry] = []

    def add(self, entry: MemoryEntry) -> None:
        with self._lock:
            self._items.append(entry)

    def query(
        self,
        query_embedding: List[float],
        person_id: Optional[str],
        top_k: int,
        include_roles: Optional[List[str]] = None,
        include_tags: Optional[List[str]] = None,
        after_time: Optional[float] = None,
        before_time: Optional[float] = None,
        conversation_id: Optional[str] = None,
        channel: Optional[str] = None,
    ) -> List[Tuple[MemoryEntry, float]]:
        """
        Return top_k results as (entry, score) using cosine similarity approximated via
        normalized dot product. If embeddings not normalized, we normalize on-the-fly.
        """
        if not query_embedding:
            return []

        # Normalize
        def _norm(v: List[float]) -> float:
            return math.sqrt(sum(x * x for x in v)) or 1.0

        qn = _norm(query_embedding)
        qv = [x / qn for x in query_embedding]

        results: List[Tuple[MemoryEntry, float]] = []
        with self._lock:
            for it in self._items:
                # Filtering
                if person_id and it.person_id != person_id:
                    continue
                if include_roles and it.role not in include_roles:
                    continue
                if include_tags and not (set(include_tags) & set(it.tags or [])):
                    continue
                if conversation_id and it.conversation_id != conversation_id:
                    continue
                if channel and it.channel != channel:
                    continue
                if after_time and it.timestamp <= after_time:
                    continue
                if before_time and it.timestamp >= before_time:
                    continue

                # Cosine similarity
                vn = _norm(it.embedding)
                vv = [x / vn for x in it.embedding]
                sim = sum(a * b for a, b in zip(qv, vv))
                score = max(0.0, min(1.0, (sim + 1.0) / 2.0))  # map [-1,1] -> [0,1]
                results.append((it, score))

        # Sort by score desc
        results.sort(key=lambda t: t[1], reverse=True)
        return results[: max(1, top_k)]


# ------------------------------
# Sentence-Transformer Embedding Wrapper
# ------------------------------


class STEmbedder:
    def __init__(self, model_name: str, node: Node):
        self._node = node
        self._model_name = model_name
        self._model = None
        self._lock = threading.Lock()

    def ensure_loaded(self) -> None:
        if not _HAVE_SENTENCE_TRANSFORMERS:
            raise RuntimeError(
                f"SentenceTransformers not available: {_SENTENCE_TRANSFORMERS_IMPORT_ERROR}"
            )
        with self._lock:
            if self._model is None:
                self._node.get_logger().info(
                    f"Loading embedding model: {self._model_name}"
                )
                self._model = SentenceTransformer(self._model_name)

    def embed_text(self, text: str) -> List[float]:
        self.ensure_loaded()
        assert self._model is not None
        vec = self._model.encode([text], normalize_embeddings=True)  # shape (1, D)
        arr = vec[0]
        # Convert to Python list of floats
        return [float(x) for x in arr.tolist()]

    def embed_texts(self, texts: List[str]) -> List[List[float]]:
        self.ensure_loaded()
        assert self._model is not None
        vecs = self._model.encode(texts, normalize_embeddings=True)
        out: List[List[float]] = []
        for row in vecs:
            out.append([float(x) for x in row.tolist()])
        return out


# ------------------------------
# Memory Server Node
# ------------------------------


class MemoryServer(Node):
    def __init__(self):
        super().__init__("memory_server")

        # Parameters
        self.declare_parameter("base_dir", os.path.expanduser("~/.synapse"))
        self.declare_parameter("sqlite_path", "")
        self.declare_parameter("chroma_path", "")
        self.declare_parameter("chroma_collection", "synapse_memories")
        self.declare_parameter("embed_model_name", "all-MiniLM-L6-v2")
        self.declare_parameter("face_encoding_dim", 128)

        base_dir = self.get_parameter("base_dir").get_parameter_value().string_value
        ensure_dir(base_dir)

        sqlite_path = (
            self.get_parameter("sqlite_path").get_parameter_value().string_value
        )
        if not sqlite_path:
            sqlite_path = os.path.join(base_dir, "memory.db")

        chroma_path = (
            self.get_parameter("chroma_path").get_parameter_value().string_value
        )
        if not chroma_path:
            chroma_path = os.path.join(base_dir, "chroma")

        self.collection_name = (
            self.get_parameter("chroma_collection").get_parameter_value().string_value
        )
        self.face_encoding_dim = int(self.get_parameter("face_encoding_dim").value)

        # Initialize backends
        self._sqlite_lock = threading.Lock()
        self._conn = sqlite3.connect(sqlite_path, check_same_thread=False)
        self._conn.execute("PRAGMA journal_mode=WAL;")
        self._conn.execute("PRAGMA synchronous=NORMAL;")
        self._conn.execute("PRAGMA foreign_keys=ON;")
        self._create_sqlite_schema()

        self._chroma_client = None
        self._chroma_collection = None
        self._fallback_store = None

        if _HAVE_CHROMA:
            ensure_dir(chroma_path)
            try:
                self._chroma_client = chromadb.Client(
                    ChromaSettings(
                        persist_directory=chroma_path,
                        anonymized_telemetry=False,
                    )
                )
                self._chroma_collection = self._chroma_client.get_or_create_collection(
                    name=self.collection_name,
                    metadata={"hnsw:space": "cosine"},
                )
                self.get_logger().info(
                    f"ChromaDB initialized at: {chroma_path}, collection: {self.collection_name}"
                )
            except Exception as exc:
                self.get_logger().warn(
                    f"Failed to initialize ChromaDB ({exc}); using in-memory fallback."
                )
                self._fallback_store = InMemoryVectorStore()
        else:
            self.get_logger().warn(
                f"ChromaDB not available: {_CHROMA_IMPORT_ERROR}; using in-memory fallback."
            )
            self._fallback_store = InMemoryVectorStore()

        # Embedding model (lazy)
        model_name = (
            self.get_parameter("embed_model_name").get_parameter_value().string_value
        )
        self._embedder = (
            STEmbedder(model_name, self) if _HAVE_SENTENCE_TRANSFORMERS else None
        )
        if not _HAVE_SENTENCE_TRANSFORMERS:
            self.get_logger().warn(
                f"SentenceTransformers not available: {_SENTENCE_TRANSFORMERS_IMPORT_ERROR}. "
                "Requests must provide embeddings explicitly."
            )

        # Services
        self._srv_add_person = self.create_service(
            AddPerson, "/memory/add_person", self.handle_add_person
        )
        self._srv_get_person_by_face = self.create_service(
            GetPersonByFace,
            "/memory/get_person_by_face",
            self.handle_get_person_by_face,
        )
        self._srv_add_conv = self.create_service(
            AddConversationMemory,
            "/memory/add_conversation_memory",
            self.handle_add_conversation_memory,
        )
        self._srv_retrieve = self.create_service(
            RetrieveRelevantMemories,
            "/memory/retrieve_relevant_memories",
            self.handle_retrieve_relevant_memories,
        )

        self.get_logger().info(
            "MemoryServer is up. SQLite: %s | Vector store: %s"
            % (
                sqlite_path,
                "ChromaDB" if self._chroma_collection is not None else "InMemory",
            )
        )

    # --------------------------
    # SQLite schema and helpers
    # --------------------------

    def _create_sqlite_schema(self) -> None:
        with self._sqlite_lock:
            cur = self._conn.cursor()
            cur.execute(
                """
                CREATE TABLE IF NOT EXISTS persons (
                    person_id TEXT PRIMARY KEY,
                    name TEXT,
                    face_encoding TEXT,
                    first_seen_iso TEXT,
                    notes TEXT,
                    created_at TEXT DEFAULT (datetime('now'))
                );
                """
            )
            self._conn.commit()

    def _insert_person(
        self,
        person_id: str,
        name: str,
        face_encoding: List[float],
        first_seen_iso: str,
        notes: str,
    ) -> None:
        enc_json = json.dumps(list(map(float, face_encoding)))
        with self._sqlite_lock:
            cur = self._conn.cursor()
            cur.execute(
                """
                INSERT INTO persons (person_id, name, face_encoding, first_seen_iso, notes)
                VALUES (?, ?, ?, ?, ?)
                """,
                (person_id, name, enc_json, first_seen_iso, notes),
            )
            self._conn.commit()

    def _fetch_persons(self) -> List[Tuple[str, str, List[float]]]:
        """
        Returns: list of (person_id, name, face_encoding_vector)
        """
        with self._sqlite_lock:
            cur = self._conn.cursor()
            cur.execute("SELECT person_id, name, face_encoding FROM persons")
            rows = cur.fetchall()
        out: List[Tuple[str, str, List[float]]] = []
        for pid, name, enc_json in rows:
            try:
                vec = json.loads(enc_json) if enc_json else []
                vec = [float(x) for x in vec]
            except Exception:
                vec = []
            out.append((pid, name, vec))
        return out

    # --------------------------
    # Service: AddPerson
    # --------------------------

    def handle_add_person(
        self, request: AddPerson.Request, response: AddPerson.Response
    ) -> AddPerson.Response:
        try:
            # Validate face encoding
            enc = list(map(float, request.face_encoding or []))
            if enc and len(enc) != self.face_encoding_dim:
                response.success = False
                response.person_id = ""
                response.message = f"Invalid face_encoding length: {len(enc)} (expected {self.face_encoding_dim})"
                return response

            person_id = str(uuid.uuid4())
            ts = ros_time_to_unix_seconds(request.first_seen)
            first_seen_iso = (
                unix_seconds_to_iso(ts) if ts > 0 else unix_seconds_to_iso(time.time())
            )

            self._insert_person(
                person_id=person_id,
                name=request.name or "",
                face_encoding=enc,
                first_seen_iso=first_seen_iso,
                notes=request.notes or "",
            )

            response.success = True
            response.person_id = person_id
            response.message = "Person added"
            return response
        except Exception as exc:
            response.success = False
            response.person_id = ""
            response.message = f"Exception: {exc}"
            return response

    # --------------------------
    # Service: GetPersonByFace
    # --------------------------

    def handle_get_person_by_face(
        self, request: GetPersonByFace.Request, response: GetPersonByFace.Response
    ) -> GetPersonByFace.Response:
        try:
            q = list(map(float, request.face_encoding or []))
            if not q:
                response.success = False
                response.person_id = ""
                response.name = ""
                response.match_confidence = 0.0
                response.candidate_person_ids = []
                response.candidate_names = []
                response.candidate_confidences = []
                response.message = "Empty face_encoding"
                return response

            tol = float(request.tolerance or 0.6)  # default typical tolerance
            k = int(request.k or 3)

            persons = self._fetch_persons()
            if not persons:
                response.success = False
                response.person_id = ""
                response.name = ""
                response.match_confidence = 0.0
                response.candidate_person_ids = []
                response.candidate_names = []
                response.candidate_confidences = []
                response.message = "No persons registered"
                return response

            # Compute distances
            scored: List[Tuple[str, str, float]] = []  # (person_id, name, distance)
            for pid, name, enc in persons:
                if not enc:
                    continue
                d = l2_distance(q, enc)
                scored.append((pid, name, d))

            if not scored:
                response.success = False
                response.person_id = ""
                response.name = ""
                response.match_confidence = 0.0
                response.candidate_person_ids = []
                response.candidate_names = []
                response.candidate_confidences = []
                response.message = "No valid encodings stored"
                return response

            scored.sort(key=lambda t: t[2])  # ascending distance
            top = scored[: max(1, k)]

            # Fill candidates
            c_ids: List[str] = []
            c_names: List[str] = []
            c_scores: List[float] = []
            for pid, name, dist in top:
                c_ids.append(pid)
                c_names.append(name)
                c_scores.append(normalize_score_from_distance(dist, tol))

            # Best match
            best_pid, best_name, best_dist = scored[0]
            best_conf = normalize_score_from_distance(best_dist, tol)
            success = best_dist <= tol

            response.success = bool(success)
            response.person_id = best_pid if success else ""
            response.name = best_name if success else ""
            response.match_confidence = float(best_conf if success else 0.0)
            response.candidate_person_ids = c_ids
            response.candidate_names = c_names
            response.candidate_confidences = c_scores
            response.message = (
                "ok"
                if success
                else f"no match within tolerance (best_dist={best_dist:.3f} tol={tol:.3f})"
            )
            return response
        except Exception as exc:
            response.success = False
            response.person_id = ""
            response.name = ""
            response.match_confidence = 0.0
            response.candidate_person_ids = []
            response.candidate_names = []
            response.candidate_confidences = []
            response.message = f"Exception: {exc}"
            return response

    # --------------------------
    # Service: AddConversationMemory
    # --------------------------

    def handle_add_conversation_memory(
        self,
        request: AddConversationMemory.Request,
        response: AddConversationMemory.Response,
    ) -> AddConversationMemory.Response:
        try:
            text = request.text or ""
            role = request.role or ""
            person_id = request.person_id or ""
            channel = request.channel or ""
            tags = list(request.tags or [])
            conversation_id = request.conversation_id or str(uuid.uuid4())
            ts = ros_time_to_unix_seconds(request.timestamp)
            ts = ts if ts > 0 else time.time()

            embedding: List[float] = list(map(float, request.embedding or []))
            if not embedding:
                # Attempt to embed if model available and text present
                if self._embedder is not None and text:
                    try:
                        embedding = self._embedder.embed_text(text)
                    except Exception as e:
                        response.success = False
                        response.memory_id = ""
                        response.conversation_id = conversation_id
                        response.message = f"Embedding failed: {e}"
                        return response
                else:
                    response.success = False
                    response.memory_id = ""
                    response.conversation_id = conversation_id
                    response.message = "Missing embedding and embedding model unavailable or text empty"
                    return response

            memory_id = str(uuid.uuid4())

            if self._chroma_collection is not None:
                # ChromaDB persistent
                try:
                    # Chroma expects lists
                    self._chroma_collection.add(
                        ids=[memory_id],
                        documents=[text],
                        embeddings=[embedding],
                        metadatas=[
                            {
                                "person_id": person_id,
                                "conversation_id": conversation_id,
                                "role": role,
                                "channel": channel,
                                "tags": tags,
                                "timestamp": ts,
                            }
                        ],
                    )
                    # Persist on disk
                    try:
                        self._chroma_client.persist()
                    except Exception:
                        pass
                except Exception as exc:
                    response.success = False
                    response.memory_id = ""
                    response.conversation_id = conversation_id
                    response.message = f"Chroma add failed: {exc}"
                    return response
            else:
                # Fallback in-memory
                entry = MemoryEntry(
                    memory_id=memory_id,
                    person_id=person_id,
                    conversation_id=conversation_id,
                    role=role,
                    text=text,
                    embedding=embedding,
                    timestamp=ts,
                    tags=tags,
                    channel=channel,
                )
                assert self._fallback_store is not None
                self._fallback_store.add(entry)

            response.success = True
            response.memory_id = memory_id
            response.conversation_id = conversation_id
            response.message = "ok"
            return response
        except Exception as exc:
            response.success = False
            response.memory_id = ""
            response.conversation_id = request.conversation_id or ""
            response.message = f"Exception: {exc}"
            return response

    # --------------------------
    # Service: RetrieveRelevantMemories
    # --------------------------

    def handle_retrieve_relevant_memories(
        self,
        request: RetrieveRelevantMemories.Request,
        response: RetrieveRelevantMemories.Response,
    ) -> RetrieveRelevantMemories.Response:
        try:
            top_k = int(request.top_k or 5)
            min_score = float(request.min_score or 0.0)
            person_filter = request.person_id or ""

            # Time filters
            after_ts = (
                ros_time_to_unix_seconds(request.after_time)
                if request.after_time is not None
                else 0.0
            )
            before_ts = (
                ros_time_to_unix_seconds(request.before_time)
                if request.before_time is not None
                else 0.0
            )
            after_ts = after_ts if after_ts > 0 else None
            before_ts = before_ts if before_ts > 0 else None

            include_roles = list(request.include_roles or [])
            include_tags = list(request.include_tags or [])
            conversation_id = request.conversation_id or None
            channel = request.channel or None

            # Resolve query embedding
            query_embedding: List[float] = list(request.query_embedding or [])
            if not query_embedding:
                qtext = request.query_text or ""
                if not qtext:
                    response.success = False
                    response.message = (
                        "Either query_text or query_embedding must be provided"
                    )
                    response.returned_k = 0
                    return response
                if self._embedder is None:
                    response.success = False
                    response.message = (
                        "Embedding model unavailable and query_embedding not provided"
                    )
                    response.returned_k = 0
                    return response
                try:
                    query_embedding = self._embedder.embed_text(qtext)
                except Exception as e:
                    response.success = False
                    response.message = f"Embedding failed: {e}"
                    response.returned_k = 0
                    return response

            # Query backend
            results: List[
                Tuple[str, Dict[str, Any], str, float]
            ] = []  # (id, meta, doc, score)

            if self._chroma_collection is not None:
                # Build Chroma filters (limited expressiveness; additional filtering applied client-side)
                where: Dict[str, Any] = {}
                if person_filter:
                    where["person_id"] = person_filter
                if conversation_id:
                    where["conversation_id"] = conversation_id
                if channel:
                    where["channel"] = channel
                # Time filters are not consistently supported across all backends; filter client-side after query.

                # Over-fetch to allow client-side tag/role/time filtering
                n_fetch = max(top_k * 5, top_k)
                try:
                    query_res = self._chroma_collection.query(
                        query_embeddings=[query_embedding],
                        n_results=max(1, n_fetch),
                        where=where if where else None,
                        include=["documents", "metadatas", "distances"],
                    )
                    ids = query_res.get("ids", [[]])[0]
                    docs = query_res.get("documents", [[]])[0]
                    metas = query_res.get("metadatas", [[]])[0]
                    dists = query_res.get("distances", [[]])[0]

                    for i, mid in enumerate(ids):
                        meta = metas[i] if i < len(metas) else {}
                        doc = docs[i] if i < len(docs) else ""
                        dist = float(dists[i]) if i < len(dists) else 1.0
                        score = max(
                            0.0, min(1.0, 1.0 - dist)
                        )  # cosine distance -> score
                        results.append((mid, meta, doc, score))
                except Exception as exc:
                    response.success = False
                    response.message = f"Chroma query failed: {exc}"
                    response.returned_k = 0
                    return response
            else:
                # In-memory fallback
                assert self._fallback_store is not None
                fetched = self._fallback_store.query(
                    query_embedding=query_embedding,
                    person_id=person_filter or None,
                    top_k=max(1, top_k * 5),
                    include_roles=include_roles or None,
                    include_tags=include_tags or None,
                    after_time=after_ts,
                    before_time=before_ts,
                    conversation_id=conversation_id,
                    channel=channel,
                )
                for entry, score in fetched:
                    meta = {
                        "person_id": entry.person_id,
                        "conversation_id": entry.conversation_id,
                        "role": entry.role,
                        "channel": entry.channel,
                        "tags": entry.tags,
                        "timestamp": entry.timestamp,
                    }
                    results.append((entry.memory_id, meta, entry.text, float(score)))

            # Client-side filtering on tags/roles/time
            filtered: List[Tuple[str, Dict[str, Any], str, float]] = []
            for mid, meta, doc, score in results:
                # Time
                ts = (
                    float(meta.get("timestamp", 0.0)) if isinstance(meta, dict) else 0.0
                )
                if after_ts and ts <= after_ts:
                    continue
                if before_ts and ts >= before_ts:
                    continue
                # Roles
                if include_roles and (meta.get("role") not in include_roles):
                    continue
                # Tags (intersection)
                if include_tags:
                    tags_meta = meta.get("tags", [])
                    if not isinstance(tags_meta, list):
                        tags_meta = []
                    if not (set(include_tags) & set(tags_meta)):
                        continue
                if score < min_score:
                    continue
                filtered.append((mid, meta, doc, score))

            # Sort by score desc and cut to top_k
            filtered.sort(key=lambda t: t[3], reverse=True)
            final = filtered[: max(1, top_k)]

            # Fill response
            response.success = True
            response.message = "ok"
            response.returned_k = len(final)

            response.memory_ids = []
            response.person_ids = []
            response.conversation_ids = []
            response.roles = []
            response.channels = []
            response.texts = []
            response.tags_csv = []
            response.timestamps = []
            response.scores = []

            for mid, meta, doc, score in final:
                response.memory_ids.append(str(mid))
                response.person_ids.append(str(meta.get("person_id", "")))
                response.conversation_ids.append(str(meta.get("conversation_id", "")))
                response.roles.append(str(meta.get("role", "")))
                response.channels.append(str(meta.get("channel", "")))
                response.texts.append(str(doc))

                tags_meta = meta.get("tags", [])
                if not isinstance(tags_meta, list):
                    tags_meta = []
                response.tags_csv.append(",".join(map(str, tags_meta)))

                # Timestamp
                ts = float(meta.get("timestamp", 0.0))
                ts_msg = RosTime()
                if ts > 0.0:
                    sec = int(ts)
                    nsec = int((ts - sec) * 1e9)
                    ts_msg.sec = sec
                    ts_msg.nanosec = nsec
                else:
                    ts_msg.sec = 0
                    ts_msg.nanosec = 0
                response.timestamps.append(ts_msg)

                response.scores.append(float(score))

            return response
        except Exception as exc:
            response.success = False
            response.message = f"Exception: {exc}"
            response.returned_k = 0
            response.memory_ids = []
            response.person_ids = []
            response.conversation_ids = []
            response.roles = []
            response.channels = []
            response.texts = []
            response.tags_csv = []
            response.timestamps = []
            response.scores = []
            return response

    # --------------------------
    # Shutdown
    # --------------------------

    def destroy_node(self) -> bool:
        try:
            if self._chroma_client is not None:
                try:
                    self._chroma_client.persist()
                except Exception:
                    pass
            with self._sqlite_lock:
                self._conn.commit()
                self._conn.close()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MemoryServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
