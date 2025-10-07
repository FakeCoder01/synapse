#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Project Synapse — Conversation Manager

A ROS 2 node that coordinates:
- STT (Speech-to-Text) to capture a user's speech
- Memory retrieval to fetch relevant past interactions
- LLM to generate a response with context and current emotion
- TTS (Text-to-Speech) to speak the response
- Memory writing to store the conversation as long-term semantic memories

High-level flow:
1) Subscribe to /detected_persons to detect a nearby known person and trigger a session.
2) Subscribe to /emotion/state to include current mood in the prompt.
3) When a session triggers:
   - Capture user's utterance via STT
   - Retrieve relevant memories for that person via RetrieveRelevantMemories
   - Build a prompt (persona + memory summary + emotion state + user message)
   - Call LLM (OpenAI or Google; fallback to local rule-based if unavailable)
   - Speak response via TTS (Coqui TTS or gTTS; fallback to console)
   - Store user and robot utterances via AddConversationMemory
   - Repeat for a small number of turns or until timeout

Notes:
- Dependencies for STT/TTS/LLM are optional and selected via parameters/environment.
- If external APIs are not available, the node still runs with graceful fallbacks.

Parameters (examples with defaults):
- activation.person_distance_threshold: 1.5
- activation.recognition_conf_threshold: 0.50
- activation.cooldown_sec: 45.0
- stt.engine: "vosk" | "whisper" | "dummy"
- stt.sample_rate: 16000
- stt.device: ""   (implementation-specific)
- tts.engine: "coqui" | "gtts" | "dummy"
- tts.voice: ""    (implementation-specific)
- llm.provider: "openai" | "google" | "dummy"
- llm.openai_model: "gpt-4o-mini"
- llm.google_model: "gemini-1.5-flash"
- memory.retrieve_service: "/memory/retrieve_relevant_memories"
- memory.add_service: "/memory/add_conversation_memory"
- topics.detected_persons: "/detected_persons"
- topics.emotion_state: "/emotion/state"
- topics.transcript_pub: "/conversation/transcript"
- topics.response_pub: "/conversation/response"
- session.max_turns: 2
- session.user_timeout_sec: 20.0
- session.max_session_sec: 90.0
- session.top_k_memories: 6
- session.min_memory_score: 0.30

Environment variables:
- OPENAI_API_KEY, GOOGLE_API_KEY
- SYNAPSE_TTS_ENGINE, SYNAPSE_STT_ENGINE, LLM_PROVIDER (optional overrides)

Security:
- Do not hardcode API keys. Provide them via environment or secret management.

License: Apache-2.0
"""

from __future__ import annotations

import os
import sys
import time
import uuid
import math
import json
import queue
import tempfile
import threading
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import String
from synapse_interfaces.msg import DetectedPersons, EmotionState
from synapse_interfaces.srv import (
    RetrieveRelevantMemories,
    AddConversationMemory,
)

# Optional dependencies (import guarded)
_HAVE_VOSK = False
_HAVE_SD = False
_HAVE_OPENAI = False
_HAVE_GOOGLE_GENAI = False
_HAVE_TTS_COQUI = False
_HAVE_TTS_GTTS = False
_HAVE_SENTENCE_TRANSFORMERS = False

try:
    import vosk  # type: ignore

    _HAVE_VOSK = True
except Exception:
    pass

try:
    import sounddevice as sd  # type: ignore

    _HAVE_SD = True
except Exception:
    pass

try:
    # OpenAI new SDK (>=1.0)
    from openai import OpenAI  # type: ignore

    _HAVE_OPENAI = True
except Exception:
    try:
        # Legacy fallback
        import openai  # type: ignore

        _HAVE_OPENAI = True
    except Exception:
        pass

try:
    import google.generativeai as genai  # type: ignore

    _HAVE_GOOGLE_GENAI = True
except Exception:
    pass

try:
    # Coqui TTS
    from TTS.api import TTS  # type: ignore

    _HAVE_TTS_COQUI = True
except Exception:
    pass

try:
    # gTTS fallback
    from gtts import gTTS  # type: ignore

    _HAVE_TTS_GTTS = True
except Exception:
    pass

try:
    from sentence_transformers import SentenceTransformer  # type: ignore

    _HAVE_SENTENCE_TRANSFORMERS = True
except Exception:
    pass


# ------------------------------
# Utility
# ------------------------------


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def now_ts() -> float:
    return time.time()


def safe_getenv(name: str, default: Optional[str] = None) -> Optional[str]:
    val = os.getenv(name, default)
    if val is not None and len(val.strip()) == 0:
        return default
    return val


def shorten(text: str, max_len: int = 180) -> str:
    if len(text) <= max_len:
        return text
    return text[: max_len - 1] + "…"


# ------------------------------
# STT Engines
# ------------------------------


class STTBase:
    def transcribe_once(self, timeout_sec: float) -> Optional[str]:
        raise NotImplementedError("STT not implemented")


class STTVosk(STTBase):
    """
    Simple Vosk streaming recognizer using sounddevice for mic capture.
    Captures until silence or timeout, returns a single utterance string.
    """

    def __init__(
        self,
        model_path: Optional[str] = None,
        sample_rate: int = 16000,
        device: Optional[str] = None,
    ):
        if not (_HAVE_VOSK and _HAVE_SD):
            raise RuntimeError("Vosk/sounddevice not available")
        self.sample_rate = sample_rate
        self.device = device
        if model_path is None:
            # The user must provide the Vosk model path or install default model
            # Attempt to use VOSK_MODEL_PATH env
            model_path = safe_getenv("VOSK_MODEL_PATH", None)
        if not model_path or not os.path.isdir(model_path):
            raise RuntimeError(
                "Vosk model path not set or invalid (env VOSK_MODEL_PATH)"
            )
        self._model = vosk.Model(model_path)

    def transcribe_once(self, timeout_sec: float) -> Optional[str]:
        rec = vosk.KaldiRecognizer(self._model, self.sample_rate)
        rec.SetWords(True)
        deadline = now_ts() + timeout_sec
        buff_q: queue.Queue = queue.Queue()

        def _callback(indata, frames, time_info, status):
            try:
                buff_q.put(bytes(indata))
            except Exception:
                pass

        try:
            stream = sd.RawInputStream(
                samplerate=self.sample_rate,
                blocksize=8000,
                device=self.device,
                dtype="int16",
                channels=1,
                callback=_callback,
            )
        except Exception as exc:
            raise RuntimeError(f"Failed to open audio input: {exc}") from exc

        text_accum: List[str] = []
        silence_counter = 0
        silence_limit = int(0.8 / (8000 / self.sample_rate))  # approx 0.8s of silence

        with stream:
            while now_ts() < deadline:
                try:
                    data = buff_q.get(timeout=0.2)
                except queue.Empty:
                    continue
                if rec.AcceptWaveform(data):
                    res = json.loads(rec.Result())
                    if "text" in res and res["text"]:
                        text_accum.append(res["text"])
                        # reset silence
                        silence_counter = 0
                else:
                    # partial results; check for silence heuristics via final() style not available; simple timeout
                    silence_counter += 1
                    if silence_counter >= silence_limit:
                        break

        # Final result merge
        if text_accum:
            joined = " ".join(text_accum).strip()
            return joined if joined else None
        else:
            # Try final partial
            try:
                res = json.loads(rec.FinalResult())
                if "text" in res and res["text"]:
                    return res["text"]
            except Exception:
                pass
        return None


class STTDummy(STTBase):
    """
    Fallback STT that prompts the developer to type the user's utterance.
    Useful for development when audio/STT isn't configured.
    """

    def transcribe_once(self, timeout_sec: float) -> Optional[str]:
        sys.stdout.write("[STT dummy] Type user's utterance (or blank to cancel): ")
        sys.stdout.flush()
        try:
            # No strict timeout; simulate by empty input
            line = sys.stdin.readline()
            if not line:
                return None
            line = line.strip()
            return line if line else None
        except Exception:
            return None


# ------------------------------
# TTS Engines
# ------------------------------


class TTSBase:
    def speak(self, text: str) -> None:
        raise NotImplementedError("TTS not implemented")


class TTSCoqui(TTSBase):
    def __init__(self, model_name: Optional[str] = None, speaker: Optional[str] = None):
        if not _HAVE_TTS_COQUI:
            raise RuntimeError("Coqui TTS not available")
        # If model_name is None, Coqui will try to download a default model.
        self._tts = TTS(model_name) if model_name else TTS()
        self._speaker = speaker

    def speak(self, text: str) -> None:
        if not text:
            return
        try:
            with tempfile.NamedTemporaryFile(
                prefix="synapse_tts_", suffix=".wav", delete=False
            ) as f:
                wav_path = f.name
            self._tts.tts_to_file(text=text, file_path=wav_path, speaker=self._speaker)
            # Play via system player (aplay/ffplay)
            self._play_file(wav_path)
        finally:
            try:
                os.unlink(wav_path)
            except Exception:
                pass

    @staticmethod
    def _play_file(path: str) -> None:
        # Try ffplay, then aplay
        for cmd in (
            ["ffplay", "-nodisp", "-autoexit", "-loglevel", "quiet", path],
            ["aplay", path],
        ):
            try:
                import subprocess

                subprocess.run(cmd, check=True)
                return
            except Exception:
                continue
        # Fallback: no playback
        sys.stdout.write(f"[TTS] Audio ready at: {path}\n")
        sys.stdout.flush()


class TTSgTTS(TTSBase):
    def __init__(self, lang: str = "en"):
        if not _HAVE_TTS_GTTS:
            raise RuntimeError("gTTS not available")
        self.lang = lang

    def speak(self, text: str) -> None:
        if not text:
            return
        try:
            tts = gTTS(text=text, lang=self.lang)
            with tempfile.NamedTemporaryFile(
                prefix="synapse_tts_", suffix=".mp3", delete=False
            ) as f:
                mp3_path = f.name
            tts.save(mp3_path)
            self._play_file(mp3_path)
        finally:
            try:
                os.unlink(mp3_path)
            except Exception:
                pass

    @staticmethod
    def _play_file(path: str) -> None:
        # Try mpg123, then ffplay
        for cmd in (
            ["mpg123", "-q", path],
            ["ffplay", "-nodisp", "-autoexit", "-loglevel", "quiet", path],
        ):
            try:
                import subprocess

                subprocess.run(cmd, check=True)
                return
            except Exception:
                continue
        sys.stdout.write(f"[TTS] Audio ready at: {path}\n")
        sys.stdout.flush()


class TTSDummy(TTSBase):
    def speak(self, text: str) -> None:
        sys.stdout.write(f"[TTS dummy] {text}\n")
        sys.stdout.flush()


# ------------------------------
# LLM Clients
# ------------------------------


class LLMBase:
    def complete(self, system_prompt: str, user_text: str) -> str:
        raise NotImplementedError("LLM not implemented")


class LLMOpenAI(LLMBase):
    def __init__(self, model: str = "gpt-4o-mini", api_key: Optional[str] = None):
        if not _HAVE_OPENAI:
            raise RuntimeError("openai SDK not available")
        self.model = model
        self.api_key = api_key or safe_getenv("OPENAI_API_KEY", None)
        if not self.api_key:
            raise RuntimeError("OPENAI_API_KEY is required for LLMOpenAI")

        # Try new SDK first
        self._client_new: Optional[Any] = None
        self._legacy = False
        try:
            self._client_new = OpenAI(api_key=self.api_key)  # type: ignore
        except Exception:
            # fallback to legacy
            self._legacy = True
            try:
                import openai  # type: ignore

                openai.api_key = self.api_key
                self._openai_legacy = openai
            except Exception as exc:
                raise RuntimeError(f"Failed to init OpenAI client: {exc}") from exc

    def complete(self, system_prompt: str, user_text: str) -> str:
        messages = [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_text},
        ]
        try:
            if not self._legacy and self._client_new is not None:
                resp = self._client_new.chat.completions.create(
                    model=self.model,
                    messages=messages,
                    temperature=0.7,
                )
                return (resp.choices[0].message.content or "").strip()
            else:
                # legacy
                resp = self._openai_legacy.ChatCompletion.create(
                    model=self.model,
                    messages=messages,
                    temperature=0.7,
                )
                return (resp["choices"][0]["message"]["content"] or "").strip()
        except Exception as exc:
            return f"(LLM error) {exc}"


class LLMGoogle(LLMBase):
    def __init__(self, model: str = "gemini-1.5-flash", api_key: Optional[str] = None):
        if not _HAVE_GOOGLE_GENAI:
            raise RuntimeError("google.generativeai not available")
        self.model = model
        self.api_key = api_key or safe_getenv("GOOGLE_API_KEY", None)
        if not self.api_key:
            raise RuntimeError("GOOGLE_API_KEY is required for LLMGoogle")
        genai.configure(api_key=self.api_key)

    def complete(self, system_prompt: str, user_text: str) -> str:
        try:
            model = genai.GenerativeModel(self.model)
            # Provide system prompt as the first part
            resp = model.generate_content(
                [{"role": "user", "text": f"{system_prompt}\n\nUser: {user_text}"}]
            )
            if hasattr(resp, "text") and resp.text:
                return resp.text.strip()
            # Some responses may include candidates
            if getattr(resp, "candidates", None):
                parts = []
                for cand in resp.candidates:
                    try:
                        parts.append(cand.content.parts[0].text)
                    except Exception:
                        pass
                joined = "\n".join([p for p in parts if p]).strip()
                if joined:
                    return joined
            return ""
        except Exception as exc:
            return f"(LLM error) {exc}"


class LLMDummy(LLMBase):
    def complete(self, system_prompt: str, user_text: str) -> str:
        # Simple rule-based fallback
        user_text_l = user_text.lower()
        if "hello" in user_text_l or "hi" in user_text_l:
            return "Hi there! It's great to see you."
        if "how are you" in user_text_l:
            return "I'm feeling good and ready to help."
        return "I hear you. Tell me more."


# ------------------------------
# Conversation Manager Node
# ------------------------------


@dataclass
class PersonCandidate:
    person_id: str
    name: str
    distance: float
    rec_conf: float


class ConversationManager(Node):
    def __init__(self):
        super().__init__("conversation_manager")

        # Parameters
        self.declare_parameter("activation.person_distance_threshold", 1.5)
        self.declare_parameter("activation.recognition_conf_threshold", 0.50)
        self.declare_parameter("activation.cooldown_sec", 45.0)

        self.declare_parameter("stt.engine", safe_getenv("SYNAPSE_STT_ENGINE", "dummy"))
        self.declare_parameter("stt.sample_rate", 16000)
        self.declare_parameter("stt.device", "")

        self.declare_parameter("tts.engine", safe_getenv("SYNAPSE_TTS_ENGINE", "dummy"))
        self.declare_parameter("tts.voice", "")
        self.declare_parameter("tts.lang", "en")

        self.declare_parameter("llm.provider", safe_getenv("LLM_PROVIDER", "dummy"))
        self.declare_parameter("llm.openai_model", "gpt-4o-mini")
        self.declare_parameter("llm.google_model", "gemini-1.5-flash")

        self.declare_parameter(
            "memory.retrieve_service", "/memory/retrieve_relevant_memories"
        )
        self.declare_parameter("memory.add_service", "/memory/add_conversation_memory")

        self.declare_parameter("topics.detected_persons", "/detected_persons")
        self.declare_parameter("topics.emotion_state", "/emotion/state")
        self.declare_parameter("topics.transcript_pub", "/conversation/transcript")
        self.declare_parameter("topics.response_pub", "/conversation/response")

        self.declare_parameter("session.max_turns", 2)
        self.declare_parameter("session.user_timeout_sec", 20.0)
        self.declare_parameter("session.max_session_sec", 90.0)
        self.declare_parameter("session.top_k_memories", 6)
        self.declare_parameter("session.min_memory_score", 0.30)
        self.declare_parameter("persona.name", "Synapse")
        self.declare_parameter("persona.style", "kind, curious, supportive, concise")

        # Resolve parameters
        self.dist_threshold = float(
            self.get_parameter("activation.person_distance_threshold").value
        )
        self.rec_conf_threshold = float(
            self.get_parameter("activation.recognition_conf_threshold").value
        )
        self.cooldown_sec = float(self.get_parameter("activation.cooldown_sec").value)

        self.stt_engine_name = (
            self.get_parameter("stt.engine").get_parameter_value().string_value
        )
        self.stt_sample_rate = int(self.get_parameter("stt.sample_rate").value)
        self.stt_device = (
            self.get_parameter("stt.device").get_parameter_value().string_value
        )

        self.tts_engine_name = (
            self.get_parameter("tts.engine").get_parameter_value().string_value
        )
        self.tts_voice = (
            self.get_parameter("tts.voice").get_parameter_value().string_value
        )
        self.tts_lang = (
            self.get_parameter("tts.lang").get_parameter_value().string_value
        )

        self.llm_provider = (
            self.get_parameter("llm.provider").get_parameter_value().string_value
        )
        self.openai_model = (
            self.get_parameter("llm.openai_model").get_parameter_value().string_value
        )
        self.google_model = (
            self.get_parameter("llm.google_model").get_parameter_value().string_value
        )

        self.retrieve_srv_name = (
            self.get_parameter("memory.retrieve_service")
            .get_parameter_value()
            .string_value
        )
        self.addmem_srv_name = (
            self.get_parameter("memory.add_service").get_parameter_value().string_value
        )

        self.detected_topic = (
            self.get_parameter("topics.detected_persons")
            .get_parameter_value()
            .string_value
        )
        self.emotion_topic = (
            self.get_parameter("topics.emotion_state")
            .get_parameter_value()
            .string_value
        )
        self.transcript_topic = (
            self.get_parameter("topics.transcript_pub")
            .get_parameter_value()
            .string_value
        )
        self.response_topic = (
            self.get_parameter("topics.response_pub").get_parameter_value().string_value
        )

        self.max_turns = int(self.get_parameter("session.max_turns").value)
        self.user_timeout_sec = float(
            self.get_parameter("session.user_timeout_sec").value
        )
        self.max_session_sec = float(
            self.get_parameter("session.max_session_sec").value
        )
        self.top_k_memories = int(self.get_parameter("session.top_k_memories").value)
        self.min_memory_score = float(
            self.get_parameter("session.min_memory_score").value
        )
        self.persona_name = (
            self.get_parameter("persona.name").get_parameter_value().string_value
        )
        self.persona_style = (
            self.get_parameter("persona.style").get_parameter_value().string_value
        )

        # Runtime state
        self._last_emotion: EmotionState = EmotionState()
        self._last_seen_person: Dict[str, float] = {}  # person_id -> last_ts
        self._session_lock = threading.Lock()
        self._session_active = False

        # Publishers
        self.pub_transcript = self.create_publisher(String, self.transcript_topic, 10)
        self.pub_response = self.create_publisher(String, self.response_topic, 10)

        # Subscribers
        self.create_subscription(
            DetectedPersons, self.detected_topic, self._on_detected_persons, 10
        )
        self.create_subscription(
            EmotionState, self.emotion_topic, self._on_emotion_state, 10
        )

        # Service clients
        self.cli_retrieve = self.create_client(
            RetrieveRelevantMemories, self.retrieve_srv_name
        )
        self.cli_addmem = self.create_client(
            AddConversationMemory, self.addmem_srv_name
        )

        # Embedding model optional (to pre-embed texts before storing)
        self._embedder = None
        if _HAVE_SENTENCE_TRANSFORMERS:
            try:
                model_name = safe_getenv("SYNAPSE_EMBED_MODEL", "all-MiniLM-L6-v2")
                self._embedder = SentenceTransformer(model_name)  # type: ignore
            except Exception:
                self._embedder = None

        # STT engine
        self._stt = self._init_stt()
        # TTS engine
        self._tts = self._init_tts()
        # LLM client
        self._llm = self._init_llm()

        self.get_logger().info(
            "ConversationManager ready:\n"
            f"  Activation: dist<= {self.dist_threshold:.2f} m, rec_conf>= {self.rec_conf_threshold:.2f}, cooldown {self.cooldown_sec:.0f}s\n"
            f"  STT: {self.stt_engine_name}, TTS: {self.tts_engine_name}, LLM: {self.llm_provider}\n"
            f"  Memory services: retrieve='{self.retrieve_srv_name}', add='{self.addmem_srv_name}'\n"
            f"  Topics: detected='{self.detected_topic}', emotion='{self.emotion_topic}'\n"
            f"  Session: max_turns={self.max_turns}, user_timeout={self.user_timeout_sec:.0f}s, max_session={self.max_session_sec:.0f}s\n"
        )

    # -------------------------
    # Initializers
    # -------------------------

    def _init_stt(self) -> STTBase:
        name = (self.stt_engine_name or "dummy").lower()
        if name == "vosk":
            try:
                return STTVosk(
                    model_path=None,
                    sample_rate=self.stt_sample_rate,
                    device=(self.stt_device or None),
                )
            except Exception as exc:
                self.get_logger().warn(
                    f"STT Vosk unavailable: {exc}. Falling back to dummy."
                )
        # "whisper" can be added here with a wrapper.
        return STTDummy()

    def _init_tts(self) -> TTSBase:
        name = (self.tts_engine_name or "dummy").lower()
        if name == "coqui":
            try:
                return TTSCoqui(model_name=None, speaker=(self.tts_voice or None))
            except Exception as exc:
                self.get_logger().warn(
                    f"TTS Coqui unavailable: {exc}. Falling back to gTTS/dummy."
                )
                # Intentional fallthrough to gTTS
        if name in ("gtts", "google"):
            try:
                return TTSgTTS(lang=self.tts_lang or "en")
            except Exception as exc:
                self.get_logger().warn(
                    f"TTS gTTS unavailable: {exc}. Falling back to dummy."
                )
        return TTSDummy()

    def _init_llm(self) -> LLMBase:
        prov = (self.llm_provider or "dummy").lower()
        if prov == "openai":
            try:
                return LLMOpenAI(model=self.openai_model)
            except Exception as exc:
                self.get_logger().warn(
                    f"LLM OpenAI unavailable: {exc}. Falling back to dummy."
                )
        elif prov in ("google", "gemini"):
            try:
                return LLMGoogle(model=self.google_model)
            except Exception as exc:
                self.get_logger().warn(
                    f"LLM Google unavailable: {exc}. Falling back to dummy."
                )
        return LLMDummy()

    # -------------------------
    # Subscribers
    # -------------------------

    def _on_emotion_state(self, msg: EmotionState) -> None:
        self._last_emotion = msg

    def _on_detected_persons(self, msg: DetectedPersons) -> None:
        if self._session_active:
            return
        candidate = self._select_candidate(msg)
        if not candidate:
            return
        # Cooldown per-person
        last_ts = self._last_seen_person.get(candidate.person_id, 0.0)
        if (now_ts() - last_ts) < self.cooldown_sec:
            return

        # Start session thread
        self._last_seen_person[candidate.person_id] = now_ts()
        threading.Thread(
            target=self._run_session,
            args=(candidate.person_id, candidate.name),
            daemon=True,
        ).start()

    def _select_candidate(self, msg: DetectedPersons) -> Optional[PersonCandidate]:
        best: Optional[PersonCandidate] = None

        # Prefer primary if known and within thresholds
        if 0 <= msg.primary_index < len(msg.persons):
            p = msg.persons[msg.primary_index]
            if p.is_known and (p.recognition_confidence >= self.rec_conf_threshold):
                dist_ok = (p.distance_m > 0.0) and (p.distance_m <= self.dist_threshold)
                if dist_ok:
                    return PersonCandidate(
                        person_id=p.person_id or "",
                        name=p.name or "",
                        distance=float(p.distance_m),
                        rec_conf=float(p.recognition_confidence),
                    )

        # Fallback: scan all persons
        for p in msg.persons:
            if not p.is_known:
                continue
            if (p.recognition_confidence or 0.0) < self.rec_conf_threshold:
                continue
            if not (p.distance_m > 0.0 and p.distance_m <= self.dist_threshold):
                continue
            c = PersonCandidate(
                person_id=p.person_id or "",
                name=p.name or "",
                distance=float(p.distance_m),
                rec_conf=float(p.recognition_confidence),
            )
            if (best is None) or (c.distance < best.distance):
                best = c
        return best

    # -------------------------
    # Session
    # -------------------------

    def _run_session(self, person_id: str, person_name: str) -> None:
        if not self._session_lock.acquire(blocking=False):
            return
        self._session_active = True
        try:
            self.get_logger().info(
                f"Starting conversation session with {person_name or person_id}..."
            )
            started = now_ts()
            conversation_id = str(uuid.uuid4())
            turns = 0

            # Optional: initial greeting
            greeting = self._make_greeting(person_name)
            self._say(greeting)
            self._store_memory(person_id, "robot", greeting, conversation_id)

            while (
                turns < self.max_turns and (now_ts() - started) < self.max_session_sec
            ):
                # Capture user utterance
                user_text = self._listen(self.user_timeout_sec)
                if not user_text:
                    self.get_logger().info("No user input captured; ending session.")
                    break
                self._publish_transcript(user_text)
                self._store_memory(person_id, "user", user_text, conversation_id)

                # Retrieve relevant memories
                memories = self._retrieve_memories(
                    person_id, user_text, top_k=self.top_k_memories
                )
                memory_summary = self._format_memories(memories)

                # Build system prompt
                sys_prompt = self._build_system_prompt(person_name, memory_summary)
                # Ask LLM
                answer = self._ask_llm(sys_prompt, user_text)
                answer = answer.strip() if answer else ""
                if not answer:
                    answer = "Hmm, I need a moment to think about that. Could you rephrase it?"
                # Speak answer
                self._say(answer)
                self._publish_response(answer)
                self._store_memory(person_id, "robot", answer, conversation_id)

                turns += 1

            self.get_logger().info(f"Session finished with {person_name or person_id}.")
        finally:
            self._session_active = False
            self._session_lock.release()

    def _make_greeting(self, name: str) -> str:
        emo = self._last_emotion
        flair = {
            "happy": "It's lovely to see you!",
            "curious": "What's new today?",
            "excited": "I was hoping you'd stop by!",
            "neutral": "How can I help?",
            "tired": "I'm here for you.",
            "anxious": "All good on your side?",
            "annoyed": "I might be a little on edge, but I'm here.",
            "sad": "I could use a friendly chat.",
            "surprised": "Oh! Hi there!",
        }.get((emo.state_label or "").lower(), "How can I help?")
        if name:
            return f"Hi {name}. {flair}"
        return f"Hi there. {flair}"

    def _listen(self, timeout_sec: float) -> Optional[str]:
        try:
            return self._stt.transcribe_once(timeout_sec=timeout_sec)
        except Exception as exc:
            self.get_logger().warn(f"STT error: {exc}")
            return None

    def _say(self, text: str) -> None:
        try:
            self._tts.speak(text)
        except Exception as exc:
            self.get_logger().warn(f"TTS error: {exc}")
            # print fallback
            sys.stdout.write(f"[Speak] {text}\n")
            sys.stdout.flush()

    def _publish_transcript(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.pub_transcript.publish(msg)

    def _publish_response(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.pub_response.publish(msg)

    # -------------------------
    # Memory integration
    # -------------------------

    def _retrieve_memories(
        self, person_id: str, query_text: str, top_k: int
    ) -> List[Tuple[str, float]]:
        """
        Returns a list of (text, score) of relevant memories.
        """
        if not self.cli_retrieve.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn("RetrieveRelevantMemories service not available.")
            return []
        req = RetrieveRelevantMemories.Request()
        req.person_id = person_id
        req.query_text = query_text
        req.top_k = max(1, top_k)
        req.min_score = float(self.min_memory_score)
        # Optional filters:
        req.include_roles = []  # any
        req.include_tags = []  # any
        # time filters: leave unset

        fut = self.cli_retrieve.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.5)
        if not fut.done():
            self.get_logger().warn("RetrieveRelevantMemories timed out.")
            return []
        resp = fut.result()
        if not (resp and resp.success):
            return []
        out: List[Tuple[str, float]] = []
        for text, score in zip(resp.texts, resp.scores):
            out.append((text, float(score)))
        return out

    def _store_memory(
        self, person_id: str, role: str, text: str, conversation_id: str
    ) -> None:
        if not text:
            return
        if not self.cli_addmem.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn("AddConversationMemory service not available.")
            return
        req = AddConversationMemory.Request()
        req.person_id = person_id or ""
        req.role = role
        req.text = text
        req.timestamp = self.get_clock().now().to_msg()
        req.tags = []  # could add contextual tags
        req.conversation_id = conversation_id
        req.channel = "voice"

        # Embedding optional; if server has embedder, it can embed when missing.
        if self._embedder is not None:
            try:
                emb = self._embedder.encode([text], normalize_embeddings=True)[0]
                req.embedding = [float(x) for x in emb.tolist()]
            except Exception:
                req.embedding = []

        fut = self.cli_addmem.call_async(req)
        # Fire-and-forget with short wait
        rclpy.spin_until_future_complete(self, fut, timeout_sec=1.0)

    # -------------------------
    # Prompting
    # -------------------------

    def _format_memories(self, memories: List[Tuple[str, float]]) -> str:
        if not memories:
            return "(no notable past context)"
        lines = []
        for text, score in memories[: self.top_k_memories]:
            lines.append(f"- ({score:.2f}) {shorten(text, 220)}")
        return "\n".join(lines)

    def _build_system_prompt(self, person_name: str, memory_summary: str) -> str:
        emo = self._last_emotion
        emo_label = emo.state_label or "neutral"
        intensity = clamp(emo.intensity or 0.0, 0.0, 1.0)
        valence = clamp(emo.valence or 0.0, -1.0, 1.0)
        arousal = clamp(emo.arousal or 0.0, 0.0, 1.0)

        persona = (
            f"You are {self.persona_name}, a friendly home robot companion. "
            f"Your style is {self.persona_style}. "
            f"You speak concisely and helpfully, with warmth and empathy."
        )

        emotion_note = (
            f"Current mood: {emo_label} "
            f"(intensity={intensity:.2f}, valence={valence:.2f}, arousal={arousal:.2f}). "
            f"Reflect this mood subtly in your tone."
        )

        memory_note = (
            "Relevant past memories with this person (most relevant first):\n"
            f"{memory_summary}\n"
            "Use these memories to personalize your response. If a memory references sensitive data, handle it respectfully."
        )

        if person_name:
            user_note = f"The user you are speaking to is named {person_name}."
        else:
            user_note = "You are speaking to a known user."

        guidelines = (
            "Guidelines:\n"
            "- Be brief (2–3 sentences) unless the user asks for details.\n"
            "- Ask a friendly follow-up question when appropriate.\n"
            "- Avoid making up facts. If unsure, say so and ask for clarification.\n"
        )

        prompt = "\n\n".join(
            [persona, user_note, emotion_note, memory_note, guidelines]
        ).strip()
        return prompt

    def _ask_llm(self, system_prompt: str, user_text: str) -> str:
        try:
            return self._llm.complete(system_prompt, user_text)
        except Exception as exc:
            self.get_logger().warn(f"LLM error: {exc}")
            return "I'm having trouble thinking right now, but I'm here with you."


# ------------------------------
# Main
# ------------------------------


def main(args=None):
    rclpy.init(args=args)
    node = ConversationManager()
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
