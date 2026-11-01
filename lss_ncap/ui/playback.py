"""Display-independent playback state for animation and timeline scrubbing."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(slots=True)
class PlaybackState:
    frame_count: int
    index: int = 0
    playing: bool = False

    def __post_init__(self) -> None:
        if self.frame_count < 1:
            raise ValueError("Playback requires at least one frame")

    def reset(self, frame_count: int) -> None:
        if frame_count < 1:
            raise ValueError("Playback requires at least one frame")
        self.frame_count = frame_count
        self.index = 0
        self.playing = False

    def scrub(self, value: float) -> None:
        self.playing = False
        self.index = max(0, min(round(value), self.frame_count - 1))

    def advance(self) -> bool:
        """Advance one sample; return whether another sample remains."""
        if not self.playing:
            return False
        self.index = min(self.index + 1, self.frame_count - 1)
        if self.index == self.frame_count - 1:
            self.playing = False
        return self.playing


@dataclass(slots=True)
class PlaybackClock:
    """Maps elapsed wall time to deterministic simulation sample indices."""

    started_at_s: float = 0.0
    start_index: int = 0

    def start(self, index: int, now_s: float) -> None:
        self.start_index = index
        self.started_at_s = now_s

    def index_at(self, now_s: float, sample_period_s: float,
                 frame_count: int) -> int:
        if sample_period_s <= 0 or frame_count < 1:
            raise ValueError("Playback timing and frame count must be positive")
        elapsed_s = max(0.0, now_s - self.started_at_s)
        elapsed_frames = int(elapsed_s / sample_period_s)
        return min(self.start_index + elapsed_frames, frame_count - 1)
