"""Explicit lifecycle transitions, independent from ROS lifecycle machinery."""

from enum import Enum


class SlamState(str, Enum):
    WAITING = "WAITING"
    RUNNING = "RUNNING"
    PAUSED = "PAUSED"
    ERROR = "ERROR"
    STOPPED = "STOPPED"


class StateMachine:
    def __init__(self):
        self.state = SlamState.WAITING
        self._resume_state = SlamState.WAITING

    def start(self):
        if self.state != SlamState.WAITING:
            raise RuntimeError(f"Cannot start from {self.state.value}")
        self.state = SlamState.RUNNING

    def pause(self):
        if self.state not in (SlamState.WAITING, SlamState.RUNNING):
            raise RuntimeError(f"Cannot pause from {self.state.value}")
        self._resume_state = self.state
        self.state = SlamState.PAUSED

    def resume(self):
        if self.state != SlamState.PAUSED:
            raise RuntimeError(f"Cannot resume from {self.state.value}")
        self.state = self._resume_state

    def fail(self):
        self.state = SlamState.ERROR

    def stop(self):
        self.state = SlamState.STOPPED

    def reset(self):
        self.state = SlamState.WAITING
        self._resume_state = SlamState.WAITING
