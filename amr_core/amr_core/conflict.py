# Copyright 2026 Delhivery RSE Assignment
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Space-time conflict detection and the yielding protocol (Section 3.2)."""

import math

PROCEED = 0
SLOW = 1
YIELD = 2


class YieldState:
    """Traffic-control state machine (Section 3)."""

    NORMAL = "NORMAL"
    CONFLICT_DETECTED = "CONFLICT_DETECTED"
    YIELD_REQUESTED = "YIELD_REQUESTED"
    DECELERATING = "DECELERATING"
    WAITING = "WAITING"
    PASSAGE_CLEAR = "PASSAGE_CLEAR"
    RESUME = "RESUME"


class TrajectorySample:
    __slots__ = ("x", "y", "t", "speed")

    def __init__(self, x, y, t, speed=0.0):
        self.x = float(x)
        self.y = float(y)
        self.t = float(t)
        self.speed = float(speed)


class Trajectory:
    def __init__(self, robot_id, samples, footprint_radius, yield_priority):
        self.robot_id = robot_id
        self.samples = list(samples)
        self.footprint_radius = float(footprint_radius)
        self.yield_priority = int(yield_priority)


class Conflict:
    def __init__(self, time_to_conflict, separation, threshold):
        self.time_to_conflict = time_to_conflict
        self.separation = separation
        self.threshold = threshold

    def __repr__(self):
        return (f"Conflict(t={self.time_to_conflict:.2f}s, "
                f"sep={self.separation:.2f}m, thr={self.threshold:.2f}m)")


def find_conflict(a, b, margin=0.35, time_tolerance=0.5):
    """Earliest space-time conflict between two trajectories, or None."""
    threshold = a.footprint_radius + b.footprint_radius + float(margin)
    earliest = None
    for sa in a.samples:
        for sb in b.samples:
            if abs(sa.t - sb.t) > time_tolerance:
                continue
            separation = math.hypot(sa.x - sb.x, sa.y - sb.y)
            if separation < threshold:
                when = min(sa.t, sb.t)
                if earliest is None or when < earliest.time_to_conflict:
                    earliest = Conflict(when, separation, threshold)
    return earliest


class TrafficPolicy:
    """Turns a detected conflict into a directive for each robot."""

    def __init__(self, margin=0.35, clear_margin=1.0, slow_speed_scale=0.35,
                 react_seconds=3.0, hard_yield_seconds=1.5, time_tolerance=0.5):
        self.margin = float(margin)
        self.clear_margin = float(clear_margin)
        self.slow_speed_scale = float(slow_speed_scale)
        self.react_seconds = float(react_seconds)
        self.hard_yield_seconds = float(hard_yield_seconds)
        self.time_tolerance = float(time_tolerance)
        self._active = set()
        self.states = {}
        self._clear_counts = {}
        self.resume_clear_cycles = 3
        self.max_wait_cycles = 200
        self._wait_counts = {}

    def _advance(self, robot_id, conflicting, speed, keeper=None, conflict=None):
        """Advance one robot's yield state machine by a single evaluation."""
        state = self.states.get(robot_id, YieldState.NORMAL)
        self._wait_counts.setdefault(robot_id, 0)
        self._clear_counts.setdefault(robot_id, 0)

        if conflicting:
            self._clear_counts[robot_id] = 0
            if state in (YieldState.NORMAL, YieldState.PASSAGE_CLEAR,
                         YieldState.RESUME):
                state = YieldState.CONFLICT_DETECTED
            elif state == YieldState.CONFLICT_DETECTED:
                state = YieldState.YIELD_REQUESTED
            elif state == YieldState.YIELD_REQUESTED:
                state = YieldState.DECELERATING
            elif state == YieldState.DECELERATING:
                state = YieldState.WAITING if abs(speed) < 0.05 else YieldState.DECELERATING
            elif state == YieldState.WAITING:
                self._wait_counts[robot_id] += 1
                if self._wait_counts[robot_id] > self.max_wait_cycles:
                    self._wait_counts[robot_id] = 0
                    state = YieldState.RESUME
        else:
            self._wait_counts[robot_id] = 0
            if state in (YieldState.NORMAL, YieldState.RESUME):
                state = YieldState.NORMAL
                self._clear_counts[robot_id] = 0
            else:
                self._clear_counts[robot_id] += 1
                if self._clear_counts[robot_id] >= self.resume_clear_cycles:
                    state = YieldState.PASSAGE_CLEAR
                    self._clear_counts[robot_id] = 0

        if state == YieldState.PASSAGE_CLEAR:
            state = YieldState.RESUME
        elif state == YieldState.RESUME:
            state = YieldState.NORMAL

        self.states[robot_id] = state
        return state

    @staticmethod
    def _action_for(state):
        """Map a yield state onto the directive the smoother receives."""
        if state in (YieldState.DECELERATING, YieldState.WAITING):
            return YIELD, 0.0
        if state in (YieldState.CONFLICT_DETECTED, YieldState.YIELD_REQUESTED):
            return SLOW, 0.35
        return PROCEED, 1.0

    def evaluate(self, trajectories, speeds=None):
        """Return {robot_id: (action, speed_scale, reason)}."""
        speeds = speeds or {}
        conflicts = {}

        for i, a in enumerate(trajectories):
            for b in trajectories[i + 1:]:
                pair = tuple(sorted((a.robot_id, b.robot_id)))
                conflict = find_conflict(a, b, self.margin, self.time_tolerance)

                if conflict is None and pair in self._active:
                    conflict = find_conflict(
                        a, b, self.margin + self.clear_margin, self.time_tolerance)
                    if conflict is None:
                        self._active.discard(pair)

                if conflict is None or conflict.time_to_conflict > self.react_seconds:
                    continue
                self._active.add(pair)

                giver, keeper = (a, b) if a.yield_priority < b.yield_priority else (b, a)
                if a.yield_priority == b.yield_priority:
                    giver, keeper = sorted((a, b), key=lambda t: t.robot_id)
                conflicts[giver.robot_id] = (keeper.robot_id, conflict)

        directives = {}
        for t in trajectories:
            entry = conflicts.get(t.robot_id)
            keeper_id, conflict = entry if entry else (None, None)
            state = self._advance(
                t.robot_id, entry is not None, speeds.get(t.robot_id, 1.0),
                keeper_id, conflict)
            action, scale = self._action_for(state)
            if entry:
                reason = (f"{state}: {keeper_id} has priority "
                          f"(t={conflict.time_to_conflict:.1f}s, "
                          f"sep={conflict.separation:.2f}m)")
            else:
                reason = "" if state == YieldState.NORMAL else state
            directives[t.robot_id] = (action, scale, reason)
        return directives
