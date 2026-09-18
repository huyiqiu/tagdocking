"""Abstract base class for robot chassis adapters — stop-and-go paradigm.

In the stop-and-go paradigm the adapters no longer compute velocities from
errors.  They simply publish the constant-rate commands issued by the
ActionExecutor: jog (straight-line forward), turn (rotate in place),
lateral movement, and stop.
"""

from abc import ABC, abstractmethod


class BaseAdapter(ABC):
    """Minimal interface for publishing discrete motion commands.

    Subclasses:
        OmniAdapter — Twist(linear.x, linear.y, angular.z)
    """

    @abstractmethod
    def publish_jog(self, linear_rate: float):
        """Publish a straight-line forward/reverse jog at the given rate (m/s)."""

    @abstractmethod
    def publish_turn(self, angular_rate: float):
        """Publish an in-place rotation at the given rate (rad/s)."""

    @abstractmethod
    def publish_lateral(self, lateral_rate: float):
        """Publish a pure-lateral velocity (positive = left, REP-103)."""

    @abstractmethod
    def publish_stop(self):
        """Publish zero velocity on all axes."""
