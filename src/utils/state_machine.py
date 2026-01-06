"""
StateMachine - Mission Phase Tracking

Provides state management for mission execution with transition
validation and event callbacks.
"""

import time
import logging
from enum import Enum
from typing import Optional, Callable, Dict, List, Set
from dataclasses import dataclass, field

logger = logging.getLogger(__name__)


class MissionState(Enum):
    """
    Mission states for drone operation.
    
    States represent the current phase of mission execution.
    """
    IDLE = "idle"               # Grounded, waiting for mission
    PREFLIGHT = "preflight"     # Pre-flight checks in progress
    ARMED = "armed"             # Armed and ready for takeoff
    TAKEOFF = "takeoff"         # Taking off to initial altitude
    SURVEY = "survey"           # Executing survey pattern (Drone 1)
    DETECTED = "detected"       # Human detected, processing
    LOITER = "loiter"           # Holding position (Drone 2 waiting)
    TRANSIT = "transit"         # Moving to target location
    APPROACH = "approach"       # Approaching drop point
    DROP = "drop"               # Executing payload drop
    RTL = "rtl"                 # Returning to launch
    LANDING = "landing"         # Landing in progress
    LANDED = "landed"           # Successfully landed
    EMERGENCY = "emergency"     # Emergency state (failsafe triggered)
    ERROR = "error"             # Error state


@dataclass
class StateTransition:
    """Record of a state transition."""
    from_state: MissionState
    to_state: MissionState
    timestamp: float
    reason: str


class StateMachine:
    """
    Mission state machine with transition validation and callbacks.
    
    Tracks current mission state, validates transitions, and triggers
    callbacks on state changes for mission coordination.
    """
    
    # Valid state transitions (from_state -> set of valid to_states)
    VALID_TRANSITIONS: Dict[MissionState, Set[MissionState]] = {
        MissionState.IDLE: {
            MissionState.PREFLIGHT,
            MissionState.EMERGENCY,
            MissionState.ERROR
        },
        MissionState.PREFLIGHT: {
            MissionState.ARMED,
            MissionState.IDLE,
            MissionState.ERROR
        },
        MissionState.ARMED: {
            MissionState.TAKEOFF,
            MissionState.IDLE,
            MissionState.EMERGENCY
        },
        MissionState.TAKEOFF: {
            MissionState.SURVEY,
            MissionState.LOITER,
            MissionState.RTL,
            MissionState.EMERGENCY
        },
        MissionState.SURVEY: {
            MissionState.DETECTED,
            MissionState.RTL,
            MissionState.LOITER,
            MissionState.EMERGENCY
        },
        MissionState.DETECTED: {
            MissionState.SURVEY,
            MissionState.RTL,
            MissionState.EMERGENCY
        },
        MissionState.LOITER: {
            MissionState.TRANSIT,
            MissionState.RTL,
            MissionState.EMERGENCY
        },
        MissionState.TRANSIT: {
            MissionState.APPROACH,
            MissionState.LOITER,
            MissionState.RTL,
            MissionState.EMERGENCY
        },
        MissionState.APPROACH: {
            MissionState.DROP,
            MissionState.TRANSIT,
            MissionState.RTL,
            MissionState.EMERGENCY
        },
        MissionState.DROP: {
            MissionState.LOITER,
            MissionState.TRANSIT,
            MissionState.RTL,
            MissionState.EMERGENCY
        },
        MissionState.RTL: {
            MissionState.LANDING,
            MissionState.EMERGENCY
        },
        MissionState.LANDING: {
            MissionState.LANDED,
            MissionState.EMERGENCY
        },
        MissionState.LANDED: {
            MissionState.IDLE,
            MissionState.PREFLIGHT
        },
        MissionState.EMERGENCY: {
            MissionState.RTL,
            MissionState.LANDING,
            MissionState.LANDED,
            MissionState.IDLE
        },
        MissionState.ERROR: {
            MissionState.IDLE,
            MissionState.EMERGENCY
        }
    }
    
    def __init__(self, initial_state: MissionState = MissionState.IDLE):
        """
        Initialize StateMachine.
        
        Args:
            initial_state: Initial mission state
        """
        self.current_state = initial_state
        self.previous_state: Optional[MissionState] = None
        self.state_entered_time = time.time()
        
        # Transition history
        self.history: List[StateTransition] = []
        self.max_history = 100
        
        # Callbacks
        self.on_state_change: Optional[Callable[[MissionState, MissionState, str], None]] = None
        self.state_callbacks: Dict[MissionState, List[Callable[[], None]]] = {}
        
        logger.info(f"StateMachine initialized in {initial_state.value} state")
    
    def transition_to(self, new_state: MissionState, reason: str = "") -> bool:
        """
        Attempt to transition to a new state.
        
        Args:
            new_state: Target state
            reason: Reason for transition (for logging)
            
        Returns:
            True if transition successful, False if invalid
        """
        # Check if transition is valid
        valid_targets = self.VALID_TRANSITIONS.get(self.current_state, set())
        
        if new_state not in valid_targets:
            logger.warning(f"Invalid transition: {self.current_state.value} -> {new_state.value}")
            return False
        
        # Record transition
        transition = StateTransition(
            from_state=self.current_state,
            to_state=new_state,
            timestamp=time.time(),
            reason=reason
        )
        
        self.history.append(transition)
        if len(self.history) > self.max_history:
            self.history.pop(0)
        
        # Update state
        self.previous_state = self.current_state
        self.current_state = new_state
        self.state_entered_time = time.time()
        
        logger.info(f"State transition: {self.previous_state.value} -> {new_state.value}"
                   + (f" ({reason})" if reason else ""))
        
        # Trigger callbacks
        if self.on_state_change:
            try:
                self.on_state_change(self.previous_state, new_state, reason)
            except Exception as e:
                logger.error(f"State change callback error: {e}")
        
        if new_state in self.state_callbacks:
            for callback in self.state_callbacks[new_state]:
                try:
                    callback()
                except Exception as e:
                    logger.error(f"State callback error: {e}")
        
        return True
    
    def force_state(self, new_state: MissionState, reason: str = "FORCED"):
        """
        Force transition to a state regardless of validity.
        
        Use sparingly - only for emergency or recovery situations.
        
        Args:
            new_state: Target state
            reason: Reason for forced transition
        """
        logger.warning(f"FORCED state transition: {self.current_state.value} -> {new_state.value}")
        
        transition = StateTransition(
            from_state=self.current_state,
            to_state=new_state,
            timestamp=time.time(),
            reason=f"FORCED: {reason}"
        )
        
        self.history.append(transition)
        self.previous_state = self.current_state
        self.current_state = new_state
        self.state_entered_time = time.time()
        
        if self.on_state_change:
            try:
                self.on_state_change(self.previous_state, new_state, reason)
            except Exception as e:
                logger.error(f"State change callback error: {e}")
    
    def get_state(self) -> MissionState:
        """Get current state."""
        return self.current_state
    
    def get_state_name(self) -> str:
        """Get current state name as string."""
        return self.current_state.value
    
    def is_in(self, *states: MissionState) -> bool:
        """
        Check if current state is one of the given states.
        
        Args:
            *states: States to check against
            
        Returns:
            True if current state matches any of the given states
        """
        return self.current_state in states
    
    def is_flying(self) -> bool:
        """Check if drone is currently airborne."""
        flying_states = {
            MissionState.TAKEOFF,
            MissionState.SURVEY,
            MissionState.DETECTED,
            MissionState.LOITER,
            MissionState.TRANSIT,
            MissionState.APPROACH,
            MissionState.DROP,
            MissionState.RTL
        }
        return self.current_state in flying_states
    
    def is_grounded(self) -> bool:
        """Check if drone is on the ground."""
        ground_states = {
            MissionState.IDLE,
            MissionState.PREFLIGHT,
            MissionState.ARMED,
            MissionState.LANDED
        }
        return self.current_state in ground_states
    
    def time_in_state(self) -> float:
        """Get time spent in current state in seconds."""
        return time.time() - self.state_entered_time
    
    def can_transition_to(self, state: MissionState) -> bool:
        """
        Check if transition to given state is valid.
        
        Args:
            state: Target state to check
            
        Returns:
            True if transition would be valid
        """
        valid_targets = self.VALID_TRANSITIONS.get(self.current_state, set())
        return state in valid_targets
    
    def register_callback(self, state: MissionState, callback: Callable[[], None]):
        """
        Register a callback to be called when entering a specific state.
        
        Args:
            state: State to trigger callback on
            callback: Function to call
        """
        if state not in self.state_callbacks:
            self.state_callbacks[state] = []
        self.state_callbacks[state].append(callback)
    
    def get_history(self, last_n: int = None) -> List[StateTransition]:
        """
        Get transition history.
        
        Args:
            last_n: Optional limit on number of entries
            
        Returns:
            List of StateTransition objects
        """
        if last_n:
            return self.history[-last_n:]
        return self.history.copy()
    
    def reset(self):
        """Reset state machine to IDLE state."""
        self.current_state = MissionState.IDLE
        self.previous_state = None
        self.state_entered_time = time.time()
        self.history.clear()
        logger.info("StateMachine reset to IDLE")
