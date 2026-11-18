from __future__ import annotations
from dataclasses import dataclass

IDLE = 0
GOING_TO_PICKUP = 1
WAITING_PICKUP = 2
GOING_TO_DROPOFF = 3
WAITING_DROPOFF = 4
COMPLETED = 5
CANCELED = 6
ERROR = 7

NAMES = {
    IDLE: 'IDLE', GOING_TO_PICKUP: 'GOING_TO_PICKUP',
    WAITING_PICKUP: 'WAITING_PICKUP', GOING_TO_DROPOFF: 'GOING_TO_DROPOFF',
    WAITING_DROPOFF: 'WAITING_DROPOFF', COMPLETED: 'COMPLETED',
    CANCELED: 'CANCELED', ERROR: 'ERROR',
}


@dataclass
class StateMachine:
    state: int = IDLE
    message: str = 'Idle'

    @property
    def active(self) -> bool:
        return self.state in (GOING_TO_PICKUP, WAITING_PICKUP, GOING_TO_DROPOFF, WAITING_DROPOFF)

    def start(self):
        if self.active:
            raise ValueError('Mission already active')
        self.state = GOING_TO_PICKUP; self.message = 'Navigating to pickup'

    def pickup_arrived(self, require_confirmation: bool):
        if self.state != GOING_TO_PICKUP:
            raise ValueError('Not going to pickup')
        if require_confirmation:
            self.state = WAITING_PICKUP; self.message = 'Waiting pickup confirmation'
        else:
            self.state = GOING_TO_DROPOFF; self.message = 'Navigating to dropoff'

    def confirm_pickup(self):
        if self.state != WAITING_PICKUP:
            raise ValueError('Not waiting pickup')
        self.state = GOING_TO_DROPOFF; self.message = 'Navigating to dropoff'

    def dropoff_arrived(self, require_confirmation: bool):
        if self.state != GOING_TO_DROPOFF:
            raise ValueError('Not going to dropoff')
        if require_confirmation:
            self.state = WAITING_DROPOFF; self.message = 'Waiting dropoff confirmation'
        else:
            self.state = COMPLETED; self.message = 'Mission completed'

    def confirm_dropoff(self):
        if self.state != WAITING_DROPOFF:
            raise ValueError('Not waiting dropoff')
        self.state = COMPLETED; self.message = 'Mission completed'

    def cancel(self):
        self.state = CANCELED; self.message = 'Mission canceled'

    def fail(self, message: str):
        self.state = ERROR; self.message = message

    def reset(self):
        if self.active:
            raise ValueError('Cannot reset active mission')
        self.state = IDLE; self.message = 'Idle'
