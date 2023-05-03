from typing import Protocol

class AutoRoutine(Protocol):
    def run(self):
        ...

    def reset(self):
        ...

