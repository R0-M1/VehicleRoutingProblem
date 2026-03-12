from dataclasses import dataclass
import math

@dataclass(frozen=True)
class Client:
    id: int
    name: str
    x: float
    y: float
    demand: int
    ready_time: float
    due_time: float
    service_time: float

    def distance_to(self, other: "Client") -> float:
        return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)

    @property
    def is_depot(self) -> bool:
        return self.id == 0
