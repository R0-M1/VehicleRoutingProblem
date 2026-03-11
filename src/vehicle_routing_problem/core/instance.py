import numpy as np
from dataclasses import dataclass, field
from .client import Client

@dataclass
class Instance:
    clients: list[Client]   # clients[0] = dépôt
    capacity: int
    dist_matrix: np.ndarray = field(init=False)

    def __post_init__(self):
        coords = np.array([(c.x, c.y) for c in self.clients])
        diff = coords[:, None, :] - coords[None, :, :]
        self.dist_matrix = np.sqrt((diff ** 2).sum(axis=-1))

    @property
    def depot(self) -> Client:
        return self.clients[0]

    @property
    def nb_clients(self) -> int:
        return len(self.clients) - 1
