from __future__ import annotations
from abc import ABC, abstractmethod
from typing import Iterator

from vehicle_routing_problem.core.solution import Solution
from vehicle_routing_problem.core.instance import Instance


class BaseMetaheuristic(ABC):
    """Classe abstraite pour algorithme métaheuristiques."""

    def __init__(self, instance: Instance):
        self._inst = instance

    @abstractmethod
    def solve(self, current_solution: Solution) -> Iterator[Solution]:
        """Génère itérativement les nouvelles solutions."""
        ...
