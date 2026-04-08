from abc import ABC, abstractmethod

class VRPObserver(ABC):
    """Interface pour les objets qui écoutent l'algorithme."""
    @abstractmethod
    def update(self, data: dict):
        pass

class DataVisitor(ABC):
    """Interface pour les objets qui traitent les données stockées."""
    @abstractmethod
    def visit(self, storage):
        pass