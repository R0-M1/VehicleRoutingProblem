# export/datastorage.py
from .exporter import VRPObserver

class DataStorage(VRPObserver):
    history = []

    @staticmethod
    def update(data: dict):
        DataStorage.history.append(data)

    @staticmethod
    def accept(visitor):
        """Donne la classe elle-même au visiteur."""
        return visitor.visit(DataStorage)
    
    @staticmethod
    def clear():
        """Optionnel : pour vider l'historique entre deux tests."""
        DataStorage.history = []