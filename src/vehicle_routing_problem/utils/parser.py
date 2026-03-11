import re
from pathlib import Path

import numpy as np

from vehicle_routing_problem.core.client import Client
from vehicle_routing_problem.core.instance import Instance


class VRPParser:
    """Parse un fichier .vrp au format VRPTW et construit une Instance."""

    # Patterns compilés une seule fois
    _HEADER_PATTERN  = re.compile(r"^(\w+)\s*:\s*(.+)$", re.MULTILINE)
    _DEPOT_PATTERN   = re.compile(r"DATA_DEPOTS\s*\[.*?\]:\s*\n((?:.+\n?)*?)(?=\n[A-Z]|\Z)")
    _CLIENTS_PATTERN = re.compile(r"DATA_CLIENTS\s*\[.*?\]:\s*\n((?:.+\n?)*)")

    @classmethod
    def parse(cls, filepath: Path) -> Instance:
        with open(filepath) as f:
            content = f.read()

        headers     = cls._parse_headers(content)
        capacity    = cls._get_capacity(headers)
        clients     = cls._parse_depot(content) + cls._parse_clients(content)

        return Instance(clients, capacity)

    # ------------------------------------------------------------------
    # Helpers privés
    # ------------------------------------------------------------------

    @classmethod
    def _parse_headers(cls, content: str) -> dict:
        """Extrait tous les headers clé/valeur (NAME, TYPE, NB_CLIENTS, ...)."""
        return {k.upper(): v.strip() for k, v in cls._HEADER_PATTERN.findall(content)}

    @classmethod
    def _get_capacity(cls, headers: dict) -> int:
        if "MAX_QUANTITY" not in headers:
            raise ValueError("MAX_QUANTITY introuvable dans le fichier.")
        return int(headers["MAX_QUANTITY"])

    @classmethod
    def _parse_depot(cls, content: str) -> list[Client]:
        """Parse DATA_DEPOTS — format: idName x y readyTime dueTime."""
        block = cls._DEPOT_PATTERN.search(content)
        if not block:
            raise ValueError("Bloc DATA_DEPOTS introuvable.")
        depots = []
        for line in block.group(1).strip().splitlines():
            parts = line.split()
            if len(parts) == 5:
                depots.append(Client(
                    id=0,
                    x=float(parts[1]), y=float(parts[2]),
                    demand=0,
                    ready_time=float(parts[3]),
                    due_time=float(parts[4]),
                    service_time=0.0,
                ))
        if not depots:
            raise ValueError("Aucune ligne valide dans DATA_DEPOTS.")
        return depots

    @classmethod
    def _parse_clients(cls, content: str) -> list[Client]:
        """Parse DATA_CLIENTS — format: idName x y readyTime dueTime demand service."""
        block = cls._CLIENTS_PATTERN.search(content)
        if not block:
            raise ValueError("Bloc DATA_CLIENTS introuvable.")
        clients = []
        for client_id, line in enumerate(block.group(1).strip().splitlines(), start=1):
            parts = line.split()
            if len(parts) == 7:
                clients.append(Client(
                    id=client_id,
                    x=float(parts[1]), y=float(parts[2]),
                    demand=int(parts[5]),
                    ready_time=float(parts[3]),
                    due_time=float(parts[4]),
                    service_time=float(parts[6]),
                ))
        if not clients:
            raise ValueError("Aucun client valide dans DATA_CLIENTS.")
        return clients