import csv
import os
from .exporter import DataVisitor
from datetime import datetime

class CSVExporter(DataVisitor):
    def __init__(self, filename="results"):
        self.output_dir = "exported"
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.filename = f"{filename}_{timestamp}.csv"
        self.filepath = os.path.join(self.output_dir, self.filename)

    def visit(self, storage_class):
        # Ici storage_class est la CLASSE DataStorage
        if not storage_class.history:
            print("Rien à exporter.")
            return
        os.makedirs(self.output_dir, exist_ok=True)

        headers = storage_class.history[0].keys()
        with open(self.filepath, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=headers)
            writer.writeheader()
            writer.writerows(storage_class.history)
        print(f"Export CSV terminé : {self.filepath}")