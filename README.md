# VehicleRoutingProblem
Vehicle Routing Problem with Time Windows

idée de structure :
vrptw_solver/
│
├── pyproject.toml          # 1. Dépendances Poetry + metadata projet
├── README.md               # 2. Instructions + démo GIF
├── config.yaml             # 3. Paramètres algos/instances
│
├── src/
│   └── vrptw_solver/       # 4. Package principal
│       ├── __init__.py
│       ├── core/           # Modélisation de base
│       │   ├── __init__.py
│       │   ├── instance.py     # 📊 Instance + parser TON format
│       │   └── solution.py     # 🛣️ Routes/Solutions + validation TW
│       ├── operators/      # Opérateurs voisinage (1 fichier/op)
│       │   ├── __init__.py
│       │   ├── base_operator.py  # 🎯 Abstract base + registry
│       │   ├── two_opt.py        # ✂️ 2-opt intra/inter
│       │   ├── relocate.py       # 📦 Relocate client
│       │   ├── exchange.py       # 🔄 Exchange clients
│       │   └── or_opt.py         # 🔀 Or-opt segments
│       ├── metaheuristics/     # Métaheuristiques Q4
│       │   ├── __init__.py
│       │   ├── tabu_search.py   # 🧠 Tabu Search principal
│       │   └── simulated_annealing.py # 🔥 SA avec refroidissement
│       ├── heuristics/      # Heuristiques Q2+Q3
│       │   ├── __init__.py
│       │   ├── bin_packing.py   # 📦 Min véhicules
│       │   └── random_gen.py    # 🎲 Générateur aléatoire
│       └── utils/          # Outils transversaux
│           ├── __init__.py
│           ├── parser.py       # 🔍 Parse DATA_DEPOTS/CLIENTS
│           ├── visualizer.py   # 📈 Plotly/Matplotlib tournées
│           └── logger.py       # 📝 Logging structuré
│
├── tests/                  # Tests unitaires 95% coverage
│   ├── __init__.py
│   ├── test_instance.py
│   ├── test_operators.py
│   ├── test_metaheuristics.py
│   └── fixtures/           # 📄 Tes instances test
│       └── sample_data.txt
│
├── examples/               # 🚀 Scripts exécutables
│   ├── solve_instance.py      # python solve_instance.py data.txt
│   ├── benchmark.py           # Tests multiples instances
│   └── visualize.py           # Plots interactifs
│
├── docs/                   # 📖 Rapport
│   └── analysis.ipynb     # → Export PDF automatique
│
└── experiments/            # 🧪 Résultats Q5
    └── results/            # CSV auto-générés
