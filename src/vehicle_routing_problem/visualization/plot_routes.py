import plotly.graph_objects as go
import plotly.express as px
from typing import List
import numpy as np

from vehicle_routing_problem.core.instance import Instance
from vehicle_routing_problem.core.solution import Solution


def plot_vrptw_solution(solution: 'Solution', instance: 'Instance',
                        title: str = "Solution VRPTW", save_path: str = None) -> go.Figure:
    """
    Visualisation moderne des tournées VRPTW.

    Args:
        solution: Ta classe Solution
        instance: Ta classe Instance
        title: Titre personnalisé
        save_path: Sauvegarde PNG (optionnel)

    Retourne: Figure Plotly interactive
    """
    fig = go.Figure()

    # Palette moderne (10 couleurs distinctes)
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd',
              '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf']

    # 1. CLIENTS (taille & couleur par demande)
    client_x = [c.x for c in instance.clients[1:]]  # sans dépôt
    client_y = [c.y for c in instance.clients[1:]]
    demands = [c.demand for c in instance.clients[1:]]

    fig.add_trace(go.Scatter(
        x=client_x, y=client_y,
        mode='markers+text',
        marker=dict(
            size=[12 + d * 1.5 for d in demands],  # Taille ∝ demande
            color=demands,
            colorscale='Viridis',
            colorbar=dict(title="Demande", x=1.02),
            line=dict(width=2, color='white')
        ),
        text=[c.name for c in instance.clients[1:]],
        textposition="top center",
        textfont=dict(size=10, color="black", family="Arial Black"),
        name="Clients",
        hovertemplate="<b>%{text}</b><br>Demande: %{marker.color}<br>(%{x:.0f}, %{y:.0f})<extra></extra>"
    ))

    # 2. DÉPÔT (étoile orange centrale)
    depot = instance.clients[0]
    fig.add_trace(go.Scatter(
        x=[depot.x], y=[depot.y],
        mode='markers+text',
        marker=dict(size=30, color='orange', symbol='star', line=dict(width=4, color='darkorange')),
        text=['🏢 Dépôt'],
        textposition="bottom center",
        textfont=dict(size=16, color="darkorange", family="Arial Black"),
        name="Dépôt",
        hovertemplate="<b>🏢 Dépôt central</b><extra></extra>"
    ))

    # 3. ROUTES (lignes épaisses par véhicule)
    for idx, (route, color) in enumerate(zip(solution.routes, colors)):
        # Chemin complet : Dépôt → clients → Dépôt
        full_x = [instance.clients[0].x] + [instance.clients[cid].x for cid in route.client_ids] + [
            instance.clients[0].x]
        full_y = [instance.clients[0].y] + [instance.clients[cid].y for cid in route.client_ids] + [
            instance.clients[0].y]

        fig.add_trace(go.Scatter(
            x=full_x, y=full_y,
            mode='lines+markers',
            line=dict(width=2, color=color),
            marker=dict(size=14, color=color),
            name=f'🚛 Véhicule {idx + 1} (d={route.distance:.0f})',
            hovertemplate=f"<b>Véhicule {idx + 1}</b><br>" +
                          f"Distance: {route.distance:.1f}<br>" +
                          " → ".join([instance.clients[cid].name for cid in route.client_ids]) + "<extra></extra>",
            showlegend=True
        ))

    # 4. LAYOUT MODERNE
    fig.update_layout(
        title=dict(
            text=f"{title}<br><sub>{solution.nb_vehicles} véhicules | {solution.total_distance:.1f} unités | {len(instance.clients) - 1} clients</sub>",
            x=0.5, xanchor='center', font=dict(size=24, family="Arial Black")
        ),
        width=1000, height=800,
        plot_bgcolor='white', paper_bgcolor='white',
        font=dict(family="Arial", size=12),
        legend=dict(
            orientation="v", yanchor="top", y=1, xanchor="left", x=1.02,
            bgcolor="rgba(255,255,255,0.95)", bordercolor="gray", borderwidth=1
        ),
        margin=dict(l=40, r=250, t=100, b=40)
    )

    # Axes carrés + grille subtile
    fig.update_xaxes(title="X (coordonnées)", gridcolor='lightgray', gridwidth=1)
    fig.update_yaxes(title="Y (coordonnées)", gridcolor='lightgray', gridwidth=1,
                     scaleanchor="x", scaleratio=1)

    if save_path:
        fig.write_image(save_path)

    return fig
