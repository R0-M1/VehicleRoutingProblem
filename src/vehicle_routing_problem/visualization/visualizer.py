import math

import matplotlib.pyplot as plt
from typing import Optional
import numpy as np

from vehicle_routing_problem.core.client import Client

from ..core.solution import Solution
from ..core.instance import Instance
from ..core.route import Route


class Visualizer:
    @staticmethod
    def visualize_solution(
        solution: Solution,
        instance: Instance,
        title: str = "Vehicle Routing Problem - Solution",
        figsize: tuple = (12, 8),
        save_path: Optional[str] = None,
        show: bool = True
    ) -> None:
        fig, ax = plt.subplots(figsize=figsize)

        colors = plt.cm.tab20(np.linspace(0, 1, max(20, len(solution.routes))))

        # Dépôt
        depot = instance.depot
        ax.plot(
            depot.x, depot.y,
            marker='s',
            markersize=15,
            color='red',
            label='Depot',
            zorder=5,
            markeredgecolor='darkred',
            markeredgewidth=2
        )
        ax.text(
            depot.x,
            depot.y + 1.5,
            'Depot\n(0)',
            ha='center',
            fontsize=9,
            fontweight='bold'
        )

        # Routes
        for route_idx, route in enumerate(solution.routes):
            color = colors[route_idx % len(colors)]

            path_ids = [0] + route.client_ids + [0]
            path_coords = [instance.clients[cid] for cid in path_ids]

            xs = [c.x for c in path_coords]
            ys = [c.y for c in path_coords]

            ax.plot(
                xs, ys,
                color=color,
                linewidth=2,
                alpha=0.7,
                label=f'Route {route_idx + 1} (dist: {route.distance:.1f}, load: {route.total_demand})'
            )

            for client_id in route.client_ids:
                client = instance.clients[client_id]
                ax.plot(
                    client.x, client.y,
                    marker='o',
                    markersize=8,
                    color=color,
                    alpha=0.9,
                    zorder=4,
                    markeredgecolor='black',
                    markeredgewidth=0.5
                )
                ax.text(
                    client.x,
                    client.y - 1.2,
                    str(client_id),
                    ha='center',
                    fontsize=8,
                    color=color,
                    fontweight='bold'
                )

        visited_ids = set(cid for route in solution.routes for cid in route.client_ids)
        all_client_ids = set(range(1, len(instance.clients)))
        unvisited_ids = all_client_ids - visited_ids

        if unvisited_ids:
            for client_id in unvisited_ids:
                client = instance.clients[client_id]
                ax.plot(
                    client.x, client.y,
                    marker='x',
                    markersize=10,
                    color='gray',
                    alpha=0.5,
                    zorder=3,
                    markeredgewidth=2
                )
                ax.text(
                    client.x,
                    client.y - 1.2,
                    f'{client_id}(x)',
                    ha='center',
                    fontsize=7,
                    color='gray'
                )

        ax.set_xlabel('X coordinate', fontsize=11, fontweight='bold')
        ax.set_ylabel('Y coordinate', fontsize=11, fontweight='bold')
        ax.set_title(title, fontsize=13, fontweight='bold', pad=20)
        ax.legend(loc='upper left', fontsize=9, framealpha=0.95)
        ax.grid(True, alpha=0.3, linestyle='--')
        ax.set_aspect('equal')

        stats_text = (
            f"Vehicles used: {solution.nb_vehicles}\n"
            f"Total distance: {solution.total_distance:.2f}\n"
            f"Clients visited: {len(visited_ids)}/{len(all_client_ids)}"
        )
        ax.text(
            0.02, 0.98,
            stats_text,
            transform=ax.transAxes,
            fontsize=10,
            verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8)
        )

        plt.tight_layout()

        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"Solution visualization saved to: {save_path}")

        if show:
            plt.show()

    @staticmethod
    def single_route(
        solution: Solution,
        instance: Instance,
        route_idx: int,
        title: Optional[str] = None,
        figsize: tuple = (10, 8),
        save_path: Optional[str] = None,
        show: bool = True
    ) -> None:
        """
        Affiche une seule route de la solution.
        """
        if route_idx < 0 or route_idx >= len(solution.routes):
            raise IndexError(f"route_idx invalide: {route_idx}")

        route = solution.routes[route_idx]
        fig, ax = plt.subplots(figsize=figsize)

        depot = instance.depot
        color = "blue"

        # Dépôt
        ax.plot(
            depot.x, depot.y,
            marker='s',
            markersize=15,
            color='red',
            label='Depot',
            zorder=5,
            markeredgecolor='darkred',
            markeredgewidth=2
        )
        ax.text(
            depot.x,
            depot.y + 1.5,
            'Depot\n(0)',
            ha='center',
            fontsize=9,
            fontweight='bold'
        )

        # Chemin complet
        path_ids = [0] + route.client_ids + [0]
        path_coords = [instance.clients[cid] for cid in path_ids]

        xs = [c.x for c in path_coords]
        ys = [c.y for c in path_coords]

        ax.plot(
            xs, ys,
            color=color,
            linewidth=2.5,
            alpha=0.8,
            label=f'Route {route_idx + 1}'
        )

        # Clients de la route
        for order, client_id in enumerate(route.client_ids, start=1):
            client = instance.clients[client_id]
            ax.plot(
                client.x, client.y,
                marker='o',
                markersize=9,
                color=color,
                alpha=0.95,
                zorder=4,
                markeredgecolor='black',
                markeredgewidth=0.6
            )
            ax.text(
                client.x,
                client.y - 2.2,
                client.name,
                ha='center',
                fontsize=8,
                color='black',
                fontweight='bold'
            )
            ax.text(
                client.x + 0.8,
                client.y + 0.8,
                f'{order}',
                fontsize=7,
                color='black'
            )

        if title is None:
            title = f"Route {route_idx + 1}"

        ax.set_xlabel('X coordinate', fontsize=11, fontweight='bold')
        ax.set_ylabel('Y coordinate', fontsize=11, fontweight='bold')
        ax.set_title(title, fontsize=13, fontweight='bold', pad=20)
        ax.grid(True, alpha=0.3, linestyle='--')
        ax.set_aspect('equal')
        ax.legend()

        stats_text = (
            f"Route index: {route_idx}\n"
            f"Clients: {len(route.client_ids)}\n"
            f"Load: {route.total_demand}\n"
            f"Distance: {route.distance:.2f}"
        )
        ax.text(
            0.02, 0.98,
            stats_text,
            transform=ax.transAxes,
            fontsize=10,
            verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8)
        )

        plt.tight_layout()

        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"Single route visualization saved to: {save_path}")

        if show:
            plt.show()

    @staticmethod
    def compare_solutions(
        solutions: list[Solution],
        instance: Instance,
        titles: Optional[list[str]] = None,
        figsize: tuple = (15, 5),
        save_path: Optional[str] = None,
        show: bool = True
    ) -> None:
        nb_solutions = len(solutions)

        if titles is None:
            titles = [f"Solution {i+1}" for i in range(nb_solutions)]

        cols = min(3, nb_solutions)
        rows = math.ceil(nb_solutions / cols)

        fig, axes = plt.subplots(rows, cols, figsize=(5*cols, 5*rows))

        if nb_solutions == 1:
            axes = [axes]

        all_colors = plt.cm.tab20(np.linspace(0, 1, 20))

        for solution, ax, title in zip(solutions, axes, titles):
            colors = all_colors[np.linspace(0, 19, max(20, len(solution.routes))).astype(int)]

            depot = instance.depot
            ax.plot(depot.x, depot.y, marker='s', markersize=12, color='red', zorder=5)

            for route_idx, route in enumerate(solution.routes):
                color = colors[route_idx % len(colors)]

                path_ids = [0] + route.client_ids + [0]
                path_coords = [instance.clients[cid] for cid in path_ids]

                xs = [c.x for c in path_coords]
                ys = [c.y for c in path_coords]

                ax.plot(xs, ys, color=color, linewidth=1.5, alpha=0.7)

                for client_id in route.client_ids:
                    client = instance.clients[client_id]
                    ax.plot(client.x, client.y, marker='o', markersize=6, color=color, alpha=0.9)

            ax.set_xlabel('X', fontsize=9)
            ax.set_ylabel('Y', fontsize=9)
            ax.set_title(f"{title}\nDist: {solution.total_distance:.1f}", fontsize=10, fontweight='bold')
            ax.grid(True, alpha=0.3)
            ax.set_aspect('equal')

        plt.tight_layout()

        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"Comparison saved to: {save_path}")

        if show:
            plt.show()

    @staticmethod
    def compare_routes(
        routes: list[Route],
        instance: Instance,
        titles: Optional[list[str]] = None,
        highlight_clients: Optional[list[int]] = None,
        show: bool = True
    ) -> None:

        if titles is None:
            titles = [f"Route {i}" for i in range(len(routes))]

        fig, axes = plt.subplots(1, len(routes), figsize=(7 * len(routes), 6))

        if len(routes) == 1:
            axes = [axes]

        for ax, route, title in zip(axes, routes, titles):

            path_ids = [0] + route.client_ids + [0]
            path_coords = [instance.clients[cid] for cid in path_ids]

            xs = [client.x for client in path_coords]
            ys = [client.y for client in path_coords]

            ax.plot(xs, ys, marker="o")

            depot = instance.depot
            ax.scatter(
                depot.x,
                depot.y,
                s=150,
                marker="s",
                label="Depot"
            )

            for i, client_id in enumerate(route.client_ids):
                client = instance.clients[client_id]

                ax.annotate(
                    f"{client.name} ({i+1})",
                    (client.x, client.y),
                    textcoords="offset points",
                    xytext=(5, 5)
                )

                if highlight_clients and client_id in highlight_clients:
                    ax.scatter(
                        client.x,
                        client.y,
                        s=250,
                        facecolors="none",
                        edgecolors="black",
                        linewidths=2,
                        zorder=6
                    )

            ax.set_title(title)
            ax.legend()

        if show:
            plt.show()