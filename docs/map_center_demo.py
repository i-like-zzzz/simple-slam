#!/usr/bin/env python3
# Copyright 2026 zwc
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from __future__ import annotations

import argparse
import math
import os
from dataclasses import dataclass


@dataclass(frozen=True)
class Point2D:
    x: float
    y: float


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


class SubmapGridDemo:
    def __init__(
        self,
        resolution: float,
        width: int,
        height: int,
        map_center: Point2D,
        origin: Pose2D,
    ) -> None:
        self.resolution = resolution
        self.width = width
        self.height = height
        self.map_center = map_center
        self.origin = origin

    @property
    def size_x_m(self) -> float:
        return self.width * self.resolution

    @property
    def size_y_m(self) -> float:
        return self.height * self.resolution

    @property
    def min_x(self) -> float:
        return self.map_center.x - 0.5 * self.size_x_m

    @property
    def min_y(self) -> float:
        return self.map_center.y - 0.5 * self.size_y_m

    @property
    def max_x(self) -> float:
        return self.min_x + self.size_x_m

    @property
    def max_y(self) -> float:
        return self.min_y + self.size_y_m

    def world_to_grid(self, point: Point2D) -> tuple[int, int] | None:
        cell_x = math.floor((point.x - self.min_x) / self.resolution)
        cell_y = math.floor((point.y - self.min_y) / self.resolution)
        if not (0 <= cell_x < self.width and 0 <= cell_y < self.height):
            return None
        return cell_x, cell_y

    def grid_to_world_center(self, index: tuple[int, int]) -> Point2D:
        cell_x, cell_y = index
        return Point2D(
            self.min_x + (cell_x + 0.5) * self.resolution,
            self.min_y + (cell_y + 0.5) * self.resolution,
        )


def parse_point(text: str) -> Point2D:
    x_text, y_text = text.split(",")
    return Point2D(float(x_text), float(y_text))


def parse_pose(text: str) -> Pose2D:
    x_text, y_text, yaw_text = text.split(",")
    return Pose2D(float(x_text), float(y_text), float(yaw_text))


def build_default_samples(grid: SubmapGridDemo) -> list[Point2D]:
    return [
        Point2D(grid.map_center.x, grid.map_center.y),
        Point2D(grid.map_center.x + 2.2, grid.map_center.y + 1.3),
        Point2D(grid.map_center.x - 4.6, grid.map_center.y + 3.7),
        Point2D(grid.max_x - 0.2, grid.max_y - 0.2),
        Point2D(grid.max_x + 0.3, grid.map_center.y),
    ]


def print_explanation(grid: SubmapGridDemo, samples: list[Point2D]) -> None:
    print("Submap grid demo")
    print("================")
    print(f"resolution       : {grid.resolution:.3f} m/cell")
    print(f"grid size        : {grid.width} x {grid.height} cells")
    print(f"map size         : {grid.size_x_m:.3f} m x {grid.size_y_m:.3f} m")
    print(
        "map_center       : "
        f"({grid.map_center.x:.3f}, {grid.map_center.y:.3f})  <- center of the whole grid"
    )
    print(
        "origin pose      : "
        f"({grid.origin.x:.3f}, {grid.origin.y:.3f}, yaw={grid.origin.yaw:.3f})"
    )
    print(
        "map bounds       : "
        f"x[{grid.min_x:.3f}, {grid.max_x:.3f}] "
        f"y[{grid.min_y:.3f}, {grid.max_y:.3f}]"
    )
    print()
    print("Sample world points")
    print("-------------------")
    for point in samples:
        index = grid.world_to_grid(point)
        if index is None:
            print(
                f"world ({point.x:.3f}, {point.y:.3f}) -> outside grid "
                f"because it is not inside x[{grid.min_x:.3f}, {grid.max_x:.3f}) "
                f"and y[{grid.min_y:.3f}, {grid.max_y:.3f})"
            )
            continue
        cell_center = grid.grid_to_world_center(index)
        print(
            f"world ({point.x:.3f}, {point.y:.3f}) -> grid {index} "
            f"-> cell center ({cell_center.x:.3f}, {cell_center.y:.3f})"
        )


def render_plot(
    grid: SubmapGridDemo,
    samples: list[Point2D],
    output_path: str | None,
    show_window: bool,
) -> None:
    headless = not os.environ.get("DISPLAY")
    if output_path or headless:
        import matplotlib

        matplotlib.use("Agg")

    import matplotlib.pyplot as plt
    from matplotlib.patches import FancyArrowPatch, Rectangle

    figure, axis = plt.subplots(figsize=(8, 8))

    grid_patch = Rectangle(
        (grid.min_x, grid.min_y),
        grid.size_x_m,
        grid.size_y_m,
        fill=False,
        linewidth=2.0,
        edgecolor="tab:blue",
        label="submap bounds",
    )
    axis.add_patch(grid_patch)

    axis.scatter(
        [grid.map_center.x],
        [grid.map_center.y],
        color="tab:red",
        s=100,
        marker="x",
        label="map_center",
        zorder=4,
    )

    axis.scatter(
        [grid.origin.x],
        [grid.origin.y],
        color="tab:green",
        s=80,
        marker="o",
        label="origin pose",
        zorder=4,
    )

    arrow_length = max(grid.size_x_m, grid.size_y_m) * 0.1
    yaw_end = Point2D(
        grid.origin.x + arrow_length * math.cos(grid.origin.yaw),
        grid.origin.y + arrow_length * math.sin(grid.origin.yaw),
    )
    axis.add_patch(
        FancyArrowPatch(
            (grid.origin.x, grid.origin.y),
            (yaw_end.x, yaw_end.y),
            arrowstyle="->",
            mutation_scale=15,
            linewidth=2.0,
            color="tab:green",
        )
    )

    inside_points: list[tuple[int, Point2D, tuple[int, int]]] = []
    for index, point in enumerate(samples, start=1):
        grid_index = grid.world_to_grid(point)
        color = "tab:purple" if grid_index is not None else "tab:gray"
        axis.scatter([point.x], [point.y], color=color, s=50, zorder=5)
        if grid_index is not None:
            inside_points.append((index, point, grid_index))
            cell_center = grid.grid_to_world_center(grid_index)
            cell_patch = Rectangle(
                (
                    cell_center.x - 0.5 * grid.resolution,
                    cell_center.y - 0.5 * grid.resolution,
                ),
                grid.resolution,
                grid.resolution,
                fill=True,
                alpha=0.15,
                linewidth=1.0,
                edgecolor=color,
                facecolor=color,
            )
            axis.add_patch(cell_patch)
            label = f"P{index} -> {grid_index}"
        else:
            label = f"P{index} -> outside"
        axis.annotate(label, (point.x, point.y), textcoords="offset points", xytext=(6, 6))

    axis.annotate(
        "map_center = center of the grid rectangle",
        (grid.map_center.x, grid.map_center.y),
        textcoords="offset points",
        xytext=(10, -22),
    )
    axis.annotate(
        "origin = pose stored when this submap was created",
        (grid.origin.x, grid.origin.y),
        textcoords="offset points",
        xytext=(10, 12),
    )

    axis.scatter([grid.min_x], [grid.min_y], color="tab:blue", s=35, zorder=5)
    axis.annotate(
        f"lower-left corner\n({grid.min_x:.1f}, {grid.min_y:.1f})",
        (grid.min_x, grid.min_y),
        textcoords="offset points",
        xytext=(8, 8),
    )

    formula_text = "\n".join(
        [
            "Grid placement formulas",
            (
                f"size_x = width * resolution = {grid.width} * "
                f"{grid.resolution:.2f} = {grid.size_x_m:.1f} m"
            ),
            (
                f"size_y = height * resolution = {grid.height} * "
                f"{grid.resolution:.2f} = {grid.size_y_m:.1f} m"
            ),
            (
                f"min_x = map_center.x - size_x / 2 = {grid.map_center.x:.1f} - "
                f"{grid.size_x_m:.1f}/2 = {grid.min_x:.1f}"
            ),
            (
                f"min_y = map_center.y - size_y / 2 = {grid.map_center.y:.1f} - "
                f"{grid.size_y_m:.1f}/2 = {grid.min_y:.1f}"
            ),
            "Current implementation uses only map_center.x/y for indexing.",
            "origin.yaw is stored metadata here and does not rotate the grid.",
        ]
    )
    axis.text(
        0.02,
        0.98,
        formula_text,
        transform=axis.transAxes,
        va="top",
        ha="left",
        fontsize=9,
        bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.9},
    )

    if inside_points:
        point_index, point, grid_index = (
            inside_points[1] if len(inside_points) > 1 else inside_points[0]
        )
        sample_formula_text = "\n".join(
            [
                f"Indexing example with P{point_index}",
                "cell_x = floor((x - min_x) / resolution)",
                (
                    f"       = floor(({point.x:.2f} - {grid.min_x:.2f}) / "
                    f"{grid.resolution:.2f}) = {grid_index[0]}"
                ),
                "cell_y = floor((y - min_y) / resolution)",
                (
                    f"       = floor(({point.y:.2f} - {grid.min_y:.2f}) / "
                    f"{grid.resolution:.2f}) = {grid_index[1]}"
                ),
            ]
        )
        axis.text(
            0.52,
            0.08,
            sample_formula_text,
            transform=axis.transAxes,
            va="bottom",
            ha="left",
            fontsize=9,
            bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.9},
        )

    padding = max(grid.size_x_m, grid.size_y_m) * 0.15
    axis.set_xlim(grid.min_x - padding, grid.max_x + padding)
    axis.set_ylim(grid.min_y - padding, grid.max_y + padding)
    axis.set_aspect("equal", adjustable="box")
    axis.grid(True, linestyle="--", alpha=0.3)
    axis.set_xlabel("world x (m)")
    axis.set_ylabel("world y (m)")
    axis.set_title("How map_center places the fixed submap grid in world coordinates")
    axis.legend(loc="upper right")

    if output_path:
        figure.savefig(output_path, dpi=180, bbox_inches="tight")
        print(f"\nSaved figure to: {output_path}")

    if show_window and not headless:
        plt.show()

    plt.close(figure)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Visualize how map_center_ defines the placement of a fixed submap grid."
    )
    parser.add_argument("--resolution", type=float, default=0.05)
    parser.add_argument("--width", type=int, default=400)
    parser.add_argument("--height", type=int, default=400)
    parser.add_argument("--map-center", type=parse_point, default=Point2D(10.0, 5.0))
    parser.add_argument("--origin", type=parse_pose, default=Pose2D(10.0, 5.0, 0.6))
    parser.add_argument(
        "--sample-point",
        type=parse_point,
        action="append",
        default=[],
        help="World point as x,y. Can be repeated.",
    )
    parser.add_argument("--output", type=str, default=None, help="Optional output image path.")
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Do not open an interactive window.",
    )
    args = parser.parse_args()

    grid = SubmapGridDemo(
        resolution=args.resolution,
        width=args.width,
        height=args.height,
        map_center=args.map_center,
        origin=args.origin,
    )
    samples = args.sample_point or build_default_samples(grid)

    print_explanation(grid, samples)
    render_plot(grid, samples, args.output, show_window=not args.no_show)


if __name__ == "__main__":
    main()
