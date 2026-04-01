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


def rotate_point(point: Point2D, yaw: float) -> Point2D:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return Point2D(c * point.x - s * point.y, s * point.x + c * point.y)


def transform_point(point: Point2D, pose: Pose2D) -> Point2D:
    rotated = rotate_point(point, pose.yaw)
    return Point2D(rotated.x + pose.x, rotated.y + pose.y)


def inverse_transform_point(point: Point2D, pose: Pose2D) -> Point2D:
    dx = point.x - pose.x
    dy = point.y - pose.y
    c = math.cos(pose.yaw)
    s = math.sin(pose.yaw)
    return Point2D(c * dx + s * dy, -s * dx + c * dy)


class AxisAlignedSubmap:
    """Current simple-slam style submap: axis-aligned, no rotation."""

    def __init__(self, resolution: float, width: int, height: int, map_center: Point2D) -> None:
        self.resolution = resolution
        self.width = width
        self.height = height
        self.map_center = map_center

    @property
    def size_x_m(self) -> float:
        return self.width * self.resolution

    @property
    def size_y_m(self) -> float:
        return self.height * self.resolution

    @property
    def lower_left(self) -> Point2D:
        return Point2D(
            self.map_center.x - 0.5 * self.size_x_m,
            self.map_center.y - 0.5 * self.size_y_m,
        )

    @property
    def upper_right(self) -> Point2D:
        return Point2D(self.lower_left.x + self.size_x_m, self.lower_left.y + self.size_y_m)

    def world_bounds(self) -> tuple[float, float, float, float]:
        lower_left = self.lower_left
        upper_right = self.upper_right
        return lower_left.x, lower_left.y, upper_right.x, upper_right.y

    def corners_world(self) -> list[Point2D]:
        lower_left = self.lower_left
        return [
            lower_left,
            Point2D(lower_left.x + self.size_x_m, lower_left.y),
            Point2D(lower_left.x + self.size_x_m, lower_left.y + self.size_y_m),
            Point2D(lower_left.x, lower_left.y + self.size_y_m),
        ]

    def grid_to_world_center(self, cell_x: int, cell_y: int) -> Point2D:
        lower_left = self.lower_left
        return Point2D(
            lower_left.x + (cell_x + 0.5) * self.resolution,
            lower_left.y + (cell_y + 0.5) * self.resolution,
        )

    def world_to_grid(self, point: Point2D) -> tuple[int, int] | None:
        lower_left = self.lower_left
        cell_x = math.floor((point.x - lower_left.x) / self.resolution)
        cell_y = math.floor((point.y - lower_left.y) / self.resolution)
        if not (0 <= cell_x < self.width and 0 <= cell_y < self.height):
            return None
        return cell_x, cell_y


class RotatedSubmap:
    """Rotated submap: the grid has its own local frame and a world yaw."""

    def __init__(self, resolution: float, width: int, height: int, pose: Pose2D) -> None:
        self.resolution = resolution
        self.width = width
        self.height = height
        self.pose = pose

    @property
    def size_x_m(self) -> float:
        return self.width * self.resolution

    @property
    def size_y_m(self) -> float:
        return self.height * self.resolution

    def local_lower_left(self) -> Point2D:
        return Point2D(-0.5 * self.size_x_m, -0.5 * self.size_y_m)

    def local_corners(self) -> list[Point2D]:
        lower_left = self.local_lower_left()
        return [
            lower_left,
            Point2D(lower_left.x + self.size_x_m, lower_left.y),
            Point2D(lower_left.x + self.size_x_m, lower_left.y + self.size_y_m),
            Point2D(lower_left.x, lower_left.y + self.size_y_m),
        ]

    def corners_world(self) -> list[Point2D]:
        return [transform_point(corner, self.pose) for corner in self.local_corners()]

    def world_bounds(self) -> tuple[float, float, float, float]:
        corners = self.corners_world()
        xs = [corner.x for corner in corners]
        ys = [corner.y for corner in corners]
        return min(xs), min(ys), max(xs), max(ys)

    def grid_to_world_center(self, cell_x: int, cell_y: int) -> Point2D:
        local_lower_left = self.local_lower_left()
        local_center = Point2D(
            local_lower_left.x + (cell_x + 0.5) * self.resolution,
            local_lower_left.y + (cell_y + 0.5) * self.resolution,
        )
        return transform_point(local_center, self.pose)

    def world_to_grid(self, point: Point2D) -> tuple[int, int] | None:
        local_point = inverse_transform_point(point, self.pose)
        local_lower_left = self.local_lower_left()
        cell_x = math.floor((local_point.x - local_lower_left.x) / self.resolution)
        cell_y = math.floor((local_point.y - local_lower_left.y) / self.resolution)
        if not (0 <= cell_x < self.width and 0 <= cell_y < self.height):
            return None
        return cell_x, cell_y


def format_point(point: Point2D) -> str:
    return f"({point.x:.3f}, {point.y:.3f})"


def print_submap_info(name: str, submap: AxisAlignedSubmap | RotatedSubmap) -> None:
    min_x, min_y, max_x, max_y = submap.world_bounds()
    print(name)
    print("=" * len(name))
    print(f"resolution     : {submap.resolution:.3f} m/cell")
    print(f"grid size      : {submap.width} x {submap.height} cells")
    print(f"map size       : {submap.size_x_m:.3f} m x {submap.size_y_m:.3f} m")
    if isinstance(submap, AxisAlignedSubmap):
        print(f"map_center     : {format_point(submap.map_center)}")
        print("rotation       : none, grid axes are aligned with world x/y")
    else:
        print(
            "submap pose    : "
            f"({submap.pose.x:.3f}, {submap.pose.y:.3f}, "
            f"yaw={submap.pose.yaw:.3f})"
        )
        print("rotation       : grid axes are rotated relative to world x/y")
    print(
        f"world bounds   : x[{min_x:.3f}, {max_x:.3f}] "
        f"y[{min_y:.3f}, {max_y:.3f}]"
    )
    print("world corners  :")
    for index, corner in enumerate(submap.corners_world()):
        print(f"  corner {index}: {format_point(corner)}")
    print()


def compare_sample_points(
    axis_aligned: AxisAlignedSubmap,
    rotated: RotatedSubmap,
    sample_points: list[Point2D],
) -> None:
    print("Sample point lookup")
    print("===================")
    for point in sample_points:
        print(f"world point {format_point(point)}")
        axis_cell = axis_aligned.world_to_grid(point)
        rotated_cell = rotated.world_to_grid(point)
        print(f"  axis-aligned -> {axis_cell}")
        print(f"  rotated      -> {rotated_cell}")
    print()


def compare_sample_cells(
    axis_aligned: AxisAlignedSubmap,
    rotated: RotatedSubmap,
    sample_cells: list[tuple[int, int]],
) -> None:
    print("Sample cell centers")
    print("===================")
    for cell_x, cell_y in sample_cells:
        axis_world = axis_aligned.grid_to_world_center(cell_x, cell_y)
        rotated_world = rotated.grid_to_world_center(cell_x, cell_y)
        print(f"cell ({cell_x}, {cell_y})")
        print(f"  axis-aligned -> {format_point(axis_world)}")
        print(f"  rotated      -> {format_point(rotated_world)}")
    print()


def print_figure_guide() -> None:
    print("Figure guide")
    print("============")
    print(
        "Left panel: axis-aligned submap. "
        "The blue rectangle itself is the world footprint."
    )
    print(
        "Left panel: because the local axes match world x/y, "
        "lower_left + width/height is enough."
    )
    print(
        "Right panel: rotated submap. "
        "The blue polygon is the real rotated footprint in world coordinates."
    )
    print(
        "Right panel: the red dashed box is the axis-aligned world "
        "bounding box computed from 4 rotated corners."
    )
    print(
        "Orange arrow means local +x. Purple arrow means local +y. "
        "Gray arrows are world x/y."
    )
    print(
        "Pink labels such as (0,0) and (7,3) are sample grid-cell centers "
        "after mapping to world."
    )
    print()


def draw_submap_panel(
    axis,
    title: str,
    submap: AxisAlignedSubmap | RotatedSubmap,
    sample_cells: list[tuple[int, int]],
) -> None:
    from matplotlib.patches import Polygon, Rectangle

    corners = submap.corners_world()
    polygon_points = [(corner.x, corner.y) for corner in corners]
    polygon = Polygon(
        polygon_points,
        closed=True,
        fill=True,
        facecolor="#7db7ff",
        edgecolor="#1f4e79",
        linewidth=2.0,
        alpha=0.35,
    )
    axis.add_patch(polygon)

    xs = [corner.x for corner in corners]
    ys = [corner.y for corner in corners]
    for index, corner in enumerate(corners):
        axis.scatter([corner.x], [corner.y], color="#0f3057", s=35, zorder=5)
        axis.annotate(
            f"C{index}",
            (corner.x, corner.y),
            textcoords="offset points",
            xytext=(5, 5),
        )

    min_x, min_y, max_x, max_y = submap.world_bounds()
    bounds = Rectangle(
        (min_x, min_y),
        max_x - min_x,
        max_y - min_y,
        fill=False,
        edgecolor="#b22222",
        linewidth=1.6,
        linestyle="--",
    )
    axis.add_patch(bounds)
    axis.annotate(
        "world bounds",
        (max_x, max_y),
        textcoords="offset points",
        xytext=(-75, 6),
        color="#b22222",
    )

    if isinstance(submap, AxisAlignedSubmap):
        center = submap.map_center
        yaw = 0.0
    else:
        center = Point2D(submap.pose.x, submap.pose.y)
        yaw = submap.pose.yaw

    axis.scatter([center.x], [center.y], color="#1b5e20", s=45, zorder=6)
    axis.annotate(
        "center",
        (center.x, center.y),
        textcoords="offset points",
        xytext=(5, -14),
        color="#1b5e20",
    )

    axis.quiver(
        center.x,
        center.y,
        0.8,
        0.0,
        angles="xy",
        scale_units="xy",
        scale=1.0,
        color="#555555",
        width=0.008,
    )
    axis.quiver(
        center.x,
        center.y,
        0.0,
        0.8,
        angles="xy",
        scale_units="xy",
        scale=1.0,
        color="#555555",
        width=0.008,
    )
    axis.annotate(
        "world x",
        (center.x + 0.8, center.y),
        textcoords="offset points",
        xytext=(4, -10),
        color="#555555",
    )
    axis.annotate(
        "world y",
        (center.x, center.y + 0.8),
        textcoords="offset points",
        xytext=(4, 4),
        color="#555555",
    )

    axis.quiver(
        center.x,
        center.y,
        1.0 * math.cos(yaw),
        1.0 * math.sin(yaw),
        angles="xy",
        scale_units="xy",
        scale=1.0,
        color="#f57c00",
        width=0.010,
    )
    axis.quiver(
        center.x,
        center.y,
        -0.6 * math.sin(yaw),
        0.6 * math.cos(yaw),
        angles="xy",
        scale_units="xy",
        scale=1.0,
        color="#8e24aa",
        width=0.010,
    )
    axis.annotate(
        "local +x",
        (center.x + math.cos(yaw), center.y + math.sin(yaw)),
        textcoords="offset points",
        xytext=(4, 4),
        color="#f57c00",
    )
    axis.annotate(
        "local +y",
        (center.x - 0.6 * math.sin(yaw), center.y + 0.6 * math.cos(yaw)),
        textcoords="offset points",
        xytext=(4, 4),
        color="#8e24aa",
    )

    for cell_x, cell_y in sample_cells:
        cell_center = submap.grid_to_world_center(cell_x, cell_y)
        axis.scatter([cell_center.x], [cell_center.y], color="#d81b60", s=30, zorder=6)
        axis.annotate(
            f"({cell_x},{cell_y})",
            (cell_center.x, cell_center.y),
            textcoords="offset points",
            xytext=(5, 5),
            color="#d81b60",
        )

    padding = 0.9
    axis.set_xlim(min(xs + [min_x]) - padding, max(xs + [max_x]) + padding)
    axis.set_ylim(min(ys + [min_y]) - padding, max(ys + [max_y]) + padding)
    axis.set_aspect("equal", adjustable="box")
    axis.grid(True, linewidth=0.5, alpha=0.35)
    axis.set_title(title)
    axis.set_xlabel("world x")
    axis.set_ylabel("world y")


def render_plot(
    axis_aligned: AxisAlignedSubmap,
    rotated: RotatedSubmap,
    sample_cells: list[tuple[int, int]],
    output_path: str,
    show_window: bool,
) -> None:
    headless = not os.environ.get("DISPLAY")
    if headless:
        import matplotlib

        matplotlib.use("Agg")

    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(
        1, 2, figsize=(13, 6.5), constrained_layout=True
    )
    draw_submap_panel(axes[0], "Axis-aligned Submap", axis_aligned, sample_cells)
    draw_submap_panel(axes[1], "Rotated Submap", rotated, sample_cells)
    figure.suptitle("Submap geometry: axis-aligned vs rotated", fontsize=14)
    figure.text(
        0.5,
        0.02,
        "Blue shape: real submap footprint. Red dashed box: world "
        "bounding box. Rotated submaps need corner rotation before "
        "min/max bounds can be computed.",
        ha="center",
        fontsize=10,
        bbox={
            "boxstyle": "round,pad=0.35",
            "facecolor": "#fff7e6",
            "edgecolor": "#d9a441",
        },
    )
    figure.savefig(output_path, dpi=180, bbox_inches="tight")
    if show_window and not headless:
        plt.show()
    plt.close(figure)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Show axis-aligned and rotated submap geometry.")
    parser.add_argument(
        "--output",
        default=os.path.join(os.path.dirname(__file__), "submap_rotation_demo.png"),
        help="Path to save the generated figure.",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Do not open a matplotlib window even if DISPLAY is available.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    axis_aligned = AxisAlignedSubmap(
        resolution=0.5,
        width=8,
        height=4,
        map_center=Point2D(10.0, 5.0),
    )
    rotated = RotatedSubmap(
        resolution=0.5,
        width=8,
        height=4,
        pose=Pose2D(10.0, 5.0, math.radians(30.0)),
    )

    print_submap_info("Axis-aligned submap", axis_aligned)
    print_submap_info("Rotated submap", rotated)

    sample_points = [
        Point2D(10.0, 5.0),
        Point2D(8.3, 4.4),
        Point2D(11.6, 5.8),
        Point2D(12.1, 6.8),
    ]
    compare_sample_points(axis_aligned, rotated, sample_points)

    sample_cells = [(0, 0), (3, 1), (7, 3)]
    compare_sample_cells(axis_aligned, rotated, sample_cells)
    print_figure_guide()

    render_plot(axis_aligned, rotated, sample_cells, args.output, show_window=not args.no_show)
    print(f"Figure saved to: {args.output}")

    print("Key difference")
    print("==============")
    print("Axis-aligned submap can compute max_x/max_y by lower_left + width/height * resolution.")
    print("Rotated submap must rotate its four corners first, then build the world bounding box.")


if __name__ == "__main__":
    main()
