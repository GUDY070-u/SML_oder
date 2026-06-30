#!/usr/bin/env python3

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime
import json
import math
from pathlib import Path
import tkinter as tk
from tkinter import messagebox, ttk
from typing import Callable, Dict, Iterable, List, Optional, Tuple

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from sml_messages.msg import Order, Task


PACKAGE_NAME = "sml_worldcup_gui"
DEFAULT_TOPIC = "/eai/task"
WORKSPACE_LAYOUT_PATH = Path(
    "/home/user/ros2_ws/src/GUI/config/sml_worldcup_2026_layout.json"
)

SIDE_ALIASES = {
    "a": "side_a",
    "side_a": "side_a",
    "b": "side_b",
    "side_b": "side_b",
    "all": "all",
    "both": "all",
}

# Crops are expressed in the coordinate system of the World Cup layout JSON.
# Both side views overlap at the shared storage in the center.
SIDE_VIEWPORTS = {
    "side_a": (120.0, 45.0, 180.0, 235.0),
    "side_b": (255.0, 45.0, 180.0, 235.0),
}

BACKGROUND = "#111827"
PANEL = "#182235"
PANEL_ALT = "#202c40"
TEXT = "#e5edf8"
MUTED = "#94a3b8"
ACCENT = "#38bdf8"
SUCCESS = "#34d399"
WARNING = "#fbbf24"

STATION_COLORS = {
    "customer_counter": "#f59e0b",
    "workbench": "#a855f7",
    "hybrid_work_shelf": "#c084fc",
    "storage_shelf": "#22c55e",
    "shared_storage": "#10b981",
}

ZONE_COLORS = {
    "side_a": "#172554",
    "side_b": "#3b172b",
    "center": "#1f2937",
    "customers": "#3b2a15",
    "robot_fleets": "#163047",
    "wait_a": "#1d3557",
    "wait_b": "#4a1d32",
    "warehouse": "#193425",
    "workbench_area_b": "#332047",
}

OBJECT_NAMES = {
    1: "2x2 red",
    2: "2x2 green",
    3: "2x2 blue",
    4: "2x2 yellow",
    5: "4x2 red",
    6: "4x2 green",
    7: "4x2 blue",
    8: "4x2 yellow",
    10: "2x2 red batch",
    20: "2x2 green batch",
    30: "2x2 blue batch",
    40: "2x2 yellow batch",
    50: "4x2 red batch",
    60: "4x2 green batch",
    70: "4x2 blue batch",
    80: "4x2 yellow batch",
    90: "mixed batch",
    13: "Magnet",
    34: "Battery",
    81: "E-Stop",
    241: "Traffic Light",
    442: "Carrot",
    462: "Small Tree",
    711: "Hammer",
    4482: "Big Carrot",
    8518: "Burger",
    48132: "Ice Cream",
    46262: "Big Tree",
}


@dataclass(frozen=True)
class OrderState:
    name: str
    order_type: int
    product_id: int

    @property
    def type_name(self) -> str:
        if self.order_type == Order.OT_PRODUCE:
            return "PRODUCE"
        if self.order_type == Order.OT_RECYCLE:
            return "RECYCLE"
        return f"UNKNOWN({self.order_type})"


@dataclass(frozen=True)
class StationState:
    station_id: int
    name: str
    station_type: int
    material_ids: Tuple[int, ...]


@dataclass
class TaskState:
    orders: List[OrderState] = field(default_factory=list)
    stations: Dict[int, StationState] = field(default_factory=dict)
    received_at: Optional[datetime] = None
    message_count: int = 0

    def update(self, msg: Task) -> None:
        self.orders = [
            OrderState(
                name=order.name,
                order_type=order.order_type,
                product_id=order.product_id,
            )
            for order in msg.order_list
        ]
        self.stations = {
            station.station_id: StationState(
                station_id=station.station_id,
                name=station.name,
                station_type=station.station_type,
                material_ids=tuple(station.material_ids),
            )
            for station in msg.arena_layout
        }
        self.received_at = datetime.now()
        self.message_count += 1


class LayoutModel:
    def __init__(self, path: Path) -> None:
        self.path = path
        with path.open(encoding="utf-8") as stream:
            self.data = json.load(stream)
        self._validate()

        map_size = self.data["coordinate_system"]["map_size_px"]
        self.width = float(map_size["width"])
        self.height = float(map_size["height"])
        self.stations_by_id = {
            int(station["station_id"]): station
            for station in self.data["stations"]
        }

    def _validate(self) -> None:
        required = {
            "coordinate_system",
            "zones",
            "stations",
            "start_areas",
            "walls",
        }
        missing = sorted(required.difference(self.data))
        if missing:
            raise ValueError(f"Layout JSON missing keys: {', '.join(missing)}")

        station_ids = [int(station["station_id"]) for station in self.data["stations"]]
        if len(station_ids) != len(set(station_ids)):
            raise ValueError("Layout JSON contains duplicate station_id values")


class TaskListenerNode(Node):
    def __init__(self) -> None:
        super().__init__("sml_worldcup_gui")
        self.declare_parameter("topic_name", DEFAULT_TOPIC)
        self.declare_parameter("layout_file", "")
        self.declare_parameter("refresh_ms", 50)
        self.declare_parameter("side", "all")
        self._callback: Optional[Callable[[Task], None]] = None

        topic = self.get_parameter("topic_name").value
        self._subscription = self.create_subscription(
            Task,
            topic,
            self._on_task,
            10,
        )
        self.get_logger().info(f"GUI listening for tasks on {topic}")

    def set_task_callback(self, callback: Callable[[Task], None]) -> None:
        self._callback = callback

    def _on_task(self, msg: Task) -> None:
        if self._callback is not None:
            self._callback(msg)


class WorldCupGui:
    def __init__(
        self,
        root: tk.Tk,
        layout: LayoutModel,
        topic_name: str,
        selected_side: str,
    ) -> None:
        self.root = root
        self.layout = layout
        self.topic_name = topic_name
        self.selected_side = selected_side
        self.task = TaskState()
        self.selected_station_id: Optional[int] = None
        self._transform = (1.0, 0.0, 0.0)
        self.visible_station_ids = {
            station_id
            for station_id, station in self.layout.stations_by_id.items()
            if self._is_visible_side(station.get("side", "shared"))
        }

        self._configure_window()
        self._configure_styles()
        self._build_widgets()
        self.root.after_idle(self.redraw_map)

    def _configure_window(self) -> None:
        self.root.title("SML World Cup 2026 — Match Visualizer")
        self.root.geometry("1280x780")
        self.root.minsize(1000, 640)
        self.root.configure(bg=BACKGROUND)

    def _configure_styles(self) -> None:
        style = ttk.Style(self.root)
        style.theme_use("clam")
        style.configure(
            "Treeview",
            background=PANEL_ALT,
            fieldbackground=PANEL_ALT,
            foreground=TEXT,
            rowheight=28,
            borderwidth=0,
        )
        style.configure(
            "Treeview.Heading",
            background="#273449",
            foreground=TEXT,
            relief="flat",
            font=("TkDefaultFont", 10, "bold"),
        )
        style.map("Treeview", background=[("selected", "#075985")])
        style.configure("TSeparator", background="#334155")

    def _build_widgets(self) -> None:
        header = tk.Frame(self.root, bg=BACKGROUND, padx=18, pady=12)
        header.pack(fill=tk.X)

        tk.Label(
            header,
            text=f"SML WORLD CUP 2026  /  {self._side_title()}",
            bg=BACKGROUND,
            fg=TEXT,
            font=("TkDefaultFont", 18, "bold"),
        ).pack(side=tk.LEFT)

        self.connection_label = tk.Label(
            header,
            text=f"● WAITING  {self.topic_name}",
            bg=BACKGROUND,
            fg=WARNING,
            font=("TkDefaultFont", 10, "bold"),
        )
        self.connection_label.pack(side=tk.RIGHT)

        body = tk.PanedWindow(
            self.root,
            orient=tk.HORIZONTAL,
            bg=BACKGROUND,
            sashwidth=6,
            sashrelief=tk.FLAT,
            bd=0,
        )
        body.pack(fill=tk.BOTH, expand=True, padx=14, pady=(0, 14))

        map_panel = tk.Frame(body, bg=PANEL, highlightthickness=1, highlightbackground="#334155")
        side_panel = tk.Frame(body, bg=PANEL, width=385)
        body.add(map_panel, stretch="always", minsize=620)
        body.add(side_panel, stretch="never", minsize=350)

        map_header = tk.Frame(map_panel, bg=PANEL, padx=12, pady=9)
        map_header.pack(fill=tk.X)
        tk.Label(
            map_header,
            text=f"{self.layout.data.get('name', 'Arena Layout')} — {self._side_title()}",
            bg=PANEL,
            fg=TEXT,
            font=("TkDefaultFont", 11, "bold"),
        ).pack(side=tk.LEFT)
        tk.Label(
            map_header,
            text="Click a station for details",
            bg=PANEL,
            fg=MUTED,
        ).pack(side=tk.RIGHT)

        self.canvas = tk.Canvas(
            map_panel,
            bg="#0b1220",
            highlightthickness=0,
        )
        self.canvas.pack(fill=tk.BOTH, expand=True, padx=8, pady=(0, 8))
        self.canvas.bind("<Configure>", lambda _event: self.redraw_map())

        self._build_side_panel(side_panel)

    def _build_side_panel(self, panel: tk.Frame) -> None:
        summary = tk.Frame(panel, bg=PANEL, padx=14, pady=12)
        summary.pack(fill=tk.X)
        self.summary_label = tk.Label(
            summary,
            text="No task received",
            bg=PANEL,
            fg=TEXT,
            anchor="w",
            font=("TkDefaultFont", 12, "bold"),
        )
        self.summary_label.pack(fill=tk.X)
        self.time_label = tk.Label(
            summary,
            text="Waiting for /eai/task …",
            bg=PANEL,
            fg=MUTED,
            anchor="w",
        )
        self.time_label.pack(fill=tk.X, pady=(4, 0))

        ttk.Separator(panel).pack(fill=tk.X, padx=12)
        tk.Label(
            panel,
            text="ORDERS",
            bg=PANEL,
            fg=ACCENT,
            anchor="w",
            font=("TkDefaultFont", 10, "bold"),
            padx=14,
            pady=10,
        ).pack(fill=tk.X)

        order_frame = tk.Frame(panel, bg=PANEL)
        order_frame.pack(fill=tk.BOTH, expand=True, padx=12)
        self.order_tree = ttk.Treeview(
            order_frame,
            columns=("type", "product", "name"),
            show="headings",
            height=9,
        )
        self.order_tree.heading("type", text="Type")
        self.order_tree.heading("product", text="Product")
        self.order_tree.heading("name", text="Order name")
        self.order_tree.column("type", width=78, anchor=tk.CENTER, stretch=False)
        self.order_tree.column("product", width=95, anchor=tk.W, stretch=False)
        self.order_tree.column("name", width=175, anchor=tk.W)
        order_scroll = ttk.Scrollbar(
            order_frame,
            orient=tk.VERTICAL,
            command=self.order_tree.yview,
        )
        self.order_tree.configure(yscrollcommand=order_scroll.set)
        self.order_tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        order_scroll.pack(side=tk.RIGHT, fill=tk.Y)

        ttk.Separator(panel).pack(fill=tk.X, padx=12, pady=(12, 0))
        tk.Label(
            panel,
            text="STATION DETAILS",
            bg=PANEL,
            fg=ACCENT,
            anchor="w",
            font=("TkDefaultFont", 10, "bold"),
            padx=14,
            pady=10,
        ).pack(fill=tk.X)

        self.station_detail = tk.Label(
            panel,
            text="Select a station on the map.",
            bg=PANEL_ALT,
            fg=TEXT,
            justify=tk.LEFT,
            anchor="nw",
            padx=12,
            pady=10,
            wraplength=330,
            height=7,
        )
        self.station_detail.pack(fill=tk.X, padx=12, pady=(0, 12))

        legend = tk.Frame(panel, bg=PANEL, padx=12, pady=4)
        legend.pack(fill=tk.X)
        for label, color in (
            ("Storage", STATION_COLORS["storage_shelf"]),
            ("Workbench", STATION_COLORS["workbench"]),
            ("Hybrid", STATION_COLORS["hybrid_work_shelf"]),
            ("Customer", STATION_COLORS["customer_counter"]),
        ):
            item = tk.Frame(legend, bg=PANEL)
            item.pack(side=tk.LEFT, padx=(0, 10))
            tk.Label(item, text="■", bg=PANEL, fg=color).pack(side=tk.LEFT)
            tk.Label(item, text=label, bg=PANEL, fg=MUTED).pack(side=tk.LEFT)

    def update_task(self, msg: Task) -> None:
        self.task.update(msg)
        received = self.task.received_at
        received_text = received.strftime("%H:%M:%S") if received else "-"
        self.connection_label.configure(
            text=f"● LIVE  {self.topic_name}",
            fg=SUCCESS,
        )
        self.summary_label.configure(
            text=(
                f"{len(self.task.orders)} orders  •  "
                f"{sum(station_id in self.visible_station_ids for station_id in self.task.stations)} "
                "visible stations"
            )
        )
        self.time_label.configure(
            text=f"Last update {received_text}  •  message #{self.task.message_count}"
        )
        self._refresh_order_tree()
        self.redraw_map()
        if self.selected_station_id is not None:
            self._show_station_details(self.selected_station_id)

    def _refresh_order_tree(self) -> None:
        self.order_tree.delete(*self.order_tree.get_children())
        for order in self.task.orders:
            product = OBJECT_NAMES.get(order.product_id, f"ID {order.product_id}")
            self.order_tree.insert(
                "",
                tk.END,
                values=(order.type_name, f"{order.product_id} {product}", order.name),
            )

    def redraw_map(self) -> None:
        width = max(self.canvas.winfo_width(), 1)
        height = max(self.canvas.winfo_height(), 1)
        if width < 20 or height < 20:
            return

        self.canvas.delete("all")
        padding = 24.0
        view_x, view_y, view_width, view_height = self._viewport()
        scale = min(
            (width - 2 * padding) / view_width,
            (height - 2 * padding) / view_height,
        )
        offset_x = (width - view_width * scale) / 2 - view_x * scale
        offset_y = (height - view_height * scale) / 2 - view_y * scale
        self._transform = (scale, offset_x, offset_y)

        self._draw_zones()
        self._draw_walls()
        self._draw_start_areas()
        self._draw_stations()

    def _xy(self, x: float, y: float) -> Tuple[float, float]:
        scale, offset_x, offset_y = self._transform
        return offset_x + x * scale, offset_y + y * scale

    def _draw_zones(self) -> None:
        for zone in self.layout.data["zones"]:
            bounds = zone["bounds"]
            x1, y1 = self._xy(bounds["x"], bounds["y"])
            x2, y2 = self._xy(
                bounds["x"] + bounds["width"],
                bounds["y"] + bounds["height"],
            )
            color = ZONE_COLORS.get(zone["id"], "#1f2937")
            self.canvas.create_rectangle(
                x1,
                y1,
                x2,
                y2,
                fill=color,
                outline="#475569",
                width=1,
            )
            self.canvas.create_text(
                x1 + 5,
                y1 + 5,
                text=zone["name"],
                fill="#7f93ad",
                anchor=tk.NW,
                font=("TkDefaultFont", 8),
            )

    def _draw_walls(self) -> None:
        scale, _, _ = self._transform
        for wall in self.layout.data["walls"]:
            start_x, start_y = self._xy(wall["start"]["x"], wall["start"]["y"])
            end_x, end_y = self._xy(wall["end"]["x"], wall["end"]["y"])
            color = "#22d3ee" if wall["type"] == "wall_100cm" else "#c4b5fd"
            self.canvas.create_line(
                start_x,
                start_y,
                end_x,
                end_y,
                fill=color,
                width=max(2, wall.get("thickness", 3) * scale),
                capstyle=tk.ROUND,
            )

    def _draw_start_areas(self) -> None:
        for area in self.layout.data["start_areas"]:
            if not self._is_visible_side(area.get("side", "shared")):
                continue
            center = area["center"]
            size = area["size"]
            x1, y1 = self._xy(
                center["x"] - size["width"] / 2,
                center["y"] - size["height"] / 2,
            )
            x2, y2 = self._xy(
                center["x"] + size["width"] / 2,
                center["y"] + size["height"] / 2,
            )
            self.canvas.create_rectangle(
                x1,
                y1,
                x2,
                y2,
                outline="#ef4444",
                dash=(5, 3),
                width=2,
            )
            self.canvas.create_text(
                (x1 + x2) / 2,
                (y1 + y2) / 2,
                text=area["name"],
                fill="#fca5a5",
                font=("TkDefaultFont", 7, "bold"),
                width=max(30, x2 - x1 - 4),
            )

    def _draw_stations(self) -> None:
        for station in self.layout.data["stations"]:
            station_id = int(station["station_id"])
            if station_id not in self.visible_station_ids:
                continue
            received = self.task.stations.get(station_id)
            color = STATION_COLORS.get(station["type"], "#64748b")
            outline = (
                "#ffffff"
                if station_id == self.selected_station_id
                else SUCCESS if received is not None else "#cbd5e1"
            )
            polygon = self._rotated_rectangle(station)
            tag = f"station_{station_id}"
            self.canvas.create_polygon(
                *polygon,
                fill=color,
                outline=outline,
                width=3 if station_id == self.selected_station_id else 2,
                tags=(tag, "station"),
            )

            center_x, center_y = self._xy(
                station["center"]["x"],
                station["center"]["y"],
            )
            self.canvas.create_text(
                center_x,
                center_y,
                text=str(station_id),
                fill="#ffffff",
                font=("TkDefaultFont", 9, "bold"),
                tags=(tag, "station"),
            )

            label = station["name"].replace("side_a_", "A ").replace("side_b_", "B ")
            label = label.replace("shared_", "Shared ")
            label_x, label_y = self._xy(
                station["label_position"]["x"],
                station["label_position"]["y"],
            )
            self.canvas.create_text(
                label_x,
                label_y,
                text=label,
                fill=TEXT,
                font=("TkDefaultFont", 7),
                tags=(tag, "station"),
            )

            if received is not None and received.material_ids:
                material_text = ",".join(str(value) for value in received.material_ids)
                badge_y = center_y + 15
                badge = self.canvas.create_text(
                    center_x,
                    badge_y,
                    text=f"[{material_text}]",
                    fill="#04111d",
                    font=("TkDefaultFont", 7, "bold"),
                    tags=(tag, "station"),
                )
                bounds = self.canvas.bbox(badge)
                if bounds:
                    background = self.canvas.create_rectangle(
                        bounds[0] - 3,
                        bounds[1] - 1,
                        bounds[2] + 3,
                        bounds[3] + 1,
                        fill="#bae6fd",
                        outline="",
                        tags=(tag, "station"),
                    )
                    self.canvas.tag_lower(background, badge)

            self.canvas.tag_bind(
                tag,
                "<Button-1>",
                lambda _event, value=station_id: self.select_station(value),
            )
            self.canvas.tag_bind(tag, "<Enter>", lambda _event: self.canvas.configure(cursor="hand2"))
            self.canvas.tag_bind(tag, "<Leave>", lambda _event: self.canvas.configure(cursor=""))

    def _rotated_rectangle(self, station: dict) -> List[float]:
        center = station["center"]
        size = station["size"]
        angle = math.radians(float(station.get("rotation_deg", 0)))
        half_width = size["width"] / 2
        half_height = size["height"] / 2
        points: List[float] = []
        for local_x, local_y in (
            (-half_width, -half_height),
            (half_width, -half_height),
            (half_width, half_height),
            (-half_width, half_height),
        ):
            rotated_x = local_x * math.cos(angle) - local_y * math.sin(angle)
            rotated_y = local_x * math.sin(angle) + local_y * math.cos(angle)
            canvas_x, canvas_y = self._xy(
                center["x"] + rotated_x,
                center["y"] + rotated_y,
            )
            points.extend((canvas_x, canvas_y))
        return points

    def select_station(self, station_id: int) -> None:
        if station_id not in self.visible_station_ids:
            return
        self.selected_station_id = station_id
        self._show_station_details(station_id)
        self.redraw_map()

    def _show_station_details(self, station_id: int) -> None:
        if station_id not in self.visible_station_ids:
            return
        layout_station = self.layout.stations_by_id.get(station_id)
        if layout_station is None:
            return
        state = self.task.stations.get(station_id)
        materials = state.material_ids if state is not None else ()
        material_lines = self._format_materials(materials)
        live_status = "Task data received" if state is not None else "No task data"
        text = (
            f"#{station_id}  {layout_station['name']}\n"
            f"Type: {layout_station['type']}\n"
            f"Side: {layout_station['side']}\n"
            f"Status: {live_status}\n"
            f"Materials: {material_lines}"
        )
        self.station_detail.configure(text=text)

    @staticmethod
    def _format_materials(materials: Iterable[int]) -> str:
        values = list(materials)
        if not values:
            return "empty"
        return ", ".join(
            f"{value} ({OBJECT_NAMES.get(value, 'unknown')})"
            for value in values
        )

    def _viewport(self) -> Tuple[float, float, float, float]:
        if self.selected_side in SIDE_VIEWPORTS:
            return SIDE_VIEWPORTS[self.selected_side]
        return (0.0, 0.0, self.layout.width, self.layout.height)

    def _is_visible_side(self, item_side: str) -> bool:
        if self.selected_side == "all":
            return True
        return item_side in {self.selected_side, "shared"}

    def _side_title(self) -> str:
        return {
            "side_a": "SIDE A",
            "side_b": "SIDE B",
            "all": "FULL ARENA",
        }[self.selected_side]


def default_layout_path() -> Path:
    if WORKSPACE_LAYOUT_PATH.is_file():
        return WORKSPACE_LAYOUT_PATH
    share = Path(get_package_share_directory(PACKAGE_NAME))
    return share / "config" / "sml_worldcup_2026_layout.json"


def normalize_side(value: str) -> str:
    normalized = SIDE_ALIASES.get(value.strip().lower())
    if normalized is None:
        valid = ", ".join(("a", "b", "all"))
        raise ValueError(f"Unsupported side '{value}'. Valid values: {valid}")
    return normalized


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TaskListenerNode()
    root: Optional[tk.Tk] = None

    try:
        configured_path = str(node.get_parameter("layout_file").value).strip()
        layout_path = Path(configured_path).expanduser() if configured_path else default_layout_path()
        layout = LayoutModel(layout_path)

        root = tk.Tk()
        topic_name = str(node.get_parameter("topic_name").value)
        selected_side = normalize_side(str(node.get_parameter("side").value))
        refresh_ms = max(10, int(node.get_parameter("refresh_ms").value))
        gui = WorldCupGui(root, layout, topic_name, selected_side)
        node.set_task_callback(gui.update_task)

        closed = False

        def close() -> None:
            nonlocal closed
            if closed:
                return
            closed = True
            root.destroy()

        def pump_ros() -> None:
            if closed:
                return
            try:
                rclpy.spin_once(node, timeout_sec=0.0)
            except Exception as error:  # Keep Tk alive long enough to show the error.
                messagebox.showerror("ROS 2 error", str(error), parent=root)
                close()
                return
            root.after(refresh_ms, pump_ros)

        root.protocol("WM_DELETE_WINDOW", close)
        root.after(refresh_ms, pump_ros)
        root.mainloop()
    except Exception as error:
        if root is not None:
            messagebox.showerror("SML GUI error", str(error), parent=root)
        else:
            node.get_logger().error(str(error))
        raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
