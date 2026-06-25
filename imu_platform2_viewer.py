from __future__ import annotations

"""Transport-agnostic cube viewer driven by caller-provided callbacks."""

import math
import sys
from typing import Callable, Optional

from imu_platform2_frames import MotionFrame

try:
    import tkinter as tk
except ImportError as exc:  # pragma: no cover - depends on target environment
    tk = None
    TK_IMPORT_ERROR = exc
else:
    TK_IMPORT_ERROR = None

VERTICES = (
    (-1.0, -1.0, -1.0),
    (1.0, -1.0, -1.0),
    (1.0, 1.0, -1.0),
    (-1.0, 1.0, -1.0),
    (-1.0, -1.0, 1.0),
    (1.0, -1.0, 1.0),
    (1.0, 1.0, 1.0),
    (-1.0, 1.0, 1.0),
)

FACES = (
    (4, 5, 6, 7),  # front
    (0, 1, 2, 3),  # back
    (0, 4, 7, 3),  # left
    (1, 5, 6, 2),  # right
    (3, 7, 6, 2),  # top
    (0, 4, 5, 1),  # bottom
)

FACE_COLORS = (
    "#ffff00",
    "#00ff00",
    "#ff00ff",
    "#ff0000",
    "#0000ff",
    "#00ffff",
)

FACE_NORMALS = (
    (0.0, 0.0, 1.0),
    (0.0, 0.0, -1.0),
    (-1.0, 0.0, 0.0),
    (1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
    (0.0, -1.0, 0.0),
)

BACKGROUND_COLOR = "#ffffff"
INFO_TEXT_COLOR = "#1f2328"
CUBE_OUTLINE_COLOR = "#202020"
GRID_MINOR_COLOR = "#d0d0d0"
GRID_MAJOR_COLOR = "#9a9a9a"
GRID_AXIS_COLOR = "#666666"


def normalize_quaternion(
    quaternion: tuple[float, float, float, float]
) -> tuple[float, float, float, float]:
    w, x, y, z = quaternion
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm < 1e-9:
        return (1.0, 0.0, 0.0, 0.0)
    return (w / norm, x / norm, y / norm, z / norm)


def quaternion_to_rotation_matrix(
    quaternion: tuple[float, float, float, float]
) -> tuple[
    tuple[float, float, float],
    tuple[float, float, float],
    tuple[float, float, float],
]:
    w, x, y, z = normalize_quaternion(quaternion)
    return (
        (
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * z + y * w),
        ),
        (
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w),
        ),
        (
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y),
        ),
    )


def rotate_vertex(
    vertex: tuple[float, float, float],
    rotation_matrix: tuple[
        tuple[float, float, float],
        tuple[float, float, float],
        tuple[float, float, float],
    ],
) -> tuple[float, float, float]:
    x, y, z = vertex
    return (
        rotation_matrix[0][0] * x + rotation_matrix[0][1] * y + rotation_matrix[0][2] * z,
        rotation_matrix[1][0] * x + rotation_matrix[1][1] * y + rotation_matrix[1][2] * z,
        rotation_matrix[2][0] * x + rotation_matrix[2][1] * y + rotation_matrix[2][2] * z,
    )


def multiply_rotation_matrices(
    left: tuple[
        tuple[float, float, float],
        tuple[float, float, float],
        tuple[float, float, float],
    ],
    right: tuple[
        tuple[float, float, float],
        tuple[float, float, float],
        tuple[float, float, float],
    ],
) -> tuple[
    tuple[float, float, float],
    tuple[float, float, float],
    tuple[float, float, float],
]:
    return tuple(
        tuple(
            sum(left[row][idx] * right[idx][col] for idx in range(3))
            for col in range(3)
        )
        for row in range(3)
    )


def build_view_rotation_matrix(
    yaw_degrees: float,
    pitch_degrees: float,
) -> tuple[
    tuple[float, float, float],
    tuple[float, float, float],
    tuple[float, float, float],
]:
    yaw_radians = math.radians(yaw_degrees)
    pitch_radians = math.radians(pitch_degrees)

    cos_yaw = math.cos(yaw_radians)
    sin_yaw = math.sin(yaw_radians)
    cos_pitch = math.cos(pitch_radians)
    sin_pitch = math.sin(pitch_radians)

    yaw_matrix = (
        (cos_yaw, 0.0, sin_yaw),
        (0.0, 1.0, 0.0),
        (-sin_yaw, 0.0, cos_yaw),
    )
    pitch_matrix = (
        (1.0, 0.0, 0.0),
        (0.0, cos_pitch, -sin_pitch),
        (0.0, sin_pitch, cos_pitch),
    )
    return multiply_rotation_matrices(pitch_matrix, yaw_matrix)


class CubeViewer:
    def __init__(
        self,
        *,
        width: int,
        height: int,
        fps: int,
        poll_callback: Optional[Callable[[], None]] = None,
        reset_callback: Optional[Callable[[], None]] = None,
        window_title: str = "IMU Platform2 Cube Viewer",
        info_hint_text: str = "Click to reset the attitude estimate",
    ) -> None:
        if tk is None:
            raise RuntimeError(str(TK_IMPORT_ERROR))

        self.poll_callback = poll_callback
        self.reset_callback = reset_callback
        self.info_hint_text = info_hint_text
        self.running = True
        self.current_quaternion = (1.0, 0.0, 0.0, 0.0)
        self.current_imu_counter = -1

        self.root = tk.Tk()
        self.root.title(window_title)
        self.root.geometry(f"{width}x{height}")
        self.root.configure(bg=BACKGROUND_COLOR)
        self.root.protocol("WM_DELETE_WINDOW", self.request_stop)

        self.info_bar_height = 44

        self.canvas = tk.Canvas(
            self.root,
            width=width,
            height=max(1, height - self.info_bar_height),
            bg=BACKGROUND_COLOR,
            highlightthickness=0,
        )

        self.info_var = tk.StringVar()
        self.info_label = tk.Label(
            self.root,
            textvariable=self.info_var,
            anchor="w",
            justify="left",
            bg=BACKGROUND_COLOR,
            fg=INFO_TEXT_COLOR,
            font=("Courier New", 11),
            padx=6,
            pady=4,
        )
        self.info_label.pack(side="bottom", fill="x")
        self.canvas.pack(side="top", fill="both", expand=True)
        self.canvas.bind("<Button-1>", self.on_canvas_click)

        self.frame_interval_ms = max(1, int(round(1000 / max(1, fps))))
        self.camera_distance = 10
        self.view_rotation_matrix = build_view_rotation_matrix(0.0, 20.0)
        self.cube_scale = min(width, height) * 0.1
        self.axis_overlay_scale = min(width, height) * 0.055

    def request_stop(self) -> None:
        self.running = False

    def on_canvas_click(self, _event: object) -> None:
        if self.reset_callback is None:
            return
        try:
            self.reset_callback()
        except Exception as exc:  # pragma: no cover - callback owned by caller
            print(f"[WARN] Click handler failed: {exc}", file=sys.stderr)

    def apply_motion_frame(self, frame: MotionFrame) -> None:
        self.current_quaternion = frame.viewer_quaternion
        self.current_imu_counter = frame.imu_counter

    def project_vertex(
        self,
        vertex: tuple[float, float, float],
    ) -> tuple[float, float, float]:
        x, y, z = vertex

        canvas_width = max(1, self.canvas.winfo_width())
        canvas_height = max(1, self.canvas.winfo_height())

        center_x = canvas_width * 0.5
        center_y = canvas_height * 0.43

        screen_x = center_x + x * self.cube_scale
        screen_y = center_y - y * self.cube_scale
        z_shifted = z + self.camera_distance

        return screen_x, screen_y, z_shifted

    def transform_scene_vertex(
        self,
        vertex: tuple[float, float, float],
    ) -> tuple[float, float, float]:
        return rotate_vertex(vertex, self.view_rotation_matrix)

    def project_axis_overlay_vertex(
        self,
        vertex: tuple[float, float, float],
    ) -> tuple[float, float, float]:
        x, y, z = vertex

        canvas_width = max(1, self.canvas.winfo_width())
        canvas_height = max(1, self.canvas.winfo_height())

        center_x = max(72.0, canvas_width * 0.12)
        center_y = min(canvas_height - 72.0, canvas_height * 0.82)

        screen_x = center_x + x * self.axis_overlay_scale
        screen_y = center_y - y * self.axis_overlay_scale
        z_shifted = z + self.camera_distance

        return screen_x, screen_y, z_shifted

    def draw_background_grid(self) -> None:
        grid_half_extent = 10
        grid_y = -1.3

        for offset in range(-grid_half_extent, grid_half_extent + 1):
            color = GRID_AXIS_COLOR if offset == 0 else GRID_MAJOR_COLOR if offset % 2 == 0 else GRID_MINOR_COLOR
            width = 3 if offset == 0 else 2 if offset % 2 == 0 else 1

            start = self.transform_scene_vertex((-grid_half_extent, grid_y, float(offset)))
            end = self.transform_scene_vertex((grid_half_extent, grid_y, float(offset)))
            sx, sy, _ = self.project_vertex(start)
            ex, ey, _ = self.project_vertex(end)
            self.canvas.create_line(sx, sy, ex, ey, fill=color, width=width)

        for offset in range(-grid_half_extent, grid_half_extent + 1):
            color = GRID_AXIS_COLOR if offset == 0 else GRID_MAJOR_COLOR if offset % 2 == 0 else GRID_MINOR_COLOR
            width = 3 if offset == 0 else 2 if offset % 2 == 0 else 1

            start = self.transform_scene_vertex((float(offset), grid_y, -grid_half_extent))
            end = self.transform_scene_vertex((float(offset), grid_y, grid_half_extent))
            sx, sy, _ = self.project_vertex(start)
            ex, ey, _ = self.project_vertex(end)
            self.canvas.create_line(sx, sy, ex, ey, fill=color, width=width)

    def draw_axis_arrows(
        self,
        rotation_matrix: tuple[
            tuple[float, float, float],
            tuple[float, float, float],
            tuple[float, float, float],
        ],
    ) -> None:
        axis_length = 1.65
        origin_3d = (0.0, 0.0, 0.0)
        axes = (
            ("X", (axis_length, 0.0, 0.0), "#ff5555"),
            ("Y", (0.0, 0.0, -axis_length), "#55ff55"),
            ("Z", (0.0, axis_length, 0.0), "#5599ff"),
        )

        origin_rotated = self.transform_scene_vertex(rotate_vertex(origin_3d, rotation_matrix))
        ox, oy, _ = self.project_axis_overlay_vertex(origin_rotated)

        for label, axis_end_3d, color in axes:
            end_rotated = self.transform_scene_vertex(rotate_vertex(axis_end_3d, rotation_matrix))
            ex, ey, _ = self.project_axis_overlay_vertex(end_rotated)
            label_x = ox + (ex - ox) * 1.18
            label_y = oy + (ey - oy) * 1.18

            self.canvas.create_line(
                ox,
                oy,
                ex,
                ey,
                fill=color,
                width=4,
                arrow=tk.LAST,
                arrowshape=(14, 18, 7),
            )

            self.canvas.create_text(
                label_x,
                label_y,
                text=label,
                fill=color,
                font=("Courier New", 16, "bold"),
                anchor="center",
            )

    def draw_scene(self) -> None:
        self.canvas.delete("all")
        self.draw_background_grid()

        rotation_matrix = quaternion_to_rotation_matrix(self.current_quaternion)
        rotated_vertices = [
            self.transform_scene_vertex(rotate_vertex(vertex, rotation_matrix))
            for vertex in VERTICES
        ]
        projected_vertices = [self.project_vertex(vertex) for vertex in rotated_vertices]

        visible_faces = []
        for face_index, face_normal in enumerate(FACE_NORMALS):
            rotated_normal = self.transform_scene_vertex(rotate_vertex(face_normal, rotation_matrix))
            if rotated_normal[2] > 1e-6:
                visible_faces.append(face_index)

        face_order = sorted(
            visible_faces,
            key=lambda idx: sum(projected_vertices[v][2] for v in FACES[idx]) / 4.0,
        )

        for face_index in face_order:
            face = FACES[face_index]
            points = []
            for vertex_index in face:
                x, y, _ = projected_vertices[vertex_index]
                points.extend((x, y))

            self.canvas.create_polygon(
                points,
                fill=FACE_COLORS[face_index],
                outline=CUBE_OUTLINE_COLOR,
                width=2,
            )

        if 0 in visible_faces:
            front_face = FACES[0]
            front_center_x = sum(projected_vertices[v][0] for v in front_face) / 4.0
            front_center_y = sum(projected_vertices[v][1] for v in front_face) / 4.0

            marker_r = max(5.0, min(self.canvas.winfo_width(), self.canvas.winfo_height()) * 0.012)
            self.canvas.create_oval(
                front_center_x - marker_r,
                front_center_y - marker_r,
                front_center_x + marker_r,
                front_center_y + marker_r,
                fill="#ffffff",
                outline=CUBE_OUTLINE_COLOR,
                width=2,
            )

        self.draw_axis_arrows(rotation_matrix)

        w, x, y, z = normalize_quaternion(self.current_quaternion)
        self.info_var.set(
            f"imu_counter={self.current_imu_counter}  "
            f"q=({w:+.4f}, {x:+.4f}, {y:+.4f}, {z:+.4f})\n"
            f"{self.info_hint_text}"
        )

    def tick(self) -> None:
        if not self.running:
            self.root.quit()
            return
        if self.poll_callback is not None:
            self.poll_callback()
        self.draw_scene()
        self.root.after(self.frame_interval_ms, self.tick)

    def run(self) -> None:
        self.tick()
        self.root.mainloop()
