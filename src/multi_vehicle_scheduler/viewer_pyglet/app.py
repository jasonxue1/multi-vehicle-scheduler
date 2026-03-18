from __future__ import annotations

from copy import deepcopy

import pyglet
from pyglet import shapes
from pyglet.window import key

from multi_vehicle_scheduler.scheduler_backend import SceneScheduler, SchedulerOutput

from .models import ScenarioConfig, VehicleState, WorldState
from .simulation import apply_scheduler_output, build_scheduler_input_with_mode


class SimulationViewerApp:
    def __init__(
        self,
        scenario: ScenarioConfig,
        initial_world: WorldState,
        scheduler: SceneScheduler,
        fixed_dt: float = 1 / 30,
    ) -> None:
        self.ego_color = (255, 70, 70)
        self.bg_color = (80, 150, 255)
        self.scenario = scenario
        self.initial_world = deepcopy(initial_world)
        self.world = deepcopy(initial_world)
        self.scheduler = scheduler
        self.fixed_dt = fixed_dt
        self.latest_output = SchedulerOutput()
        self.paused = False
        self.show_reference_lines = True
        self.show_vehicle_labels = True
        self.ego_speed_step = 5
        self.ego_lane_change_speed = 180.0
        self.bg_lane_change_speed = 180.0
        self.camera_x = 0.0
        self.behavior_mode = "follow"
        self.behavior_start_time: float | None = None
        self.tick_counter = 0
        self.last_ego_input_tick = -1
        self.held_ego_control_keys: set[int] = set()

        self.window = pyglet.window.Window(
            width=scenario.map_width,
            height=scenario.map_height,
            caption=f"Multi Vehicle Scheduler - {scenario.name}",
        )
        self.batch = pyglet.graphics.Batch()
        self.vehicle_shapes: dict[str, shapes.Circle] = {}
        self.vehicle_labels: dict[str, pyglet.text.Label] = {}
        self.reference_lines: list[shapes.Line] = []

        self._build_static_scene()
        self._build_vehicle_scene()
        self._bind_events()
        self._update_camera_offset()
        self._sync_vehicle_shapes()
        self._sync_reference_lines()
        self._update_hud_text()

    def _build_static_scene(self) -> None:
        road_color = (35, 35, 35)
        lane_mark_color = (230, 230, 120)

        shapes.Rectangle(
            x=0,
            y=self.scenario.road_y_min,
            width=self.scenario.map_width,
            height=self.scenario.road_y_max - self.scenario.road_y_min,
            color=road_color,
            batch=self.batch,
        )
        for y in self.scenario.lane_center_y:
            shapes.Line(
                x=0,
                y=y,
                x2=self.scenario.map_width,
                y2=y,
                thickness=2,
                color=lane_mark_color,
                batch=self.batch,
            )

        self.hud_label = pyglet.text.Label(
            "",
            font_name="Menlo",
            font_size=14,
            x=16,
            y=0,
            anchor_x="left",
            anchor_y="center",
            color=(250, 250, 250, 255),
            batch=self.batch,
        )
        self._update_hud_position()

    def _build_vehicle_scene(self) -> None:
        for vehicle_id, vehicle in self.world.vehicles.items():
            self._ensure_vehicle_artifacts(vehicle_id, vehicle)

    def _bind_events(self) -> None:
        @self.window.event
        def on_draw() -> None:
            self.window.clear()
            self.batch.draw()

        @self.window.event
        def on_key_press(symbol: int, modifiers: int) -> None:
            del modifiers
            ego_control_keys = {key.UP, key.DOWN, key.A, key.D, key.LEFT, key.RIGHT}
            if symbol in ego_control_keys:
                if symbol in self.held_ego_control_keys:
                    self._update_hud_text()
                    return
                self.held_ego_control_keys.add(symbol)

            if symbol == key.SPACE:
                self.paused = not self.paused
            if symbol == key.R:
                self._reset_world()
            if symbol in (key.T, key.TAB):
                self.show_reference_lines = not self.show_reference_lines
                self._sync_reference_lines()
            if symbol == key.I:
                self.show_vehicle_labels = not self.show_vehicle_labels
                self._apply_vehicle_label_visibility()
            if symbol == key.N and self.paused:
                self._tick_once()
            if symbol == key.UP and self._try_consume_ego_input():
                self._adjust_ego_speed(self.ego_speed_step)
            if symbol == key.DOWN and self._try_consume_ego_input():
                self._adjust_ego_speed(-self.ego_speed_step)
            if symbol in (key.A, key.LEFT) and self._try_consume_ego_input():
                self._request_ego_lane_change("left")
            if symbol in (key.D, key.RIGHT) and self._try_consume_ego_input():
                self._request_ego_lane_change("right")
            if symbol == key.X:
                self._trigger_background_overtake()
            self._update_hud_text()

        @self.window.event
        def on_key_release(symbol: int, modifiers: int) -> None:
            del modifiers
            self.held_ego_control_keys.discard(symbol)

        @self.window.event
        def on_resize(width: int, height: int) -> None:
            del width, height
            self._update_hud_position()

        @self.window.event
        def on_scale(scale: float, dpi: int) -> None:
            del scale, dpi
            self._update_hud_position()

    def _update_hud_position(self) -> None:
        _, framebuffer_height = self.window.get_framebuffer_size()
        self.hud_label.y = framebuffer_height - 28

    def _try_consume_ego_input(self) -> bool:
        if self.last_ego_input_tick == self.tick_counter:
            return False
        self.last_ego_input_tick = self.tick_counter
        return True

    def _reset_world(self) -> None:
        self.world = deepcopy(self.initial_world)
        self.latest_output = SchedulerOutput()
        self.behavior_mode = "follow"
        self.behavior_start_time = None
        self._update_camera_offset()
        self._sync_vehicle_shapes()
        self._sync_reference_lines()
        self._update_hud_text()

    def _find_ego_vehicle(self) -> VehicleState | None:
        for vehicle in self.world.vehicles.values():
            if vehicle.is_ego:
                return vehicle
        return None

    def _get_sorted_lane_centers(self) -> list[float]:
        return sorted(self.scenario.lane_center_y)

    def _nearest_lane_index(self, y: float) -> int:
        lane_centers = self._get_sorted_lane_centers()
        return min(range(len(lane_centers)), key=lambda idx: abs(lane_centers[idx] - y))

    def _adjust_ego_speed(self, delta: float) -> None:
        ego = self._find_ego_vehicle()
        if ego is None:
            return
        ego.speed = int(max(0, ego.speed + delta))

    def _request_ego_lane_change(self, direction: str) -> None:
        ego = self._find_ego_vehicle()
        if ego is None:
            return
        lane_centers = self._get_sorted_lane_centers()
        current_idx = self._nearest_lane_index(
            ego.lane_target_y if ego.lane_target_y is not None else ego.y
        )
        if direction == "left":
            target_idx = min(len(lane_centers) - 1, current_idx + 1)
        else:
            target_idx = max(0, current_idx - 1)
        ego.lane_target_y = lane_centers[target_idx]

    def _trigger_background_overtake(self) -> None:
        self.behavior_mode = "intercept_run"
        self.behavior_start_time = self.world.sim_time

    def _all_background_in_front(
        self, front_margin: float = 20.0, lane_tolerance: float = 8.0
    ) -> bool:
        ego = self._find_ego_vehicle()
        if ego is None:
            return False
        for vehicle in self.world.vehicles.values():
            if vehicle.is_ego:
                continue
            if vehicle.x < ego.x + front_margin:
                return False
            if abs(vehicle.y - ego.y) > lane_tolerance:
                return False
        return True

    def _update_ego_manual_state(self) -> None:
        ego = self._find_ego_vehicle()
        if ego is None:
            return

        if ego.lane_target_y is None:
            ego.lane_target_y = ego.y

        dy = ego.lane_target_y - ego.y
        max_step = self.ego_lane_change_speed * self.fixed_dt
        if abs(dy) <= max_step:
            ego.y = ego.lane_target_y
        else:
            ego.y += max_step if dy > 0 else -max_step

        ego.x += ego.speed * self.fixed_dt

    def _update_camera_offset(self) -> None:
        ego = self._find_ego_vehicle()
        if ego is None:
            self.camera_x = 0.0
            return
        self.camera_x = ego.x - self.scenario.map_width * 0.5

    def _world_to_screen_x(self, world_x: float) -> float:
        return world_x - self.camera_x

    def _ensure_vehicle_artifacts(self, vehicle_id: str, vehicle: VehicleState) -> None:
        if vehicle_id not in self.vehicle_shapes:
            self.vehicle_shapes[vehicle_id] = shapes.Circle(
                x=vehicle.x,
                y=vehicle.y,
                radius=11,
                color=vehicle.color,
                batch=self.batch,
            )

        if vehicle_id not in self.vehicle_labels:
            self.vehicle_labels[vehicle_id] = pyglet.text.Label(
                "",
                font_name="Menlo",
                font_size=10,
                x=vehicle.x,
                y=vehicle.y,
                anchor_x="center",
                anchor_y="bottom",
                color=(245, 245, 245, 255),
                batch=self.batch,
            )

    def _sync_vehicle_shapes(self) -> None:
        active_vehicle_ids = set(self.world.vehicles.keys())
        for vehicle_id, vehicle in self.world.vehicles.items():
            self._ensure_vehicle_artifacts(vehicle_id, vehicle)
            screen_x = self._world_to_screen_x(vehicle.x)
            circle = self.vehicle_shapes[vehicle_id]
            circle.x = screen_x
            circle.y = vehicle.y
            circle.visible = True
            circle.radius = 13 if vehicle.is_ego else 11
            circle.color = self.ego_color if vehicle.is_ego else self.bg_color

            label = self.vehicle_labels[vehicle_id]
            label.x = screen_x
            label.y = vehicle.y + 13
            role = "ego" if vehicle.is_ego else "bg"
            label.text = f"{vehicle_id} [{role}] v={int(vehicle.speed)}"

        for vehicle_id, circle in self.vehicle_shapes.items():
            if vehicle_id not in active_vehicle_ids:
                circle.visible = False
        for vehicle_id, label in self.vehicle_labels.items():
            if vehicle_id not in active_vehicle_ids:
                label.visible = False

        self._apply_vehicle_label_visibility()

    def _apply_vehicle_label_visibility(self) -> None:
        for label in self.vehicle_labels.values():
            label.visible = self.show_vehicle_labels

    def _sync_reference_lines(self) -> None:
        for line in self.reference_lines:
            line.delete()
        self.reference_lines.clear()
        if not self.show_reference_lines:
            return

        for vehicle_id, trajectory in self.latest_output.trajectories.items():
            if not trajectory or vehicle_id not in self.world.vehicles:
                continue
            current = self.world.vehicles[vehicle_id]
            target = trajectory[0]
            self.reference_lines.append(
                shapes.Line(
                    x=self._world_to_screen_x(current.x),
                    y=current.y,
                    x2=self._world_to_screen_x(target.x),
                    y2=target.y,
                    thickness=2,
                    color=(100, 220, 255),
                    batch=self.batch,
                )
            )

    def _tick_once(self) -> None:
        self.tick_counter += 1
        self._update_ego_manual_state()
        behavior_elapsed = 0.0
        if (
            self.behavior_mode in {"intercept_run", "intercept_hold"}
            and self.behavior_start_time is not None
        ):
            behavior_elapsed = self.world.sim_time - self.behavior_start_time

        scheduler_input = build_scheduler_input_with_mode(
            self.world,
            self.scenario.lane_center_y,
            behavior_mode=self.behavior_mode,
            behavior_elapsed=behavior_elapsed,
        )
        self.latest_output = self.scheduler.plan(scheduler_input, self.fixed_dt)
        apply_scheduler_output(
            self.world,
            self.latest_output,
            self.fixed_dt,
            lateral_speed=self.bg_lane_change_speed,
        )

        if self.behavior_mode == "intercept_run" and self._all_background_in_front():
            self.behavior_mode = "intercept_hold"
            self.behavior_start_time = self.world.sim_time
        elif self.behavior_mode == "intercept_hold" and behavior_elapsed >= 1.0:
            self.behavior_mode = "follow"
            self.behavior_start_time = None

        self._update_camera_offset()
        self._sync_vehicle_shapes()
        self._sync_reference_lines()

    def _update_hud_text(self) -> None:
        status = "paused" if self.paused else "running"
        refs_status = "on" if self.show_reference_lines else "off"
        ego = self._find_ego_vehicle()
        ego_speed_text = f"{int(ego.speed)}" if ego else "-"
        ego_lane_text = f"{ego.y:.0f}" if ego else "-"
        self.hud_label.text = " | ".join(
            [
                f"t={self.world.sim_time:.2f}s",
                f"mode={self.behavior_mode}",
                f"vehicles={len(self.world.vehicles)}",
                f"ego_v={ego_speed_text}",
                f"ego_lane_y={ego_lane_text}",
                f"refs={refs_status}",
                status,
                "Space pause/resume",
                "N step",
                "R reset",
                "T/Tab refs",
                "I labels",
                "Up/Down ego speed",
                "A/D ego lane change",
                "X bg intercept",
            ]
        )

    def update(self, dt: float) -> None:
        del dt
        if self.paused:
            self._update_hud_text()
            return
        self._tick_once()
        self._update_hud_text()

    def run(self) -> None:
        pyglet.clock.schedule_interval(self.update, self.fixed_dt)
        pyglet.app.run()
