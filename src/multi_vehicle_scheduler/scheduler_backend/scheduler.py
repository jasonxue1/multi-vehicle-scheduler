import numpy as np

from .models import SchedulerInput, SchedulerOutput, TrajectoryPoint, VehicleCommand


class SceneScheduler:
    """Scene scheduler scaffold.

    V1 only keeps the scheduling interface and returns a simple
    constant-velocity reference trajectory based on current state.
    """

    def __init__(
        self,
        horizon_steps: int = 10,
        follow_base_gap: float = 28.0,
        max_accel_per_step: float = 1.0,
        max_decel_per_step: float = 4.0,
        intercept_max_accel_per_step: float = 10.0,
        intercept_max_decel_per_step: float = 6.0,
        intercept_speed_upper: float = 150.0,
    ) -> None:
        self.horizon_steps = horizon_steps
        self.follow_base_gap = follow_base_gap
        self.max_accel_per_step = max_accel_per_step
        self.max_decel_per_step = max_decel_per_step
        self.intercept_max_accel_per_step = intercept_max_accel_per_step
        self.intercept_max_decel_per_step = intercept_max_decel_per_step
        self.intercept_speed_upper = intercept_speed_upper

    def _nearest_lane_center(self, y: float, lane_centers: tuple[float, ...]) -> float:
        if not lane_centers:
            return y
        lane_array = np.asarray(lane_centers, dtype=np.float64)
        return float(lane_array[np.argmin(np.abs(lane_array - y))])

    def _other_lane_center(
        self, lane_centers: tuple[float, ...], current_y: float
    ) -> float:
        if len(lane_centers) < 2:
            return current_y
        lane_array = np.asarray(lane_centers, dtype=np.float64)
        current_idx = int(np.argmin(np.abs(lane_array - current_y)))
        other_idx = (
            1 - current_idx
            if len(lane_array) == 2
            else (current_idx + 1) % len(lane_array)
        )
        return float(lane_array[other_idx])

    def _apply_front_safety(
        self,
        idx: int,
        lane_center_y: float,
        xs: np.ndarray,
        ys: np.ndarray,
        speeds: np.ndarray,
        next_speed: float,
    ) -> float:
        in_lane_mask = np.abs(ys - lane_center_y) <= 35.0
        ahead_mask = xs > xs[idx]
        candidate_indices = np.flatnonzero(in_lane_mask & ahead_mask)
        if candidate_indices.size == 0:
            return max(0.0, next_speed)

        gaps = xs[candidate_indices] - xs[idx]
        nearest_local_idx = int(np.argmin(gaps))
        front_idx = int(candidate_indices[nearest_local_idx])
        front_gap = float(gaps[nearest_local_idx])
        front_speed = float(speeds[front_idx])

        hard_buffer = 12.0
        soft_buffer = 20.0 + max(0.0, float(speeds[idx])) * 0.25
        if front_gap <= hard_buffer:
            return 0.0

        safe_cap = front_speed + max(0.0, (front_gap - soft_buffer) * 0.45)
        return max(0.0, min(next_speed, safe_cap))

    def plan(self, scheduler_input: SchedulerInput, dt: float) -> SchedulerOutput:
        if dt <= 0.0:
            return SchedulerOutput()
        accel_up_limit = self.max_accel_per_step / dt
        accel_down_limit = self.max_decel_per_step / dt
        intercept_accel_up_limit = self.intercept_max_accel_per_step / dt
        intercept_accel_down_limit = self.intercept_max_decel_per_step / dt

        commands: dict[str, VehicleCommand] = {}
        trajectories: dict[str, list[TrajectoryPoint]] = {}
        vehicles = list(scheduler_input.vehicles.values())
        if not vehicles:
            return SchedulerOutput()

        vehicle_ids = np.asarray(
            [vehicle.vehicle_id for vehicle in vehicles], dtype=object
        )
        xs = np.asarray([vehicle.x for vehicle in vehicles], dtype=np.float64)
        ys = np.asarray([vehicle.y for vehicle in vehicles], dtype=np.float64)
        speeds = np.asarray([vehicle.speed for vehicle in vehicles], dtype=np.float64)
        desired_speeds = np.asarray(
            [vehicle.desired_speed for vehicle in vehicles], dtype=np.float64
        )
        manual_flags = np.asarray(
            [vehicle.is_manual for vehicle in vehicles], dtype=bool
        )
        bg_indices = np.flatnonzero(~manual_flags)
        sorted_bg_indices = (
            bg_indices[np.argsort(xs[bg_indices])]
            if bg_indices.size > 0
            else np.asarray([], dtype=int)
        )
        bg_rank_map = {
            int(vehicle_idx): rank
            for rank, vehicle_idx in enumerate(sorted_bg_indices.tolist())
        }
        ego_vehicle_id = scheduler_input.ego_vehicle_id
        ego_idx = -1
        for idx, vehicle in enumerate(vehicles):
            if vehicle.vehicle_id == ego_vehicle_id:
                ego_idx = idx
                break
        if ego_idx < 0:
            manual_indices = np.flatnonzero(manual_flags)
            if manual_indices.size > 0:
                ego_idx = int(manual_indices[0])

        ego_x = float(xs[ego_idx]) if ego_idx >= 0 else 0.0
        ego_y = (
            float(ys[ego_idx])
            if ego_idx >= 0
            else self._nearest_lane_center(0.0, scheduler_input.lane_centers)
        )
        ego_speed = float(speeds[ego_idx]) if ego_idx >= 0 else 0.0
        ego_lane = self._nearest_lane_center(ego_y, scheduler_input.lane_centers)
        pass_lane = self._other_lane_center(scheduler_input.lane_centers, ego_lane)

        for idx, vehicle in enumerate(vehicles):
            vehicle_id = str(vehicle_ids[idx])
            if manual_flags[idx]:
                commands[vehicle_id] = VehicleCommand(
                    vehicle_id=vehicle_id,
                    action="manual",
                    acceleration=0.0,
                    change_lane=False,
                    target_lane_y=float(ys[idx]),
                )
                continue

            bg_rank = bg_rank_map.get(idx, 0)
            lane_center_y = ego_lane
            action = "follow"

            if scheduler_input.behavior_mode == "intercept_run":
                desired_front_x = ego_x + 36.0 + bg_rank * 16.0
                slot_error = desired_front_x - float(xs[idx])
                if float(xs[idx]) < ego_x + 18.0:
                    action = "intercept_run"
                    lane_center_y = pass_lane
                    target_speed = max(
                        ego_speed + 45.0,
                        float(desired_speeds[idx]) + 35.0,
                        float(speeds[idx]),
                    )
                    target_speed += max(0.0, slot_error) * 0.25
                else:
                    action = "intercept_merge"
                    lane_center_y = ego_lane
                    target_speed = (
                        ego_speed + 12.0 + np.clip(slot_error * 0.20, -10.0, 20.0)
                    )
                target_speed = float(
                    np.clip(target_speed, 0.0, self.intercept_speed_upper)
                )
                speed_delta = target_speed - float(speeds[idx])
                speed_delta = float(
                    np.clip(
                        speed_delta,
                        -self.intercept_max_decel_per_step,
                        self.intercept_max_accel_per_step,
                    )
                )
                tentative_next_speed = max(0.0, float(speeds[idx]) + speed_delta)
            elif scheduler_input.behavior_mode == "intercept_hold":
                action = "intercept_hold"
                lane_center_y = ego_lane
                desired_front_x = ego_x + 32.0 + bg_rank * 14.0
                slot_error = desired_front_x - float(xs[idx])
                target_speed = ego_speed + 4.0 + np.clip(slot_error * 0.16, -8.0, 16.0)
                target_speed = float(
                    np.clip(target_speed, 0.0, self.intercept_speed_upper)
                )
                speed_delta = target_speed - float(speeds[idx])
                speed_delta = float(
                    np.clip(
                        speed_delta,
                        -self.intercept_max_decel_per_step,
                        self.intercept_max_accel_per_step,
                    )
                )
                tentative_next_speed = max(0.0, float(speeds[idx]) + speed_delta)
            else:
                desired_slot_x = ego_x - (self.follow_base_gap + bg_rank * 14.0)
                slot_error = desired_slot_x - float(xs[idx])
                relative_speed = ego_speed - float(speeds[idx])
                ahead_of_ego = float(xs[idx]) > ego_x + 6.0
                on_side_lane = abs(float(ys[idx]) - pass_lane) <= 20.0
                retreating = ahead_of_ego or (
                    on_side_lane and float(xs[idx]) > desired_slot_x + 3.0
                )
                if retreating:
                    action = "follow_retreat"
                    lane_center_y = pass_lane
                    retreat_bias = -3.0 if ahead_of_ego else -1.0
                else:
                    retreat_bias = 0.0
                follow_acc = 0.24 * slot_error + 0.70 * relative_speed + retreat_bias
                follow_acc = max(-accel_down_limit, min(accel_up_limit, follow_acc))
                tentative_next_speed = max(0.0, float(speeds[idx]) + follow_acc * dt)

            tentative_next_speed = self._apply_front_safety(
                idx=idx,
                lane_center_y=lane_center_y,
                xs=xs,
                ys=ys,
                speeds=speeds,
                next_speed=tentative_next_speed,
            )
            is_intercept_action = action.startswith("intercept_")
            if is_intercept_action:
                min_speed = max(
                    0.0, float(speeds[idx]) - self.intercept_max_decel_per_step
                )
                max_speed = float(speeds[idx]) + self.intercept_max_accel_per_step
            else:
                min_speed = max(0.0, float(speeds[idx]) - self.max_decel_per_step)
                max_speed = float(speeds[idx]) + self.max_accel_per_step
            next_speed = min(max(tentative_next_speed, min_speed), max_speed)
            if is_intercept_action:
                next_speed = min(next_speed, self.intercept_speed_upper)
            next_speed = float(round(next_speed))
            acceleration = (next_speed - float(speeds[idx])) / dt
            if is_intercept_action:
                acceleration = max(
                    -intercept_accel_down_limit,
                    min(intercept_accel_up_limit, acceleration),
                )
            else:
                acceleration = max(-accel_down_limit, min(accel_up_limit, acceleration))
            next_speed = max(0.0, float(speeds[idx]) + acceleration * dt)
            change_lane = abs(lane_center_y - float(ys[idx])) > 2.0

            points: list[TrajectoryPoint] = []
            for step in range(1, self.horizon_steps + 1):
                offset = step * dt
                points.append(
                    TrajectoryPoint(
                        x=float(xs[idx]) + next_speed * offset,
                        y=lane_center_y,
                        yaw=vehicle.yaw,
                        speed=next_speed,
                        time_offset=offset,
                    )
                )
            trajectories[vehicle_id] = points
            commands[vehicle_id] = VehicleCommand(
                vehicle_id=vehicle_id,
                action=action,
                acceleration=acceleration,
                change_lane=change_lane,
                target_lane_y=lane_center_y if change_lane else None,
            )

        return SchedulerOutput(commands=commands, trajectories=trajectories)
