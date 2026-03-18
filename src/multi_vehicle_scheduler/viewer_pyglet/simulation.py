from multi_vehicle_scheduler.scheduler_backend import (
    SchedulerInput,
    SchedulerOutput,
    VehicleObservation,
)

from .models import WorldState


def build_scheduler_input(
    world_state: WorldState, lane_centers: tuple[float, ...]
) -> SchedulerInput:
    ego_vehicle_id = "ego"
    for vehicle in world_state.vehicles.values():
        if vehicle.is_ego:
            ego_vehicle_id = vehicle.vehicle_id
            break

    vehicles = {
        vehicle_id: VehicleObservation(
            vehicle_id=vehicle.vehicle_id,
            x=vehicle.x,
            y=vehicle.y,
            yaw=vehicle.yaw,
            speed=vehicle.speed,
            desired_speed=vehicle.desired_speed,
            is_manual=vehicle.is_ego,
        )
        for vehicle_id, vehicle in world_state.vehicles.items()
    }
    return SchedulerInput(
        sim_time=world_state.sim_time,
        lane_centers=lane_centers,
        ego_vehicle_id=ego_vehicle_id,
        vehicles=vehicles,
    )


def build_scheduler_input_with_mode(
    world_state: WorldState,
    lane_centers: tuple[float, ...],
    behavior_mode: str,
    behavior_elapsed: float,
) -> SchedulerInput:
    scheduler_input = build_scheduler_input(world_state, lane_centers)
    scheduler_input.behavior_mode = behavior_mode
    scheduler_input.behavior_elapsed = behavior_elapsed
    return scheduler_input


def apply_scheduler_output(
    world_state: WorldState,
    scheduler_output: SchedulerOutput,
    dt: float,
    lateral_speed: float = 180.0,
    min_longitudinal_gap: float = 26.0,
) -> None:
    if dt <= 0.0:
        return

    for vehicle_id, vehicle in world_state.vehicles.items():
        command = scheduler_output.commands.get(vehicle_id)
        if command is None:
            vehicle.x += vehicle.speed * dt
            continue

        if command.action == "manual":
            continue

        vehicle.speed = int(max(0, round(vehicle.speed + command.acceleration * dt)))
        if command.change_lane and command.target_lane_y is not None:
            vehicle.lane_target_y = command.target_lane_y
        vehicle.x += vehicle.speed * dt

    max_lateral_step = lateral_speed * dt
    for vehicle in world_state.vehicles.values():
        if vehicle.is_ego or vehicle.lane_target_y is None:
            continue
        dy = vehicle.lane_target_y - vehicle.y
        if abs(dy) <= max_lateral_step:
            vehicle.y = vehicle.lane_target_y
        else:
            vehicle.y += max_lateral_step if dy > 0 else -max_lateral_step

    lane_groups: dict[int, list] = {}
    for vehicle in world_state.vehicles.values():
        lane_key = int(round(vehicle.y / 20.0))
        lane_groups.setdefault(lane_key, []).append(vehicle)

    for lane_vehicles in lane_groups.values():
        lane_vehicles.sort(key=lambda item: item.x, reverse=True)
        for idx in range(len(lane_vehicles) - 1):
            front = lane_vehicles[idx]
            back = lane_vehicles[idx + 1]
            gap = front.x - back.x
            if gap < min_longitudinal_gap:
                # Keep ego fully manual: do not apply anti-collision correction on ego vehicle.
                if back.is_ego:
                    continue
                back.x = front.x - min_longitudinal_gap
                if back.speed > front.speed:
                    back.speed = int(front.speed)

    world_state.sim_time += dt
