from __future__ import annotations

from pathlib import Path

from .models import ScenarioConfig, VehicleState, WorldState
from .scenario_schema import ScenarioPayload

DEFAULT_SCENARIO_PATH = (
    Path(__file__).resolve().parent / "scenarios" / "bootstrap_city_block.json"
)
DEFAULT_SCENARIO_PAYLOAD = {
    "name": "bootstrap_city_block",
    "map_width": 1200,
    "map_height": 720,
    "road_y_min": 190,
    "road_y_max": 530,
    "lane_center_y": [280, 440],
    "vehicles": [
        {
            "vehicle_id": "ego",
            "x": 120,
            "y": 280,
            "yaw": 0,
            "speed": 60,
            "desired_speed": 60,
            "is_ego": True,
            "color": [255, 70, 70],
        },
        {
            "vehicle_id": "bg_01",
            "x": 40,
            "y": 280,
            "yaw": 0,
            "speed": 58,
            "desired_speed": 58,
            "color": [80, 150, 255],
        },
        {
            "vehicle_id": "bg_02",
            "x": 0,
            "y": 440,
            "yaw": 0,
            "speed": 68,
            "desired_speed": 68,
            "color": [80, 150, 255],
        },
    ],
}


def load_scenario(path: str | Path | None = None) -> tuple[ScenarioConfig, WorldState]:
    scenario_path = Path(path) if path else DEFAULT_SCENARIO_PATH
    if scenario_path.exists():
        payload = ScenarioPayload.model_validate_json(
            scenario_path.read_text(encoding="utf-8")
        )
    elif path is None:
        payload = ScenarioPayload.model_validate(DEFAULT_SCENARIO_PAYLOAD)
    else:
        raise FileNotFoundError(f"Scenario file not found: {scenario_path}")

    scenario = ScenarioConfig(
        name=payload.name,
        map_width=payload.map_width,
        map_height=payload.map_height,
        road_y_min=payload.road_y_min,
        road_y_max=payload.road_y_max,
        lane_center_y=tuple(payload.lane_center_y),
    )
    vehicles = {
        item.vehicle_id: VehicleState(
            vehicle_id=item.vehicle_id,
            x=item.x,
            y=item.y,
            yaw=item.yaw,
            speed=item.speed,
            desired_speed=item.desired_speed
            if item.desired_speed is not None
            else item.speed,
            is_ego=item.is_ego,
            lane_target_y=item.y,
            color=item.color,
        )
        for item in payload.vehicles
    }
    world_state = WorldState(sim_time=0.0, vehicles=vehicles)
    return scenario, world_state
