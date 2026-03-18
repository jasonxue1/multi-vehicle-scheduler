from dataclasses import dataclass, field


@dataclass(slots=True)
class VehicleState:
    vehicle_id: str
    x: float
    y: float
    yaw: float = 0.0
    speed: int = 0
    desired_speed: int = 0
    is_ego: bool = False
    lane_target_y: float | None = None
    color: tuple[int, int, int] = (60, 170, 255)


@dataclass(slots=True)
class WorldState:
    sim_time: float = 0.0
    vehicles: dict[str, VehicleState] = field(default_factory=dict)


@dataclass(slots=True)
class ScenarioConfig:
    name: str
    map_width: int = 1200
    map_height: int = 720
    road_y_min: float = 190.0
    road_y_max: float = 530.0
    lane_center_y: tuple[float, float] = (280.0, 440.0)
