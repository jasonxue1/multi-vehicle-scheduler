from dataclasses import dataclass, field


@dataclass(slots=True)
class VehicleObservation:
    vehicle_id: str
    x: float
    y: float
    yaw: float = 0.0
    speed: int = 0
    desired_speed: int = 0
    is_manual: bool = False


@dataclass(slots=True)
class TrajectoryPoint:
    x: float
    y: float
    yaw: float
    speed: float
    time_offset: float


@dataclass(slots=True)
class SchedulerOutput:
    commands: dict[str, "VehicleCommand"] = field(default_factory=dict)
    trajectories: dict[str, list[TrajectoryPoint]] = field(default_factory=dict)


@dataclass(slots=True)
class SchedulerInput:
    sim_time: float = 0.0
    lane_centers: tuple[float, ...] = ()
    ego_vehicle_id: str = "ego"
    behavior_mode: str = "follow"
    behavior_elapsed: float = 0.0
    vehicles: dict[str, VehicleObservation] = field(default_factory=dict)


@dataclass(slots=True)
class VehicleCommand:
    vehicle_id: str
    action: str
    acceleration: float
    change_lane: bool
    target_lane_y: float | None = None
