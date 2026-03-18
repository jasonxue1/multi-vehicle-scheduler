from pydantic import BaseModel, Field


class ScenarioVehicle(BaseModel):
    vehicle_id: str
    x: float
    y: float
    yaw: float = 0.0
    speed: int = 0
    desired_speed: int | None = None
    is_ego: bool = False
    color: tuple[int, int, int] = (60, 170, 255)


class ScenarioPayload(BaseModel):
    name: str
    map_width: int = Field(ge=100)
    map_height: int = Field(ge=100)
    road_y_min: float
    road_y_max: float
    lane_center_y: tuple[float, ...]
    vehicles: list[ScenarioVehicle]
