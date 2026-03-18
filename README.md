## Multi Vehicle Scheduler (Bootstrap)

### Run

```bash
uv sync
uv run pyglet
```

### Runtime Options

```bash
# load a custom scenario json
uv run pyglet --scenario path/to/scenario.json
```

默认场景文件在：
`src/multi_vehicle_scheduler/viewer_pyglet/scenarios/bootstrap_city_block.json`

### Keyboard Controls

- `Space`: pause/resume
- `N`: single step when paused
- `R`: reset
- `T`: show/hide reference trajectory
- `I`: show/hide vehicle labels
- `Up/Down`: ego speed +/-
- `A/D` or `Left/Right`: ego lane change
- `X`: trigger background intercept event
