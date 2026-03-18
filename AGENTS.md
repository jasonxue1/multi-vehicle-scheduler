# Repository Guidelines

## Project Structure & Module Organization

This repository currently uses a single Python package layout:

- `src/multi_vehicle_scheduler/`: main package root.
- `src/multi_vehicle_scheduler/scheduler_backend/`: scheduling/backend logic.
- `src/multi_vehicle_scheduler/viewer_pyglet/`: `pyglet` viewer entrypoint and UI-side code.
- `pyproject.toml`: project metadata and script entrypoints.
- `uv.lock`: locked dependency set for reproducible environments.

Keep new code inside `src/multi_vehicle_scheduler/<module_name>/`.

## Build, Test, and Development Commands

Use `uv` for all local workflows:

- `uv sync`: create/update the virtual environment from `pyproject.toml` and `uv.lock`.
- `uv run pyglet`: run the configured viewer script (`multi_vehicle_scheduler.viewer_pyglet:main`).
- `uv build`: build source/wheel distributions.

## Coding Style & Naming Conventions

- Follow PEP 8 with 4-space indentation.
- Prefer explicit type hints on public functions and interfaces.
- Module/function names: `snake_case`; class names: `PascalCase`; constants: `UPPER_SNAKE_CASE`.
- Keep backend logic deterministic and side-effect-light; isolate rendering concerns in `viewer_pyglet`.
- Write small, composable functions instead of large monolithic loops.

## Commit Guidelines

This branch currently has no commit history, so use Conventional Commits moving forward:

- `feat: add lane-priority conflict resolver`
- `fix: prevent negative dt in replay clock`
- `chore: update uv lockfile`

## Security & Configuration Tips

- Do not commit secrets, tokens, or device credentials.
- Keep environment-specific settings out of source; prefer local config files ignored by Git.
