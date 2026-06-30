# SYMROS: Enabling Symbolic Execution for ROS 2 Robotic Systems

SYMROS performs compilation-based symbolic execution on ROS 2 nodes: it builds the
target node with an instrumented compiler (SymCC) against instrumented C++
libraries, drives the node through its lifecycle / entry code, and explores its
input space to construct test drivers, run symbolic execution, and perform
bug-guided input space exploration.

## Repository layout

| Path | Description |
|------|-------------|
| `SYMROS/` | Main symbolic execution engine (driver `symros.py`, SymCC compiler, `ros-nodds` runtime, build scripts). |
| `workspace/` | Test targets, each built into a concrete / symbolic / recovery workspace (`conc_ws`, `sym_ws`, `chk_ws`). |
| `external/` | Third-party libraries supporting compilation-based symbolic execution (installed builds: `gm` = GraphicsMagick, `geographiclib` = GeographicLib). |
| `libcxx_clang_install/`, `libcxx_symros_install/` | C++ standard libraries instrumented by SymCC. |
| `SYMROS/samples/` | Pre-provided samples to reproduce the SYMROS evaluation results. |

### Test targets (`workspace/`)

- navigation2
- robot_localization
- slam_toolbox

## Getting started

1. Set up the environment (compiler, instrumented libc++, ROS runtime, system
   packages): see [`SETUP.md`](SETUP.md).
2. Build the three workspaces:

   ```bash
   cd SYMROS
   ./scripts/conc_build.bash ../workspace/conc_ws
   ./scripts/sym_build.bash ../workspace/sym_ws
   ./scripts/chk_build.bash ../workspace/chk_ws
   ```

## Reproducing evaluation results

Each evaluated package ships a step-by-step procedure (configuration, construct
test driver, symbolic execution, bug-guided input space exploration) under
`SYMROS/samples/<package>/README.md`:

- [`nav2_amcl`](SYMROS/samples/nav2_amcl/README.md)
- [`nav2_planner`](SYMROS/samples/nav2_planner/README.md)
- [`nav2_controller`](SYMROS/samples/nav2_controller/README.md)
- [`nav2_behaviors`](SYMROS/samples/nav2_behaviors/README.md)
- [`nav2_bt_navigator`](SYMROS/samples/nav2_bt_navigator/README.md)
- [`slam_toolbox`](SYMROS/samples/slam_toolbox/README.md)
- [`robot_localization`](SYMROS/samples/robot_localization/README.md)
