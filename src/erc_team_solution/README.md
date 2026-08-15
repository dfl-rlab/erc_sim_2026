# ERC Team Solution

Autonomous TIAGo Pro library-assistant solution for the Emirates Robotics Competition 2026.

## Required evaluator command

```bash
ros2 launch erc_team_solution solution.launch.py \
  shelf_column_number:=2 \
  book_colour:=red
```

The valid shelf-column range is 1 through 5.

The valid book colours are:

- red
- green
- blue
- yellow

## Build

```bash
cd /opt/erc_ws
colcon build \
  --symlink-install \
  --packages-select erc_team_solution
source install/setup.bash
```

## Day 1 architecture

The initial package contains one orchestrator node that validates the evaluation arguments and creates the two required scoring publishers.

Planned modular nodes:

1. orchestrator
2. shelf marker detector
3. book and bin detector
4. navigation controller
5. manipulation controller
6. safety monitor
7. trial evidence and metrics logger

No autonomous motion is implemented on Day 1.
