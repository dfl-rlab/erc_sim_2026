# Day 1 Foundation

Date: 14 August 2026

## Machine role

Primary integration and simulation machine.

## Environment

- Native Ubuntu 22.04 LTS
- x86_64
- Intel i7-1255U
- 32 GB RAM
- Intel integrated graphics
- No NVIDIA GPU
- Docker Engine
- Docker Compose v2
- ROS 2 Humble inside Docker
- Gazebo Harmonic inside Docker
- ROS_DOMAIN_ID 23

## Completed

- Competition repository cloned
- Official upstream remote added
- ERC Docker image built
- ROS workspace built with symlink install
- Gazebo simulation launched
- TIAGo Pro spawned
- Sensor topics inspected
- Controllers inspected
- Low-speed base test completed
- Head, torso, and left gripper tested
- erc_team_solution package created
- Evaluator-compatible solution.launch.py created
- Required arguments validated
- Required scoring publishers created
- Invalid arguments tested

## Day 1 limitations

- No autonomous perception
- No autonomous navigation
- No arm motion
- No grasping
- No bin placement
- No scored outputs published
