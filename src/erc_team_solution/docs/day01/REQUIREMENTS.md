# ERC Phase 1 Requirements Baseline

## Evaluation launch contract

```bash
ros2 launch erc_team_solution solution.launch.py \
  shelf_column_number:=2 \
  book_colour:=red
```

## Required inputs

- shelf_column_number: integer from 1 to 5
- book_colour: red, green, blue, or yellow

## Required scored outputs

- /erc/shelf_column_identification
- /erc/shelf_row_identification
- timestamped annotated shelf-column image
- timestamped annotated target-book image

## Maximum trial score

- Column topic: 1
- Column annotated image: 2
- Correct shelf navigation: 3
- Row topic: 1
- Book annotated image: 2
- Grasp: 3
- Return with book: 3
- Gentle placement: 4
- Maximum: 19
- Collision: -0.5 per event

## Day 1 acceptance

- [ ] Native Ubuntu 22.04 confirmed
- [ ] Docker works without sudo
- [ ] Docker image builds
- [ ] Workspace builds
- [ ] Gazebo starts
- [ ] Robot spawns
- [ ] Twenty books spawn
- [ ] Five number markers spawn
- [ ] RGB camera publishes
- [ ] Depth camera publishes
- [ ] Front and rear LiDAR publish
- [ ] Seven controllers are active
- [ ] Base responds to a low-speed command
- [ ] Head responds
- [ ] Torso responds
- [ ] Left gripper responds
- [ ] Team package builds
- [ ] Required launch arguments work
- [ ] Invalid inputs fail clearly
- [ ] Pull request reviewed
