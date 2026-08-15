# Day 1 Proposed Architecture

```mermaid
flowchart LR
    RGB[RGB Camera] --> ShelfDetector[Shelf Marker Detector]
    RGB --> ObjectDetector[Book and Bin Detector]
    Depth[Depth Camera] --> ObjectDetector
    Lidar[Front and Rear LiDAR] --> Navigation[Navigation Controller]
    Odom[Odometry] --> Navigation

    ShelfDetector --> Orchestrator[Orchestrator State Machine]
    ObjectDetector --> Orchestrator
    Navigation --> Orchestrator
    Manipulation[Manipulation Controller] --> Orchestrator
    Safety[Safety Monitor] --> Orchestrator

    Orchestrator --> Navigation
    Orchestrator --> Manipulation
    Orchestrator --> Evidence[Evidence and Metrics Logger]

    ShelfDetector --> ShelfTopic[/erc/shelf_column_identification]
    ObjectDetector --> RowTopic[/erc/shelf_row_identification]
    Evidence --> Images[Timestamped Annotated Images]
```

## State machine

1. STARTUP
2. FIND_COLUMN
3. NAVIGATE_TO_SHELF
4. FIND_BOOK
5. GRASP_BOOK
6. RETURN_TO_START
7. FIND_BIN
8. PLACE_BOOK
9. DONE
10. RECOVERY

Every motion state must include a timeout, stop command, verification condition, and bounded retry count.
