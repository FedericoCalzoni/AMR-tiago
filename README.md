# AMR - Project

## Setup without docker using a compatible ubuntu distro

- run the setup.sh script:
  
  ```bash
  bash setup.sh
  ```

- rebuild ros2_ws and tiago_ws:
  
  ```bash
  rebuild-ws
  ```

- rebuild exam_ws:
  
  ```bash
  rebuild-AMR-tiago
  ```
  

## Run the simulation

### Mapping

```bash
run-gazebo
run-rviz-slam
ros2 run tiago_task_manager mapping_manager
```

Altenratively:

```bash
ros2 launch tiago_task_manager mapping.launch.py
```

### Pick & Place

```bash
ros2 launch tiago_task_manager pick_and_place.launch.py
```

Altenratively:

```bash
run-gazebo
run-rviz
ros2 run tiago_task_manager task_manager
```
