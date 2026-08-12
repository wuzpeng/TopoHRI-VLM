# TopoHRI-VLM

**TopoHRI-VLM: Topology-Aware Human–AI Collaboration for Multi-Robot Semantic Search**

TopoHRI-VLM is a ROS-based multi-robot semantic search framework for initially unknown indoor environments. It integrates topology-aware frontier exploration, event-driven vision-language model (VLM) reasoning, human–AI interaction, and heterogeneous multi-robot coordination.

The current system is developed and evaluated with **two UAVs and one UGV** in Gazebo.

## Features

- **Topology-aware frontier exploration** for reducing redundant multi-robot search.
- **Hierarchical VLM reasoning** with Local-VLM perception and Central-VLM task coordination.
- **Event-driven decision making** for efficient semantic search and target verification.
- **Human–AI interaction** supporting target-query updates and human-priority search regions.
- **Heterogeneous UAV–UGV coordination** with robot-specific accessibility constraints.
- **Multi-view target verification** and semantic evidence fusion.
- Four experimental environments:
  - H1: corridor and side-room environment
  - H2: ring-structured environment
  - H3: dense-obstacle open environment
  - H4: multi-branch environment
- Automatic logging of search success, task time, team travel distance, and map coverage.

## Environment

The project has been developed with:

- Ubuntu 20.04
- ROS Noetic
- Gazebo Classic
- PX4 SITL
- MAVROS
- Python 3

## Build

Clone the repository:

```bash
git clone https://github.com/wuzpeng/TopoHRI-VLM.git
cd TopoHRI-VLM
```

Build the catkin workspace:

```bash
catkin_make
source devel/setup.bash
```

## Run

For example, to run the H4 experiment:

### Terminal 1: Start TopoHRI-VLM

```bash
source devel/setup.bash

roslaunch uav_semantic_search human_ai_vlm_stage5.launch \
  experiment_config:=$(rospack find uav_semantic_search)/config/experiments/h4.yaml
```

### Terminal 2: Start PX4 UAVs

```bash
source devel/setup.bash

rosrun uav_semantic_search run_px4_two_uav.sh
```

### Terminal 3: Start Frontier Extraction

```bash
source devel/setup.bash

rosrun uav_semantic_search frontier_extractor.py
```

The experiment configuration can be changed to:

```text
config/experiments/h1.yaml
config/experiments/h2.yaml
config/experiments/h3.yaml
config/experiments/h4.yaml
```

## Project Structure

```text
TopoHRI-VLM/
├── src/
│   ├── uav_semantic_search/
│   │   ├── config/          # System and experiment configurations
│   │   ├── docs/            # Detailed documentation
│   │   ├── launch/          # ROS launch files
│   │   ├── models/          # Simulation models
│   │   ├── msg/             # Custom ROS messages
│   │   ├── rviz/            # RViz configurations
│   │   ├── scripts/         # Search, VLM, HRI, planning and control nodes
│   │   └── worlds/          # Gazebo experimental environments
│   └── go2_description/     # UGV robot description
└── README.md
```

## Documentation

More detailed installation, configuration, and experiment instructions are available in:

```text
src/uav_semantic_search/docs/
```

## Status

This repository contains the research implementation of **TopoHRI-VLM** and is currently under active development.
