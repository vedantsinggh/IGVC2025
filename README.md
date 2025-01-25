# IGVC Autonav 2025

This repository contains the ROS2 workspace for the IGVC 2025 competition.

## Repository Structure
- `bot_ws/`: ROS2 workspace containing all packages.
- `Dockerfile`: Docker configuration for ROS2.
- `docker-compose.yml`: Multi-container setup.
- `scripts/`: Helper scripts for building and running the project.

## Prerequisites
- Docker
- Docker Compose

## Building the Docker Image
```bash
docker build -t igvc_ros2 .
```

## Running the container
```bash
docker run -it --rm --net=host igvc_ros2
```

## Using Docker Compose
```bash
docker-compose up
```
