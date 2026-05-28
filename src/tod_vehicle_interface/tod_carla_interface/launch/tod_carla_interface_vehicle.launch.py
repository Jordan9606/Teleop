"""
Empty vehicle launch file for tod_carla_interface.

When vehicleID=carla and mode=vehicle, the TUM launcher loads this file.
The actual CARLA bridging is done by the carla-bridge Docker service
(software-stack-main/bridge/bridge_node.py), which connects to CARLA
via the Python API and publishes/subscribes on the correct TUM topics.

To run CARLA integration:
  1. Start CARLA server:   docker compose -f software-stack-main/docker-compose.yml up carla
  2. Start CARLA bridge:   docker compose -f software-stack-main/docker-compose.yml up carla-bridge
  3. Start TUM operator:   docker compose up tod_operator
  4. Start TUM vehicle:    docker compose up tod_vehicle   (uses this launch file)

Make sure launch_setup.yaml has:
  vehicleID: 'carla'
  mode: 'vehicle'
"""
from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription([])
