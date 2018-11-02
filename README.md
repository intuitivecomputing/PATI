# PATI
## Introduction
This is the repository accompanying the paper *PATI: A Projection-Based Augmented Table-Top Interface for Robot Programming*. In this work, we present PATI—a Projection-based Augmented Table-top Interface for robot programming—through which users are able to use sim- ple, common gestures (e.g., pinch gestures) and tools (e.g., shape tools) to specify table-top manipulation tasks (such as pick-and-place) for a robot manipulator. PATI allows users to interact with the environment directly when providing task specifications.

## Usage
To launch kinect and server for Unity communication
`roslaunch ropi_tangible_surface all_in_one.launch`

To launch main program:
`rosrun ropi_tangible_surface tangible_surface.py`

## Structure
- ropi_msgs: ROS messages, services and actions
- ropi_tangible_surface: main package
    - ropi_tangible_surface: python package for backend processing
        - base_class.py: base class of all detection and tracking clsses.
        - finger_detection.py: detect fingertip positions
        - fingertip_tracking.py: filter and track detections
        - object_detection.py: detect tabletop objects
        - object_tracking.py: filter and track object detections
    - tangible_surface.py: main script