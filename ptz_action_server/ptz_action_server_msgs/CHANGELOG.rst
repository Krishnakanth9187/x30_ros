^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ptz_action_server_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.3 (2024-11-01)
------------------
* Added new general Ptz message (`#6 <https://github.com/clearpathrobotics/ptz_action_server/issues/6>`_)
* Contributors: Jose Mastrangelo

2.0.2 (2024-05-24)
------------------
* Add dependency on action_msgs
* Contributors: Chris Iverach-Brereton

2.0.1 (2024-05-23)
------------------

2.0.0 (2024-05-23)
------------------
* Initial release for ROS 2
* Contributors: Chris Iverach-Brereton, Michael Hosmar

0.1.9 (2024-01-09)
------------------

0.1.8 (2024-01-08)
------------------

0.1.7 (2023-11-22)
------------------

0.1.6 (2023-11-15)
------------------
* Major overhaul to add new control modes (`#4 <https://github.com/clearpathrobotics/ptz_action_server/issues/4>`_)
  * Rename PtzPosition.msg to PtzState.msg, add support for velocity control. Rename .../move_ptz action to .../position, add .../velocity to Axis camera to allow velocity control
  * Add support for move_ptz/position_abs and move_ptz/position_rel actions
  * Update the readme
  * Add support for absolute & relative positions for the flir PTU
  * Move the axis server's actions into the move_ptz namespace, add position_abs and position_rel
  * Update the limits config files, readme for the axis action server
  * Keep all nodes called "ptz_action_server_node" for all nodes by default to make searching params easier
  * Make the ROS parameters consistent across all implementations. Add limits file for the simulation node
  * Publish the sign of the direction of travel in velocity control feedback. Check if the camera has reached end-of-travel and signal the end of the velocity control action if this occurs
  * Debug the new action servers on actual hardware
  * Add the first-pass at supporting pan/tilt angles relative to an external frame. Not yet supported by the Flir PTU
  * Create a temporary vector frame we can use to calculate the necessary pan/tilt angles, rather than ignoring the tilt. This should allow the camera to operate correctly even on hills. Temporary frame has a UUID in the name so it shouldn't conflict with other frames in the robot
  * Modify the axis & simulation packages to use positive pan = clockwise instead of anticlockwise. Add a new action to pan/tilt/zoom to a specific frame
  * Fix message dependencies
  * Implement the new action to point the camera AT a specific frame, instead of just parallel with its X axis
* Contributors: Chris Iverach-Brereton

0.1.5 (2023-10-19)
------------------
* Changes
* Contributors: Chris Iverach-Brereton

0.1.4 (2023-08-18)
------------------

0.1.3 (2023-08-04)
------------------

0.1.2 (2023-05-05)
------------------

0.1.1 (2023-04-11)
------------------

0.1.0 (2023-03-27)
------------------
* Initial release
* Contributors: Chris Iverach-Brereton
