# Waypoint Node

This node is responsible for flying to a specific waypoint.

## Internal Logic

The following actions are performed when a new command was received from Mission Control:

1. Wait for `pre_wait_time_ms` time
1. Fly to the cruise height at the current position with the `vertical_speed_mps` speed
1. Fly to the target location at cruise height with the `horizontal_speed_mps` speed
1. Fly to the target height at the current position with the `vertical_speed_mps` speed
1. Wait for `post_wait_time_ms` time

## Communication with Mission Control

Take a look at the [Mission Control Documentation](https://docs.google.com/document/d/1BV6CUhAO0_3A8xje5DsFQydK13K3FuRfkojx6XGCMnk/edit?usp=drive_link).
