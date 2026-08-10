"""Gazebo Harmonic simulation bridge for the self-balancing wheeled biped.

Modules
-------
standing_mode
    Drives the 5-bar leg linkages to the calibrated stance when the robot is
    armed, and holds the fallen posture otherwise.
rc_teleop
    Maps ``/cmd_vel`` onto the balance controller's velocity target and turn
    bias, reproducing the FlySky RC channel scaling.
wireless_gui_bridge
    Serves the STM32 firmware's ASCII tuning protocol over TCP so the existing
    wireless GUI can drive the simulated robot unchanged.

The balance controller itself is the C++ node ``balance_point``.
"""

__version__ = "1.0.0"
