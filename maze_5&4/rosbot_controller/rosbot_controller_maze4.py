#!/usr/bin/env python3
"""Webots controller entry point.

Keep this file named `rosbot_controller.py` in the Webots controller folder.
Importing `mission_runtime` starts the unchanged mission loop.
"""

import mission_runtime  # noqa: F401  (runs the controller loop)
# this file is the main gateway to the controller, importing the mission runtime (which is the main loop file)