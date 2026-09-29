"""sidewalk_bringup — shared Python library for the sidewalk VSLAM project.

Installed as a real Python package via catkin_python_setup(), so that sibling
packages can `from sidewalk_bringup.run_logger import RunLogger` and have it
work identically from a source workspace and from an install space.

Before this existed, siblings reached into sidewalk_bringup's scripts/ folder
by absolute path. That works when running from source and silently fails after
a catkin install, because catkin puts scripts in lib/<pkg>/ rather than on the
Python path — so the logging would quietly degrade to a fallback stub and the
project-wide RUNLOG.md index would stop being written.
"""
