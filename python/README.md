# threed-mouse

Python utilities for working with 3D mice, including SpaceMouse-style devices and teleoperation helpers for robotics workflows.

## Install

```bash
pip install threed-mouse
```

## Basic usage

```python
from threed_mouse import ThreeDMouse

mouse = ThreeDMouse()
mouse.run()

state = mouse.get_controller_state()
print(state)

mouse.close()
```

## Project

Source, issues, and demos live in the main repository:

- https://github.com/hcrlab/3dmouse-hid
