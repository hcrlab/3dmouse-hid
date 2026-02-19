from dataclasses import dataclass

import pyspacemouse
import numpy as np
from pyspacemouse import DeviceInfo


@dataclass(frozen=True, slots=True)
class SpaceMouseData:
    t: float
    xyz: np.ndarray
    rpy: np.ndarray
    buttons: np.ndarray

# Use pyspacemouse specs directly as the source of truth.
DEVICE_SPECS: dict[str, DeviceInfo] = pyspacemouse.get_device_specs()
DEVICE_NAMES = list(DEVICE_SPECS.keys())
