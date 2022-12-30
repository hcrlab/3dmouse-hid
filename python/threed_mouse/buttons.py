from typing import Union, Set, Dict
import time

from srl.spacemouse.device import DEVICE_SPECS


DEVICE_BUTTON_STRUCT_INDICES: Dict[str, Dict[str, int]] = {}

for device_name, spec in DEVICE_SPECS.items():
    # We use the order that the buttons are listed in the spec to define the packing arrangement for a bitfield representation
    # of button state.
    DEVICE_BUTTON_STRUCT_INDICES[device_name] = {name: index for index, (name, _, _, _) in enumerate(spec.button_mapping)}


class ButtonState(list):
    def __int__(self):
        # Button state is a list of bools, so convert to a compact int bitfield representation
        return sum((b << i) for (i, b) in enumerate(self))


class ButtonStateStruct:
    def __init__(self, value: int, name_to_index: Dict[str, int]):
        """
        Args:
            value (int): the packed bitfield representation of the button state
            mapping (List[str]): name of each index in the bitfield
        """
        self.value = value
        self.name_to_index = name_to_index

    def __getitem__(self, name: str) -> bool:
        if name not in self.name_to_index.keys():
            return False
        index = self.name_to_index[name]
        return (self.value >> index) & 1


class SpaceMouseButtonDebouncer:
    """ Limit the number of times a change in state of the SpaceMouse's buttons is reported
        API modeled on lodash's debounce: https://lodash.com/docs#debounce
    """
    def __init__(self, name_to_index: Dict[str, int], leading: Union[bool, Set[str]], trailing: Union[bool, Set[str]], max_wait: float) -> None:
        # NOTE: Asking for both leading and trailing events for the same button will give you an
        # immediate event on the first activation of the button, then silence until the button is released and max_wait
        # seconds has passed. Asking for neither leading nor trailing will pass through the original signal.
        self._leading_preference = leading
        self._trailing_preference = trailing
        self.max_wait = max_wait
        self.num_inputs = len(name_to_index)
        self._current_device_map = name_to_index

        if isinstance(self._leading_preference, set):
            leading_mask = 0
            for name, struct_index in name_to_index.items():
                if name in self._leading_preference:
                    leading_mask |= 1 << struct_index
            self.leading = leading_mask
        else:
            self.leading = -1 if self._leading_preference else 0
        if isinstance(self._trailing_preference, set):
            trailing_mask = 0
            for name, struct_index in name_to_index.items():
                if name in self._trailing_preference:
                    trailing_mask |= 1 << struct_index
            self.trailing = trailing_mask
        else:
            self.trailing = -1 if self._trailing_preference else 0

        self.ignore = (~self.leading & ~self.trailing)

        self._last_change_per_field = [float('-inf') for _ in range(self.num_inputs)]

        self._last_value = 0
        self._debounced_value = 0
        self._last_update_timestamp = None

    def update(self, current_button_state: int):
        """ Pass in the latest available value, get back processed button states.

        Args:
            current_button_state (ButtonStateStruct): _description_

        Returns:
            _type_: _description_
        """
        if current_button_state is not None:
            new_value = current_button_state
        else:
            # Interpret the absence of a new reading as a repeat of the old value
            new_value = self._last_value

        # Which bits flipped
        changed = self._last_value ^ new_value
        rising = changed & new_value
        falling = changed & ~new_value
        should_notify = 0
        passthrough = self.ignore
        current_stamp = time.time()
        for i in range(self.num_inputs):
            mask = (1 << i)
            # User didn't ask for leading or trailing on these bits
            if self.ignore & mask:
                continue
            field_changed = (changed >> i) & 1
            last_change_stamp = self._last_change_per_field[i]
            value_stale = current_stamp - last_change_stamp > self.max_wait
            if field_changed and value_stale:
                # It's been long enough since the last change that we need to report this change
                should_notify |= mask
                self._last_change_per_field[i] = current_stamp

        # Check whether they wanted to know about the kind of change that happened
        should_notify &= (rising & self.leading) | (falling & self.trailing)
        # Apply changes, but mask to those that we determined aren't spurious high frequency input via timestamp
        self._debounced_value = (should_notify & changed)
        # Passthrough bits that we aren't debouncing
        self._debounced_value = (~passthrough & self._debounced_value) | (passthrough & new_value)
        self._last_value = new_value
        self._last_update_timestamp = current_stamp
        # print(f"{changed} {should_notify} {self._debounced_value}")
        return ButtonStateStruct(self._debounced_value, self._current_device_map)