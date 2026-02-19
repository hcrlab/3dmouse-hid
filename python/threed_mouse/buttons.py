from typing import Union, Set, Dict, Sequence
import time
import numpy as np

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

        self.ignore = (~self.leading & ~self.trailing) & ((1 << self.num_inputs) - 1)
        self._last_change_per_field = [float("-inf") for _ in range(self.num_inputs)]
        self._debounced_state = np.zeros(self.num_inputs, dtype=int)
        self._last_update_timestamp = None

    def update(self, current_button_state: Sequence[int] | np.ndarray | None) -> np.ndarray:
        """Pass in raw button states, get debounced raw button states.

        Args:
            current_button_state: Sequence of 0/1 (or bool) button values.
        """
        if current_button_state is not None:
            desired_state = np.asarray(current_button_state, dtype=int).reshape(-1)
            if desired_state.size < self.num_inputs:
                desired_state = np.pad(
                    desired_state,
                    (0, self.num_inputs - desired_state.size),
                    mode="constant",
                )
            elif desired_state.size > self.num_inputs:
                desired_state = desired_state[: self.num_inputs]
            desired_state = (desired_state != 0).astype(int)
        else:
            desired_state = self._debounced_state.copy()

        current_stamp = time.time()
        for i in range(self.num_inputs):
            desired = int(desired_state[i])
            current = int(self._debounced_state[i])
            if desired == current:
                continue

            mask = (1 << i)
            if desired == 1:
                edge_allowed = bool(self.leading & mask)
            else:
                edge_allowed = bool(self.trailing & mask)

            if self.ignore & mask:
                edge_allowed = True

            if not edge_allowed:
                continue

            value_stale = (current_stamp - self._last_change_per_field[i]) > self.max_wait
            if value_stale:
                self._debounced_state[i] = desired
                self._last_change_per_field[i] = current_stamp

        self._last_update_timestamp = current_stamp
        return self._debounced_state.copy()
