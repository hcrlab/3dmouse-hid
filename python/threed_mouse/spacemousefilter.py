import numpy as np

EPS = np.finfo(float).eps


class SpaceMouseFilter:

    def __init__(self,
        smoothing_factor,
        softmax_temp,
        translation_modifier,
        rotation_modifer,
        translation_deadband,
        rotation_deadband,
        translation_enabled,
        rotation_enabled) -> None:
        self._world = None
        self._device = None
        self.spacemouse_prim = None
        self.smoothing_factor = smoothing_factor
        self.softmax_temp = softmax_temp
        self.rotation_modifier = rotation_modifer
        self.translation_modifier = translation_modifier
        self.rotation_deadband = rotation_deadband
        self.translation_deadband = translation_deadband
        self.translation_enabled = translation_enabled
        self.rotation_enabled = rotation_enabled

        self.prev_trans = np.array((0.,0.,0.))
        self.prev_rot = np.array((0.,0.,0.))

    def _rotation_modifier(self, rot):
        if not self.rotation_enabled:
            rot[:] = 0
            self.prev_rot[:] = 0
            return

        rot[np.abs(rot) < self.rotation_deadband] = 0

        magnitude = np.linalg.norm(rot)
        # Not sure if you can actuall max out multiple rotation dimensions at once,
        # but we're going to limit you to 1 anyways.
        # This helps avoid the case where you've maxed out one axis, then adding
        # small input to another axis increases the magnitude, but softmax
        # puts this mass straight back into the axis you've already maxed out.
        magnitude = min(magnitude, 1.0)
        if magnitude == 0:
            rot[:] = (1 - self.smoothing_factor) * rot + self.smoothing_factor * self.prev_rot
            rot[np.abs(rot) < EPS] = 0
            self.prev_rot[:] = rot[:]
            return

        rot_exp = np.exp(np.abs(rot) / self.softmax_temp)
        softmax = rot_exp / np.sum(rot_exp)
        rot[:] *= softmax # redistribute
        rot[:] *= magnitude / np.linalg.norm(rot) # renormalize to original scale
        rot[:] *= self.rotation_modifier # apply user scale factor

        rot[:] = (1 - self.smoothing_factor) * rot + self.smoothing_factor * self.prev_rot
        rot[np.abs(rot) < EPS] = 0
        self.prev_rot[:] = rot[:]

    def _translation_modifier(self, trans):
        if not self.translation_enabled:
            trans[:] = 0
            self.prev_trans[:] = 0
            return

        # Cut small values completely
        trans[np.abs(trans) < self.translation_deadband] = 0

        magnitude = np.linalg.norm(trans)
        # You _can_ max out multiple dimensions at once (X+Z or Y+Z) which would allow
        # you a total input magnitude of 2, but we're going to limit you to 1.
        # This helps avoid the case where you've maxed out one axis, then adding
        # small input to another axis increases the magnitude, but softmax
        # puts this mass straight back into the axis you've already maxed out.
        magnitude = min(magnitude, 1.0)
        if magnitude == 0:
            trans[:] = (1 - self.smoothing_factor) * trans + self.smoothing_factor * self.prev_trans
            trans[np.abs(trans) < EPS] = 0
            self.prev_trans[:] = trans[:]
            return

        # Redistribute mass, smoothly favoring the stronger components
        trans_exp = np.exp(np.abs(trans) / self.softmax_temp)
        softmax = trans_exp / np.sum(trans_exp)
        trans[:] *= softmax # redistribute
        trans[:] *= magnitude / np.linalg.norm(trans) # renormalize to original scale
        trans[:] *= self.translation_modifier # apply user scale factor

        trans[:] = (1 - self.smoothing_factor) * trans + self.smoothing_factor * self.prev_trans
        trans[np.abs(trans) < EPS] = 0
        self.prev_trans[:] = trans[:]
