"""Mass-switch variant initialized from whole-robot CoM, with fixed body height.

The underlying approximation and landing planner are inherited unchanged.
"""
import numpy as np
from LIPM.models.LIPM_3D_mass_switch import LIPM3DMassSwitch, DEFAULT_BODY_MASS, DEFAULT_FOOT_MASS


class LIPM3DMassSwitchTotalCoM(LIPM3DMassSwitch):
    def initializeModel(self, COM_pos, left_foot_pos, right_foot_pos):
        """COM_pos is initial whole-robot CoM, unlike the body-based variant."""
        whole = np.asarray(COM_pos, dtype=float)
        feet = np.asarray([left_foot_pos, right_foot_pos], dtype=float)
        if whole.shape != (3,) or feet.shape != (2,3) or not np.isfinite(whole).all() or not np.isfinite(feet).all():
            raise ValueError('Initial positions must be finite 3D vectors')
        if whole[2] <= 0:
            raise ValueError('Initial whole-CoM height must be positive')
        total_mass = self.body_mass+2*self.foot_mass
        body = (total_mass*whole-self.foot_mass*feet.sum(axis=0))/self.body_mass
        super().initializeModel(body, feet[0], feet[1])
        self.initial_whole_com = whole.copy()

    def initializeWholeVelocity(self, velocity):
        velocity = np.asarray(velocity, dtype=float)
        if velocity.shape != (2,) or not np.isfinite(velocity).all():
            raise ValueError('Initial whole-CoM velocity must have two finite components')
        total_mass = self.body_mass+2*self.foot_mass
        self.body_velocity[:2] = (total_mass*velocity-self.foot_mass*self.foot_velocity[:,:2].sum(axis=0))/self.body_mass
        self.resetEffectiveState()

    def snapshot(self):
        record = super().snapshot()
        record['position'] = record['whole_com'][:2].copy()
        record['velocity'] = record['whole_com_velocity'][:2].copy()
        return record
