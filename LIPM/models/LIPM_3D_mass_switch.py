"""Experimental fixed-height effective-CoM model; not full rigid-body dynamics."""
import numpy as np
from LIPM.models.LIPM_3D_double_support import LIPM3DDoubleSupport


DEFAULT_BODY_MASS = 40.0
DEFAULT_FOOT_MASS = 10.0


class LIPM3DMassSwitch(LIPM3DDoubleSupport):
    def __init__(self, *args, body_mass=DEFAULT_BODY_MASS, foot_mass=DEFAULT_FOOT_MASS, **kwargs):
        if not np.isfinite([body_mass, foot_mass]).all() or body_mass <= 0 or foot_mass < 0:
            raise ValueError('body_mass must be positive and foot_mass nonnegative')
        self.body_mass, self.foot_mass = body_mass, foot_mass
        self.alpha = foot_mass/(body_mass+foot_mass)
        self.foot_velocity = np.zeros((2, 3))
        super().__init__(*args, **kwargs)

    def initializeModel(self, COM_pos, left_foot_pos, right_foot_pos):
        super().initializeModel(COM_pos, left_foot_pos, right_foot_pos)
        self.body_pos = np.array(COM_pos, dtype=float)
        self.body_velocity = np.zeros(3)
        self.body_height = self.body_pos[2]
        self.active_foot = 1 if self.support_leg == 'left_leg' else 0
        self.resetEffectiveState()

    def feet(self):
        return np.array([self.left_foot_pos, self.right_foot_pos])

    def resetEffectiveState(self):
        a = self.alpha
        effective = (1-a)*self.body_pos+a*self.feet()[self.active_foot]
        velocity = (1-a)*self.body_velocity+a*self.foot_velocity[self.active_foot]
        self.zc = effective[2]
        if self.zc <= 0:
            raise ValueError('Effective planning height must be positive')
        self.w_0 = np.sqrt(9.81/self.zc)
        self.x_t, self.y_t = effective[:2]-self.support_foot_pos[:2]
        self.vx_t, self.vy_t = velocity[:2]
        self.x_0, self.y_0 = self.x_t, self.y_t
        self.vx_0, self.vy_0 = self.vx_t, self.vy_t
        self._update_COM_pos()

    def initializePeriodicState(self, theta=0.):
        """Fixed point of two closed-loop steps, including both touchdown resets.

        This initializes the current planner's periodic gait. With finite foot
        mass its achieved speed/width need not equal the nominal references.
        """
        import copy
        if not np.isfinite(theta):
            raise ValueError('theta must be finite')
        if self.phase != 'SSP' or self.phase_count != 0 or self.steps != 0:
            raise ValueError('Periodic initialization is only supported before walking')
        template = copy.deepcopy(self)
        support_index = 0 if self.support_leg == 'left_leg' else 1
        swing_index = 1-support_index

        def cycle(state, return_model=False):
            q = copy.deepcopy(template)
            q.support_foot_pos = np.zeros(3)
            feet = np.zeros((2,3))
            feet[swing_index,:2] = state[4:]
            q.left_foot_pos,q.right_foot_pos = feet.copy()
            q.active_foot = swing_index
            q.foot_velocity[:] = 0.
            q.body_pos[:2] = (state[:2]-q.alpha*state[4:])/(1-q.alpha)
            q.body_velocity[:2] = state[2:4]/(1-q.alpha)
            q.resetEffectiveState()
            for _ in range(2):
                q.calculateFootLocationForNextStepXcoMWorld(theta)
                target = np.array([q.u_x,q.u_y,0.])
                old = q.support_foot_pos[:2].copy()
                c,v = q.propagate(q.COM_pos[:2],np.array([q.vx_t,q.vy_t]),old,old,q.T,q.w_0)
                q.x_t,q.y_t = c-old
                q.vx_t,q.vy_t = v
                q._update_COM_pos()
                if q.support_leg == 'left_leg': q.right_foot_pos = target
                else: q.left_foot_pos = target
                # Both feet have zero endpoint velocity; swing details do not affect this map.
                q.updateBody()
                if q.T_ds:
                    q.beginDoubleSupport()
                    c,v = q.propagate(q.COM_pos[:2],np.array([q.vx_t,q.vy_t]),old,target[:2],q.T_ds,q.w_0)
                    q.x_t,q.y_t = c-old
                    q.vx_t,q.vy_t = v
                    q._update_COM_pos()
                    q.updateBody()
                q.switchSupportLeg()
            if return_model: return q
            swing = q.feet()[swing_index,:2]-q.support_foot_pos[:2]
            return np.r_[q.COM_pos[:2]-q.support_foot_pos[:2],q.vx_t,q.vy_t,swing]

        # The endpoint map is affine in horizontal CoM/velocity and trailing-foot position.
        b = cycle(np.zeros(6))
        A = np.column_stack([cycle(np.eye(6)[i])-b for i in range(6)])
        state = np.linalg.solve(np.eye(6)-A,b)
        if not np.isfinite(state).all() or not np.allclose(cycle(state),state,rtol=1e-8,atol=1e-9):
            raise ValueError('Could not obtain a consistent periodic initial state')
        lateral = np.array([-np.sin(theta),np.cos(theta)])
        # Center the two initial feet laterally without changing the periodic relative state.
        origin = -0.5*np.dot(state[4:],lateral)*lateral
        feet = np.zeros((2,3))
        feet[support_index,:2] = origin
        feet[swing_index,:2] = origin+state[4:]
        self.left_foot_pos,self.right_foot_pos = feet.copy()
        self.support_foot_pos = feet[support_index].copy()
        self.active_foot = swing_index
        self.foot_velocity[:] = 0.
        self.body_pos[:2] = origin+(state[:2]-self.alpha*state[4:])/(1-self.alpha)
        self.body_velocity[:2] = state[2:4]/(1-self.alpha)
        self.resetEffectiveState()
        self.ZMP_pos = self.support_foot_pos.copy()
        self.periodic_velocity = cycle(state,True).support_foot_pos[:2]/(2*self.T_d)
        self.calculateFootLocationForNextStepXcoMWorld(theta)

    def updateBody(self):
        # Horizontal effective state is dynamic; physical body height stays fixed.
        a = self.alpha
        self.body_pos[:2] = (self.COM_pos[:2]-a*self.feet()[self.active_foot,:2])/(1-a)
        self.body_velocity[:2] = (np.array([self.vx_t,self.vy_t])-a*self.foot_velocity[self.active_foot,:2])/(1-a)

    def beginDoubleSupport(self):
        # Called after touchdown and body reconstruction; physical states are unchanged.
        self.active_foot = 0 if self.support_leg == 'left_leg' else 1
        self.resetEffectiveState()
        super().beginDoubleSupport()

    def switchSupportLeg(self):
        if self.T_ds == 0:
            self.active_foot = 0 if self.support_leg == 'left_leg' else 1
            self.resetEffectiveState()
        super().switchSupportLeg()

    def calculateFootLocationForNextStepXcoMWorld(self, theta=0.):
        # Solve an affine terminal-DCM constraint including the touchdown reset.
        # This is a capture-style target, not a proof of periodic whole-body speed.
        a = self.alpha
        old = self.support_foot_pos[:2].copy()
        c = self.COM_pos[:2].copy()
        v = np.array([self.vx_t,self.vy_t])
        next_w = np.sqrt(9.81/((1-a)*self.body_height))
        K = np.expm1(next_w*self.T_ds)/(next_w*self.T_ds) if self.T_ds else 1.
        E = np.exp(next_w*self.T_d)
        side = -1 if self.support_leg == 'left_leg' else 1
        forward = np.array([np.cos(theta),np.sin(theta)])
        lateral = np.array([-np.sin(theta),np.cos(theta)])
        offset = K*self.s_d/(E-1)*forward-side*K*self.w_d/(E+1)*lateral
        end_c,end_v = self.propagate(c,v,old,old,self.T,self.w_0)
        self.eICP_x,self.eICP_y = end_c+end_v/self.w_0
        def residual(target):
            # Swing endpoint velocity is zero; new included foot is the old support.
            switched_c = end_c+a*(old-target)
            switched_v = end_v
            if self.T_ds:
                switched_c,switched_v = self.propagate(switched_c,switched_v,old,target,self.T_ds,next_w)
            return switched_c+switched_v/next_w-target-offset
        base=residual(np.zeros(2))
        matrix=np.column_stack([residual(np.eye(2)[i])-base for i in range(2)])
        self.u_x,self.u_y=np.linalg.solve(matrix,-base)

    def snapshot(self):
        record=super().snapshot()
        feet=self.feet()
        whole=(self.body_mass*self.body_pos+self.foot_mass*feet.sum(axis=0))/(self.body_mass+2*self.foot_mass)
        whole_v=(self.body_mass*self.body_velocity+self.foot_mass*self.foot_velocity.sum(axis=0))/(self.body_mass+2*self.foot_mass)
        effective=(1-self.alpha)*self.body_pos+self.alpha*feet
        record.update(position=self.body_pos[:2].copy(), velocity=self.body_velocity[:2].copy(),
                      whole_com=whole, whole_com_velocity=whole_v,
                      body_position=self.body_pos.copy(), body_velocity=self.body_velocity.copy(),
                      effective_com=effective, active_foot=self.active_foot,
                      planning_com=self.COM_pos.copy(), planning_height=self.zc,
                      foot_velocity=self.foot_velocity.copy())
        return record
