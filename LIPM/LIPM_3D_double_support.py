"""3D LIPM with double support, following the structure of LIPM_3D.py.

Like the original, x_t/y_t are relative to support_foot_pos. Foot positions,
COM_pos and ZMP_pos are world coordinates. During DSP the old support frame
is retained until switchSupportLeg() is explicitly called by the demo.
"""
import numpy as np


class LIPM3DDoubleSupport:
    def __init__(self, dt=0.01, T=0.34, T_ds=0.08, s_d=0.21, w_d=0.3,
                 support_leg='left_leg'):
        if dt <= 0 or T <= 0 or T_ds < 0 or w_d <= 0:
            raise ValueError('Invalid time or step width')
        for duration in (T, T_ds):
            if not np.isclose(duration/dt, round(duration/dt)):
                raise ValueError('SSP and DSP durations must be integer multiples of dt')
        if support_leg not in ('left_leg', 'right_leg'):
            raise ValueError('Unknown support leg')
        self.dt = dt
        self.t = 0.0
        self.T = T                       # Single support duration
        self.T_ds = T_ds                 # Double support duration
        self.T_d = T + T_ds              # Complete step duration
        self.s_d = s_d
        self.w_d = w_d
        self.eICP_x = self.eICP_y = 0.0
        self.u_x = self.u_y = 0.0

        # CoM initial and current states, relative to the support foot.
        self.x_0 = self.vx_0 = self.y_0 = self.vy_0 = 0.0
        self.x_t = self.vx_t = self.y_t = self.vy_t = 0.0
        self.support_leg = support_leg
        self.support_foot_pos = np.zeros(3)
        self.left_foot_pos = np.zeros(3)
        self.right_foot_pos = np.zeros(3)
        self.COM_pos = np.zeros(3)

        # Additional DSP states.
        self.phase = 'SSP'
        self.phase_count = 0
        self.time = 0.0
        self.steps = 0
        self.new_load = 0.0
        self.ZMP_pos = np.zeros(3)
        self.dsp_start_pos = np.zeros(3)
        self.dsp_end_pos = np.zeros(3)

    def initializeModel(self, COM_pos, left_foot_pos, right_foot_pos):
        self.COM_pos = np.array(COM_pos, dtype=float)
        self.left_foot_pos = np.array(left_foot_pos, dtype=float)
        self.right_foot_pos = np.array(right_foot_pos, dtype=float)
        if any(p.shape != (3,) or not np.isfinite(p).all()
               for p in (self.COM_pos, self.left_foot_pos, self.right_foot_pos)):
            raise ValueError('Positions must have three finite components')
        if self.COM_pos[2] <= 0 or self.left_foot_pos[2] != 0 or self.right_foot_pos[2] != 0:
            raise ValueError('A positive CoM height and ground-level feet are required')
        self.support_foot_pos = (self.left_foot_pos if self.support_leg == 'left_leg'
                                 else self.right_foot_pos).copy()
        self.zc = self.COM_pos[2]
        self.w_0 = np.sqrt(9.81/self.zc)
        self.x_0, self.y_0 = self.COM_pos[:2] - self.support_foot_pos[:2]
        self.x_t, self.y_t = self.x_0, self.y_0
        self.vx_0 = self.vy_0 = self.vx_t = self.vy_t = 0.0
        self.ZMP_pos = self.support_foot_pos.copy()
        self.phase, self.phase_count = 'SSP', 0
        self.t = self.time = self.new_load = 0.0
        self.steps = 0

    def initializePeriodicState(self, theta=0.0):
        """Optional periodic start; the original demo instead assigns x_0/vx_0/etc."""
        forward = np.array([np.cos(theta), np.sin(theta)])
        lateral = np.array([-np.sin(theta), np.cos(theta)])
        E = np.exp(self.w_0*self.T_d)
        decay = 1/E
        K = self._double_support_gain()
        H = (-np.expm1(-self.w_0*self.T_ds)/(self.w_0*self.T_ds)
             if self.T_ds else 1.0)
        side = 1 if self.support_leg == 'left_leg' else -1
        xi = K*self.s_d/(E-1)*forward - side*K*self.w_d/(E+1)*lateral
        eta = -H*self.s_d/(1-decay)*forward - side*H*self.w_d/(1+decay)*lateral
        self.x_0, self.y_0 = 0.5*(xi+eta)
        self.vx_0, self.vy_0 = 0.5*self.w_0*(xi-eta)
        self.x_t, self.y_t = self.x_0, self.y_0
        self.vx_t, self.vy_t = self.vx_0, self.vy_0
        self._update_COM_pos()

    def step(self):
        """Advance only CoM/ZMP. The demo updates feet and switches phases."""
        if self.phase == 'SSP':
            p0 = p1 = self.support_foot_pos[:2]
        else:
            count = round(self.T_ds/self.dt)
            p0 = self.dsp_start_pos[:2] + self.phase_count/count*(self.dsp_end_pos[:2]-self.dsp_start_pos[:2])
            self.new_load = (self.phase_count+1)/count
            p1 = self.dsp_start_pos[:2] + self.new_load*(self.dsp_end_pos[:2]-self.dsp_start_pos[:2])
        position = np.array([self.x_t, self.y_t]) + self.support_foot_pos[:2]
        velocity = np.array([self.vx_t, self.vy_t])
        position, velocity = self.propagate(position, velocity, p0, p1, self.dt, self.w_0)
        self.x_t, self.y_t = position - self.support_foot_pos[:2]
        self.vx_t, self.vy_t = velocity
        self.ZMP_pos[:2] = p1
        self.phase_count += 1
        self.t = self.phase_count*self.dt
        self.time += self.dt
        self._update_COM_pos()

    def calculateXfVf(self):
        """Predict the end of SSP in the support frame, as in the original."""
        ch, sh = np.cosh(self.T*self.w_0), np.sinh(self.T*self.w_0)
        x_f = self.x_0*ch + self.vx_0*sh/self.w_0
        vx_f = self.x_0*self.w_0*sh + self.vx_0*ch
        y_f = self.y_0*ch + self.vy_0*sh/self.w_0
        vy_f = self.y_0*self.w_0*sh + self.vy_0*ch
        return x_f, vx_f, y_f, vy_f

    def calculateFootLocationForNextStepXcoMWorld(self, theta=0.0):
        x_f, vx_f, y_f, vy_f = self.calculateXfVf()
        self.eICP_x = x_f + self.support_foot_pos[0] + vx_f/self.w_0
        self.eICP_y = y_f + self.support_foot_pos[1] + vy_f/self.w_0
        K = self._double_support_gain()
        E = np.exp(self.w_0*self.T_d)
        b_x = K*self.s_d/(E-1)
        b_y = K*self.w_d/(E+1)
        next_side = -1 if self.support_leg == 'left_leg' else 1
        b_next = np.array([np.cos(theta)*b_x + np.sin(theta)*next_side*b_y,
                           np.sin(theta)*b_x - np.cos(theta)*next_side*b_y])
        xi = np.array([self.x_0+self.vx_0/self.w_0, self.y_0+self.vy_0/self.w_0])
        target = self.support_foot_pos[:2] + (E*xi-b_next)/K
        self.u_x, self.u_y = target

    def beginDoubleSupport(self):
        if self.T_ds <= 0:
            raise ValueError('No DSP configured; call switchSupportLeg directly')
        self.dsp_start_pos = self.support_foot_pos.copy()
        self.dsp_end_pos = (self.right_foot_pos if self.support_leg == 'left_leg'
                            else self.left_foot_pos).copy()
        self.phase, self.phase_count = 'DSP', 0
        self.t = self.new_load = 0.0

    def switchSupportLeg(self):
        """Preserve world CoM and velocity; reset the origin to the new support."""
        self._update_COM_pos()
        if self.support_leg == 'left_leg':
            self.support_leg = 'right_leg'
            self.support_foot_pos = self.right_foot_pos.copy()
        else:
            self.support_leg = 'left_leg'
            self.support_foot_pos = self.left_foot_pos.copy()
        self.x_0, self.y_0 = self.COM_pos[:2] - self.support_foot_pos[:2]
        self.vx_0, self.vy_0 = self.vx_t, self.vy_t
        self.x_t, self.y_t = self.x_0, self.y_0
        self.ZMP_pos = self.support_foot_pos.copy()
        self.phase, self.phase_count = 'SSP', 0
        self.t = self.new_load = 0.0
        self.steps += 1

    # DSP-specific mathematical helpers.
    def _double_support_gain(self):
        return np.expm1(self.w_0*self.T_ds)/(self.w_0*self.T_ds) if self.T_ds else 1.0

    def _update_COM_pos(self):
        self.COM_pos = np.array([self.x_t+self.support_foot_pos[0],
                                 self.y_t+self.support_foot_pos[1], self.zc])

    @staticmethod
    def propagate(position, velocity, p0, p1, duration, omega):
        """Exact constant-height propagation for linearly moving ZMP."""
        zmp_velocity = (p1-p0)/duration
        relative = position-p0
        relative_velocity = velocity-zmp_velocity
        ch, sh = np.cosh(omega*duration), np.sinh(omega*duration)
        return (p1+ch*relative+sh/omega*relative_velocity,
                zmp_velocity+omega*sh*relative+ch*relative_velocity)

    def snapshot(self):
        """World-coordinate logging only; canonical states retain original names."""
        return dict(time=self.time, position=self.COM_pos[:2].copy(),
                    velocity=np.array([self.vx_t, self.vy_t]), zmp=self.ZMP_pos[:2].copy(),
                    feet=np.array([self.left_foot_pos, self.right_foot_pos]), phase=self.phase,
                    support=0 if self.support_leg == 'left_leg' else 1,
                    new_load=self.new_load, target=np.array([self.u_x, self.u_y]))
