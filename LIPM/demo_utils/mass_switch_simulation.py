"""Shared stepping for the body-CoM and total-CoM mass-switch demos.

Model construction and interpretation of recorded values belong to each demo.
The double-support baseline has different body/foot updates and stays separate.
"""
import numpy as np


class MassSwitchSimulation:
    """Run a supplied mass-switch model with a foot and command schedule."""

    def __init__(self, model):
        self.model = model

    def swing_trajectory(self, clearance):
        """Compute swing positions and store the model's swing velocities."""
        LIPM_model = self.model
        if clearance < 0:
            raise ValueError('clearance must be nonnegative')
        swing_data_len = round(LIPM_model.T/LIPM_model.dt)
        start = (LIPM_model.right_foot_pos if LIPM_model.support_leg == 'left_leg'
                 else LIPM_model.left_foot_pos)
        target = np.array([LIPM_model.u_x, LIPM_model.u_y, 0.])
        phase = np.arange(1, swing_data_len+1)/swing_data_len
        blend = 10*phase**3-15*phase**4+6*phase**5
        swing_foot_pos = start + blend[:, None]*(target-start)
        swing_foot_pos[:, 2] = clearance*64*phase**3*(1-phase)**3
        swing_foot_pos[-1] = target
        LIPM_model.swing_velocity = np.zeros_like(swing_foot_pos)
        LIPM_model.swing_velocity[:, :2] = ((30*phase**2-60*phase**3+30*phase**4)/LIPM_model.T)[:, None]*(target-start)[:2]
        LIPM_model.swing_velocity[:, 2] = clearance*192*phase**2*(1-phase)**2*(1-2*phase)/LIPM_model.T
        LIPM_model.swing_velocity[-1] = 0.
        return swing_foot_pos

    def run(self, total_time, theta=0., clearance=0.1,
            step_to_cmdv=None, COM_dvel_list=None, w_d_list=None):
        """Advance the model and record the same post-transition state each step."""
        LIPM_model = self.model
        # Calculate the next step locations and the foot positions for swing phase.
        LIPM_model.calculateFootLocationForNextStepXcoMWorld(theta)
        swing_foot_pos = self.swing_trajectory(clearance)
        swing_data_len = len(swing_foot_pos)
        double_support_data_len = round(LIPM_model.T_ds/LIPM_model.dt)
        # With no schedule, retain a fixed command (used by numerical tests).
        if step_to_cmdv is None:
            step_to_cmdv = []
        if COM_dvel_list is None:
            speed = LIPM_model.s_d/LIPM_model.T_d
            COM_dvel_list = np.array([[speed*np.cos(theta), speed*np.sin(theta)]])
        if w_d_list is None:
            w_d_list = np.full(len(COM_dvel_list), LIPM_model.w_d)
        if len(COM_dvel_list) != len(step_to_cmdv)+1 or len(w_d_list) != len(COM_dvel_list):
            raise ValueError('A command schedule needs one more value than transition steps')
        COM_dvel = np.asarray(COM_dvel_list[0])
        support_foot_pos = LIPM_model.support_foot_pos.copy()
        prev_support_foot_pos = support_foot_pos.copy()

        def record_data():
            record = LIPM_model.snapshot()
            delta = support_foot_pos[:2]-prev_support_foot_pos[:2]
            record.update(command=COM_dvel.copy(), dstep_length=LIPM_model.s_d,
                          dstep_width=LIPM_model.w_d, step_num=LIPM_model.steps,
                          step_length=np.cos(theta)*delta[0]+np.sin(theta)*delta[1],
                          step_width=abs(-np.sin(theta)*delta[0]+np.cos(theta)*delta[1]))
            return record

        records = [record_data()]

        for i in range(1, round(total_time/LIPM_model.dt)+1):
            # Update body (CoM) state: x_t, vx_t, y_t, vy_t.
            LIPM_model.step()
            switch_support = False
            if LIPM_model.phase == 'SSP':
                j = LIPM_model.phase_count-1
                if LIPM_model.support_leg == 'left_leg':
                    LIPM_model.right_foot_pos = swing_foot_pos[j].copy()
                else:
                    LIPM_model.left_foot_pos = swing_foot_pos[j].copy()
                swing_index = 1 if LIPM_model.support_leg == 'left_leg' else 0
                LIPM_model.foot_velocity[swing_index] = LIPM_model.swing_velocity[j]
                LIPM_model.updateBody()
                if LIPM_model.phase_count == swing_data_len:
                    if double_support_data_len:
                        LIPM_model.beginDoubleSupport()
                    else:
                        switch_support = True
            else:
                LIPM_model.updateBody()
                if LIPM_model.phase_count == double_support_data_len:
                    switch_support = True

            # Switch the support leg only after DSP, then plan the next swing.
            if switch_support:
                prev_support_foot_pos = support_foot_pos.copy()
                LIPM_model.switchSupportLeg()
                support_foot_pos = LIPM_model.support_foot_pos.copy()
                step_num = LIPM_model.steps
                command_index = sum(step_num >= threshold for threshold in step_to_cmdv)
                COM_dvel = np.asarray(COM_dvel_list[command_index])
                theta = np.arctan2(COM_dvel[1], COM_dvel[0])
                # Include DSP in the duration so the average speed reference remains meaningful.
                LIPM_model.s_d = np.linalg.norm(COM_dvel)*LIPM_model.T_d
                LIPM_model.w_d = w_d_list[command_index]
                LIPM_model.calculateFootLocationForNextStepXcoMWorld(theta)
                swing_foot_pos = self.swing_trajectory(clearance)

            # Record data after the phase transition, preserving previous demo semantics.
            records.append(record_data())
        return {key: np.array([record[key] for record in records]) for key in records[0]}
