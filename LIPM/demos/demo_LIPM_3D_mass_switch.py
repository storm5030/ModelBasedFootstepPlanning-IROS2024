"""Static results: python LIPM/demos/demo_LIPM_3D_mass_switch.py
Animation (optional): add --animate
Headless: add --headless --output-dir outputs/lipm_double_support
"""
# Allow both direct script execution and python -m from the repository root.
if __package__ in (None, ''):
    import sys
    from pathlib import Path as _Path
    sys.path.insert(0, str(_Path(__file__).resolve().parents[2]))

import argparse
from pathlib import Path
import numpy as np
from LIPM.models.LIPM_3D_mass_switch import LIPM3DMassSwitch, DEFAULT_BODY_MASS, DEFAULT_FOOT_MASS


# ---------------------------------------------------------------- Animation helpers
class Ball:
    def __init__(self, ax, size=10, shape='o', color=None, label=None):
        self.scatter, = ax.plot([], [], [], shape, markersize=size, color=color, label=label)

    def update(self, pos):
        self.scatter.set_data_3d([pos[0]], [pos[1]], [pos[2]])


class Line:
    def __init__(self, ax, size=1, color='g'):
        self.line, = ax.plot([], [], [], linewidth=size, color=color)

    def update(self, pos):
        self.line.set_data_3d(pos)


class LIPM_3D_Animate:
    def __init__(self, ax):
        self.ax = ax
        self.origin = Ball(ax, size=2, color='k')
        self.COM_trajectory = Line(ax, color='g')
        self.COM_head = Ball(ax, size=2, color='r')
        self.left_foot = Ball(ax, size=5, color='b', label='Left foot')
        self.right_foot = Ball(ax, size=5, color='m', label='Right foot')
        self.left_leg = Line(ax, size=3, color='b')
        self.right_leg = Line(ax, size=3, color='m')
        self.COM = Ball(ax, size=16, color='r', label='Body CoM')
        self.ZMP = Ball(ax, size=12, shape='*', color='darkorange', label='ZMP')
        self.support_line = Line(ax, size=2, color='darkorange')

    def update(self, COM_pos, COM_pos_trajectory, left_foot_pos, right_foot_pos,
               ZMP_pos, double_support):
        self.origin.update([0, 0, 0])
        self.COM.update(COM_pos)
        self.COM_trajectory.update(COM_pos_trajectory)
        self.COM_head.update(COM_pos_trajectory[:, -1])
        self.left_foot.update(left_foot_pos)
        self.right_foot.update(right_foot_pos)
        self.left_leg.update(np.array([COM_pos, left_foot_pos]).T)
        self.right_leg.update(np.array([COM_pos, right_foot_pos]).T)
        self.ZMP.update(ZMP_pos)
        self.support_line.update(np.array([left_foot_pos, right_foot_pos]).T
                                 if double_support else np.empty((3, 0)))
        xshift = min(COM_pos[0], 0) + max(COM_pos[0]-3, 0)
        yshift = min(COM_pos[1]+1.5, 0) + max(COM_pos[1]-1.5, 0)
        self.ax.set_xlim(-1+xshift, 4+xshift)
        self.ax.set_ylim(-2+yshift, 2+yshift)
        return [self.origin.scatter, self.COM.scatter, self.COM_trajectory.line,
                self.COM_head.scatter, self.left_foot.scatter, self.right_foot.scatter,
                self.left_leg.line, self.right_leg.line, self.ZMP.scatter, self.support_line.line]


# ---------------------------------------------------------------- LIPM control

def create_model(dt=0.02, t_ss=0.48, t_ds=0.12, height=0.6, velocity=(0.3, 0.),
                 width=0.4, body_mass=DEFAULT_BODY_MASS, foot_mass=DEFAULT_FOOT_MASS, periodic_start=False):
    velocity = np.asarray(velocity, dtype=float)
    if velocity.shape != (2,) or not np.isfinite(velocity).all():
        raise ValueError('velocity must contain two finite components')
    model = LIPM3DMassSwitch(dt=dt,T=t_ss,T_ds=t_ds,
                            s_d=np.linalg.norm(velocity)*(t_ss+t_ds),w_d=width,
                            body_mass=body_mass,foot_mass=foot_mass)
    model.initializeModel([0.,0.,height],[0.,0.2,0.],[0.,-0.2,0.])
    model.body_velocity[:2] = velocity
    model.resetEffectiveState()
    if periodic_start:
        model.initializePeriodicState(np.arctan2(velocity[1],velocity[0]))
    model.calculateFootLocationForNextStepXcoMWorld(np.arctan2(velocity[1],velocity[0]))
    return model


def calculateSwingFootTrajectory(LIPM_model, clearance):
    """As in the original demo, foot trajectories belong to the demo, not the model."""
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


def simulate(LIPM_model, total_time, theta=0., clearance=0.1,
             step_to_cmdv=None, COM_dvel_list=None, w_d_list=None):
    # Calculate the next step locations and the foot positions for swing phase.
    LIPM_model.calculateFootLocationForNextStepXcoMWorld(theta)
    swing_foot_pos = calculateSwingFootTrajectory(LIPM_model, clearance)
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
            swing_foot_pos = calculateSwingFootTrajectory(LIPM_model, clearance)

        # Record data after the phase transition, preserving previous demo semantics.
        records.append(record_data())
    return {key: np.array([record[key] for record in records]) for key in records[0]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ss', type=float, default=0.48, help='Single support duration [s]')
    parser.add_argument('--ds', type=float, default=0.12, help='Double support duration [s]; 0 for baseline')
    parser.add_argument('--dt', type=float, default=0.02)
    parser.add_argument('--duration', type=float, default=10.0)
    parser.add_argument('--vx', type=float, default=0.3)
    parser.add_argument('--vy', type=float, default=0.0)
    parser.add_argument('--width', type=float, default=None, help='Override the original width schedule with a fixed width')
    parser.add_argument('--height', type=float, default=0.6)
    parser.add_argument('--clearance', type=float, default=0.1)
    parser.add_argument('--body-mass', type=float, default=DEFAULT_BODY_MASS, help='Mass excluding both feet [kg]')
    parser.add_argument('--foot-mass', type=float, default=DEFAULT_FOOT_MASS, help='Mass of each foot [kg]')
    parser.add_argument('--periodic-start', action='store_true', help='Start on the mass-switch planner periodic gait')
    parser.add_argument('--animate', action='store_true', help='Replay the animation instead of showing only final results')
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--output-dir', type=Path)
    args = parser.parse_args()
    if args.duration <= 0:
        parser.error('duration must be positive')
    import matplotlib
    if args.headless:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    step_to_cmdv = [10, 20, 30]
    COM_dvel_list = np.array([[args.vx, args.vy]]*4)
    w_d_list = np.array([0.4, 0.6, 0.4, 0.4]) if args.width is None else np.full(4, args.width)
    LIPM_model = create_model(dt=args.dt, t_ss=args.ss, t_ds=args.ds,
                              height=args.height, velocity=COM_dvel_list[0], width=w_d_list[0],
                              body_mass=args.body_mass, foot_mass=args.foot_mass, periodic_start=args.periodic_start)
    print(f'Model mass [kg]: body={LIPM_model.body_mass:g}, each foot={LIPM_model.foot_mass:g}; alpha={LIPM_model.alpha:g}')
    if args.periodic_start:
        print('Periodic gait mean velocity [m/s] (before command/width changes):', LIPM_model.periodic_velocity)
    theta = np.arctan2(args.vy, args.vx)
    data = simulate(LIPM_model, args.duration, theta, args.clearance,
                    step_to_cmdv, COM_dvel_list, w_d_list)
    t, c, v, z, feet = (data[k] for k in ('time', 'position', 'velocity', 'zmp', 'feet'))
    if not all(np.isfinite(a).all() for a in (c, v, z, feet)):
        raise RuntimeError('Non-finite simulation state')
    # Match the original demo: 3D / velocity above, top view / step parameters below.
    fig = plt.figure(figsize=(10, 10))
    grid = fig.add_gridspec(2, 2, height_ratios=[2.5, 1], left=0.09, right=0.97,
                           bottom=0.09, top=0.90, wspace=0.32, hspace=0.30)
    ax = fig.add_subplot(grid[0, 0], projection='3d')
    bx = fig.add_subplot(grid[1, 0], autoscale_on=False)
    cx = fig.add_subplot(grid[0, 1])
    dx = fig.add_subplot(grid[1, 1])
    fig.suptitle(f'Mass-switch LIPM ({LIPM_model.body_mass:g}+2x{LIPM_model.foot_mass:g} kg) | SSP {args.ss:.2f}s + DSP {args.ds:.2f}s | '
                 f'command ({args.vx:.2f}, {args.vy:.2f}) m/s')
    dsp = data['phase'] == 'DSP'

    step_length, step_width = data['step_length'], data['step_width']
    dstep_length, dstep_width = data['dstep_length'], data['dstep_width']
    COM_vel_x, COM_vel_y = v[:, 0], v[:, 1]
    COM_dvel_x, COM_dvel_y = data['command'][:, 0], data['command'][:, 1]

    ax.set(xlim=(-1, 4), ylim=(-2, 2), zlim=(-0.01, max(1., args.height*1.25)),
            xlabel='x (m)', ylabel='y (m)', zlabel='z (m)')
    ax.view_init(20, -150)
    LIPM_3D_ani = LIPM_3D_Animate(ax)
    effective_3d = [Ball(ax, size=8, shape='D', color=color, label=label) for color, label in [('teal', 'Effective: left swing'), ('goldenrod', 'Effective: right swing')]]
    ax.legend(loc='upper left', fontsize=7)

    bx.set(xlim=(-0.5, 5), ylim=(-0.8, 0.8), xlabel='x (m)', ylabel='y (m)')
    bx.set_aspect('equal')
    bx.plot([0], [0], 'ko', ms=2)
    COM_traj_ani, = bx.plot([], [], 'g-', lw=1)
    COM_pos_ani, = bx.plot([], [], 'ro', ms=6)
    effective_2d = [bx.plot([], [], 'D', color=color, ms=6)[0] for color in ('teal','goldenrod')]
    left_foot_pos_ani, = bx.plot([], [], 'bo', ms=10)
    right_foot_pos_ani, = bx.plot([], [], 'mo', ms=10)
    path_zmp, = bx.plot([], [], '*', color='darkorange', ms=10)
    zmp_trail, = bx.plot([], [], color='darkorange', lw=1, alpha=0.7)
    ani_text_COM_pos = bx.text(0.05, 0.90, '', transform=bx.transAxes, fontsize=9)

    cx.set(xlim=(0, t[-1]), ylim=(min(v.min(), args.vx, args.vy)-0.1,
                                   max(v.max(), args.vx, args.vy)+0.1),
              xlabel='time (s)', ylabel='Body CoM velocity (m/s)')
    COM_vel_x_ani, = cx.plot([], [], color='k', label='Body CoM velocity x')
    COM_dvel_x_ani, = cx.plot([], [], 'k--', label='desired Body CoM velocity x')
    COM_vel_y_ani, = cx.plot([], [], color='purple', label='Body CoM velocity y')
    COM_dvel_y_ani, = cx.plot([], [], color='purple', ls='--', label='desired Body CoM velocity y')
    dx.set(xlim=(0, t[-1]),
               ylim=(min(step_length.min(), step_width.min(), dstep_length.min(), dstep_width.min())-0.1,
                     max(step_length.max(), step_width.max(), dstep_length.max(), dstep_width.max())+0.1),
               xlabel='time (s)', ylabel='scale (m)')
    step_length_ani, = dx.plot([], [], color='gray', label='step length')
    dstep_length_ani, = dx.plot([], [], color='gray', ls='--', label='desired step length')
    step_width_ani, = dx.plot([], [], color='cyan', label='step width')
    dstep_width_ani, = dx.plot([], [], color='cyan', ls='--', label='desired step width')
    histories = [(COM_vel_x_ani, COM_vel_x), (COM_dvel_x_ani, COM_dvel_x),
                 (COM_vel_y_ani, COM_vel_y), (COM_dvel_y_ani, COM_dvel_y),
                 (step_length_ani, step_length), (dstep_length_ani, dstep_length),
                 (step_width_ani, step_width), (dstep_width_ani, dstep_width)]

    # Reveal DSP shading only up to the displayed time, including on replay.
    from matplotlib.patches import Rectangle
    edges = np.diff(np.r_[False, dsp, False].astype(int))
    spans = []
    for start, stop in zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)):
        end_time = t[stop] if stop < len(t) else t[-1]
        for axis in (cx, dx):
            patch = Rectangle((t[start], 0), 0, 1, transform=axis.get_xaxis_transform(),
                              facecolor='darkorange', alpha=0.12, edgecolor='none', zorder=0)
            axis.add_patch(patch)
            spans.append((patch, t[start], end_time))
    for axis in (bx, cx, dx):
        axis.grid(ls='--', alpha=0.5)
    for axis in (cx, dx):
        axis.set_xticks(np.linspace(0, t[-1], 6))
        axis.legend(loc='upper right', fontsize=8)

    # ------------------------------------------------- Animation callbacks
    def ani_3D_init():
        return ani_3D_update(0)

    def ani_3D_update(i):
        COM_pos = data['body_position'][i]
        COM_pos_trajectory = np.vstack((c[:i+1].T, np.zeros(i+1)))
        artists = LIPM_3D_ani.update(COM_pos, COM_pos_trajectory, feet[i, 0], feet[i, 1],
                                    np.r_[z[i], 0.], dsp[i])
        for k, marker in enumerate(effective_3d):
            marker.update(data['effective_com'][i,k])
            marker.scatter.set_markersize(10 if k == data['active_foot'][i] else 5)
            artists.append(marker.scatter)
        label = ('DSP: new-foot load ' + f'{100*data["new_load"][i]:.0f}%'
                 if dsp[i] else f'SSP: {"left" if data["support"][i] == 0 else "right"} support')
        ax.set_title(f't = {t[i]:.2f}s | {label}', fontsize=9)
        return artists

    def ani_2D_init():
        return ani_2D_update(0)

    def ani_2D_update(i):
        for k, marker in enumerate(effective_2d):
            point = data['effective_com'][i,k]
            marker.set_data([point[0]], [point[1]])
            marker.set_markersize(9 if k == data['active_foot'][i] else 5)
        COM_traj_ani.set_data(c[:i+1, 0], c[:i+1, 1])
        zmp_trail.set_data(z[:i+1, 0], z[:i+1, 1])
        for artist, point in ((COM_pos_ani, c[i]), (left_foot_pos_ani, feet[i, 0]),
                              (right_foot_pos_ani, feet[i, 1]), (path_zmp, z[i])):
            artist.set_data([point[0]], [point[1]])
        ani_text_COM_pos.set_text(f'Body CoM = ({c[i, 0]:.2f}, {c[i, 1]:.2f})')
        bx.set_xlim(c[i, 0]-2, c[i, 0]+3)
        bx.set_ylim(c[i, 1]-0.8, c[i, 1]+0.8)
        return effective_2d + [COM_traj_ani, zmp_trail, COM_pos_ani, left_foot_pos_ani, right_foot_pos_ani, path_zmp, ani_text_COM_pos]

    def COM_vel_2D_init():
        return COM_vel_2D_update(0)

    def COM_vel_2D_update(i):
        COM_vel_x_ani.set_data(t[:i+1], COM_vel_x[:i+1])
        COM_vel_y_ani.set_data(t[:i+1], COM_vel_y[:i+1])
        COM_dvel_x_ani.set_data(t[:i+1], COM_dvel_x[:i+1])
        COM_dvel_y_ani.set_data(t[:i+1], COM_dvel_y[:i+1])
        return [COM_vel_x_ani, COM_vel_y_ani, COM_dvel_x_ani, COM_dvel_y_ani]

    def step_params_2D_init():
        return step_params_2D_update(0)

    def step_params_2D_update(i):
        step_length_ani.set_data(t[:i+1], step_length[:i+1])
        step_width_ani.set_data(t[:i+1], step_width[:i+1])
        dstep_length_ani.set_data(t[:i+1], dstep_length[:i+1])
        dstep_width_ani.set_data(t[:i+1], dstep_width[:i+1])
        return [step_length_ani, step_width_ani, dstep_length_ani, dstep_width_ani]

    def _init_func():
        for patch, _, _ in spans:
            patch.set_width(0)
        return ani_3D_init()+ani_2D_init()+COM_vel_2D_init()+step_params_2D_init()

    def _update_func(i):
        artists = ani_3D_update(i)+ani_2D_update(i)+COM_vel_2D_update(i)+step_params_2D_update(i)
        for patch, start, stop in spans:
            patch.set_width(max(0., min(t[i], stop)-start))
        return artists

    def show_final_result():
        # Keep all animation callbacks above; draw just one final frame by default.
        _update_func(len(t)-1)
        # A static result should include the entire walk, not only the final camera window.
        points = np.vstack((c, z, feet[:, :, :2].reshape(-1, 2)))
        lower, upper = points.min(axis=0)-0.2, points.max(axis=0)+0.2
        ax.set_xlim(lower[0], upper[0])
        ax.set_ylim(lower[1], upper[1])
        bx.set_xlim(lower[0], upper[0])
        bx.set_ylim(lower[1], upper[1])

    if args.output_dir:
        args.output_dir.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(args.output_dir / 'trajectory.npz', **data)
        frame = min(len(t)-1, round((args.ss + args.ds/2) / args.dt))
        _update_func(frame)
        fig.savefig(args.output_dir / 'overview.png', dpi=150)
        show_final_result()
        fig.savefig(args.output_dir / 'overview_final.png', dpi=150)
        print('Saved:', args.output_dir.resolve())
    if args.headless:
        show_final_result()
        fig.canvas.draw()
        assert all(len(line.get_xdata()) == len(t) for line, _ in histories)
        assert len(COM_traj_ani.get_xdata()) == len(t)
        plt.close(fig)
    elif args.animate:
        # Original animation path retained for use on a faster machine.
        stride = max(1, round(0.02 / args.dt))
        _update_func(0)
        animation = FuncAnimation(fig, _update_func, init_func=_init_func, frames=range(0, len(t), stride),
                                  interval=1000*args.dt*stride, blit=False, repeat=False, cache_frame_data=False)
        plt.show()
    else:
        show_final_result()
        plt.show()
    print(f'Completed steps: {LIPM_model.steps}; DSP fraction: {args.ds/(args.ss+args.ds):.1%}')
    print('Mean body CoM velocity [m/s]:', (c[-1]-c[0])/(t[-1]-t[0]))
    print('Velocity ranges [m/s]:', v.min(axis=0), v.max(axis=0))


if __name__ == '__main__':
    main()
