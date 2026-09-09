"""Double-support CoM/ZMP/ICP analysis. Static plots by default; no video writer."""
# Allow both direct script execution and python -m from the repository root.
if __package__ in (None, ''):
    import sys
    from pathlib import Path as _Path
    sys.path.insert(0, str(_Path(__file__).resolve().parents[2]))

import argparse
from pathlib import Path
from LIPM.demo_utils.output_paths import default_output_dir
import numpy as np
from LIPM.demos.demo_LIPM_3D_double_support import create_model, simulate
from LIPM.models.LIPM_3D_double_support import LIPM3DDoubleSupport


def analyze(data, height, ss, ds):
    w = np.sqrt(9.81/height)
    c, v, z = data['position'], data['velocity'], data['zmp']
    icp = c+v/w
    predicted_ss = np.full_like(c, np.nan)
    predicted_end = np.empty_like(c)
    for i,t in enumerate(data['time']):
        elapsed = np.clip(t-data['step_num'][i]*(ss+ds),0.,ss+ds)
        cp,vp = c[i].copy(),v[i].copy()
        if data['phase'][i] == 'SSP':
            remaining = max(0.,ss-elapsed)
            if remaining:
                cp,vp = LIPM3DDoubleSupport.propagate(cp,vp,z[i],z[i],remaining,w)
            predicted_ss[i] = cp+vp/w
            remaining_ds = ds
        else:
            remaining_ds = max(0.,ss+ds-elapsed)
        if remaining_ds:
            cp,vp = LIPM3DDoubleSupport.propagate(cp,vp,z[i],data['target'][i],remaining_ds,w)
        predicted_end[i] = cp+vp/w
    # Exact DCM equation check over each sample interval, accounting for moving ZMP.
    residual = np.zeros_like(c)
    for i,dt in enumerate(np.diff(data['time']),1):
        p0=z[i-1]
        p1=z[i] if data['phase'][i-1] == 'DSP' else p0
        u=(p1-p0)/dt
        expected=p1+u/w+np.exp(w*dt)*(icp[i-1]-p0-u/w)
        residual[i]=icp[i]-expected
    return dict(icp=icp, icp_dot=w*(icp-z), icp_zmp=icp-z,
                predicted_ss_end_icp=predicted_ss, predicted_step_end_icp=predicted_end,
                dcm_residual=residual, omega=np.array(w), height=np.array(height))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ss',type=float,default=0.48)
    parser.add_argument('--ds',type=float,default=0.12)
    parser.add_argument('--dt',type=float,default=0.02)
    parser.add_argument('--duration',type=float,default=10.)
    parser.add_argument('--vx',type=float,default=0.3)
    parser.add_argument('--vy',type=float,default=0.)
    parser.add_argument('--height',type=float,default=0.6)
    parser.add_argument('--width',type=float)
    parser.add_argument('--clearance',type=float,default=0.1)
    parser.add_argument('--periodic-start',action='store_true')
    parser.add_argument('--start',type=float,default=None,help='Plot window start [s]; simulate from t=0')
    parser.add_argument('--end',type=float,help='Plot window end [s]')
    parser.add_argument('--headless',action='store_true')
    parser.add_argument('--output-dir',type=Path,default=default_output_dir(__file__))
    parser.add_argument('--snapshot',type=float,help='Time of the robot marker within the window [s]')
    parser.add_argument('--icp-kind',choices=['instantaneous','ssp-end','step-end'],default='instantaneous')
    parser.add_argument('--diagnostics',action='store_true',help='Also display the earlier multi-panel diagnostics')
    args=parser.parse_args()
    if args.duration <= 0:
        parser.error('duration must be positive')
    args.start=min(2.4,args.duration/4) if args.start is None else args.start
    end=min(args.duration,args.start+2.4) if args.end is None else args.end
    if not 0 <= args.start < end <= args.duration:
        parser.error('Require 0 <= start < end <= duration')
    model=create_model(dt=args.dt,t_ss=args.ss,t_ds=args.ds,height=args.height,
                       velocity=(args.vx,args.vy),width=0.4 if args.width is None else args.width,
                       periodic_start=args.periodic_start)
    widths=np.array([0.4,0.6,0.4,0.4]) if args.width is None else np.full(4,args.width)
    d=simulate(model,args.duration,np.arctan2(args.vy,args.vx),args.clearance,
               [10,20,30],np.array([[args.vx,args.vy]]*4),widths)
    d.update(analyze(d,args.height,args.ss,args.ds))
    if not all(np.isfinite(d[k]).all() for k in ('position','velocity','icp','predicted_step_end_icp')):
        raise RuntimeError('Non-finite simulation result')
    import matplotlib
    if args.headless:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    mask=(d['time'] >= args.start-1e-10)&(d['time'] <= end+1e-10)
    if mask.sum()<2:
        parser.error('Plot window must contain at least two samples')
    t=d['time'][mask]; c=d['position'][mask]; v=d['velocity'][mask]
    z=d['zmp'][mask]; xi=d['icp'][mask]; feet=d['feet'][mask]
    title=f'Double-support LIPM | SSP {args.ss:g}s + DSP {args.ds:g}s | ICP = CoM + velocity / omega'
    fig=plt.figure(figsize=(12,9),layout='constrained')
    grid=fig.add_gridspec(2,2,height_ratios=[2.5,1])
    ax=fig.add_subplot(grid[0,0],projection='3d')
    top=fig.add_subplot(grid[1,0]); vel=fig.add_subplot(grid[0,1]); step=fig.add_subplot(grid[1,1])
    fig.suptitle(title)
    ax.plot(c[:,0],c[:,1],np.full(len(c),args.height),color='red',label='CoM')
    ax.plot(xi[:,0],xi[:,1],np.zeros(len(c)),color='goldenrod',ls='--',label='Instantaneous ICP (ground)')
    ax.plot(z[:,0],z[:,1],np.zeros(len(c)),color='darkorange',label='ZMP')
    for j,color in enumerate(['blue','magenta']):
        ax.plot(feet[:,j,0],feet[:,j,1],feet[:,j,2],color=color,label=['Left foot','Right foot'][j])
        ax.plot([c[-1,0],feet[-1,j,0]],[c[-1,1],feet[-1,j,1]],[args.height,feet[-1,j,2]],color=color)
        top.plot(feet[:,j,0],feet[:,j,1],color=color,lw=0.8)
    ax.set(xlabel='x (m)',ylabel='y (m)',zlabel='z (m)'); ax.view_init(20,-150)
    for points,color,label in [(c,'red','CoM'),(xi,'goldenrod','ICP'),(z,'darkorange','ZMP')]:
        top.plot(points[:,0],points[:,1],color=color,label=label)
    # Targets only at planning instants, rather than every repeated sample.
    changes=np.r_[True,np.diff(d['step_num'])!=0]&mask
    targets=d['target'][changes]
    top.scatter(targets[:,0],targets[:,1],marker='x',color='black',label='Planned landing')
    top.set(xlabel='x (m)',ylabel='y (m)'); top.set_aspect('equal',adjustable='datalim')
    for j,color in enumerate(['black','purple']):
        vel.plot(t,v[:,j],color=color,label=f'CoM velocity {"xy"[j]}')
        vel.plot(t,d['command'][mask,j],color=color,ls='--',label=f'Reference {"xy"[j]}')
    vel.set(xlabel='time (s)',ylabel='velocity (m/s)')
    for key,color in [('length','gray'),('width','teal')]:
        step.plot(t,d['step_'+key][mask],color=color,label='Actual '+key)
        step.plot(t,d['dstep_'+key][mask],color=color,ls='--',label='Desired '+key)
    step.set(xlabel='time (s)',ylabel='step scale (m)')
    detail,axes=plt.subplots(2,2,figsize=(12,8),layout='constrained')
    detail.suptitle('ICP diagnostics | orange shading: DSP')
    for j in range(2):
        a=axes[0,j]
        for points,label,color in [(c,'CoM','red'),(z,'ZMP','darkorange'),(xi,'Instantaneous ICP','goldenrod')]:
            a.plot(t,points[:,j],label=label,color=color)
        a.plot(t,d['predicted_ss_end_icp'][mask,j],ls=':',color='blue',label='Predicted SSP-end ICP')
        a.plot(t,d['predicted_step_end_icp'][mask,j],ls='--',color='green',label='Predicted DSP-end ICP')
        a.set(xlabel='time (s)',ylabel=f'{"xy"[j]} position (m)')
        a=axes[1,j]
        a.plot(t,(xi-z)[:,j],label='ICP - ZMP',color='goldenrod')
        a.plot(t,(xi-c)[:,j],label='ICP - CoM = velocity / omega',color='red')
        a.axhline(0,color='gray',lw=0.7)
        a.set(xlabel='time (s)',ylabel=f'{"xy"[j]} offset (m)')
    edges=np.diff(np.r_[False,d['phase']=='DSP',False].astype(int))
    for a in [vel,step,*axes.flat]:
        for first,last in zip(np.flatnonzero(edges==1),np.flatnonzero(edges==-1)):
            lo=max(args.start,d['time'][first]); hi=min(end,d['time'][last] if last<len(d['time']) else d['time'][-1])
            if hi>lo: a.axvspan(lo,hi,color='darkorange',alpha=0.12)
        a.set_xlim(t[0],t[-1])
    for a in [ax,top,vel,step,*axes.flat]:
        a.legend(fontsize=7); a.grid(alpha=0.3)
    args.output_dir.mkdir(parents=True,exist_ok=True)
    np.savez_compressed(args.output_dir/'trajectory_analysis.npz',**d)
    columns=np.column_stack([d['time'],d['step_num'],(d['phase']=='DSP').astype(int),d['position'],d['velocity'],d['zmp'],d['icp'],d['target'],d['predicted_ss_end_icp'],d['predicted_step_end_icp']])
    np.savetxt(args.output_dir/'icp_analysis.csv',columns,delimiter=',',comments='',
               header='time,step,dsp,com_x,com_y,vx,vy,zmp_x,zmp_y,icp_x,icp_y,target_x,target_y,pred_ss_x,pred_ss_y,pred_step_x,pred_step_y')
    for figure,name in [(fig,'overview'),(detail,'icp_diagnostics')]:
        figure.savefig(args.output_dir/(name+'.png'),dpi=150)
        figure.savefig(args.output_dir/(name+'.pdf'))
    from LIPM.analysis.plot_double_support_analysis import plot_spatial
    try:
        fig3,fig2=plot_spatial(d,args.height,args.start,end,args.snapshot,args.icp_kind)
    except ValueError as exc:
        parser.error(str(exc))
    for figure,name in [(fig3,'LIP_3D'),(fig2,'LIP_2D')]:
        figure.savefig(args.output_dir/(name+'.pdf'),bbox_inches='tight')
        figure.savefig(args.output_dir/(name+'.png'),dpi=150,bbox_inches='tight')
    if not args.diagnostics:
        plt.close(fig); plt.close(detail)
    error=np.max(np.abs(d['dcm_residual']))
    print(f'Completed {model.steps} steps; max exact DCM propagation residual: {error:.3e} m')
    print('Saved:',args.output_dir.resolve())
    if args.headless: plt.close('all')
    else: plt.show()


if __name__ == '__main__':
    main()
