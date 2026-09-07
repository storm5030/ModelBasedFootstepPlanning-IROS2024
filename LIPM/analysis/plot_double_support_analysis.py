"""Standalone spatial figures matching the original LIP_2D/LIP_3D analysis."""
# Allow both direct script execution and python -m from the repository root.
if __package__ in (None, ''):
    import sys
    from pathlib import Path as _Path
    sys.path.insert(0, str(_Path(__file__).resolve().parents[2]))

import numpy as np


def plot_spatial(data, height, start, end, snapshot=None, icp_kind='instantaneous'):
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Wedge, FancyArrowPatch
    t=data['time']; mask=(t>=start-1e-9)&(t<=end+1e-9)
    ids=np.flatnonzero(mask)
    if len(ids)<2: raise ValueError('Select at least two samples')
    chosen=(start+end)/2 if snapshot is None else snapshot
    if not start <= chosen <= end: raise ValueError('snapshot must be within the plot time window')
    k=ids[np.argmin(abs(t[ids]-chosen))]
    c=data['position']; feet=data['feet']; z=data['zmp']
    key={'instantaneous':'icp','step-end':'predicted_step_end_icp','ssp-end':'predicted_ss_end_icp'}[icp_kind]
    xi=data[key]
    label={'instantaneous':'Instantaneous ICP','step-end':'Predicted DSP-end ICP','ssp-end':'Predicted SSP-end ICP'}[icp_kind]
    events=np.flatnonzero(np.r_[False,np.diff(data['step_num'])!=0])
    events=events[(t[events]>=start)&(t[events]<=end)]
    # Initial planning point within the view plus subsequent planning events.
    planning=np.unique(np.r_[ids[0],events])
    targets=data['target'][planning]
    points=np.vstack([c[mask],xi[mask],targets,feet[k,:,:2]])
    points=points[np.isfinite(points).all(axis=1)]
    low,high=points.min(axis=0)-[0.08,0.12],points.max(axis=0)+[0.08,0.10]
    radius=0.025
    style={'font.family':'serif','font.size':12,'mathtext.fontset':'dejavuserif'}
    with plt.rc_context(style):
        fig3=plt.figure(figsize=(10,8)); ax3=fig3.add_subplot(111,projection='3d')
        fig2,ax2=plt.subplots(figsize=(10,5))
        ax3.plot(c[mask,0],c[mask,1],np.full(mask.sum(),height),color='green',lw=2,label='CoM Trajectory')
        ax3.plot(c[mask,0],c[mask,1],np.zeros(mask.sum()),color='green',ls='--',alpha=.8,label='Projected CoM')
        ax2.plot(c[mask,0],c[mask,1],color='green',ls='--',lw=2,label='CoM Trajectory')
        ax3.plot(xi[mask,0],xi[mask,1],np.zeros(mask.sum()),color='#b5b500',ls='--',lw=2,label=label)
        ax2.plot(xi[mask,0],xi[mask,1],color='#b5b500',ls='--',lw=2,label=label)
        # Keep the DSP-specific addition subtle: ZMP path and snapshot marker.
        ax3.plot(z[mask,0],z[mask,1],np.zeros(mask.sum()),color='darkorange',lw=.8,alpha=.6,label='ZMP')
        ax2.plot(z[mask,0],z[mask,1],color='darkorange',lw=.8,alpha=.6,label='ZMP')
        for side,color,name in [(0,'blue','Desired Left Step'),(1,'red','Desired Right Step')]:
            take=(1-data['support'][planning])==side
            p=targets[take]
            ax3.scatter(p[:,0],p[:,1],np.zeros(len(p)),color=color,marker='x',s=55,label=name)
            ax2.scatter(p[:,0],p[:,1],color=color,marker='x',s=55,label=name)
            ax3.plot([c[k,0],feet[k,side,0]],[c[k,1],feet[k,side,1]],[height,feet[k,side,2]],color='black',lw=1.5)
            ax3.scatter(*feet[k,side],color=['cyan','magenta'][side],s=45)
        for hemi in range(2):
            phi=np.linspace(hemi*np.pi/2,(hemi+1)*np.pi/2,15)
            for quarter in range(4):
                theta=np.linspace(quarter*np.pi/2,(quarter+1)*np.pi/2,15)
                th,ph=np.meshgrid(theta,phi)
                ax3.plot_surface(c[k,0]+radius*np.sin(ph)*np.cos(th),c[k,1]+radius*np.sin(ph)*np.sin(th),
                                 height+radius*np.cos(ph),color='black' if (quarter+hemi)%2==0 else 'white',linewidth=0)
        ax2.add_patch(Circle(c[k],radius,facecolor='white',edgecolor='black',zorder=5))
        for angle in [0,180]: ax2.add_patch(Wedge(c[k],radius,angle,angle+90,color='black',zorder=6))
        cmd=data['command'][k]; arrow=.4*cmd
        ax3.quiver(c[k,0],c[k,1],height+.06,arrow[0],arrow[1],0,color='#70ed00',label='Velocity Command')
        ax2.quiver(c[k,0],c[k,1],arrow[0],arrow[1],angles='xy',scale_units='xy',scale=1,color='#70ed00',zorder=7)
        # Annotate a completed step. Distances are measured values, not hard-coded references.
        if len(events):
            interior=events[t[events]>start+0.1*(end-start)]
            e=interior[0] if len(interior) else events[0]; pre=e-1
            new=data['feet'][e,data['support'][e],:2]
            old=data['feet'][e,1-data['support'][e],:2]
            endpoint=data['icp'][e]
            def arrow_between(p,q,text,offset):
                ax2.add_patch(FancyArrowPatch(p,q,arrowstyle='<->',mutation_scale=12,lw=.8,color='black',zorder=8))
                mid=(np.asarray(p)+q)/2+offset
                ax2.text(*mid,text,ha='center',va='center',fontsize=11,zorder=9,
                         bbox=dict(facecolor='white',edgecolor='none',alpha=.7,pad=.4))
            # Axis components; for nonzero heading these are explicitly world-axis distances.
            ybar=max(new[1],old[1])+.055
            arrow_between(np.array([old[0],ybar]),np.array([new[0],ybar]),r'$s_x$',np.array([0,.023]))
            xbar=min(old[0],new[0])-.025
            arrow_between(np.array([xbar,old[1]]),np.array([xbar,new[1]]),r'$w_y$',np.array([-.032,0]))
            corner=np.array([endpoint[0],new[1]])
            arrow_between(new,corner,r'$b_x$',np.array([0,-.045]))
            arrow_between(corner,endpoint,r'$b_y$',np.array([.045,0]))
            ax2.scatter(*endpoint,color='#b5b500',s=20,zorder=8)
            ax2.annotate(r'$(p_x,p_y)$',new,xytext=(-10,-22),textcoords='offset points',fontsize=10,ha='right')
            ax2.annotate(r'$(\xi_x,\xi_y)$',endpoint,xytext=(8,10),textcoords='offset points',fontsize=10)
            # Use conventional desired-step labels only when the measured values agree.
            if np.isclose(data['command'][pre,1],0):
                annotations=ax2.texts
                for txt in annotations:
                    if txt.get_text()==r'$s_x$' and np.isclose(abs(new[0]-old[0]),data['dstep_length'][pre],atol=1e-5): txt.set_text(r'$s_d$')
                    if txt.get_text()==r'$w_y$' and np.isclose(abs(new[1]-old[1]),data['dstep_width'][pre],atol=1e-5): txt.set_text(r'$w_d$')
        ax2.set(xlim=(low[0],high[0]),ylim=(low[1],high[1]),xlabel='x (m)',ylabel='y (m)')
        ax2.set_aspect('equal'); ax2.grid(ls='--',alpha=.55)
        ax2.legend(loc='lower center',bbox_to_anchor=(.5,1.03),ncol=3,fontsize=9)
        ax3.set(xlim=(low[0],high[0]),ylim=(low[1],high[1]),zlim=(-.01,height+.10),xlabel='x (m)',ylabel='y (m)',zlabel='z (m)')
        ax3.set_box_aspect([high[0]-low[0],high[1]-low[1],height+.11]); ax3.view_init(20,-130)
        for axis in [ax3.xaxis,ax3.yaxis,ax3.zaxis]: axis.set_pane_color((1,1,1,0))
        ax3.legend(loc='upper center',bbox_to_anchor=(.5,1.03),ncol=3,fontsize=9)
        phase=data['phase'][k]
        fig2.text(.5,.02,f't = {t[k]:.2f} s ({phase}) | window {start:g}-{end:g} s',ha='center',fontsize=10)
        fig3.text(.5,.02,f't = {t[k]:.2f} s ({phase}) | window {start:g}-{end:g} s',ha='center',fontsize=10)
        fig2.tight_layout(rect=(0,.04,1,.94)); fig3.subplots_adjust(top=.89,bottom=.07)
    return fig3,fig2
