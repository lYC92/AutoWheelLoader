"""15-state error-state inertial filter with delayed LiDAR pose updates."""
from collections import deque
import copy
import numpy as np
from scipy.spatial.transform import Rotation


def skew(v):
    x,y,z=v
    return np.array([[0,-z,y],[z,0,-x],[-y,x,0.]])


class InertialFilter:
    def __init__(self,position=(0,0,0),yaw=0):
        self.p=np.array(position,dtype=float);self.v=np.zeros(3)
        self.q=Rotation.from_euler('z',yaw);self.bg=np.zeros(3);self.ba=np.zeros(3)
        self.P=np.diag([.05**2]*3+[.2**2]*3+[.05**2]*3+[.01**2]*3+[.1**2]*3)
        self.time=None;self.last_lidar=-np.inf;self.accepted=0;self.rejected=0
        self.history=deque();self.last_measurement={}

    def snapshot(self):
        return (self.time,self.p.copy(),self.v.copy(),self.q.as_quat().copy(),self.bg.copy(),self.ba.copy(),self.P.copy())

    def restore(self,state):
        self.time,self.p,self.v,q,self.bg,self.ba,self.P=copy.deepcopy(state)
        self.q=Rotation.from_quat(q)

    def predict(self,time,acceleration,gyro,remember=True):
        acceleration=np.asarray(acceleration,dtype=float);gyro=np.asarray(gyro,dtype=float)
        if not np.isfinite(np.r_[time,acceleration,gyro]).all(): raise ValueError('non-finite IMU')
        if self.time is None: self.time=time
        dt=time-self.time
        if not 0<=dt<=.1: raise ValueError('IMU clock gap or reversal')
        omega=gyro-self.bg; specific=acceleration-self.ba
        rotation=self.q.as_matrix()
        world_acc=rotation@specific+np.array([0,0,-9.80665])
        self.p+=self.v*dt+.5*world_acc*dt*dt;self.v+=world_acc*dt
        self.q=self.q*Rotation.from_rotvec(omega*dt)
        F=np.zeros((15,15));F[:3,3:6]=np.eye(3)
        F[3:6,6:9]=-rotation@skew(specific);F[3:6,12:15]=-rotation
        F[6:9,6:9]=-skew(omega);F[6:9,9:12]=-np.eye(3)
        transition=np.eye(15)+F*dt
        noise=np.diag([1e-8]*3+[.08**2]*3+[.004**2]*3+[1e-8]*3+[1e-6]*3)
        self.P=transition@self.P@transition.T+noise*dt
        self.time=time
        if remember:
            self.history.append((self.snapshot(),acceleration.copy(),gyro.copy()))
            while self.history and time-self.history[0][0][0]>1.0: self.history.popleft()

    def correct(self,time,position,quaternion,indices=None,variances=None,channel="lidar"):
        if not self.history or time<=self.last_measurement.get(channel,-np.inf): return False
        index=min(range(len(self.history)),key=lambda i:abs(self.history[i][0][0]-time))
        if abs(self.history[index][0][0]-time)>.015: self.rejected+=1;return False
        current=self.snapshot();entries=list(self.history);self.restore(entries[index][0])
        residual=np.r_[np.asarray(position)-self.p,(self.q.inv()*Rotation.from_quat(quaternion)).as_rotvec()]
        if not np.isfinite(residual).all(): self.restore(current);self.rejected+=1;return False
        H=np.zeros((6,15));H[:3,:3]=np.eye(3);H[3:,6:9]=np.eye(3)
        R=np.diag([.03**2]*3+[.015**2]*3 if variances is None else variances)
        if indices is not None:
            ids=list(indices);residual=residual[ids];H=H[ids];R=R[np.ix_(ids,ids)]
        S=H@self.P@H.T+R
        if float(residual@np.linalg.solve(S,residual))>30:
            self.restore(current);self.rejected+=1;return False
        K=np.linalg.solve(S,H@self.P).T;delta=K@residual
        self.p+=delta[:3];self.v+=delta[3:6]
        self.q=self.q*Rotation.from_rotvec(delta[6:9]);self.bg+=delta[9:12];self.ba+=delta[12:]
        # Joseph update keeps the covariance symmetric and positive semidefinite.
        A=np.eye(15)-K@H;self.P=A@self.P@A.T+K@R@K.T
        reset=np.eye(15);reset[6:9,6:9]-=.5*skew(delta[6:9])
        self.P=reset@self.P@reset.T
        entries[index]=(self.snapshot(),entries[index][1],entries[index][2])
        for k in range(index+1,len(entries)):
            state,acc,gyro=entries[k]
            self.predict(state[0],acc,gyro,remember=False)
            entries[k]=(self.snapshot(),acc,gyro)
        self.history=deque(entries);self.last_measurement[channel]=time
        if channel=="lidar":self.last_lidar=time;self.accepted+=1
        return True

    def healthy(self,now):
        return (self.time is not None and self.accepted>=3 and
                0<=now-self.time<.1 and 0<=now-self.last_lidar<.5 and
                float(np.max(np.diag(self.P)[:3]))<.15**2)
