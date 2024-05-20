import numpy as np
from functools import partial

dX = np.array([1.,0.,0.])
dY = np.array([0.,1.,0.])
dZ = np.array([0.,0.,1.])

def rodrigues_r_R(r, theta):
    kx, ky, kz = r
    K = np.array([
        [0, -kz, ky],
        [kz, 0, -kx],
        [-ky, kx, 0]
    ])
    return np.identity(3) + np.sin(theta) * K + (1 - np.cos(theta)) * (K @ K)

def deg_to_rad(*args):
    rads = []
    for arg in args:
        rads.append(arg * np.pi / 180)
    return rads

def rad_to_deg(*args):
    degs = []
    for arg in args:
        degs.append(arg * 180 / np.pi)
    return degs


class Transform():

    def __init__(self, pose):
        self._pose = pose
    
    @classmethod
    def id(cls):
        pose = np.array([
            [1,0,0,0],
            [0,1,0,0],
            [0,0,1,0],
            [0,0,0,1],
        ], dtype = np.float32)
        return cls(pose)
    
    def apply(self, vectors):
        if vectors.shape[-1] == 3:
            _vectors = np.concatenate([vectors, np.ones_like(vectors[...,0:1])], axis=-1)
        _vectors = np.tensordot(self._pose, _vectors, axes=[[-1],[-1]]).transpose(1,2,0)
        if vectors.shape[-1] == 3:
            _vectors = _vectors[...,:-1]
        return _vectors

    @classmethod
    def from_Rxyx(self, rx1, ry, rx2):
        X_1 = Transform.from_axis_angle(dX, rx1)
        Y = Transform.from_axis_angle(dY, ry)
        X_2 = Transform.from_axis_angle(dX, rx2)
        return X_1 @ Y @ X_2

    @classmethod
    def from_t(cls, t):
        T = cls.id()
        T._pose[:3,3] = t
        return Transform(T._pose)

    @classmethod
    def from_axis_angle(cls, axis, angle):
        T = cls.id()
        R = rodrigues_r_R(axis, angle)
        T._pose[:3,:3] = R
        return Transform(T._pose)

    @classmethod
    def from_t_r(cls, t, r):
        frame = cls.id()
        frame._pose[:3,3] = t
        frame._pose[:3,:3] = r
        return frame

    def R_to_XYX(self):
        y = np.arccos(self._pose[0,0])

    def __repr__(self):
        return f"TRANSFORM\nTranslation:\n{self.t()}\nRotation:\n{self.r()}"

    def __matmul__(self, other):
        return Transform(self._pose @ other._pose)

    def t(self):
        return self._pose[:3,3].T

    def r(self):
        return self._pose[:3,:3]
    
    def inv(self):
        return Transform(np.linalg.inv(self._pose))

    def quiver(self):
        X,Y,Z = np.stack(3*[self.t()]).T
        U,V,W = self.r()
        return X,Y,Z,U,V,W

    def draw_on_ax(self, ax, scale=0.1, c_i=0):
        colors = ["red", "green", "blue", "grey", "orange", "purple"]*5
        x,y,z,u,v,w = self.quiver()
        return ax.quiver(x,y,z,u,v,w, length=scale, color=["red", "green", "blue"]+6*[colors[c_i]])

class Cylinder():

    def __init__(self, radius, length, pose: Transform, color="red", x_mode=False, wf=False, strides=(1,1)):

        self.wf = wf
        self.color = color
        self.strides = strides

        if x_mode:
            pose = pose @ Transform.from_axis_angle(dY, np.pi/2)

        u = np.linspace(0, 2 * np.pi, 8)
        v = np.linspace(0, length, 2)
        r = np.linspace(0, radius, 2)

        X = radius * np.outer(np.ones(v.size), np.cos(u))
        Y = radius * np.outer(np.ones(v.size), np.sin(u))
        Z = np.outer(v, np.ones(u.size))

        disk_X = np.outer(r, np.cos(u))
        disk_Y = np.outer(r, np.sin(u))
        disk_Z = np.zeros_like(disk_X)
        disk_ZZ = length * np.ones_like(disk_X)

        self.X, self.Y, self.Z = pose.apply(np.stack([X,Y,Z], axis=-1)).transpose(2,0,1)
        self.disk_X, self.disk_Y, self.disk_Z = pose.apply(np.stack([disk_X,disk_Y,disk_Z], axis=-1)).transpose(2,0,1)
        self.disk_XX, self.disk_YY, self.disk_ZZ = pose.apply(np.stack([disk_X,disk_Y,disk_ZZ], axis=-1)).transpose(2,0,1)
   
    def plot_on_ax(self, ax):

        f = partial(ax.plot_wireframe, rstride=self.strides[0], cstride=self.strides[1]) if self.wf else ax.plot_surface

        self.span = f(self.X, self.Y, self.Z, color=self.color)
        self.bottom_cap = f(self.disk_X, self.disk_Y, self.disk_Z, color=self.color)
        self.top_cap = f(self.disk_XX, self.disk_YY, self.disk_ZZ, color=self.color)
