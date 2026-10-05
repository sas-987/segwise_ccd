"""Precomputed/mock SDF adapter for CCD tests before the real SDF exists."""
import numpy as np
class GridSDF:
    def __init__(self,sdf,grad_x,grad_y,resolution,origin_x=0.0,origin_y=0.0):
        self.grid=np.asarray(sdf,float); self.grad_x=np.asarray(grad_x,float); self.grad_y=np.asarray(grad_y,float); self.resolution=float(resolution); self.origin_x=float(origin_x); self.origin_y=float(origin_y)
    @classmethod
    def from_npz(cls,path):
        d=np.load(path); return cls(d["sdf"],d["grad_x"],d["grad_y"],d["resolution"],d["origin_x"],d["origin_y"])
    def _wc(self,x,y): return (x-self.origin_x)/self.resolution,(y-self.origin_y)/self.resolution
    @staticmethod
    def _bilinear(g,c,r):
        h,w=g.shape; c=float(np.clip(c,0,w-1.001)); r=float(np.clip(r,0,h-1.001)); c0,r0=int(np.floor(c)),int(np.floor(r)); c1,r1=c0+1,r0+1; fc,fr=c-c0,r-r0; v0=g[r0,c0]*(1-fc)+g[r0,c1]*fc; v1=g[r1,c0]*(1-fc)+g[r1,c1]*fc; return float(v0*(1-fr)+v1*fr)
    def distance(self,x,y): c,r=self._wc(x,y); return self._bilinear(self.grid,c,r)
    def gradient(self,x,y):
        c,r=self._wc(x,y); g=np.array([self._bilinear(self.grad_x,c,r),self._bilinear(self.grad_y,c,r)],float); n=np.linalg.norm(g); return g/n if n>=1e-9 else np.zeros(2)
