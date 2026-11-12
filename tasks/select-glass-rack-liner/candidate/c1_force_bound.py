"""Conservative Coulomb-contact bound, not per-contact vector reconstruction."""
import math,numpy as np

def effective_mu_bound(coefficients):
 values=np.asarray(coefficients,float)
 if values.size==0 or not np.isfinite(values).all() or np.any(values<0):raise ValueError('Invalid material coefficient bound')
 # Bounds min, max, average and multiply combine modes. Floor1 also covers
 # the installed PhysX default0.5 when a shape has no explicit material.
 mu=max(1.,float(values.max()));return max(mu,mu*mu)

def contact_norm_upper_bound(normal_magnitudes,mu_bound):
 n=np.asarray(normal_magnitudes,float)
 if not np.isfinite(n).all() or np.any(n<0) or not math.isfinite(mu_bound) or mu_bound<0:raise ValueError('Invalid Coulomb load bound')
 # For every contact |t|<=mu*n, hence |n_vector+t|<=n*sqrt(1+mu²)
 # for orthogonal normal/tangent components; summing preserves the bound.
 return float(n.sum())*math.sqrt(1.+mu_bound*mu_bound)
