"""Conservative softened dipoles; estimated mechanism, NOT measured Apple magnets.

No pose snapping, capture flags, latches, or externally supplied success states.
Pair forces derive from U; intrinsic torques rotate dipoles, with application
point moments added about each body's origin. Interactions include all sites.
"""
import numpy as np

PENCIL_MASS = .0182
IPAD_MASS = .293
LENGTH, RADIUS, FLAT = .166, .00445, .0030
IPAD_SIZE = np.array([.1954, .1348, .0063])
PENCIL_SITES = np.array([[-.045, -.0022, 0.], [.045, -.0022, 0.]])
IPAD_SITES = np.array([[-.045, .0664, 0.], [.045, .0664, 0.]])
SOFTEN = .0004
C = 1e-7
# Sets approximately 1.2 N total normal pull at the authored correct-face seat.
# This magnitude, embed depths and site locations are assumptions, not product data.
MOMENT = .0036

def pair(r, ma, mb):
    """Force on dipole a (r=pa-pb), torques at dipole centres, potential J."""
    s2 = np.dot(r, r) + SOFTEN ** 2
    s3, s5, s7 = s2 ** 1.5, s2 ** 2.5, s2 ** 3.5
    a, b, c = np.dot(ma, r), np.dot(mb, r), np.dot(ma, mb)
    force = 3*C*((c*r + b*ma + a*mb)/s5 - 5*a*b*r/s7)
    ba = C*(3*b*r/s5 - mb/s3)
    bb = C*(3*a*r/s5 - ma/s3)
    return force, np.cross(ma, ba), np.cross(mb, bb), C*(c/s3-3*a*b/s5)

def wrench(pp, rp, pi, ri, scale=1.):
    fp, tp, fi, ti = [np.zeros(3) for _ in range(4)]
    energy = 0.
    ma, mb = rp @ np.array([0., MOMENT, 0.]), ri @ np.array([0., MOMENT, 0.])
    for ap in PENCIL_SITES:
        ra = rp @ ap
        for ai in IPAD_SITES:
            rb = ri @ ai
            f, ta, tb, u = pair(pp+ra-pi-rb, ma, mb)
            fp += f; fi -= f
            tp += ta + np.cross(ra, f)
            ti += tb - np.cross(rb, f)
            energy += u
    return fp*scale, tp*scale, fi*scale, ti*scale, energy*scale

def pencil_vertices():
    """D-section body, tapered last 11 mm; metre vertices for convex hull.

    Flat width/depth and tip taper are visually plausible estimates. Full extent
    is 166 x 8.9 x 7.45 mm in x,z,y, with the flat on local negative y.
    """
    yz=[]
    for angle in np.linspace(0,2*np.pi,64,endpoint=False):
        y,z=RADIUS*np.cos(angle),RADIUS*np.sin(angle)
        if y >= -FLAT: yz.append([y,z])
    z = np.sqrt(RADIUS**2-FLAT**2)
    yz.extend([[-FLAT,z],[-FLAT,-z]])
    return np.array([[x,y,z] for x in [-LENGTH/2, LENGTH/2-.011] for y,z in yz]
                    + [[LENGTH/2,y,z] for y,z in [[-.0005,0],[.0005,0],[0,-.0005],[0,.0005]]])
