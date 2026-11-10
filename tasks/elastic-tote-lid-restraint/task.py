"Two YAM arms with parallel grippers stand on 16 x 16 x 20 cm rigid risers, bases at (0, +/-0.305, 0.95) m in the station frame. Table top z=0.7475 m. A fixed tray between the arms holds four initially loose straps: red 36 cm, blue 52 cm, yellow 58 cm, green 46 cm relaxed cord length. Each has two 50 g rigid hooks; white length cards are quarter-scale. The tall plastic tote at (0.50, 0) m has a 30 x 14 cm footprint, a blue 12 mm thick lid about 28 cm above the table when shut, 20 mm vertical lid travel, and two 6 mm diameter finite steel anchor bars on 8 cm brackets. The contents push upward by 30 N when shut and 15 N at full opening, excluding lid weight. A 26 x 14 cm low tote at (0.66, -0.42) m is a distractor. Initial hook poses, tray dimensions, tote solids and camera/mount geometry are specified by the packaged source; the fixed seed-42 layout must match the setup image.\n\nSecure the tall tote's lid using exactly one of the four straps. Seat its two hooks on the two different bars, with the cord over the lid. Release every hook and the lid, and finish with the lid gap below 3 mm continuously for 20 seconds. Do not overload a hook or let a taut loose hook snap back.\n\nRigid hooks, totes and lids use PhysX. A massless 8 mm analytic elastic cord follows the shortest route over lid edges; swept guards reject cord/solid penetration. Robot-cord collision and slack-shape dynamics are omitted; temporary cord/gripper contact can occur. Finite capsule anchor bars omit support hardware for axial entry. Contents are a spring-loaded sliding lid. Motor effort includes gravity within 27/7 Nm caps; fingers are limited to 22 N each. Hook overload is a 28 N failure proxy."
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent / "d1r_arena"))
import geometry as geo  # noqa: E402
from kit import Hook, StrapTray, Tote, current_score, install  # noqa: E402

install()
SCENE = "station"


def _tote(name):
    t = geo.tote(name)
    hx, hy = t["h"]
    return Tote(name=name, pos=tuple(float(v) for v in t["origin"]), yaw=float(t["yaw"]),
                size=(2 * (hy if abs(t["yaw"]) > 1 else hx), 2 * (hx if abs(t["yaw"]) > 1 else hy),
                      t["top"] - geo.NATIVE_TABLE_Z), mass=None)


_tray = geo.tray()
OBJECTS = [
    _tote("tote_tall"),
    _tote("tote_low"),
    StrapTray(name="strap_tray", pos=tuple(float(v) for v in _tray["origin"]), yaw=geo.YAW,
              size=(2 * geo.TRAY_H[1], 2 * geo.TRAY_H[0], 2 * geo.TRAY_T)),
    *[Hook(name=h["name"], pos=tuple(float(v) for v in h["pos"]), mass=geo.HOOK_MASS,
           size=(0.024, 0.04, 0.085)) for h in geo.hooks()],
]
SCORE_OVER = "end"


def score(objects):
    return current_score()


def demo():
    """Registration-only plan for --policy yam_scripted: both arms hold the ready pose, grippers open.

    NOT a full-score or feasibility demonstration (no strap is moved; the score stays 0).
    """
    return {}
