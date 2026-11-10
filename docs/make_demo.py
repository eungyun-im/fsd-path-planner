"""Run main_node.FSDPlannerNode offline (ROS stubbed) on a synthetic cone track and plot the result."""
import sys, types, time, math
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

SRC, OUT = sys.argv[1], sys.argv[2]

# ---------- minimal ROS stubs ----------
class Auto:
    def __init__(self, **kw):
        self.__dict__.update(kw)
    def __getattr__(self, k):
        if k.startswith("__"):
            raise AttributeError(k)
        v = Auto(); setattr(self, k, v); return v

class _T:
    def __init__(self, t): self.t = t
    def __sub__(self, o): return Auto(to_sec=lambda d=self.t - o.t: d)

PUB = {}
class Publisher:
    def __init__(self, topic, *a, **k): self.topic = topic
    def publish(self, msg): PUB.setdefault(self.topic, []).append(msg)
    def get_num_connections(self): return 1

rospy = types.ModuleType("rospy")
rospy.init_node = lambda *a, **k: None
rospy.get_param = lambda name, default=None: default
rospy.Subscriber = lambda *a, **k: None
rospy.Publisher = Publisher
rospy.Time = Auto(now=lambda: _T(time.time()))
rospy.Duration = lambda *a: a
for f in ["logwarn_throttle", "logerr_throttle", "logdebug", "logerr", "loginfo"]:
    setattr(rospy, f, lambda *a, **k: None)
rospy.ROSInterruptException = Exception

class Marker(Auto):
    LINE_STRIP, LINE_LIST, POINTS, SPHERE, ADD = 4, 5, 8, 2, 0
    def __init__(self): super().__init__(points=[])
class MarkerArray(Auto):
    def __init__(self): super().__init__(markers=[])
def Point(x=0, y=0, z=0): return Auto(x=x, y=y, z=z)

vm = types.ModuleType("visualization_msgs"); vmm = types.ModuleType("visualization_msgs.msg")
vmm.Marker, vmm.MarkerArray = Marker, MarkerArray
gm = types.ModuleType("geometry_msgs"); gmm = types.ModuleType("geometry_msgs.msg"); gmm.Point = Point
sm = types.ModuleType("std_msgs"); smm = types.ModuleType("std_msgs.msg")
smm.Float32 = lambda data=0.0: Auto(data=data); smm.Int32 = lambda data=0: Auto(data=data)
sys.modules.update({"rospy": rospy, "visualization_msgs": vm, "visualization_msgs.msg": vmm,
                    "geometry_msgs": gm, "geometry_msgs.msg": gmm, "std_msgs": sm, "std_msgs.msg": smm})
sys.path.insert(0, SRC)
import main_node  # noqa: E402

# ---------- synthetic S-curve track (vehicle frame, heading +x) ----------
s = np.linspace(0, 16, 400)
cy = 2.2 * np.sin(s / 5.0) * (s / 16)
dx, dy = np.gradient(s), np.gradient(cy)
n = np.hypot(dx, dy); nx, ny = -dy / n, dx / n
half = 1.7
arc = np.concatenate([[0], np.cumsum(np.hypot(np.diff(s), np.diff(cy)))])
idx = [np.argmin(abs(arc - d)) for d in np.arange(1.5, arc[-1], 2.5)]
left = np.c_[s[idx] + nx[idx] * half, cy[idx] + ny[idx] * half]
right = np.c_[s[idx] - nx[idx] * half, cy[idx] - ny[idx] * half]
cones = np.vstack([left, right])

node = main_node.FSDPlannerNode()
msg = Marker(); msg.header.frame_id = "os_sensor"
msg.points = [Point(x, y, 0) for x, y in cones]
for _ in range(30):
    PUB.clear()
    time.sleep(0.01)
    node.callback_cones(msg)

best = np.array([[p.x, p.y] for p in PUB["/fsd_planner/best_path"][-1].points])
cands = [np.array([[p.x, p.y] for p in m.points]) for m in PUB["/fsd_planner/candidate_paths"][-1].markers]
dl = PUB.get("/fsd_planner/delaunay_viz", [])
edges = np.array([[p.x, p.y] for p in dl[0].points]) if dl else np.empty((0, 2))
mids = np.array([[p.x, p.y] for p in dl[1].points]) if len(dl) > 1 else np.empty((0, 2))
tgt = PUB["/fsd_planner/target_point"][-1].pose.position
steer = PUB["/cone/steer"][-1].data; speed = PUB["/cone/speed"][-1].data

# ---------- plot ----------
fig, ax = plt.subplots(figsize=(11, 5.2), dpi=150)
for i in range(0, len(edges) - 1, 2):
    ax.plot(edges[i:i + 2, 0], edges[i:i + 2, 1], color="#cbd5e1", lw=0.8, zorder=1)
for i, c in enumerate(cands):
    ax.plot(c[:, 0], c[:, 1], color="#94a3b8", lw=1, alpha=0.6, zorder=2,
            label="Candidate paths" if i == 0 else None)
if len(mids):
    ax.scatter(mids[:, 0], mids[:, 1], s=10, color="#f472b6", zorder=3, label="Delaunay / Voronoi points")
ax.plot(best[:, 0], best[:, 1], color="#16a34a", lw=3, zorder=4, label="Selected path")
ax.scatter(left[:, 0], left[:, 1], s=70, color="#2563eb", edgecolor="k", zorder=5, label="Blue cones")
ax.scatter(right[:, 0], right[:, 1], s=70, color="#facc15", edgecolor="k", zorder=5, label="Yellow cones")
ax.scatter([tgt.x], [tgt.y], s=140, marker="*", color="#dc2626", zorder=6, label="Pure-pursuit target (KF)")
ax.annotate("", xy=(1.2, 0), xytext=(0, 0), arrowprops=dict(arrowstyle="-|>", lw=2.5, color="k"), zorder=6)
ax.text(0.01, 0.97, f"steer = {steer:+.1f}°   speed cmd = {speed}", transform=ax.transAxes,
        va="top", fontsize=10, family="monospace", bbox=dict(fc="white", ec="#e5e7eb"))
ax.set_aspect("equal"); ax.grid(alpha=0.3)
ax.set_xlabel("x [m] (forward)"); ax.set_ylabel("y [m] (left)")
ax.set_title("main_node.py on a synthetic S-curve track (ROS stubbed, 30 frames)")
ax.legend(loc="upper left", bbox_to_anchor=(1.01, 1), fontsize=8, framealpha=0.95)
fig.tight_layout(); fig.savefig(OUT)
print("saved", OUT, "| best path pts", len(best), "| candidates", len(cands), "| steer", round(steer, 2), "| speed", speed)
