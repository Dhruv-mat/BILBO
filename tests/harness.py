"""Stub the hardware libraries so the real BILBO logic can be exercised on any
machine, including a dev laptop with no camera, LiDAR or Pixhawk.

simple_pid is deliberately NOT stubbed. The yaw sign chain is the most important
thing these tests verify, and it depends on simple_pid computing
`error = setpoint - input`. Verifying that against a reimplementation would
prove nothing, so if the real library is missing the tests refuse to run.

    pip install simple-pid
    python tests/run_tests.py
"""

import importlib.util
import os
import sys
import types

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "detectors"))

if importlib.util.find_spec("simple_pid") is None:
    sys.stderr.write(
        "\nsimple_pid is required and must NOT be stubbed: these tests verify "
        "the yaw sign\nchain, which depends on its real error convention.\n"
        "Install it with:  pip install simple-pid\n\n"
    )
    raise SystemExit(2)


# ---------------------------------------------------------------- serial ----

class FakeSerial:
    def __init__(self, *a, **kw):
        self.buf = bytearray()
        self.closed = False
        self.kwargs = kw

    def feed(self, data):
        self.buf += data

    @property
    def in_waiting(self):
        return len(self.buf)

    def read(self, n):
        out = bytes(self.buf[:n])
        del self.buf[:n]
        return out

    def reset_input_buffer(self):
        self.buf = bytearray()

    def close(self):
        self.closed = True


serial_mod = types.ModuleType("serial")
serial_mod.Serial = FakeSerial
sys.modules.setdefault("serial", serial_mod)


# --------------------------------------------------------------- pi5neo ----

class FakeNeo:
    def __init__(self, *a, **kw):
        self.writes = []
        self._pending = None

    def fill_strip(self, r, g, b):
        # Three positional args, matching the real pi5neo signature. The
        # pre-review led.py passed a single tuple here, which raised TypeError
        # on every LED write.
        self._pending = (r, g, b)

    def update_strip(self):
        self.writes.append(self._pending)


pi5neo_mod = types.ModuleType("pi5neo")
pi5neo_mod.Pi5Neo = FakeNeo
sys.modules.setdefault("pi5neo", pi5neo_mod)


# ------------------------------------------------------------- pymavlink ----

if "pymavlink" not in sys.modules:
    mavlink_ns = types.SimpleNamespace(
        MAV_FRAME_BODY_NED=8,
        MAV_MODE_FLAG_SAFETY_ARMED=128,
        MAV_TYPE_ONBOARD_CONTROLLER=18,
        MAV_AUTOPILOT_INVALID=8,
        MAV_CMD_SET_MESSAGE_INTERVAL=511,
        MAV_DATA_STREAM_ALL=0,
        MAVLINK_MSG_ID_HEARTBEAT=0,
        MAVLINK_MSG_ID_SYS_STATUS=1,
        MAVLINK_MSG_ID_GLOBAL_POSITION_INT=33,
        MAVLINK_MSG_ID_RC_CHANNELS=65,
    )
    mavutil_mod = types.ModuleType("pymavlink.mavutil")
    mavutil_mod.mavlink = mavlink_ns
    mavutil_mod.mavlink_connection = lambda *a, **kw: None
    pymavlink_pkg = types.ModuleType("pymavlink")
    pymavlink_pkg.mavutil = mavutil_mod
    sys.modules["pymavlink"] = pymavlink_pkg
    sys.modules["pymavlink.mavutil"] = mavutil_mod


# ------------------------------------------------- picamera2 / libcamera ----

if "picamera2" not in sys.modules:
    picamera2_mod = types.ModuleType("picamera2")
    picamera2_mod.MappedArray = object
    picamera2_mod.Picamera2 = object
    devices_mod = types.ModuleType("picamera2.devices")
    devices_mod.IMX500 = object
    imx_mod = types.ModuleType("picamera2.devices.imx500")
    imx_mod.NetworkIntrinsics = object
    picamera2_mod.devices = devices_mod
    devices_mod.imx500 = imx_mod
    sys.modules["picamera2"] = picamera2_mod
    sys.modules["picamera2.devices"] = devices_mod
    sys.modules["picamera2.devices.imx500"] = imx_mod

if "libcamera" not in sys.modules:
    libcamera_mod = types.ModuleType("libcamera")
    libcamera_mod.Transform = lambda **kw: kw
    sys.modules["libcamera"] = libcamera_mod

if "cv2" not in sys.modules:
    cv2_mod = types.ModuleType("cv2")
    cv2_mod.FONT_HERSHEY_SIMPLEX = 0
    cv2_mod.rectangle = lambda *a, **kw: None
    cv2_mod.circle = lambda *a, **kw: None
    cv2_mod.putText = lambda *a, **kw: None
    sys.modules["cv2"] = cv2_mod


# -------------------------------------------------------- hermetic config ----

# config.BENCH_MODE is read from BILBO_BENCH_MODE at import time. Pin it off so
# the suite is deterministic no matter what the developer's shell happens to
# have set -- otherwise a run with bench mode enabled would silently disable
# the very engagement gates these tests exist to verify. The bench-mode section
# of test_state.py toggles it explicitly and restores it.
import config as _cfg  # noqa: E402

_cfg.BENCH_MODE = False


# ------------------------------------------------------------- utilities ----

def tf_frame(dist_cm, strength=1000, corrupt=False):
    """Build a TF Luna 9-byte frame: header, distance, strength, temp, checksum."""
    body = bytearray([0x59, 0x59,
                      dist_cm & 0xFF, (dist_cm >> 8) & 0xFF,
                      strength & 0xFF, (strength >> 8) & 0xFF,
                      0, 0])
    checksum = sum(body) & 0xFF
    if corrupt:
        checksum = (checksum + 1) & 0xFF
    return bytes(body + bytearray([checksum]))


class FakePerson:
    """Stands in for camera.Person without needing the IMX500."""

    def __init__(self, cx, cy, area, conf=0.9):
        self.center_x = float(cx)
        self.center_y = float(cy)
        self.area = int(area)
        self.confidence = conf
        self.width = int(area ** 0.5)
        self.height = int(area ** 0.5)
        self.x = int(cx - self.width / 2)
        self.y = int(cy - self.height / 2)

    def __repr__(self):
        return "P(x=%.0f,y=%.0f,a=%d)" % (self.center_x, self.center_y,
                                          self.area)


PASS, FAIL = [], []


def check(label, condition, detail=""):
    if condition:
        PASS.append(label)
        print("  PASS  %s %s" % (label, detail))
    else:
        FAIL.append(label)
        print("  FAIL  %s %s" % (label, detail))


def summary():
    print("\n%d passed, %d failed" % (len(PASS), len(FAIL)))
    if FAIL:
        print("FAILURES:")
        for f in FAIL:
            print("  - " + f)
    return 1 if FAIL else 0
