# BILBO — flying guide

Props off until §5. Read §6 (things going wrong) *before* you fly, not after.

---

## 1. Transmitter setup

| Channel | Function | Set on the Tx as |
|---|---|---|
| 1–4 | roll / pitch / throttle / yaw | normal sticks |
| **7** | **flight mode** (`FLTMODE_CH = 7`) | 6-position switch |
| **8** | **AI enable** (`CH_ENABLE = 8`) | **2-position** switch |
| **10** | **arm / disarm** (`RC10_OPTION = 153`) | 2-position switch, ideally spring-loaded |

The AI switch is now simply **OFF / ON**. There is no middle position any more — on means full tracking, yaw and forward.

### Flight modes you must configure

Your params have LOITER on `FLTMODE6` already, but **no GUIDED and no RTL**. Two values to change:

| Param | Now | Set to | Mode |
|---|---|---|---|
| `FLTMODE1` | `0` | `0` | STABILIZE |
| `FLTMODE2` | `0` | `0` | STABILIZE |
| `FLTMODE3` | `0` | **`6`** | **RTL** |
| `FLTMODE4` | `2` | `2` | ALT_HOLD |
| `FLTMODE5` | `0` | **`4`** | **GUIDED** — required for autonomy |
| `FLTMODE6` | `5` | `5` | **LOITER** — your escape position |

LOITER sits at position 6, one end of the switch travel, with GUIDED at 5 next to it — so escaping is one click, in a known direction. You want your escape mode to be the position your thumb finds without looking.

---

## 2. Arming: the thing to be clear about first

**A drone cannot fly unless it is armed.** Armed means the motors are allowed to
spin. If it is not armed, it is sitting on the ground with the props still.

So there is no such thing as "it drifted away and I hadn't armed it." If it is
in the air, it is armed, and because it is armed **every RC control is live** —
the mode switch, the sticks, RTL, all of it. The Pi never arms anything;
`drone.arm()` exists but `main.py` never calls it. You arm it, you fly it up,
and only then do you hand it to the Pi.

The one that catches people: **in GUIDED your sticks do nothing.** Roll, pitch,
yaw and throttle are all ignored while the Pi is commanding. If the drone starts
misbehaving and you shove the stick, nothing will happen. **You must change mode
first.** That is the single most important reflex to build.

## 2b. Arming order

**You cannot arm in GUIDED.** Rudder arming is disabled in GUIDED/AUTO, and GUIDED needs a position estimate to enter at all. So:

```
1. Power on, wait for GPS lock (outdoors)
2. Flight mode  -> STABILIZE  (or ALT_HOLD)
3. ARM          -> ch10 switch, or full-right rudder
4. Take off manually, climb to 3-5 m, hover steady
5. Flight mode  -> GUIDED           (drone holds position)
6. ch8          -> ON               (tracking engages)
```

Steps 5 and 6 are separate on purpose. GUIDED alone does nothing — the Pi still needs the switch. Two deliberate actions before the drone moves itself.

**To stop, at any time: flight mode to LOITER.** That is the real emergency stop, and it works even if the Pi has crashed, hung, or is on fire, because ArduPilot ignores guided setpoints outside GUIDED.

### One thing not to do

`ARMING_RUDDER = 2` in your params means full-left rudder **disarms**. ArduPilot
gates that on the vehicle being landed, so it should not fire in flight — but if
the drone starts spinning, "hold left rudder to counter it" is exactly the
instinct you might have, and in GUIDED that stick is doing nothing useful
anyway. Set **`ARMING_RUDDER = 1`** (arm only, never disarm by stick) and the
question disappears.

### If you lose the RC link as well

Then the Pixhawk's own failsafes are all that is left, which is why §6's
parameter list matters. `FS_THR_ENABLE = 1` is already set, so RC loss triggers
RTL on its own.

### Deliberately disarming in flight

It exists, it drops the aircraft out of the sky, and it is only ever the right
answer if the drone is heading somewhere that would hurt someone and nothing
else has worked. Accept that you are destroying the aircraft. Do not treat it as
a routine option.

---

## 3. What the drone must see before it will engage

If you flip ch8 on and nothing happens, the log says which gate is holding:

```
engage held: not armed
engage held: altitude 0.4 < 2.0 m
engage held: target unconfirmed (1/3)
```

All four must be true: switch cycled off→on, armed, above `MIN_TRACK_ALT_M` (2 m), and a person confirmed for 3 consecutive frames.

The switch must be **cycled**. Booting with it already on will not engage — flip it off and on.

---

## 4. LED meanings

| Colour | State |
|---|---|
| white | IDLE — not in GUIDED |
| blue | READY — in GUIDED, waiting for ch8 |
| green | TRACKING |
| yellow blinking | SEARCHING — target lost, sweeping |
| purple | RTL |
| red blinking | EMERGENCY |
| orange blinking | fault (link lost, repeated errors) |
| **two white blips every 3 s** | **the control loop is alive** |

**If the white blips stop, the software has stalled.** That is the most important indicator on the aircraft. The colour may be stale; the blips cannot be.

---

## 5. Ground test order — props OFF

```bash
python detectors/bench.py leds                 # no Pixhawk needed
python detectors/bench.py link                 # heartbeat, streams, RC
python detectors/bench.py switch               # confirm ch8 OFF/ON bands
python detectors/bench.py sensors              # LiDAR + camera health
python tools/verify_yaw_sign.py                # THE critical one
python detectors/bench.py motors --throttle 15 # props off, type REMOVED
python detectors/bench.py track                # tracking preview
```

For `verify_yaw_sign`: stand to the drone's **physical right**, confirm it reports image RIGHT and nose RIGHT. If it reports nose LEFT, **do not fly** — flip `config.YAW_PID_OUTPUT_SIGN`.

### Seeing the preview

The preview needs a display. Either:

- **HDMI monitor** on the Pi — just works.
- **VNC** — `sudo raspi-config` → Interface → VNC → Enable, then connect and run from the desktop terminal.
- **SSH with no display** — use `--no-preview` for a text readout instead.

Over plain SSH the window cannot appear; `DISPLAY` isn't set. That is not a bug.

---

## 6. When it goes wrong in the air

### The ladder, in order of how much you should trust it

1. **Flight mode → LOITER.** This is the panic button. LOITER *holds position by
   itself* using GPS — the drone stops dead and hovers, and you do not have to
   fly it. It takes effect the instant the switch moves, because ArduPilot
   ignores the Pi's setpoints outside GUIDED. Your `FLTMODE6 = 5` is already
   LOITER. **Practise finding this switch position without looking.**
2. **Flight mode → RTL.** Executed by the Pixhawk, works even with a dead Pi.
   Use it if LOITER is holding but you want it home.
3. **Flight mode → STABILIZE / ALT_HOLD.** Use if GPS is bad and LOITER will not
   hold. Note this hands you a drifting aircraft you must actively fly, which is
   why it is below LOITER rather than above it.
4. **ch8 → OFF.** Only works if the Pi is alive and reading RC — precisely the
   thing that has failed in most of the scenarios you would need it for. It is
   the weakest link in the chain. **Do not reach for it first.**

Everything above 4 works because the aircraft is armed and ArduPilot trusts the
RC link above everything else. None of them depend on the Pi being healthy.

### Prove it before you trust it

On the first flight, **before** you ever touch ch8: take off in STABILIZE, climb
to a few metres, flip to GUIDED, confirm the drone just sits there holding
position, then flip straight back to LOITER. You have now tested your whole
escape path with the Pi doing nothing. Only then engage tracking.

### Specific symptoms

| It's doing this | Why | Do this |
|---|---|---|
| Spinning and not stopping | yaw sign inverted, or lost target and searching | **LOITER**. Then re-run `verify_yaw_sign.py` on the ground |
| Flying at you | suspect a bad LiDAR reading, or the safety floor failing | **LOITER**, then land |
| Drifting off, ignoring you | Pi crashed; ArduPilot brakes after ~3 s (GUID_TIMEOUT) and holds | **LOITER**, then RTL |
| Frozen, hovering | Pi stalled — check whether the white LED blips stopped | **LOITER** |
| LEDs went orange | link lost or repeated faults; it has stopped commanding | **LOITER**, land, read the log |
| Nothing at all on ch8 | check the log for which gate is holding (§3) | land, `bench.py switch` |

### If you lose contact entirely

The Pixhawk's own failsafes are what save the aircraft, not the Pi. **These are worth setting before you fly:**

```
BATT_FS_LOW_ACT   = 2      (RTL on low battery -- currently 0 = do nothing)
BATT_FS_CRT_ACT   = 1      (LAND on critical)
FENCE_ENABLE      = 1      (your fence is already configured: 300 m, 100 m alt, RTL)
```

`FENCE_ENABLE = 1` is the single highest-value change you can make. It is the only backstop that no Pi bug can defeat — every failure mode ends at the fence.

Also worth setting on the FTDI's port, if it is TELEM1:

```
BRD_SER1_RTSCTS   = 0      (a 3-wire FTDI must not use auto flow control)
```

---

## 7. Standoff and the distance deadband

| Setting | Value | Meaning |
|---|---|---|
| `TARGET_DISTANCE_CM` | **200** | the standoff it aims to hold |
| `DIST_DEADBAND_CM` | **20** | 180–220 cm counts as "close enough" |
| `MIN_SAFE_DISTANCE_CM` | **150** | hard floor: backs off regardless |

Behaviour across the range:

```
 <150 cm   forced back-off, whatever the tracker wants
150-180    back away, proportionally
180-220    HOLD -- zero output, no micro-corrections
220-300    move forward, proportionally
 >300 cm   full forward (1.0 m/s cap)
```

The band is why the drone will sit still rather than twitching: inside 180–220
the controller commands exactly nothing. The edges are continuous, so crossing
out of the band gives ±0.01 m/s, not a jump.

`forward` negative simply means "closer than 200 cm, backing off". That is the
controller working. The bench overlay shows `want 200 cm` beside the measured
range so the comparison is visible.

**The trade in going from 4 m to 2 m:** detection is much better (bigger
bounding box, more reliable IMX500 output), but 2 m is close to a person with
props turning, and the safety floor is only 50 cm below the standoff. Brief the
person you are tracking, and keep the first flights over open ground.

## 8. Autostart

`bilbo.service` in the repo root. Edit the username and paths, then:

```bash
sudo cp bilbo.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable bilbo
sudo systemctl start bilbo
journalctl -u bilbo -f
```

**Think before enabling this.** Autostart means the tracking program is running every time the battery goes on, including while you are carrying the aircraft. It cannot engage without GUIDED + armed + altitude + the switch, so it is safe by construction — but during bring-up you probably want `systemctl start` manually and only `enable` once you trust it.

Flight logs land in `~/bilbo-logs/`: `bilbo.log` for humans, `flight-<timestamp>.csv` at full rate. The CSV is what diagnoses an anomaly; keep them.
