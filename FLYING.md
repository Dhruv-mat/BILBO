# BILBO — flying guide

Props off until §5. Read §6 (things going wrong) *before* you fly, not after.

---

## 1. Your channel map

| Ch | Function | ArduPilot param |
|---|---|---|
| 1–4 | roll / pitch / throttle / yaw | — |
| **7** | flight mode, 3-position | `FLTMODE_CH = 7` |
| **8** | **RTL** | **`RC8_OPTION = 4`** |
| **9** | **GUIDED *and* AI enable** | `RC9_OPTION = 55`, `CH_ENABLE = 9` |
| **10** | arm / disarm | `RC10_OPTION = 153` |

### Why FLTMODE3 did nothing

ArduPilot splits the mode channel into **six PWM bands**, but you have a
**3-position switch**, so it can only ever produce three of them:

| ch7 PWM | Band | Your switch |
|---|---|---|
| ≤1230 | FLTMODE1 | top — STABILIZE |
| 1231–1360 | FLTMODE2 | *unreachable* |
| 1361–1490 | **FLTMODE3** | ***unreachable*** |
| 1491–1620 | FLTMODE4 | middle — ALT_HOLD |
| 1621–1749 | FLTMODE5 | *unreachable* |
| ≥1750 | FLTMODE6 | bottom — LOITER |

Your switch produces 1025 / 1515 / 2004, hitting bands **1, 4 and 6 only**.
`FLTMODE3 = 6` is set but can never be selected. Set it back to `0` if you like;
it is harmless either way.

**RTL goes on ch8 as an aux switch instead:** `RC8_OPTION = 4`. Aux switches do
not use the mode channel's bands at all, so this works with any spare 2-position
switch.

### One switch for GUIDED and the AI

ch9 now does both: it selects GUIDED on the Pixhawk *and* is what the Pi reads
as its enable (`CH_ENABLE = 9`).

- **ch9 UP** — hands the aircraft to the Pi
- **ch9 DOWN** — takes it back, and returns to whatever ch7 says

One switch, one meaning. There is no longer a state where GUIDED is selected but
the Pi sits idle, and no way to leave autonomy armed after dropping out of
GUIDED.

**What you lose, and how to get it back.** You can no longer sit in GUIDED with
the Pi deliberately idle. The target-confirmation gate covers it: raise ch9 with
**nobody in front of the camera** and the aircraft enters GUIDED and holds
position without engaging, because it has no confirmed target. That is exactly
the "does GUIDED hold properly?" check you want before handing it the person.

> **Park ch7 on LOITER before you raise ch9.** ch9-down returns you to whatever
> ch7 says. On LOITER that means the aircraft stops and holds by itself. On
> STABILIZE it means you are suddenly hand-flying a drifting drone.

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

## 2b. The flight sequence

Automatic takeoff is enabled (`AUTO_TAKEOFF = True`). You arm; the aircraft
flies its own climb; tracking starts at altitude. You never hand-fly it.

```
1. GPS lock (outdoors), ch9 DOWN
2. ch7  -> bottom (LOITER)     sets your escape destination FIRST
3. ch10 -> ARM                 the one manual step, deliberately
4. ch9  -> UP                  GUIDED + AI: climbs to 2.2 m, then tracks
```

Four actions, one of them manual. To stop at any point: **ch9 DOWN**.

**Arming stays manual on purpose.** It is the one transition where a software
fault spins propellers with nobody expecting it, so a human stays in that loop.
Everything after it is automatic.

**Step 2 before step 4.** ch9-down returns you to whatever ch7 says, so park it
on LOITER before you ever raise ch9.

**Start with ch9 DOWN.** Engagement is edge-triggered, so a switch already
high at boot cannot engage until it is cycled.

### What you will see

| LED | Meaning |
|---|---|
| solid white | IDLE — not in GUIDED |
| solid blue | READY — waiting for ch8 |
| **blinking green** | **TAKEOFF — climbing, Pixhawk flying** |
| solid green | TRACKING |

During the climb the Pi sends **nothing**. ArduPilot's guided takeoff is an
altitude controller, and a velocity setpoint arriving mid-climb switches the
guided submode and abandons it — the aircraft would stop climbing wherever it
happened to be. So the Pi stays quiet and just watches the altitude.

### Takeoff altitude

`TAKEOFF_ALT_M = 2.2` in config.py, one line.

**Read this once before you fly it at 2.2 m.** You are 1.83 m tall and
barometric altitude drifts around ±0.5 m, so the propeller disc can end up
roughly 0.3 m above your head — or lower. 3.0 m or more is materially safer for
the same tracking behaviour. Whatever you choose, **the tracked person must
never walk underneath the aircraft.**

If you change it, `MIN_TRACK_ALT_M` must stay below
`TAKEOFF_ALT_M - TAKEOFF_ALT_TOLERANCE_M` or the engagement gate will reject the
altitude the takeoff just delivered. `tests/test_invariants.py` asserts this.

### If the climb goes wrong

| Condition | What happens |
|---|---|
| Takeoff command rejected | EMERGENCY -> hold -> RTL -> LAND |
| Never reaches altitude in 20 s | EMERGENCY, same ladder |
| ch8 switched OFF mid-climb | back to READY, hovers, you take over |
| Not armed, or already airborne | refuses to engage, logs why |

And ch9-down still works at any point during the climb — it drops out of GUIDED
into LOITER regardless of what the Pi is doing.

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

### Start here: the acceptance report

```bash
python tools/preflight_report.py
```

One command. It exercises every module, hammers the control path with 3800
deliberately absurd input combinations, measures the live LiDAR, camera and
MAVLink link, and writes `~/bilbo-logs/preflight-<timestamp>.csv` with one row
per check. It ends with a verdict: **ALL CHECKS PASSED**, **CLEARED WITH
WARNINGS**, or **DO NOT FLY** with the reasons listed.

It never commands motion. Absurd velocities are tested against a capture shim so
nothing reaches the wire; the only thing it transmits is one zero-velocity
setpoint, and it refuses even that while armed. It never spins a motor.

Stand in front of the camera when it asks, and move side to side — it needs to
see you on both sides of centre to confirm the yaw sign works in both
directions. Send me the CSV if anything fails.

Then the individual tools:


```bash
python detectors/bench.py leds                 # no Pixhawk needed
python detectors/bench.py link                 # heartbeat, streams, RC
python detectors/bench.py switch               # confirm ch8 actually moves
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

1. **Ch9 DOWN.** Fastest exit from GUIDED — one switch, Pi out of the loop
   immediately. Lands you in whatever ch7 says, which is why ch7 lives on
   LOITER. **Practise this one until it is muscle memory.**
2. **Ch7 to LOITER (bottom).** Works even with ch9 still up, because a
   flight-mode change overrides the aux switch. Your second independent exit.
3. **Ch7 to your RTL position** (once you have set one). Pixhawk flies it home,
   works with a dead Pi.
4. **Ch7 to STABILIZE (top).** Only if GPS is bad and LOITER will not hold. This
   hands you a drifting aircraft to hand-fly, which is why it is down here.
5. **Ch8 -> RTL.** Aux switch, executed by the Pixhawk. Only works if the Pi is alive and reading RC —
   precisely what has failed in most scenarios where you would want it. **Do not
   reach for this first.**

Options 1–4 all work because the aircraft is armed and ArduPilot trusts the RC
link above everything else. None of them need the Pi to be healthy, or even
running.

### Prove it before you trust it

On the first flight, **before** you ever touch ch8: take off, climb to a few
metres, park ch7 on LOITER, raise ch9 to GUIDED, confirm the drone just sits
there holding position — then drop ch9 and confirm it stays put in LOITER.

You have now tested your entire escape path with the Pi commanding nothing. Do
that twice. Only then flip ch8.

### Specific symptoms

| It's doing this | Why | Do this |
|---|---|---|
| Spinning and not stopping | yaw sign inverted, or lost target and searching | **Ch9 DOWN.** Then re-run `verify_yaw_sign.py` on the ground |
| Flying at you | suspect a bad LiDAR reading, or the safety floor failing | **Ch9 DOWN**, then land |
| Drifting off, ignoring you | Pi crashed; ArduPilot brakes after ~3 s (GUID_TIMEOUT) and holds | **Ch9 DOWN**, then ch7 to RTL |
| Frozen, hovering | Pi stalled — check whether the white LED blips stopped | **Ch9 DOWN** |
| LEDs went orange | link lost or repeated faults; it has stopped commanding | **Ch9 DOWN**, land, read the log |
| Nothing at all on ch8 | check the log for which gate is holding (§3) | land, `bench.py switch` — is a switch actually mapped to ch8? |

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
