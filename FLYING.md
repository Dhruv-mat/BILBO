# BILBO — flying guide

Props off until §5. Read §6 (things going wrong) *before* you fly, not after.

---

## 1. Your channel map

| Ch | Function | Positions |
|---|---|---|
| 1–4 | roll / pitch / throttle / yaw | sticks |
| **7** | **flight mode** (`FLTMODE_CH = 7`) | top STABILIZE / middle ALT_HOLD / bottom **LOITER** |
| **8** | **AI enable** (`CH_ENABLE = 8`) | needs a 2-position switch — see below |
| **9** | **GUIDED** (aux switch, `RC9_OPTION = 55`) | up = GUIDED, down = back to ch7 |
| **10** | **arm / disarm** (`RC10_OPTION = 153`) | 2-position |

### The thing that matters most about ch9

Ch9 is an *auxiliary mode switch*, not a flight-mode position. ArduPilot treats
those differently, and the difference is your entire escape plan:

- **Ch9 UP** → GUIDED.
- **Ch9 DOWN** → the aircraft returns to **whatever ch7 is currently set to**.

So ch9-down is your fastest way out of GUIDED — one switch, and the Pi is out of
the loop instantly. But *where you land* is decided by ch7.

> **Park ch7 on LOITER (bottom) before you ever raise ch9.**
>
> Then ch9-down drops you straight into LOITER, which holds position on its own.
> If ch7 is left on STABILIZE and you panic-flip ch9 down, you get a drifting
> aircraft you now have to hand-fly. Same switch, completely different outcome.

Moving ch7 while ch9 is up also works — the mode switch overrides — so you have
two independent ways out.

### Ch8 needs a switch assigned

You said ch8 currently does nothing. That is exactly right on the *Pixhawk* side
(`RC8_OPTION = 0` means ArduPilot ignores it and leaves it for the Pi), but the
Pi still needs a real switch producing a real pulse on ch8 from your
transmitter. Assign a 2-position switch to ch8, then confirm:

```bash
python detectors/bench.py switch
```

It must read roughly 1100 (OFF) and 1900 (ON). If ch8 is unassigned it will sit
at a fixed value or read nothing, and the AI will simply never engage.

`RC8_OPTION` must stay **0**. Anything else and ArduPilot would grab the channel
for its own aux function and fight the Pi for it.

### Ch8 reading a constant 1025 us

That is the channel sitting at its low endpoint with **no switch mapped to it on
the transmitter**. `RC8_OPTION = 0` is correct on the Pixhawk side — it means
ArduPilot ignores ch8 and leaves it for the Pi — but the Pi still needs a real
pulse to read. Map a 2-position switch to ch8 on the Tx and re-run
`bench.py switch`; it must move between roughly 1100 and 1900.

Until it moves, `read_enable()` returns OFF forever and tracking can never
engage. That is the fail-closed behaviour working as intended, not a fault.

### If `armed` flickers True/False on the bench

Fixed in the code, but worth knowing what it was: a MAVLink link is a shared bus.
If a GCS or telemetry radio is on the same link, ArduPilot routes its heartbeats
to the Pi's port too — and a GCS heartbeat has `base_mode = 0`, i.e. not armed.
The Pi was reading `armed` from *any* heartbeat, so a foreign one cleared the
flag. `mode` looked stable through the same fault only because pymavlink already
filters GCS heartbeats out of its own mode tracking.

`drone.py` now accepts telemetry only from the autopilot (matching system id,
component 1, and not a GCS/companion type). `bench.py link` prints message rates
**per source**, so if there is more than one node on your link you will see it:

```
  sys 1   comp 1    HEARTBEAT      2.0 Hz  (10)
  sys 255 comp 190  HEARTBEAT      1.0 Hz  (5)     <-- a GCS, ignored

  2 MAVLink nodes on this link: [(1, 1), (255, 190)]
```

### One param still missing

Your param file has no **RTL** on the mode switch. Set `FLTMODE_CH` position 2
(`FLTMODE2 = 6`) or any spare position, so you have a hands-off "bring it home"
that works with a dead Pi.

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
2. ch7          -> top (STABILIZE)
3. ch10         -> ARM
4. Take off manually, climb to 3-5 m, hover steady
5. ch7          -> bottom (LOITER)   <-- sets your escape destination FIRST
6. ch9          -> UP (GUIDED)       <-- drone holds position, Pi not engaged
7. ch8          -> ON                <-- tracking engages
```

Step 5 is the one people skip. It costs nothing and it decides where you end up
when you bail out. Do it before ch9, not after.

Steps 6 and 7 are separate on purpose. GUIDED alone does nothing — the Pi still
needs the enable switch. Two deliberate actions before the drone moves itself.

**To stop, at any time: ch9 DOWN** (with ch7 parked on LOITER). That is the real emergency stop, and it works even if the Pi has crashed, hung, or is on fire, because ArduPilot ignores guided setpoints outside GUIDED.

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
5. **Ch8 OFF.** *Weakest.* Only works if the Pi is alive and reading RC —
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
