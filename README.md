
## Overview

This is a custom drone I’ve been working on where I designed pretty much the entire frame from scratch instead of using a standard one.

The idea was to build something that can actually handle onboard compute and not just be a basic RC drone. Initially I was planning to use a Raspberry Pi + OAK-D Lite for perception since it kind of does everything in one go, but that didn’t really work out because of availability and cost, so I had to change things a bit.

Right now the plan is to use a RPI + RPI AI HAT with a normal RGB camera for detection and a TF-Luna LiDAR for distance. It’s definitely not as clean as the OAK-D setup, and I’m not 100% sure how well it’ll perform yet, but it should be good enough if I can get the software side working properly.

A lot of time went into redesigning parts that didn’t work. Especially the motor mounts and arm connectors, some of the earlier versions looked fine in CAD but realistically would’ve probably failed.

The frame uses carbon fiber rods for the arms and landing gear, and 3D printed connectors to hold everything together. It’s way stronger than just printing everything, and also makes it easier to tweak later.

Right now the hardware design is mostly done in CAD, next step is printing and actually seeing if it works the way I think it will.

Right now the hardware design is mostly done in CAD, and next step is printing + assembling + testing.

This new idea of the the RGB camera + a lidar sensor was inspired by this video:
https://youtu.be/Nrzs3dQ9exw?si=6kdqJGR1x-Q6_Brh

<img width="1920" height="1080" alt="Untitledd" src="https://github.com/user-attachments/assets/bf2fa519-02d8-4d9d-a449-6fe57590ec33" />


<img width="1920" height="1080" alt="Untddditfffledd" src="https://github.com/user-attachments/assets/24aa3063-ef2e-44b8-9600-d94d55ce5929" />


---

## Components

- Pixhawk 6C (flight controller)  
- Nvidia Jetsone Nano 2GB
- RPI CAM V2
- TF Luna
- 920KV brushless motors x4  
- 20A / 30A ESCs x4  
- Carbon fiber rods (1.4 cm diameter)  
- 4S LiPo battery
- GPS MODULE
- Telemtry Kit

### Hardware

- M3 screws (10mm, 16mm, 20mm)  
- M4 screws (used for locking arms)  
- M3 female standoffs  
- M4 inserts  
- Nuts and washers  

---

## 3D Printed Parts

You’ll need to print:

- Bottom plate (main frame)  
- Middle plate (for Pixhawk mounting)  
- Top plate (for Raspberry Pi)  
- Arm connectors (holds carbon rods)  
- Motor mounts (final version is reinforced)  
- Landing gear mounts + joints  
- OAK-D Lite holder  
- Raspberry Pi holder  

---

## Assembly

Start with the base plate.

<img width="1072" height="582" alt="Screenshot 2026-03-21 124707" src="https://github.com/user-attachments/assets/a45b6fdc-85a7-4e97-b796-6204e724fa5f" />

Attach the arm connectors onto the base using M3 screws (I used ~16mm mostly). Make sure these are tight because this is where all the stress goes.

<img width="596" height="342" alt="Screenshot 2026-03-21 124714" src="https://github.com/user-attachments/assets/3b21c8b4-deac-4f49-bc46-32464b942541" />


Now insert the carbon fiber rods into the connectors. The fit should already be tight, but I added a side hole so you can pass an M4 screw through and lock it in place.

<img width="664" height="432" alt="Screenshot 2026-03-21 124919" src="https://github.com/user-attachments/assets/6798683c-924d-4ed9-a32e-ee382a1d37df" />


---

Next is the motor side.
Each arm ends in a motor mount. My first design didn’t work because the alignment was off and there wasn’t enough support, so I switched to a more reinforced design (kind of like a wishbone support).
Attach the motors using standard screws and route the wires back toward the center.

<img width="1178" height="919" alt="Screenshot 2026-04-05 235839" src="https://github.com/user-attachments/assets/89b1f8a1-6316-4333-8a43-4be4a7456f50" />



---

Now for the frame stack.
Use M3 standoffs to mount the middle plate above the base. This is where the Pixhawk goes.
Then add another set of standoffs and attach the top plate. The Raspberry Pi sits on top.

<img width="947" height="613" alt="Screenshot 2026-03-26 030339" src="https://github.com/user-attachments/assets/1ea828d9-3a98-455d-ab23-bbfe00178bef" />




---

Landing gear goes on the bottom.
It uses the same carbon rods with angled connectors. I didn’t want it to be completely rigid, so there’s a bit of flexibility to absorb impact when landing.


<img width="1001" height="1089" alt="Screenshot 2026-03-21 125939" src="https://github.com/user-attachments/assets/6b0aa9ca-432d-417a-a026-d2d46343ced7" />

---

For perception, mount the RGB camera on the front using a custom holder, slightly angled forward.
The TF-Luna LiDAR is mounted alongside it to provide distance measurements.

<img width="1317" height="910" alt="Screenshot 2026-03-29 021244" src="https://github.com/user-attachments/assets/0e7a7026-0717-4b69-a45a-4502a01bf34f" />

<img width="1920" height="1080" alt="Untddditfffledd" src="https://github.com/user-attachments/assets/87cbf282-2f2b-4c3e-bf94-5ef8b9fcfab6" />


---

## Electronics Overview

Here is the full system schematic for the drone, including power distribution, flight controller connections, and companion computer setup.
<img width="2006" height="1405" alt="Screenshot 2026-03-22 002108" src="https://github.com/user-attachments/assets/0f68d68e-0572-465d-b386-e866240a8834" />


---

## Frame Stress Testing

Also I ran an a basic static stress test on the frame to get a rough idea of how it behaves under load. applied upward forces at the motor mounts to simulate thrust and checked deformation and stress distribution across the arms and connectors. Most of the stress seems to concentrate near the joints and motor mounts which i kinda expected, but nothing looks completely crazy or like it’ll instantly fail. deformation is pretty small overall, so structurally it seems fine for now, although this is still a simplified test and real-world vibrations and impacts aren’t accounted for yet.

![WhatsApp Image 2026-04-07 at 09 31 21](https://github.com/user-attachments/assets/e2ea1788-4ed3-47c8-bc7b-ec8591b2cea4)

![WhatsApp Image 2026-04-07 at 09 31 22](https://github.com/user-attachments/assets/9a3d937e-9e1d-48d8-8684-7a00127cb9e6)



---
## Notes

This took way more iteration than expected.
A lot of designs had to be scrapped after realizing they wouldn’t actually work under load. The motor mount especially had to be redone after noticing alignment issues.
Switching to carbon fiber rods instead of fully printed arms made a big difference. It’s way stronger and lighter.
Still a lot left to do but the structure is finally in a place where I can start building it properly.


## Bill of Materials (BOM)

| Part | Quantity | Price (USD) |
|------|--------|------------|
| Propellers (Pair) | 2 | $2.02 |
| RC Receiver | 1 | $45.78 |
| Telemetry Kit | 1 | $77.11 |
| GPS Module (u-blox) | 1 | $40 |
| ESC (30A BLHeli) | 4 | $38.55 |
| 920KV Brushless Motors | 4 | $31 |
| RPI AI HAT 26TOPS | 1 | $100 |
| Pixhawk 6C | 1 | $360 |
| Rpi Cam V2 | 1 | $18 |
| TF Luna | 1 | $22 |
| LiPo Battery 3S 5200mAh | 1 | $35 |
| Power Distribution Board (PDB) | 1 | $5|
| Holybro PM02 Power Module | 1 | $27|
| Carbon Fiber Rod (25cm) | 1 | $22 |
| Carbon Fiber Tube (Landing Legs) | 1 | $11.7 |
| M3 Female Inserts | 20 | $1 |
| M4 Female Inserts | 20 | $1|
| M3 Screw Set | 1 | $3 |
| M4 Screw Set | 1 | $3 |

