
## Overview

This is a custom drone I’ve been working on where I designed pretty much the entire frame from scratch instead of using a standard one.

The idea was to build something that can actually handle onboard compute (Raspberry Pi + OAK-D Lite) and not just be a basic RC drone. So the focus right now has mostly been on getting the structure right before moving into autonomy.

A lot of time went into redesigning parts that didn’t work. Especially the motor mounts and arm connectors, some of the earlier versions looked fine in CAD but would’ve probably not worked irl.

The frame uses carbon fiber rods for the arms and landing gear, and 3D printed connectors to hold everything together. This makes it stronger than just printing the whole thing, and also easier to modify later.

Right now the hardware design is mostly done in CAD, and next step is printing + assembling + testing.

<img width="1920" height="1080" alt="SUntitSSSDCFFFCCDDDCled1WSWW" src="https://github.com/user-attachments/assets/abb2c868-dd72-4a1b-86ca-1fb3d5f4d53e" />

<img width="1920" height="1080" alt="Untitled1" src="https://github.com/user-attachments/assets/8857a475-d68a-40c8-a35f-43c3ea660768" />



---

## Components

- Pixhawk 6C (flight controller)  
- Raspberry Pi 4B  
- OAK-D Lite (camera)  
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

<img width="796" height="486" alt="Screenshot 2026-03-21 125147" src="https://github.com/user-attachments/assets/320cc947-a3f6-47f2-b35d-92194f3eae6e" />


---

Now for the frame stack.
Use M3 standoffs to mount the middle plate above the base. This is where the Pixhawk goes.
Then add another set of standoffs and attach the top plate. The Raspberry Pi sits on top.

<img width="646" height="625" alt="Unteeeeeitled" src="https://github.com/user-attachments/assets/cafabd28-c59d-42ff-86f9-7970957c8b96" />



---

Landing gear goes on the bottom.
It uses the same carbon rods with angled connectors. I didn’t want it to be completely rigid, so there’s a bit of flexibility to absorb impact when landing.


<img width="1001" height="1089" alt="Screenshot 2026-03-21 125939" src="https://github.com/user-attachments/assets/6b0aa9ca-432d-417a-a026-d2d46343ced7" />

---

For the camera, mount the OAK-D Lite on the front using the custom holder.
It’s slightly angled forward so it gets a better view.


<img width="803" height="618" alt="Screenshot 2026-03-21 130130" src="https://github.com/user-attachments/assets/fe6d7273-4839-4457-b3ef-70838d0d62eb" />

---

## Electronics Overview

Here is the full system schematic for the drone, including power distribution, flight controller connections, and companion computer setup.
<img width="2006" height="1405" alt="Screenshot 2026-03-22 002108" src="https://github.com/user-attachments/assets/0f68d68e-0572-465d-b386-e866240a8834" />


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
| GPS Module (u-blox) | 1 | $40.96 |
| ESC (30A BLHeli) | 4 | $38.55 |
| 920KV Brushless Motors | 4 | $31.18 |
| Raspberry Pi 4 (8GB) | 1 | $144.58 |
| Pixhawk 6C | 1 | $361.45 |
| OAK-D Lite | 1 | $208.00 |
| LiPo Battery 3S 5200mAh | 1 | $35.00 |
| Power Distribution Board (PDB) | 1 | $5.00 |
| Holybro PM02 Power Module | 1 | $27.00 |
| Carbon Fiber Rod (25cm) | 1 | $22.00 |
| Carbon Fiber Tube (Landing Legs) | 1 | $11.70 |
| M3 Female Inserts | 20 | $1.00 |
| M4 Female Inserts | 20 | $1.00 |
| M3 Screw Set | 1 | $3.00 |
| M4 Screw Set | 1 | $3.00 |

**Total Estimated Cost: ~$1,058**
