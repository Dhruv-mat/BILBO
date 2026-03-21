
## Overview

This is a custom drone I’ve been working on where I designed pretty much the entire frame from scratch instead of using a standard one.

The idea was to build something that can actually handle onboard compute (Raspberry Pi + OAK-D Lite) and not just be a basic RC drone. So the focus right now has mostly been on getting the structure right before moving into autonomy.

A lot of time went into redesigning parts that didn’t work. Especially the motor mounts and arm connectors — some of the earlier versions looked fine in CAD but would’ve probably failed instantly in real life.

The frame uses carbon fiber rods for the arms and landing gear, and 3D printed connectors to hold everything together. This makes it stronger than just printing the whole thing, and also easier to modify later.

Right now the hardware design is mostly done in CAD, and next step is printing + assembling + testing.

---

## Components

- Pixhawk 6C (flight controller)  
- Raspberry Pi 4 / 5  
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

Attach the arm connectors onto the base using M3 screws (I used ~16mm mostly). Make sure these are tight because this is where all the stress goes.

Now insert the carbon fiber rods into the connectors. The fit should already be tight, but I added a side hole so you can pass an M4 screw through and lock it in place.



---

Next is the motor side.

Each arm ends in a motor mount. My first design didn’t work because the alignment was off and there wasn’t enough support, so I switched to a more reinforced design (kind of like a wishbone support).

Attach the motors using standard screws and route the wires back toward the center.

---

Now for the frame stack.

Use M3 standoffs to mount the middle plate above the base. This is where the Pixhawk goes.

Then add another set of standoffs and attach the top plate. The Raspberry Pi sits on top.

---

Landing gear goes on the bottom.

It uses the same carbon rods with angled connectors. I didn’t want it to be completely rigid, so there’s a bit of flexibility to absorb impact when landing.

---

For the camera, mount the OAK-D Lite on the front using the custom holder.

It’s slightly angled forward so it gets a better view.

---

## Current Status

- Frame design mostly done  
- Arm connectors working  
- Motor mounts redesigned and fixed  
- Landing gear added  
- Electronics layout planned  

Still need to:
- print everything  
- assemble  
- test stability  
- integrate electronics  

---

## Notes

This took way more iteration than expected.

A lot of designs had to be scrapped after realizing they wouldn’t actually work under load. The motor mount especially had to be redone after noticing alignment issues.

Switching to carbon fiber rods instead of fully printed arms made a big difference. It’s way stronger and lighter.

Still a lot left to do but the structure is finally in a place where I can start building it properly.
