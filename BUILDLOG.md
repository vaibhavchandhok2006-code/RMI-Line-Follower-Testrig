# Build Log

Progressive log of design decisions, what was tried, what changed, and why. 
---

### Aug 7 — Project start
- Allotted the Line Follower project, built a personal roadmap , timeline.
- Started learning Github.
- Set up this GitHub repo.

### Aug ~10-13 — Kinematics
- Learnt about differential  drive kinematics.
- Worked through how sensor mounting offset from the wheel axis affects control

### Aug 13 — Chassis and sensor design constraints research 
- Did a literature survey , used AI tools find the most relevant research papers , links , videos for the project knowledge
- Drafted design requirements sheet: 38mm line, sensor array width, 5-8 sensors, 10-19mm spacing, 5-8mm mount height, front-mounted array ahead of wheel axle.
- Made a rough component list
- Through the learnings from the literature survey defined design requirements 

### Aug 13-15 — Sensor hardware verification
- Checked actual RLS-05/RLS-08 datasheets against assumed design numbers. Found the initially planned mounting height (8-12mm) was too high for the real sensors' recommended sensing distance (RLS-08: ~6mm, RLS-05: ~3mm optimal). Corrected target to 5-6mm with adjustable mount.
- Computed array width and critical entry angle for available options:
  - RLS-08 full 8ch: 105mm width 
- Decision: proceeded with full RLS-08 (8ch) after confirming inventory availability — to be validated against actual measured track .


### Aug 15-20 — Circuit architecture, CAD Design and component finalization
- Finalized BOM: confirmed availability of ESP32, N20 motors w/ encoders, caster wheel, motor mounts, RLS-08, TB6612FNG,Battery , Buck converter and mechanical hardware in RMI inventory.
- Designed power architecture: 2S LiPo (7.4V nominal) → power switch → splits to (a) TB6612FNG VM pin directly, (b) LM2596 buck converter → 5V rail → ESP32 VIN + N20 encoder VCC + level shifter HV side.
- Caught a real issue: N20 encoder outputs run at 5V logic, which would be unsafe wired directly into ESP32's 3.3V-only GPIOs. Added a 4-channel logic level shifter to bring encoder signals down to 3.3V.
- Assigned all 8 RLS-08 analog channels to ESP32 ADC1 pins (GPIO32-39) specifically because ADC2 becomes unreliable once Wi-Fi is active — needed for the telemetry stretch objective.
- Routed battery voltage divider to GPIO25 (ADC2) as a deliberate exception, since battery monitoring doesn't need real-time precision the way line sensing does.
- Sized battery voltage divider (10kΩ / 5.6kΩ) so that worst-case 8.4V (full charge) maps to ~3.02V at the ESP32 ADC pin, safely under the 3.3V limit.
- Computed peak current budget (~3A instantaneous: 2× motor stall + Wi-Fi-active ESP32 + sensor array) to size the battery/driver requirements.
- Built the first draft of the Chasis

### Aug 20 -22 — CAD assembly
- Donwloaded the n20 motor , mounts , wheel cad models from grabcad
- Built first-pass chassis assembly in SolidWorks: N20 motors, motor mounts, wheels assembled onto chassis baseplate.


### Aug 23 — Design review

### Aug 26 — Circuit reiteration: component spec audit

Design review on the 23rd landed mid-CT, so a lot of it was deadline-chasing rather than actually thinking things through. Today's the real reiteration pass — went back to first-party specs for every major component instead of the assumed numbers from before.

- **RLS-08 IR array**: 100–150mA draw, rated 5V, gives 0–5V analog out (per Robu.in). Also runs at 3.3V, but only if the LED current-limiting resistor stage (two-stage arrangement on-board) is shorted/bypassed — needs on-board soldering rework.(to be confirmed)
  - If the bypass works: 3.3V is sufficient, no voltage divider needed on the 8 analog lines.
  - If not: fall back to 5V off the buck converter + 8× voltage dividers to bring the ADC lines to 3.3V logic.
  - Not decided yet — will try the 3.3V supply.
- **N20 motor**: 6V rated, 200RPM. No-load current 30mA, rated 60mA, stall 230mA.
- **N20 encoder**: red/blue = GND + motor power, black = encoder VCC (3.3–5V logic), yellow/green = signal feedback (logic-high tracks whatever VCC is supplied). Going with 3.3V logic to match ESP32 GPIOs directly.
- **TB6612FNG driver**: 1.2A continuous / 3.2A peak per channel, Vmotor max 15V, logic supply 2.7–5.5V. Logic-high threshold is 0.7×VCC — this is the actual reason 3.3V logic was chosen over 5V: at 3.3V VCC the threshold sits low enough for ESP32's native 3.3V GPIO high to drive it cleanly. At 5V VCC, the threshold would sit above what a 3.3V GPIO can output.
- **Battery**: 7.4V 2S LiPo, 850mAh, 25C continuous / 50C burst, ~21A max discharge.
- **ESP32**: onboard LDO accepts 5–12V in, fed from the buck-converted 5V rail → 3.3V LDO out. Max output 250mA , per GPIO - 20mA recommended, Wi-Fi/BT TX peak current 240–500mA — factored into the current budget.
- **Buck converter (LM2596)**: 2A continuous, 3A max — heatsink required above ~2A.

**Open question:** whether to feed the raw 7.4V LiPo rail straight into ESP32 Vin. Technically within the 5–12V input range but still need to confirm , some online source suggested that ideally 5-7V should be fed into esp32 LDO.

Updated circuit architecture diagram reflects all of the above: 7.4V rail → power switch → splits three ways (buck → 5V rail, direct to motor driver Vm, divider → battery-voltage ADC). Pin assignments locked: encoders on GPIO13/14/27/26, motor driver PWM+direction on 16/17/18/21/22/23, RLS-08 analog on GPIO32-39 (ADC1), push button + status LED on 2/4/5.

Next: once the RLS-08 power path is settled , will move to CAD assemble re - ideation .

### Aug 27–29 — Chassis v2: split design, adjustable sensor arm

(Didn't log day-by-day through this stretch — heads-down on CAD. Catching up in one entry covering the full 3-day iteration.)

The previous chassis was rushed to hit the design review deadline and wasn't actually good. This is the real redesign.

**Split into two parts** instead of one solid baseplate:
1. **Back chassis plate** — motor mounts + PCB area + battery mount
2. **Adjustable IR sensor arm** — separate piece, bolts on

**Back chassis plate**
- Previous version was one full rectangular block, which carried dead weight on either side of the motors for no reason. New version thins those sections down to just what the motor mounts need, cutting that mass out.
- PCB footprint minimized to 80mm × 60mm to keep mass down; PCB sits on standoffs ~1.5–2cm above the plate.
- 4mm thick, printed at 60% infill.
- The PCB Mounting fall are made as slots rather than fixed holes if a smaller PCB is possible to make it can mounted and hence the mounting holes are adjustable.

**Adjustable IR sensor arm:**
- Look-ahead distance adjustable from 8cm to 20cm, with 1cm-spaced grooves as cut guides. Once track calibration shows the ideal look-ahead distance, the arm gets trimmed to length and bolted to the chassis plate through two front mounting holes.
- Originally planned to mount this *below* the back plate, but that removed the ground clearance for the sensor head and removed any adjustability — moved it above instead.
- 3mm thick (kept minimal since it's not load-bearing), printed at 70–80% infill to compensate for the reduced thickness.
- Caster wheel sits directly below the arm's front section, with a 2mm gap between the arm's underside and the caster when mounted. No threaded mount for the caster — will be secured with double-sided tape instead of screws.

**Test prints completed:** caster wheel mounting hole, bolt diameter tolerances, IR sensor array mount. Final full-chassis prints still pending.

**Custom CAD parts modeled from scratch** (no usable models existed online): LiPo battery, caster wheel, IR sensor array — built to complete the full assembly for fit-checking.

**Full assembly**
<img width="1033" height="716" alt="image" src="https://github.com/user-attachments/assets/cc73a375-0408-4ce3-8a2b-10d4de3223b7" />

**Chasis Main Back Plate**

<img width="608" height="560" alt="image" src="https://github.com/user-attachments/assets/42123887-d6b8-4801-8923-a7390f19f48e" />

**Adjustable IR Sensor array part with Castor wheel and cutting grooves**
<img width="915" height="550" alt="image" src="https://github.com/user-attachments/assets/012f56c6-bd82-4157-8dd6-064fbe0eb37d" />

**Chassis assembly complete, all parts 3D printed**
- All chassis parts printed in PLA 3D Printer and assembled: baseplate, N20 motor mounts, wheels, caster mount.
- Final assembly photo:
<img width="3296" height="2686" alt="IMG_20260829_163451 jpg" src="https://github.com/user-attachments/assets/a01a2c25-733a-4b43-b0bb-c1a91124235f" />


- Mechanical build phase complete. Moving into electronics wiring and ESP-IDF firmware development next.

### Sep 1 — ESP-IDF setup, first GPIO program
- ESP-IDF installed and toolchain working (separate learning curve from Arduino IDE — different build system, no hand-holding on peripheral config).
- Wrote first "Hello World" program to confirm the flash/build/monitor workflow.
- Wrote first LED blink program using raw GPIO driver calls (gpio_reset_pin, gpio_set_level, vTaskDelay) instead of Arduino's digitalWrite/delay abstractions.
- Decision: spending few days on ESP-IDF fundamentals before touching sensor/motor code, given how different the peripheral driver model is from Arduino. Following an online ESP-IDF tutorial series to build this foundation properly rather than rushing into the project code cold.

## not yet resolved
- RLS-08 power path: 3.3V direct (needs resistor-stage bypass mod) vs 5V + 8× voltage dividers — pending 3.3V feasibility test
- ESP32 Vin: raw 7.4V LiPo direct vs regulated 5V rail — undecided (raw only possible if the rls sensor runs at 3.3V)

