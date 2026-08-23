# Build Log

Progressive log of design decisions, what was tried, what changed, and why. 
---

### Aug 7 — Project start
- Allotted the Line Follower project, built a personal roadmap , timeline.
- Started learning Github.
- Set up this GitHub repo.

### Aug ~10-13 — Kinematics deep-dive
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

---

## not yet resolved
- Firmware / ESP-IDF development has not yet started
