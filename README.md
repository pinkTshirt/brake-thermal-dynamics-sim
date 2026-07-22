# 🏎️ Brake Dynamics & Thermal Lab

This is an interactive web app designed to simulate the physics and temperatures of a baja vehicles's braking system 

---

## Core Features

### 1. Settings
Pre-set values of previous year data
* **Vehicle Weight:** Set to a lightweight $245\text{ kg}$ buggy.
* **Rotational Factor ($k = 1.15$):** Accounts for the extra energy needed to stop spinning parts like wheels and rotors.
* **Brake Bias Setup:** Simulates a realistic $75\%$ front and $25\%$ rear brake balance.
* **Dual Mass Tracking:** Separates the full rotor mass from the actual "rubbing surface" to show both average cooling temps and instant surface hot-spots.

### 2. Live Telemetry Chart
* Powered by **Chart.js**, the graph updates live every 50 milliseconds.
* It simultaneously tracks your **Speed**, **Front Rotor Temperature**, and **Rear Rotor Temperature** without slowing down your browser.

### 3. Integrated Thermal Lab Sheet
Right below the simulation track, a built-in lab workspace calculates your exact spreadsheet formulas instantly:
* **Total Energy ($J$):** Total kinetic energy converted during braking.
* **Power ($W$) & Heat Flux ($\text{W/m}^2$):** Shows exactly how hard the brakes are working and how concentrated the heat is on the rotor surface.
* **Temperature Spikes ($\Delta T$):** Predicts the exact degree jump your rotors will experience from a full stop.

### 4. Simulation
* Circuit simulation of a hydraulic brakes system
---

