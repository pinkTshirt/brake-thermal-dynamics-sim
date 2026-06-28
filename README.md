# 🏎️ Race Car Brake Dynamics & Thermal Lab

Welcome to the **Race Car Brake Dynamics & Thermal Lab**! This is an interactive web app designed to simulate the physics and temperatures of a race car's braking system (built with Formula Student / FSAE metrics in mind).

It helps you visualize how kinetic energy turns into heat across both front and rear brakes in real time.

---

## ✨ Core Features

### 1. Real Race Car Settings
Preloaded with real engineering data from high-performance vehicle setups:
* **Vehicle Weight:** Set to a lightweight $245\text{ kg}$ race car.
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

---

## 🛠️ Technology Stack

* **HTML5 & CSS3:** Clean layout built with a responsive grid and flexbox.
* **JavaScript (ES6+):** Pure, native code running a fast physics loop without lag or rounding bugs.
* **Chart.js:** Included via a lightweight CDN to handle smooth graphing.

---

## 📦 How to Deploy on GitHub Pages

1. Copy the code from `index.html` and paste it into your GitHub repository file.
2. Go to your repository **Settings ➔ Pages**.
3. Choose your branch (`main` or `master`), select the root folder (`/`), and click **Save**.
4. Open your live link, hit **Start Engine**, and hold down the pedal!

