"""
brake_sim.py — Tandem Hydraulic Brake Circuit Simulator (BAJA reference model)

This mirrors the physics implemented in brake-circuit.html:

    pedal --(pedal ratio)--> pushrod --(Pascal's Law)--> master cylinder
        --> FRONT circuit (secondary piston, direct)   --> front calipers
        --> REAR circuit  (primary piston, through a proportioning valve) --> rear calipers

Caliper clamp force, pad friction, and rotor radius produce a brake torque per
wheel. That torque is converted to a longitudinal force at the tire contact
patch and capped at the tire's grip limit (the same thing ABS prevents from
being exceeded on a real car). A simple per-axle thermal model layers brake
fade on top: rotor temperature rises with dissipated power, cools toward
ambient over time, and pad friction drops off above a fade threshold.

Run directly for an interactive CLI simulation with live matplotlib charts.
"""

import math
import matplotlib.pyplot as plt

GRAVITY = 9.80665  # m/s^2


class HydraulicBrakeSystem:
    """
    A tandem (dual-circuit) hydraulic brake system model.

    The front circuit (secondary master cylinder piston) feeds the front
    calipers directly. The rear circuit (primary piston) feeds the rear
    calipers through a proportioning valve, which limits how far rear line
    pressure can rise once master cylinder pressure passes a knee point —
    this is what keeps the rear wheels from locking up first as weight
    transfers onto the front tires under hard braking.
    """

    def __init__(
        self,
        vehicle_mass=245.0,          # kg — BAJA car, matches index.html default
        wheel_radius_m=0.28,         # m — rolling radius at the tire contact patch
        tire_mu=0.8987,              # tire-to-road grip limit, matches index.html
        pedal_ratio=4.5,             # mechanical advantage of the pedal lever
        mc_bore_mm=22.2,             # master cylinder bore diameter
        front_piston_dia_mm=42.0,    # front caliper piston diameter
        front_piston_count=2,        # pistons per front caliper
        front_rotor_radius_m=0.110,  # effective friction radius, front rotor
        rear_piston_dia_mm=30.0,     # rear caliper piston diameter
        rear_piston_count=1,         # pistons per rear caliper
        rear_rotor_radius_m=0.090,   # effective friction radius, rear rotor
        knee_bar=25.0,                # proportioning valve knee pressure
        valve_slope=0.35,             # rear pressure gain past the knee (0-1)
        base_pad_mu=0.45,             # pad-to-rotor friction coefficient, fresh
        front_thermal_capacity_j_per_c=121.175,  # action-surface mass * specific heat
        rear_thermal_capacity_j_per_c=121.175,
        ambient_temp_c=20.0,
    ):
        self.vehicle_mass = vehicle_mass
        self.wheel_radius_m = wheel_radius_m
        self.tire_mu = tire_mu

        self.pedal_ratio = pedal_ratio
        self.mc_bore_mm = mc_bore_mm

        self.front_piston_dia_mm = front_piston_dia_mm
        self.front_piston_count = front_piston_count
        self.front_rotor_radius_m = front_rotor_radius_m

        self.rear_piston_dia_mm = rear_piston_dia_mm
        self.rear_piston_count = rear_piston_count
        self.rear_rotor_radius_m = rear_rotor_radius_m

        self.knee_bar = knee_bar
        self.valve_slope = valve_slope

        self.base_pad_mu = base_pad_mu
        self.current_front_mu = base_pad_mu
        self.current_rear_mu = base_pad_mu

        self.front_thermal_capacity = front_thermal_capacity_j_per_c
        self.rear_thermal_capacity = rear_thermal_capacity_j_per_c
        self.ambient_temp_c = ambient_temp_c

        self.front_temp = ambient_temp_c
        self.rear_temp = ambient_temp_c
        self.vehicle_speed = 0.0

    # ---- geometry helpers ---------------------------------------------
    @staticmethod
    def _bore_area_m2(dia_mm):
        radius_m = dia_mm / 2000.0  # mm diameter -> m radius
        return math.pi * radius_m ** 2

    def mc_area_m2(self):
        return self._bore_area_m2(self.mc_bore_mm)

    def front_piston_area_m2(self):
        return self._bore_area_m2(self.front_piston_dia_mm) * self.front_piston_count

    def rear_piston_area_m2(self):
        return self._bore_area_m2(self.rear_piston_dia_mm) * self.rear_piston_count

    @staticmethod
    def _fade_factor(temp_c, fade_start_c=250.0, fade_rate=0.005, floor=0.15):
        if temp_c <= fade_start_c:
            return 1.0
        return max(floor, 1.0 - (temp_c - fade_start_c) * fade_rate)

    # ---- core hydraulic + thermal chain --------------------------------
    def apply_brakes(self, pedal_force_n, dt):
        """
        Run one timestep of the full hydraulic + thermal model and return a
        dict of every intermediate quantity, mirroring computeAt() in
        brake-circuit.html.
        """
        # 1. Pedal leverage -> pushrod force
        pushrod_force_n = pedal_force_n * self.pedal_ratio

        # 2. Pascal's Law -> master cylinder pressure
        mc_pressure_pa = pushrod_force_n / self.mc_area_m2()
        mc_pressure_bar = mc_pressure_pa / 1e5

        # 3. Front circuit is direct; rear circuit is valve-limited
        front_bar = mc_pressure_bar
        if mc_pressure_bar <= self.knee_bar:
            rear_bar = mc_pressure_bar
        else:
            rear_bar = self.knee_bar + (mc_pressure_bar - self.knee_bar) * self.valve_slope
        valve_limiting = mc_pressure_bar > self.knee_bar

        # 4. Caliper clamp force (per caliper)
        front_clamp_n = front_bar * 1e5 * self.front_piston_area_m2()
        rear_clamp_n = rear_bar * 1e5 * self.rear_piston_area_m2()

        # 5. Brake torque per wheel, using the *current* (possibly
        #    fade-degraded) pad friction coefficient from the previous step
        front_torque_per_wheel_nm = 2 * self.current_front_mu * front_clamp_n * self.front_rotor_radius_m
        rear_torque_per_wheel_nm = 2 * self.current_rear_mu * rear_clamp_n * self.rear_rotor_radius_m
        total_torque_nm = 2 * front_torque_per_wheel_nm + 2 * rear_torque_per_wheel_nm

        # 6. Convert torque to a longitudinal force at the contact patch,
        #    then cap it at the tire grip limit — beyond this point a real
        #    wheel locks up regardless of how much more line pressure exists.
        raw_friction_force_n = total_torque_nm / self.wheel_radius_m
        max_tire_force_n = self.tire_mu * self.vehicle_mass * GRAVITY
        wheel_locking = raw_friction_force_n > max_tire_force_n
        friction_force_n = min(raw_friction_force_n, max_tire_force_n)

        deceleration = friction_force_n / self.vehicle_mass

        # 7. Thermal model: dissipate (torque * angular velocity) at each
        #    axle, update rotor temps, then re-derive next step's pad mu.
        omega = (self.vehicle_speed / self.wheel_radius_m) if self.wheel_radius_m else 0.0
        front_power_w = front_torque_per_wheel_nm * omega * 2  # both front wheels
        rear_power_w = rear_torque_per_wheel_nm * omega * 2    # both rear wheels

        self.front_temp += (front_power_w * dt) / self.front_thermal_capacity
        self.rear_temp += (rear_power_w * dt) / self.rear_thermal_capacity
        self.front_temp -= (self.front_temp - self.ambient_temp_c) * 0.05 * dt
        self.rear_temp -= (self.rear_temp - self.ambient_temp_c) * 0.05 * dt

        self.current_front_mu = self.base_pad_mu * self._fade_factor(self.front_temp)
        self.current_rear_mu = self.base_pad_mu * self._fade_factor(self.rear_temp)

        return {
            "pushrod_force_n": pushrod_force_n,
            "mc_pressure_bar": mc_pressure_bar,
            "front_line_bar": front_bar,
            "rear_line_bar": rear_bar,
            "valve_limiting": valve_limiting,
            "front_clamp_n": front_clamp_n,
            "rear_clamp_n": rear_clamp_n,
            "front_torque_nm": front_torque_per_wheel_nm,
            "rear_torque_nm": rear_torque_per_wheel_nm,
            "total_torque_nm": total_torque_nm,
            "wheel_locking": wheel_locking,
            "deceleration": deceleration,
            "front_temp_c": self.front_temp,
            "rear_temp_c": self.rear_temp,
        }

    # ---- interactive CLI loop -------------------------------------------
    def simulate_interactive(self, initial_speed, dt=0.2):
        self.vehicle_speed = initial_speed
        print("\n--- Interactive Tandem Hydraulic Brake Circuit Simulation ---")
        print("Type 'q' at any time to quit.\n")

        history = {
            "time": [0.0], "speed": [self.vehicle_speed], "pedal_force": [0.0],
            "front_bar": [0.0], "rear_bar": [0.0],
            "front_temp": [self.front_temp], "rear_temp": [self.rear_temp],
        }
        current_time = 0.0

        while self.vehicle_speed > 0:
            print(f"Speed: {self.vehicle_speed:.1f} m/s | "
                  f"Front Temp: {self.front_temp:.0f}\u00b0C | Rear Temp: {self.rear_temp:.0f}\u00b0C | "
                  f"Front \u03bc: {self.current_front_mu:.2f} | Rear \u03bc: {self.current_rear_mu:.2f}")
            user_input = input("Enter brake pedal force (N) [e.g., 300]: ")

            if user_input.lower() == "q":
                print("Simulation aborted.")
                break
            try:
                pedal_force = float(user_input)
            except ValueError:
                print("Invalid input! Please type a number.")
                continue

            result = self.apply_brakes(pedal_force, dt)
            self.vehicle_speed -= result["deceleration"] * dt
            self.vehicle_speed = max(0.0, self.vehicle_speed)
            current_time += dt

            history["time"].append(current_time)
            history["speed"].append(self.vehicle_speed)
            history["pedal_force"].append(pedal_force)
            history["front_bar"].append(result["front_line_bar"])
            history["rear_bar"].append(result["rear_line_bar"])
            history["front_temp"].append(result["front_temp_c"])
            history["rear_temp"].append(result["rear_temp_c"])

            status = f"--> Decelerating at {result['deceleration']:.2f} m/s^2"
            if result["valve_limiting"]:
                status += "  [proportioning valve LIMITING rear pressure]"
            if result["wheel_locking"]:
                status += "  \u26a0\ufe0f  WHEEL LOCKUP RISK \u2014 exceeds tire grip limit"
            print(status)

            if self.vehicle_speed == 0:
                print("\nThe vehicle has safely come to a complete stop!")

        self.plot_results(history)

    def plot_results(self, h):
        print("\nGenerating charts... (Close the chart window to end the script)")

        fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, 1, figsize=(8, 12), sharex=True)

        ax1.plot(h["time"], h["speed"], "b-o", label="Speed (m/s)")
        ax1.set_ylabel("Speed (m/s)")
        ax1.grid(True)
        ax1.legend()
        ax1.set_title("Tandem Hydraulic Brake Circuit Telemetry")

        ax2.plot(h["time"], h["pedal_force"], "g-o", label="Pedal Force (N)")
        ax2.set_ylabel("Force (N)")
        ax2.grid(True)
        ax2.legend()

        ax3.plot(h["time"], h["front_bar"], "r-o", label="Front Line (bar)")
        ax3.plot(h["time"], h["rear_bar"], color="darkorange", marker="o", label="Rear Line (bar)")
        ax3.axhline(self.knee_bar, color="gray", linestyle="--", label="Valve Knee")
        ax3.set_ylabel("Line Pressure (bar)")
        ax3.grid(True)
        ax3.legend()

        ax4.plot(h["time"], h["front_temp"], "r-o", label="Front Rotor Temp (\u00b0C)")
        ax4.plot(h["time"], h["rear_temp"], color="darkorange", marker="o", label="Rear Rotor Temp (\u00b0C)")
        ax4.axhline(250, color="black", linestyle="--", label="Fade Threshold (250\u00b0C)")
        ax4.set_xlabel("Time (seconds)")
        ax4.set_ylabel("Temperature (\u00b0C)")
        ax4.grid(True)
        ax4.legend()

        plt.tight_layout()
        plt.show()  # renders the graphs on screen


# --- Run the Script ---
if __name__ == "__main__":
    brakes = HydraulicBrakeSystem()  # BAJA defaults, matching brake-circuit.html

    # Start at ~40 km/h, matching the initial speed default on the web sim
    brakes.simulate_interactive(initial_speed=11.11)
