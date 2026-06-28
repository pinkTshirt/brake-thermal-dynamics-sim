import time
import matplotlib.pyplot as plt

class HydraulicBrakeSystem:
    def __init__(self, master_cyl_area, slave_cyl_area, vehicle_mass, base_friction):
        self.master_cyl_area = master_cyl_area 
        self.slave_cyl_area = slave_cyl_area   
        self.vehicle_mass = vehicle_mass
        
        # Variables for Brake Fade simulation
        self.base_friction = base_friction
        self.current_friction = base_friction
        self.brake_temp = 20.0  # Starting ambient temperature in Celsius
        self.vehicle_speed = 0

    def apply_brakes(self, pedal_force, dt):
        # 1. Calculate pressures and clamping force
        pressure = pedal_force / self.master_cyl_area
        slave_force = pressure * self.slave_cyl_area

        # 2. Calculate friction force using the *current* friction coefficient
        friction_force = slave_force * self.current_friction

        # 3. SIMULATE BRAKE FADE
        # Heat generated is roughly proportional to friction force * speed * time
        heat_generated = friction_force * self.vehicle_speed * 0.001 * dt
        self.brake_temp += heat_generated
        
        # Brakes also cool down over time (ambient cooling)
        self.brake_temp -= (self.brake_temp - 20.0) * 0.05 * dt 

        # If brakes get hotter than 250°C, the friction coefficient starts to drop
        if self.brake_temp > 250:
            # The hotter it gets, the more friction we lose (bottoms out at 0.15)
            fade_factor = max(0.15, 1.0 - (self.brake_temp - 250) * 0.005)
            self.current_friction = self.base_friction * fade_factor
        else:
            self.current_friction = self.base_friction

        # 4. Calculate deceleration
        deceleration = friction_force / self.vehicle_mass
        return pressure, slave_force, deceleration

    def simulate_interactive(self, initial_speed, dt=1.0):
        self.vehicle_speed = initial_speed
        print("\n--- Interactive Braking Simulation ---")
        print("Type 'q' at any time to quit.")
        
        # Lists to store data for our graphs
        history_time = [0]
        history_speed = [self.vehicle_speed]
        history_force = [0]
        history_temp = [self.brake_temp]

        current_time = 0.0

        # Interactive loop
        while self.vehicle_speed > 0:
            print(f"\nSpeed: {self.vehicle_speed:.1f} m/s | Temp: {self.brake_temp:.0f} °C | Friction: {self.current_friction:.2f}")
            user_input = input("Enter brake pedal force (N) [e.g., 200]: ")
            
            if user_input.lower() == 'q':
                print("Simulation aborted.")
                break
            
            try:
                force = float(user_input)
            except ValueError:
                print("Invalid input! Please type a number.")
                continue

            # Apply the physics
            pressure, slave_force, deceleration = self.apply_brakes(force, dt)
            
            # Update the car's speed
            self.vehicle_speed -= deceleration * dt
            self.vehicle_speed = max(0, self.vehicle_speed) # Don't go backwards
            current_time += dt

            # Save the data for this time step
            history_time.append(current_time)
            history_speed.append(self.vehicle_speed)
            history_force.append(force)
            history_temp.append(self.brake_temp)

            print(f"--> Decelerating at {deceleration:.2f} m/s^2")
            
            if self.vehicle_speed == 0:
                print("\nThe vehicle has safely come to a complete stop!")

        # Step 4: Show the visualizations once the simulation is over
        self.plot_results(history_time, history_speed, history_force, history_temp)

    def plot_results(self, t, speed, force, temp):
        print("\nGenerating charts... (Close the chart window to end the script)")
        
        # Create 3 stacked charts
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(8, 10), sharex=True)
        
        # Chart 1: Speed over time
        ax1.plot(t, speed, 'b-o', label='Speed (m/s)')
        ax1.set_ylabel('Speed (m/s)')
        ax1.grid(True)
        ax1.legend()
        ax1.set_title('Brake Simulation Telemetry')

        # Chart 2: Driver Input Force
        ax2.plot(t, force, 'g-o', label='Pedal Force (N)')
        ax2.set_ylabel('Force (N)')
        ax2.grid(True)
        ax2.legend()

        # Chart 3: Brake Temperature (Brake Fade)
        ax3.plot(t, temp, 'r-o', label='Rotor Temp (°C)')
        ax3.axhline(250, color='orange', linestyle='--', label='Fade Threshold (250°C)')
        ax3.set_xlabel('Time (seconds)')
        ax3.set_ylabel('Temperature (°C)')
        ax3.grid(True)
        ax3.legend()

        plt.tight_layout()
        plt.show() # This renders the graph on your screen

# --- Run the Script ---
if __name__ == "__main__":
    brakes = HydraulicBrakeSystem(
        master_cyl_area=5.0,  
        slave_cyl_area=20.0,  
        vehicle_mass=1500,    
        base_friction=0.8
    )

    # Start at a high speed (45 m/s is roughly 100 mph) so we have enough energy to generate heat
    brakes.simulate_interactive(initial_speed=45.0)
