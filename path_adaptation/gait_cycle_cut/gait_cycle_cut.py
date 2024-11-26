

class GaitCycleDetector:
    def __init__(self, foot_force_threshold=50, max_gait_cycles=15, moving_avg_window=100):
        self.foot_force_threshold = foot_force_threshold
        self.max_gait_cycles = max_gait_cycles  # Maximum number of gait cycles to store
        self.moving_avg_window = moving_avg_window  # Window size for moving average

        self.gait_cycle = []
        self.gait_cycles = []
        self.gait_cycle_phase = ''
        self.gait_cycle_phase_prev = ''
        self.record = False
        self.foot_force_prev = 0
        self.foot_force_window = []  # List to store the last `moving_avg_window` foot forces

    def moving_average(self, new_value):
        """
        Update the moving average with a new foot force value.
        """
        self.foot_force_window.append(new_value)
        # If the window exceeds the maximum size, remove the oldest value
        if len(self.foot_force_window) > self.moving_avg_window:
            self.foot_force_window.pop(0)

        # Return the average of the values in the window
        return sum(self.foot_force_window) / len(self.foot_force_window)
    
    def process_step(self, joint_angle, foot_force_heel, foot_force_toe):
        """
        Process a single step in the gait cycle using foot force and joint angle.
        """
        foot_force_raw = foot_force_heel + foot_force_toe
        foot_force_smoothed = self.moving_average(foot_force_raw)  # Apply moving average smoothing

        # Gait phase detection
        if foot_force_smoothed >= self.foot_force_threshold and self.foot_force_prev < self.foot_force_threshold:
            self.gait_cycle_phase = 'HS'  # Heel strike
        elif foot_force_smoothed < self.foot_force_threshold and self.foot_force_prev >= self.foot_force_threshold:
            self.gait_cycle_phase = 'TO'  # Toe off
        else:
            if self.gait_cycle_phase_prev == 'HS':
                self.gait_cycle_phase = 'ST'  # Stance
            elif self.gait_cycle_phase_prev == 'TO':
                self.gait_cycle_phase = 'SW'  # Swing

        # Record gait cycle
        if self.gait_cycle_phase == 'ST' and self.gait_cycle_phase_prev == 'HS':
            self.record = True
        elif self.gait_cycle_phase == 'HS' and self.gait_cycle_phase_prev == 'SW':
            self.record = False

        if self.record:
            self.gait_cycle.append(joint_angle)  # Record joint angle
        elif not self.record:
            if len(self.gait_cycle) > 0:
                self.gait_cycles.append(self.gait_cycle)  # Save completed gait cycle
                self.gait_cycle = []  # Clear the temporary gait cycle

                # Ensure gait cycles do not exceed the maximum limit
                if len(self.gait_cycles) > self.max_gait_cycles:
                    self.gait_cycles.pop(0)  # Remove the oldest gait cycle

        # Update previous states
        self.gait_cycle_phase_prev = self.gait_cycle_phase
        self.foot_force_prev = foot_force_smoothed

    def get_latest_gait_cycle(self):
        """
        Retrieve the latest completed gait cycle, if available.
        """
        if self.gait_cycles:
            return self.gait_cycles[-1]  # Return the last detected gait cycle
        return None  # Return None if no gait cycle is available

    def get_gait_cycles(self):
        """
        Retrieve the detected gait cycles.
        """
        gait_cycles = self.gait_cycles
        return gait_cycles
    
    def get_number_gait_cycles(self):
        """
        Retrieve number of the detected gait cycles in buffer.
        """
        num_gait_cycles = len(self.gait_cycles)
        return num_gait_cycles


