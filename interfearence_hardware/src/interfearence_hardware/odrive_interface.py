import odrive
from odrive.enums import *

class OdriveInterface:
    def __init__(self):
        # CONSTANTS
        self.MIN_MOTOR_RADS = 104 # So min wheel speed is ~50 RPM
        self.MAX_MOTOR_RADS = 838 # So max wheel speed is ~400 RPM

        # For conversion between rad/s and rad/s electrical
        self.VEL_MULT = 0.73303827333

        self.ENCODER_COUNTS_PER_RAD = 4000 / (2 * 3.1415926)

        # Encoder counts from the wheels
        self._encoder_counts = [0, 0]

        # Find the Odrive - block until received
        print("\033[1;31mWaiting for Odrive...\033[0m")
        self._odrv = odrive.find_any()
        print("\033[1;32m...Found Odrive\033[0m")

        # Set ourselves to the reset state
        self.reset();

        # Ensure that we're in velocity control
        self._odrv.axis0.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
        self._odrv.axis1.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL

    # Set reset state, where motors are un-powered
    def reset(self):
        self._odrv.axis0.requested_state = AXIS_STATE_IDLE
        self._odrv.axis1.requested_state = AXIS_STATE_IDLE

        self._odrv.axis0.controller.vel_setpoint = 0
        self._odrv.axis1.controller.vel_setpoint = 0
    
    # Release the odrive from reset, powering on the motors
    def release_reset(self):
        # Prepare for new match
        self._odrv.axis0.encoder.shadow_count = 0
        self._odrv.axis1.encoder.shadow_count = 0

        self._odrv.axis0.requested_state = AXIS_STATE_SENSORLESS_CONTROL
        self._odrv.axis1.requested_state = AXIS_STATE_SENSORLESS_CONTROL
        
    # Read the wheel velocity in radians per second
    def get_wheel_vel(self, axis, dt):
        old_encoder_counts = self._encoder_counts[axis]
        if axis == 0:
            self._encoder_counts[axis] = self._odrv.axis0.encoder.shadow_count
        else:
            self._encoder_counts[axis] = self._odrv.axis1.encoder.shadow_count

        delta = self._encoder_counts[axis] - old_encoder_counts

        wheel_vel = delta / (dt * self.ENCODER_COUNTS_PER_RAD)

        return wheel_vel
    
    # Set the wheel velocity in radians per second
    def set_wheel_vel(self, axis, vel):
        # Convert wheel velocity into motor velocity
        vel *= 20
        if vel < self.MIN_MOTOR_RADS:
            vel = 0
        elif vel > self.MAX_MOTOR_RADS:
            vel = self.MAX_MOTOR_RADS

        if axis == 0:
            self._odrv.axis0.controller.vel_setpoint = vel * self.VEL_MULT
        else:
            self._odrv.axis1.controller.vel_setpoint = vel * self.VEL_MULT

    # Read the battery voltage
    def get_battery_voltage(self):
        return self._odrv.vbus_voltage

    # Read the effort(current) applied to the wheel
    def get_wheel_eff(self, axis):
        B = 0
        C = 0
        if axis == 0:
            B = self._odrv.axis0.motor.current_meas_phB
            C = self._odrv.axis0.motor.current_meas_phC
        else:
            B = self._odrv.axis1.motor.current_meas_phB
            C = self._odrv.axis1.motor.current_meas_phC

        if abs(B) > abs(C):
            return B
        return C
