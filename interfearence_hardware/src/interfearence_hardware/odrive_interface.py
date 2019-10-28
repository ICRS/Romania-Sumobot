import odrive
from odrive.enums import *
from odrive.utils import dump_errors

import time

class OdriveInterface:
    def __init__(self):
        # CONSTANTS
        self.MIN_MOTOR_RADS = 104 # So min wheel speed is ~50 RPM
        self.MAX_MOTOR_RADS = 471 # So max wheel speed is ~200 RPM

        self.MAX_EFFORT = 20

        # Encoder counts per radian the MOTOR has turned
        self.ENCODER_COUNTS_PER_RAD = 4000 / (2 * 3.1415926)

        # Encoder counts from the wheels
        self._encoder_counts = [0, 0]

        # Find the Odrive - block until received
        print("\033[1;31mWaiting for Odrive...\033[0m")
        self._odrv = odrive.find_any()
        print("\033[1;32m...Found Odrive\033[0m")

        # Do initial callibration
        self._odrv.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        self._odrv.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        while (self._odrv.axis0.current_state != AXIS_STATE_IDLE and
               self._odrv.axis1.current_state != AXIS_STATE_IDLE):
            time.sleep(0.1)

        if self._odrv.axis0.error != 0x00 or self._odrv.axis1.error != 0x00:
            dump_errors(self._odrv, True)
            raise RuntimeError("Failed to calibrate axis")

        # Set ourselves to the reset state
        self.reset();

        # Ensure that we're in velocity control
        self._odrv.axis0.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
        self._odrv.axis1.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL

        self._ctrl_modes = [CTRL_MODE_VELOCITY_CONTROL, 
                            CTRL_MODE_VELOCITY_CONTROL]

    # Set reset state, where motors are un-powered
    def reset(self):
        self._odrv.axis0.requested_state = AXIS_STATE_IDLE
        self._odrv.axis1.requested_state = AXIS_STATE_IDLE

        self._odrv.axis0.controller.vel_setpoint = 0
        self._odrv.axis1.controller.vel_setpoint = 0

        self.clear_errors()
    
    # Release the odrive from reset, powering on the motors
    def release_reset(self):
        # Prepare for new match
        self._odrv.axis0.encoder.shadow_count = 0
        self._odrv.axis1.encoder.shadow_count = 0

        self._odrv.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self._odrv.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        
    # Read the wheel velocity in radians per second
    def get_wheel_vel(self, axis, dt):
        old_encoder_counts = self._encoder_counts[axis]
        if axis == 0:
            self._encoder_counts[axis] = self._odrv.axis0.encoder.shadow_count
        else:
            self._encoder_counts[axis] = self._odrv.axis1.encoder.shadow_count

        delta = self._encoder_counts[axis] - old_encoder_counts

        wheel_vel = delta / (dt * self.ENCODER_COUNTS_PER_RAD * 20)

        return wheel_vel
    
    # Set the wheel velocity in radians per second
    def set_wheel_vel(self, axis, vel):
        # Convert wheel velocity into motor velocity
        vel *= 20
        if vel < self.MIN_MOTOR_RADS:
            vel = 0
        elif vel > self.MAX_MOTOR_RADS:
            vel = self.MAX_MOTOR_RADS

        if self._ctrl_modes[axis] != CTRL_MODE_VELOCITY_CONTROL:
            self._ctrl_modes[axis] = CTRL_MODE_VELOCITY_CONTROL
            if axis == 0:
                self._odrv.axis0.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
            else:
                self._odrv.axis1.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL

        if axis == 0:
            self._odrv.axis0.controller.vel_setpoint = vel * self.ENCODER_COUNTS_PER_RAD
        else:
            self._odrv.axis1.controller.vel_setpoint = vel * self.ENCODER_COUNTS_PER_RAD

    # Read the battery voltage
    def get_battery_voltage(self):
        return self._odrv.vbus_voltage

    # Set the effort(current) applied to the wheel
    def set_wheel_eff(self, axis, eff):
        if eff > self.MAX_EFFORT:
            eff = self.MAX_EFFORT
        elif eff < -self.MAX_EFFORT:
            eff = -self.MAX_EFFORT

        if self._ctrl_modes[axis] != CTRL_MODE_CURRENT_CONTROL:
            self._ctrl_modes[axis] = CTRL_MODE_CURRENT_CONTROL
            if axis == 0:
                self._odrv.axis0.controller.config.control_mode = CTRL_MODE_CURRENT_CONTROL
            else:
                self._odrv.axis1.controller.config.control_mode = CTRL_MODE_CURRENT_CONTROL

        if axis == 0:
            self._odrv.axis0.controller.current_setpoint = eff
        else:
            self._odrv.axis1.controller.current_setpoint = eff

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

    def check_errors(self):
        if self._odrv.axis0.error != 0x00:
            return True
        if self._odrv.axis1.error != 0x00:
            return True
        return False

    def clear_errors(self):
        dump_errors(self._odrv, True)
