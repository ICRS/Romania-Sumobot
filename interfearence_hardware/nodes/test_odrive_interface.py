#!/usr/bin/env python3

from interfearence_hardware.odrive_interface import OdriveInterface
import time

def delay(seconds):
    for i in range(seconds):
        print(".")
        time.sleep(1)

if __name__ == '__main__':
    interface = OdriveInterface()
    interface.reset()
    print("Odrive should not be moving...")

    delay(5)

    interface.release_reset()
    interface.set_wheel_vel(0, 4.0)
    interface.set_wheel_vel(1, 4.0)
    print("Motors should be moving at ~ 4.0 radians per second")
    for i in range(10):
        time.sleep(0.5)
        print("0: {}, 1: {} rad/s".format(
            interface.get_wheel_vel(0, 0.5), 
            interface.get_wheel_vel(1, 0.5)))

    interface.reset()
    print("Motors should now stop...")

    delay(5)

    interface.release_reset()
    interface.set_wheel_vel(0, -4.0)
    interface.set_wheel_vel(1, -4.0)
    print("Motors should be moving at ~ -4.0 radians per second")
    for i in range(10):
        time.sleep(0.5)
        print("0: {}, 1: {} rad/s".format(
            interface.get_wheel_vel(0, 0.5), 
            interface.get_wheel_vel(1, 0.5)))

    interface.reset()
    print("Motors should now stop...")


class HardwareInterface:
    def __init__(self):
        # Constants
        self.MIN_MOTOR_RADS = 104 # So min wheel speed is ~50 RPM
        self.MAX_MOTOR_RADS = 838 # So max wheel speed is ~400 RPM

        # For conversion between rad/s and rad/s electrical
        self.VEL_MULT = 0.73303827333

        self.ENCODER_COUNTS_PER_RAD = 4000 / (2 * 3.1415926)

        # Initialise rospy
        rospy.init_node("hardware_interface")
        
        # 100 Hz update rate
        self._rate = rospy.Rate(100)

        # In radians per second
        self._target_motor_vels = [0, 0]
        self._wheel_vels = [0, 0]

        # Encoder counts from the wheels
        self._encoder_counts = [0, 0]

        self.last_update_time = rospy.Time.now()

        # Create the reset publisher and publish that we're in the reset
        # state
        self._reset_pub = rospy.Publisher("/reset", Bool, queue_size=2)
        msg = Bool()
        msg.data = True
        self._reset_pub.publish(msg)

        # Find the Odrive - block until received
        self._odrv = odrive.find_any()

        self._prev_reset = False
        # Reset ourselves
        self.toggle_reset(True)

        # Ensure that we're in velocity control
        self._odrv.axis0.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
        self._odrv.axis1.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL

        # TODO: Don't publish this once we have the GPIO pins working
        self.toggle_reset(False)

    def toggle_reset(self, reset_status):
        if type(reset_status) != bool:
            raise TypeError("reset_status must be boolean!")
            return
        msg = Bool()
        msg.data = reset_status
        self._reset_pub.publish(msg)

        if reset_status and (reset_status != self._prev_reset):
            # Reset ourselves
            # Cut motor power
            self._odrv.axis0.requested_state = AXIS_STATE_IDLE
            self._odrv.axis1.requested_state = AXIS_STATE_IDLE
        
        elif reset_status != self._prev_reset:
            # Prepare for new match
            self._odrv.axis0.encoder.shadow_count = 0
            self._odrv.axis1.encoder.shadow_count = 0

            self._odrv.axis0.requested_state = AXIS_STATE_SENSORLESS_CONTROL
            self._odrv.axis1.requested_state = AXIS_STATE_SENSORLESS_CONTROL
        
        self._prev_reset = reset_status

    def check_reset(self):
        # TODO: Read the pins
        start_pin = True
        kill_pin = True
        # It's not in the reset state only if both pins are high
        return not (start_pin and kill_pin)

    def calculate_wheel_velocities(self, old_encoder_counts, dt):
            d0 = self._encoder_counts[0] - old_encoder_counts[0]
            d1 = self._encoder_counts[1] - old_encoder_counts[1]
            
            self._wheel_vels[0] = d0 / (dt * self.ENCODER_COUNTS_PER_RAD)
            self._wheel_vels[1] = d1 / (dt * self.ENCODER_COUNTS_PER_RAD)

            rospy.loginfo("Wheel vels: 1: {}\t 2: {}".format(
               self._wheel_vels[0], self._wheel_vels[1]));
            
    def main(self):
        while True:
            dt = rospy.Time.now() - self.last_update_time
            self.last_update_time += dt

            dt = dt.secs + dt.nsecs / 1e9
            # Prevent excessive time leaps from screwing things over
            if dt > 0.1:
                dt = 0.1

            self.toggle_reset(self.check_reset())

            # If we're in the reset state then don't control the odrive
            if self._prev_reset:
                return

            # Set the target wheel speeds
            self._odrv.axis0.controller.vel_setpoint = self._target_motor_vels[0] * self.VEL_MULT
            self._odrv.axis1.controller.vel_setpoint = self._target_motor_vels[1] * self.VEL_MULT

            # Measure the current wheel speeds
            old_encoder_counts = self._encoder_counts
            self._encoder_counts[0] = self._odrv.axis0.encoder.shadow_count
            self._encoder_counts[1] = self._odrv.axis1.encoder.shadow_count

            self.calculate_wheel_velocities(old_encoder_counts, dt)

            # Maintain frequency
            self._rate.sleep()

