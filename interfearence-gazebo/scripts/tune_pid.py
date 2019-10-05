#!/usr/bin/env python

import random
import math

import rospy
from dynamic_reconfigure.msg import Config, DoubleParameter, BoolParameter, GroupState
from control_msgs.msg import JointControllerState
from std_msgs.msg import Float64

# GOAL: Use genetic algorithms to find the best PID parameters

ITERATIONS = 20
SAMPLE_SIZE = 50
TEST_SAMPLE_POINTS = 500 # Number of errors to add when testing the GA
FREQ_A = 50 # Frequency of the square wave of A (number of samples per change)
MAG_A = 20 # Magnitude of the target velocity of controller A
FREQ_B = 250
MAG_B = 60

# We can test two controllers at once as the PID values ought to be the
# same for both
CONTROLLER_TOPIC = "/InterFEARence/interfearence/controller/velocity/"
CONTROLLER_A = "left_velocity_controller/"
CONTROLLER_B = "right_velocity_controller/"

# Publish PID values to controller A
pid_a_pub = None
# Publish wave to controller A
cmd_a_pub = None
# Publish PID values to controller B
pid_b_pub = None
# Publish wave to controller B
cmd_b_pub = None

error_a_values = []
error_b_values = []


def A_cb(msg):
    global MAG_A
    if len(error_a_values) >= TEST_SAMPLE_POINTS:
        return

    if len(error_a_values) % FREQ_A == 0:
        wave_msg = Float64()
        wave_msg.data = MAG_A
        cmd_a_pub.publish(wave_msg)
        MAG_A = -MAG_A

    error_a_values.append(msg.error)


def B_cb(msg):
    global MAG_B
    if len(error_b_values) >= TEST_SAMPLE_POINTS:
        return

    if len(error_b_values) % FREQ_B == 0:
        wave_msg = Float64()
        wave_msg.data = MAG_B
        cmd_b_pub.publish(wave_msg)
        MAG_B = -MAG_B

    error_b_values.append(msg.error)


def generate_random_bit_array(length):
    arr = []
    for i in range(length):
        arr.append(random.randint(0, 1))
    return arr


def calc_bit_array_value(array, starting_power):
    result = 0
    power = starting_power
    for bit in array:
        if bit == 1:
            result += math.pow(2.0, power)
        power += 1
    return result


class PIDSpecimen:
    def __init__(self, from_mate=False):
        self._length = 18
        self._starting_power = -6
        self._mutate_chance = 0.05 # 5%
        self._crossover_chance = 0.8 # 80%

        if not from_mate:
            self._p_bits = generate_random_bit_array(self._length)
            self._i_bits = generate_random_bit_array(self._length)
            self._d_bits = generate_random_bit_array(self._length)
            self.calculate_pid_values()

    def calculate_pid_values(self):
        self.p = calc_bit_array_value(self._p_bits, self._starting_power)
        self.i = calc_bit_array_value(self._i_bits, self._starting_power)
        self.d = calc_bit_array_value(self._d_bits, self._starting_power)

    def mate_bit_array(self, array_a, array_b):
        if random.random() > self._crossover_chance:
            return array_a, array_b
        swap_point = random.randint(0, self._length - 1)
        new_arr_a = array_a[0:swap_point] + array_b[swap_point:self._length]
        new_arr_b = array_b[0:swap_point] + array_a[swap_point:self._length]

        return new_arr_a, new_arr_b

    def mate(self, other_specimen):
        if self._length != other_specimen._length:
            raise RuntimeError("Specimens must have the same length!")

        child_a = PIDSpecimen(from_mate=True)
        child_b = PIDSpecimen(from_mate=True)

        child_a._p_bits, child_b._p_bits = self.mate_bit_array(
            self._p_bits, other_specimen._p_bits)
        child_a._i_bits, child_b._i_bits = self.mate_bit_array(
            self._i_bits, other_specimen._i_bits)
        child_a._d_bits, child_b._d_bits = self.mate_bit_array(
            self._d_bits, other_specimen._d_bits)

        child_a.mutate()
        child_b.mutate()

        child_a.calculate_pid_values()
        child_b.calculate_pid_values()

        return child_a, child_b

    def mutate_arr(self, arr):
        if random.random() < self._mutate_chance:
            bit = random.randint(0, self._length-1)
            if arr[bit] == 1:
                arr[bit] = 0
            else:
                arr[bit] = 1
        return arr

    def mutate(self):
        self._p_bits = self.mutate_arr(self._p_bits)
        self._i_bits = self.mutate_arr(self._i_bits)
        self._d_bits = self.mutate_arr(self._d_bits)

    def generate_ros_msg(self):
        msg = Config()

        b = BoolParameter()
        b.name = "antiwindup"
        b.value = False
        msg.bools = [b]

        p = DoubleParameter()
        p.name = "p"
        p.value = self.p

        i = DoubleParameter()
        i.name = "i"
        i.value = self.i

        d = DoubleParameter()
        d.name = "d"
        d.value = self.d

        msg.doubles = [p, i, d]

        g = GroupState()
        g.name = "Default"
        g.state = True
        g.id = 0
        g.parent = 0
        msg.groups = [g]

        return msg

    def weigh(self):
        global error_a_values
        global error_b_values
        pid_a_pub.publish(self.generate_ros_msg())
        pid_b_pub.publish(self.generate_ros_msg())
        rospy.sleep(0.05)

        # Setup variables to be counted
        error_a_values = []
        error_b_values = []
        while (len(error_a_values) < TEST_SAMPLE_POINTS) and (len(error_b_values) < TEST_SAMPLE_POINTS):
            rospy.sleep(0.02)

        weight = 0
        for err in error_a_values:
            weight += abs(err)
        for err in error_b_values:
            weight += abs(err)

        return weight


    def as_string(self):
        return "p: {0}\t\ti: {1}\t\td: {2}".format(self.p, self.i, self.d)


if __name__ == '__main__':
    rospy.init_node("genetic_algorithm_pid")
    rospy.sleep(0.01)

    pid_a_pub = rospy.Publisher(
        CONTROLLER_TOPIC + CONTROLLER_A + "pid/parameter_updates", Config, 
        queue_size=1)
    
    cmd_a_pub = rospy.Publisher(
        CONTROLLER_TOPIC + CONTROLLER_A + "command", Float64, queue_size=1)
    
    pid_b_pub = rospy.Publisher(
        CONTROLLER_TOPIC + CONTROLLER_B + "pid/parameter_updates", Config, 
        queue_size=1)
    
    cmd_b_pub = rospy.Publisher(
        CONTROLLER_TOPIC + CONTROLLER_B + "command", Float64, queue_size=1)
    
    rospy.Subscriber(CONTROLLER_TOPIC + CONTROLLER_A + "state",
                     JointControllerState, A_cb)
    rospy.Subscriber(CONTROLLER_TOPIC + CONTROLLER_B + "state",
                     JointControllerState, B_cb)
    random.seed(None)
    specimens = []
    for i in range(SAMPLE_SIZE):
        specimens.append(PIDSpecimen())

    try:
        for generation in range(ITERATIONS):
            # First weigh the specimens
            weights = []
            max_weight = 0
            min_weight = 999999999
            best_specimen_id = 0
            print("GENERATION {0}: Weighing specimens...".format(generation))
            for specimen in specimens:
                w = specimen.weigh()
                weights.append(w)
                if w > max_weight:
                    max_weight = w
                if w < min_weight:
                    min_weight = w
                    best_specimen_id = len(weights) - 1
                #print(specimen.as_string())

            total_weight = max_weight*len(weights) - sum(weights)
            # We want to minimise the weights, so set all weights w to MAX - w
            for i in range(len(weights)):
                weights[i] = (max_weight - weights[i]) / total_weight

            print("================================")
            #print("Weights:\n")
            #print(weights)
            #print("\n================================\n")
            print("Best Specimen:\n")
            print(specimens[best_specimen_id].as_string())
            #print("\n")

            new_generation = []
            while len(new_generation) < len(specimens):
                # Pick two specimens randomly by weight
                a = random.random()
                b = random.random()
                current_weight = 0
                parent_a = specimens[0]
                parent_b = specimens[0]
                for i in range(len(weights)):
                    current_weight += weights[i]
                    if a > current_weight:
                        parent_a = specimens[i]
                    if b > current_weight:
                        parent_b = specimens[i]

                child_a, child_b = parent_a.mate(parent_b)
                new_generation.append(child_a)
                if len(new_generation) != len(specimens):
                    new_generation.append(child_b)

            if generation != ITERATIONS - 1:
                specimens = new_generation

    except rospy.exceptions.ROSInterruptException:
        print("\n===================================")
        print("Final Specimen Set:")
        for specimen in specimens:
            print(specimen.as_string() + "\n")
    print("\n\n[DONE]\n")
