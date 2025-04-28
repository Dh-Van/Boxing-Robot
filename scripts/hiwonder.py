# hiwonder.py
"""
Hiwonder Robot Controller
-------------------------
Handles the control of the mobile base and 5-DOF robotic arm using commands received from the gamepad.
"""

import time
import numpy as np
from board_controller import BoardController
from servo_bus_controller import ServoBusController
import utils as ut
import simutils as s_ut
from five_dof_arm import FiveDOFRobot

# Robot base constants
WHEEL_RADIUS = 0.047  # meters
BASE_LENGTH_X = 0.096  # meters
BASE_LENGTH_Y = 0.105  # meters

class HiwonderRobot:
    def __init__(self):
        """Initialize motor controllers, servo bus, and default robot states."""
        self.board = BoardController()
        self.servo_bus = ServoBusController()

        self.joint_values = [0, 0, 90, -30, 0, 0]  # degrees
        self.home_position = [0, 0, 90, -30, 0, 0]  # degrees
        self.joint_limits = [
            [-120, 120], [-90, 90], [-120, 120],
            [-100, 100], [-90, 90], [-120, 30]
        ]
        self.joint_control_delay = 0.2 # secs
        self.speed_control_delay = 0.2

        self.sim = FiveDOFRobot()

        self.move_to_home_position()

    # -------------------------------------------------------------
    # Methods for interfacing with the mobile base
    # -------------------------------------------------------------

    def set_robot_commands(self, cmd: ut.GamepadCmds):
        """Updates robot base and arm based on gamepad commands.

        Args:
            cmd (GamepadCmds): Command data class with velocities and joint commands.
        """

        if cmd.arm_home:
            self.move_to_home_position()

        print(f'---------------------------------------------------------------------')
        
        # self.set_base_velocity(cmd)

        home_pos = [0.234, 0, 0.174, 0, 0, 0]

        if cmd.arm_rb:
            self.set_arm_velocity(cmd)

        if cmd.base_lb:
            self.set_base_velocity(cmd)

        if cmd.btn_x:
            self.go_to_position(home_pos)

        if cmd.btn_y:
            pos = home_pos.copy()
            pos[0] -= 0.2
            pos[1] += 0.25
            pos[2] -= 0.1
            self.go_to_position(pos)

        if cmd.btn_a:
            pos = home_pos.copy()
            pos[0] -= 0.2
            pos[1] += 0.25
            pos[2] -= 0.1
            self.move_to_position(home_pos[0:3], pos[0:3])

        ######################################################################

        position = [0]*3
        
        ######################################################################

        # print(f'[DEBUG] XYZ position: X: {round(position[0], 3)}, Y: {round(position[1], 3)}, Z: {round(position[2], 3)} \n')


    def set_base_velocity(self, cmd: ut.GamepadCmds):
        """ Computes wheel speeds based on joystick input and sends them to the board """
        """
        motor3 w0|  ↑  |w1 motor1
                 |     |
        motor4 w2|     |w3 motor2
        
        """
        x = -1 * cmd.base_vy
        y = -1 * cmd.base_vx
        spin = cmd.base_w

        norm_factor = max(abs(x) + abs(y) + abs(spin), 1)
        gain = 100

        fl = ((y + x + spin) / norm_factor) * gain
        bl = ((y - x - spin) / norm_factor) * gain
        fr = ((y - x + spin) / norm_factor) * gain
        br = ((y + x - spin) / norm_factor) * gain

        speed = [fr, br, fl, bl]

        # Send speeds to motors
        self.board.set_motor_speed(speed)
        time.sleep(self.speed_control_delay)

    # -------------------------------------------------------------
    # Methods for interfacing with the 5-DOF robotic arm
    # -------------------------------------------------------------

    def set_arm_velocity(self, cmd: ut.GamepadCmds):
        """Calculates and sets new joint angles from linear velocities.

        Args:
            cmd (GamepadCmds): Contains linear velocities for the arm.
        """
        vel = [cmd.arm_vx, cmd.arm_vy, cmd.arm_vz]

        thetalist_dot = self.sim.calc_velocity_kinematics(vel)

        # print(f'[DEBUG] Current thetalist (deg) = {self.joint_values}') 
        # print(f'[DEBUG] linear vel: {[round(vel[0], 3), round(vel[1], 3), round(vel[2], 3)]}')
        # print(f'[DEBUG] thetadot (deg/s) = {[round(td,2) for td in thetalist_dot]}')

        # Update joint angles
        dt = 1 # Fixed time step
        new_thetalist = [0.0]*6

        # linear velocity control
        for i in range(5):
            new_thetalist[i] = self.joint_values[i] + dt * thetalist_dot[i]

        print(f'[DEBUG] Commanded thetalist (deg) = {new_thetalist}')       
        
        # set new joint angles
        self.set_joint_values(new_thetalist, radians=False)

    def go_to_position(self, pos):
        EE = s_ut.EndEffector(pos[0], pos[1], pos[2], 0, 0, 0)
        theta = np.degrees(self.sim.solve_inverse_kinematics(EE, tol=0.01))
        theta = np.append(theta, 0.0)
        # print(theta)
        self.set_joint_values(theta)

    def move_to_position(self, start_pos, end_pos):
        # start_pos = np.array(start_pos))
        # end_pos = np.append(np.array(end_pos))
        total_time, time_increment, all_positions = self.sim.generateTrajectory(start_pos, end_pos)
        print("time", total_time)
        np_all_pos = np.array(all_positions)

        start_time = time.time()
        last_time = start_time
        i = 0
        while time.time() - start_time < total_time:
            current_time = time.time()
            if(current_time - last_time) >= time_increment:
                curr_pos = np_all_pos[:, 0, i]
                ik_pos = np.append(curr_pos, [0, 0, 0])
                curr_joint_angles = np.append(self.sim.solve_inverse_kinematics(ut.list_to_EE(ik_pos)), 0)
                self.set_joint_values(curr_joint_angles, duration=0, radians=True)
                last_time = current_time
                i += 1

        


    def set_joint_value(self, joint_id: int, theta: float, duration=250, radians=False):
        """ Moves a single joint to a specified angle """
        if not (1 <= joint_id <= 6):
            raise ValueError("Joint ID must be between 1 and 6.")

        if radians:
            theta = np.rad2deg(theta)

        theta = self.enforce_joint_limits(theta, joint_id=joint_id)
        self.joint_values[joint_id] = theta

        pulse = self.angle_to_pulse(theta)
        self.servo_bus.move_servo(joint_id, pulse, duration)
        
        print(f"[DEBUG] Moving joint {joint_id} to {theta}° ({pulse} pulse)")
        time.sleep(self.joint_control_delay)


    def set_joint_values(self, thetalist: list, duration=250, radians=False):
        """Moves all arm joints to the given angles.

        Args:
            thetalist (list): Target joint angles in degrees.
            duration (int): Movement duration in milliseconds.
        """
        if len(thetalist) != 6:
            raise ValueError("Provide 6 joint angles.")

        if radians:
            thetalist = [np.rad2deg(theta) for theta in thetalist]


        thetalist = self.enforce_joint_limits(thetalist)
        self.joint_values = thetalist # updates joint_values with commanded thetalist
        # print(f"thetalist={thetalist}")
        thetalist = self.remap_joints(thetalist) # remap the joint values from software to hardware


        for joint_id, theta in enumerate(thetalist, start=1):
            pulse = self.angle_to_pulse(theta)
            self.servo_bus.move_servo(joint_id, pulse, duration)


    def enforce_joint_limits(self, thetalist: list) -> list:
        """Clamps joint angles within their hardware limits.

        Args:
            thetalist (list): List of target angles.

        Returns:
            list: Joint angles within allowable ranges.
        """
        return [np.clip(theta, *limit) for theta, limit in zip(thetalist, self.joint_limits)]


    def move_to_home_position(self):
        print(f'Moving to home position...')
        self.set_joint_values(self.home_position, duration=800)
        time.sleep(2.0)
        print(f'Arrived at home position: {self.joint_values} \n')
        time.sleep(1.0)
        print(f'------------------- System is now ready!------------------- \n')


    # -------------------------------------------------------------
    # Utility Functions
    # -------------------------------------------------------------

    def angle_to_pulse(self, x: float):
        """ Converts degrees to servo pulse value """
        hw_min, hw_max = 0, 1000  # Hardware-defined range
        joint_min, joint_max = -150, 150
        return int((x - joint_min) * (hw_max - hw_min) / (joint_max - joint_min) + hw_min)


    def pulse_to_angle(self, x: float):
        """ Converts servo pulse value to degrees """
        hw_min, hw_max = 0, 1000  # Hardware-defined range
        joint_min, joint_max = -150, 150
        return round((x - hw_min) * (joint_max - joint_min) / (hw_max - hw_min) + joint_min, 2)


    def stop_motors(self):
        """ Stops all motors safely """
        self.board.set_motor_speed([0]*4)
        print("[INFO] Motors stopped.")


    def remap_joints(self, thetalist: list):
        """Reorders angles to match hardware configuration.

        Args:
            thetalist (list): Software joint order.

        Returns:
            list: Hardware-mapped joint angles.

        Note: Joint mapping for hardware
            HARDWARE - SOFTWARE
            joint[0] = gripper/EE
            joint[1] = joint[5] 
            joint[2] = joint[4] 
            joint[3] = joint[3] 
            joint[4] = joint[2] 
            joint[5] = joint[1] 
        """
        return thetalist[::-1]