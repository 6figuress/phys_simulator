from math import radians
import os
import time
import pybullet as py
from enum import Enum

from .tools import getPath


class RobotVersion(Enum):
    GRIPPER = 0
    PEN = 1


class URDFAddon:
    def __init__(self, filePath: str, position, quaternion):
        self.filePath = filePath
        self.pos = position
        self.quat = quaternion
        pass


class Simulator:
    gui: bool
    joint_mapping = {0: 2, 1: 3, 2: 4, 3: 5, 4: 6, 5: 7}

    def __init__(
        self,
        robot_version: RobotVersion = RobotVersion.PEN,
        environment: list[URDFAddon] = [
            URDFAddon(
                getPath("urdfs/pen_holder/pen_holder.urdf"),
                position=[1.0, 0, 0.0],
                quaternion=[0.0, 0.0, 0.0, 0.1],
            )
        ],
        gui: bool = False,
        deltaT: float = 1 / 1000,
        log=False,
    ):
        self.env_ids = []

        self.gui = gui

        self.deltaT = deltaT
        self.log = log

        if self.gui:
            py.connect(py.GUI)
        else:
            py.connect(py.DIRECT)
            py.setTimeStep(deltaT)

        if robot_version == RobotVersion.PEN:
            self.robot_id = py.loadURDF(
                getPath("urdfs/robot_arm/iscoin_pen.urdf"),
                useFixedBase=True,
                flags=py.URDF_USE_SELF_COLLISION,
            )
        elif robot_version == RobotVersion.GRIPPER:
            self.robot_id = py.loadURDF(
                getPath("urdfs/robot_arm/iscoin_gripper.urdf"),
                useFixedBase=True,
                flags=py.URDF_USE_SELF_COLLISION,
            )
        else:
            raise Exception("Robot version not supported")

        for e in environment:
            self.env_ids.append(
                (
                    py.loadURDF(e.filePath, basePosition=e.pos, baseOrientation=e.quat),
                    e.filePath,
                )
            )

        self.env_ids.append(
            (
                py.loadURDF(
                    getPath("urdfs/plane/plane.urdf"), basePosition=[0, 0, -0.05]
                ),
                "ground",
            )
        )
        self.pen_joint = 11

        self._log("Simulator started")

    def stepSimu(self):
        py.stepSimulation()
        if self.gui:
            time.sleep(self.deltaT * 10)

    def _log(self, *args):
        if self.log:
            print("[SIMULATOR] ", *args)

    def _isAtPosition(self, angles, tolerance=1e-2) -> bool:
        joint_positions = [
            py.getJointState(self.robot_id, self.joint_mapping[i])[0]
            for i in self.joint_mapping
        ]
        for i, a in enumerate(joint_positions):
            if abs(angles[i] - a) > tolerance:
                return False
        return True

    def _getLinkName(self, body_id, link_index) -> str:
        if link_index == -1:  # Base link case
            return py.getBodyInfo(body_id)[1].decode("utf-8")
        else:
            return py.getJointInfo(body_id, link_index)[12].decode("utf-8")


    def resetAtPosition(self, angles):
        """
        Reset the robot to the specified angles.
        :param angles: list of angles in radians
        """
        for i, joint_id in enumerate(self.joint_mapping.values()):
            py.resetJointState(self.robot_id, joint_id, angles[i])

    def isPositionValid(self, angles) -> bool:
        """
        Check if a position is valid by setting the robot position and checking for collisions
        """
        self.resetAtPosition(angles)
        self.stepSimu()
        return not self.check_collision()

    def validateTrajectory(self, trajectory) -> bool:
        """
        Check if a trajectory is valid by checking if the trajectory is valid.

        :param trajectory: list of angles in radians
        :return: True if the trajectory is valid, False if the trajectory is invalid
        """
        if len(trajectory) == 0:
            return True
        isValid = self.isPositionValid(trajectory[0])
        if not isValid or len(trajectory) == 1:
            return isValid
        for i, p in enumerate(trajectory[1:]):
            if self.moveToAngles(p, checkCollision=True):
                return False

        return True

    def isMoveValid(self, startAngles, destAngles) -> bool:
        """
        Check if a move is valid by checking if the trajectory is valid.
        :param startAngles: list of angles in radians
        :param destAngles: list of angles in radians
        :return: True if the move is valid, False if the move is invalid
        """
        self.resetAtPosition(startAngles)
        return not self.moveToAngles(destAngles, checkCollision=True)

    def motorMove(self, angles, force=1e5) -> None:
        """
        Move the robot to the specified angles. Using the joints as motors (full movemement, no tp)

        :param angles: list of angles in radians
        :param force: force to apply to the motors
        """
        for i, a in enumerate(angles):
            py.setJointMotorControl2(
                bodyUniqueId=self.robot_id,
                jointIndex=self.joint_mapping[i],
                controlMode=py.POSITION_CONTROL,
                targetPosition=a,
                force=force,
                maxVelocity=radians(360),
            )

    def moveToAngles(self, angles, checkCollision=False) -> bool:
        """
        Move the robot to the specified angles.
        :param angles: list of angles in radians
        :param checkCollision: if True, check for collision during the movement

        :return: False if no collision happened (or if the collision check is disabled), True if a collision happened
        """
        self.motorMove(angles)
        collided = False
        while not self._isAtPosition(angles):
            self.stepSimu()
            if checkCollision and self.check_collision():
                collided = True
        return collided

    def isPenInArea(
        self, x_bounds=(-0.0, 0.5), y_bounds=(-0.5, 0.5), z_bounds=(0, 0.5)
    ) -> bool:
        """
        Check if the pen is within the working area.
        :param x_bounds: tuple of (min, max) for x axis
        :param y_bounds: tuple of (min, max) for y axis
        :param z_bounds: tuple of (min, max) for z axis
        :return: True if the pen is within the working area, False otherwise
        """
        pen_pos = py.getLinkState(self.robot_id, self.pen_joint)[0]
        isInArea = True
        if pen_pos[0] < x_bounds[0] or pen_pos[0] > x_bounds[1]:
            self._log(f"The pen is currently out of the working area in X. (current X : {pen_pos[0]}, min - max : {x_bounds[0]} - {x_bounds[1]})")
            isInArea = False
        if pen_pos[1] < y_bounds[0] or pen_pos[1] > y_bounds[1]:
            self._log(f"The pen is currently out of the working area in Y. (current Y : {pen_pos[0]}, min - max : {y_bounds[0]} - {y_bounds[1]})")
            isInArea = False
        if pen_pos[2] < z_bounds[0] or pen_pos[2] > z_bounds[1]:
            self._log(f"The pen is currently out of the working area in Z. (current Z : {pen_pos[0]}, min - max : {z_bounds[0]} - {z_bounds[1]})")
            isInArea = False
        # If the pen is within the working area, return True
        return isInArea

    def check_collision(self) -> bool:
        """
        Check for collision between the robot and the environment.

        :return: True if a collision is detected, False otherwise
        """
        contact_points_robot = py.getContactPoints(
            self.robot_id, self.robot_id
        )  # Self-collision check

        hasCollision = False

        def getLinkName(id):
            return (py.getJointInfo(self.robot_id, id)[12]).decode()

        # Check for self-collisions (ignore wrist-pen collision)
        if contact_points_robot:
            for contact in contact_points_robot:
                if (
                    self._getLinkName(self.robot_id, contact[3]) == "wrist_3_link"
                    and self._getLinkName(self.robot_id, contact[4]) == "pen_link"
                ):
                    continue
                self._log(
                    f"⚠️ Self-Collision: {self._getLinkName(self.robot_id, contact[3])} and {self._getLinkName(self.robot_id, contact[4])} are colliding."
                )
                hasCollision = True

        for env_id, name in self.env_ids:
            contact_points = py.getContactPoints(self.robot_id, env_id)

            # Check for ground collision
            if contact_points:
                for contact in contact_points:
                    if (
                        self._getLinkName(self.robot_id, contact[3])
                        == "base_link_inertia"
                        and self._getLinkName(env_id, contact[4]) == "ground"
                    ):
                        continue
                    self._log(
                        f"⚠️ Collision with environment: {self._getLinkName(self.robot_id, contact[3])} and {self._getLinkName(env_id, contact[4])} are colliding."
                    )
                    hasCollision = True  # Collision detected

        return hasCollision
        




if __name__ == "__main__":
    simu = Simulator(gui = True, log=True)
    print(simu._getLinkName(11))

    # Check for potential self-collisions within 5mm
    # close_contacts_robot = py.getClosestPoints(self.robot_id, self.robot_id, distance=threshold_distance)
    # close_contacts_ground = py.getClosestPoints(self.robot_id, self.ground_id, distance=threshold_distance)

    # Check if a collision is **about to happen** within 5mm
    # for contact in close_contacts_robot:
    #     if (contact[3] == contact[4] or
    #         abs(contact[3] - contact[4]) == 1 or
    #         getLinkName(contact[3]) == "pen_link" or
    #         getLinkName(contact[4]) == "pen_link"):  # Exclude collisions between the same link, adjacent links, or involving the pen link
    #         continue
    #     if self.logs:
    #         print(f"WARNING: {getLinkName(contact[3])} is too close to {getLinkName(contact[4])} (within {threshold_distance})!")
    #     isInCollision = False  # Prevent movement if too close

    # for contact in close_contacts_ground:
    #     if getLinkName(contact[3]) == "base_link_inertia":
    #         continue
    #     if self.logs:
    #         print(f"WARNING: {getLinkName(contact[3])} is too close to the ground (within 5mm)!")
    #     isInCollision = False  # Prevent movement if too close