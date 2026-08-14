import asyncio
import sys
import helpers
from viam.components.gripper import Gripper
from viam.services.motion import MotionClient
from viam.proto.common import Pose, PoseInFrame
from helpers import connect
from viam.components.arm import Arm
from viam.proto.component.arm import JointPositions
from viam.proto.common import (
    Pose,
    PoseInFrame,
    WorldState,
    GeometriesInFrame,
    Geometry,
    RectangularPrism,
    Vector3,
)


## Constants
BOX_W, BOX_L, BOX_H = 200.0, 150.0, 100.0  # the box the pick-station presents
GRASP_DEPTH = 10.0  # press the cups this far onto the box top so they seat


def down_pose(x, y, z) -> Pose:
    """A target pose at (x, y, z) with the tool pointing straight down."""
    return Pose(x=x, y=y, z=z, o_x=0, o_y=0, o_z=-1, theta=0)


class Palletizer:
    def __init__(self, robot):
        self.robot = robot
        self.gripper = Gripper.from_robot(robot, helpers.GRIPPER)
        self.placed = []
        self._top = None

    async def resources(self):
        """List the resources the machine exposes (a first, safe call)."""
        for name in sorted(rn.name for rn in self.robot.resource_names):
            print(" ", name)

    async def wave(self):
        """Move the arm by commanding its joint angles directly."""
        arm = Arm.from_robot(self.robot, "arm-1")
        await arm.move_to_joint_positions(
            JointPositions(values=[0, -45, -30, 0, 60, 0])
        )

    async def zero(self):
        """Zero out all the joints, returning the arm to a flat, outstretched position."""
        arm = Arm.from_robot(self.robot, "arm-1")
        await arm.move_to_joint_positions(
            JointPositions(values=[0, 0, 0, 0, 0, 0])
        )

    async def move_gripper(self, pose, obstacles=None):
        """Move the gripper frame to `pose`, routing around `obstacles`."""
        motion = MotionClient.from_robot(self.robot, helpers.MOTION)
        destination = PoseInFrame(reference_frame="world", pose=pose)
        return await helpers.retry("move", lambda: motion.move(
            component_name=helpers.GRIPPER,
            destination=destination,
            world_state=obstacles,
            extra={"timeout": 15.0},
            timeout=120,
        ))

    async def move(self):
        """Send the gripper to a reachable pose, pointing down."""
        await self.move_gripper(down_pose(400, -300, 400))

    async def pick(self, seq=0):
        """Pick the box at the pick-station and lift it (box ends up held)."""
        home = await helpers.pick_home_pose(self.robot, BOX_H)
        grasp = await helpers.grasp_pose(self.robot, BOX_H)
        await self.move_gripper(home, self.obstacles(held=True))
        await helpers.show_box(self.robot, seq, grasp.x, grasp.y, grasp.z - BOX_H / 2)
        await self.move_gripper(down_pose(grasp.x, grasp.y, grasp.z - GRASP_DEPTH))
        await self.gripper.grab()
        await helpers.attach_box(self.robot, seq, BOX_H)
        await self.move_gripper(home, self.obstacles(held=True))



    async def _pallet_top(self):
        if self._top is None:
            self._top = await helpers.pallet_top(self.robot)
        return self._top

    def _place_pose(self, i, cx, cy, top):
        """Box i -> (x, y, gripper-tip z): two layers of a 2x2 grid."""
        layer, slot = i // 4, i % 4
        col, row = slot % 2, slot // 2
        x = cx + (col - 0.5) * BOX_W
        y = cy + (row - 0.5) * BOX_L
        z_tip = top + (layer + 1) * BOX_H
        return x, y, z_tip
    
    def _clear_tip(self, z_tip):
        if not self.placed:
            return z_tip
        max_top = max(z + BOX_H / 2 for (_, _, z) in self.placed)
        return max(z_tip, max_top + BOX_H + 10.0)



    async def place(self):
        """One full pick-and-place of the next box onto the pallet."""
        cx, cy, top = await self._pallet_top()
        seq = len(self.placed)
        x, y, z_tip = self._place_pose(seq, cx, cy, top)
        await self.pick(seq)
        # 1) over the slot, held box modeled, so it cannot drag through the stack
        await self.move_gripper(
            down_pose(x, y, self._clear_tip(z_tip)), self.obstacles(held=True)
        )
        # 2) straight down into the slot (a vertical lower is not a drag)
        await self.move_gripper(down_pose(x, y, z_tip), self.obstacles())
        await self.gripper.open()
        await helpers.show_box(self.robot, seq, x, y, z_tip - BOX_H / 2)
        self.placed.append((x, y, z_tip - BOX_H / 2))

        # await self.move_gripper(down_pose(x, y, z_tip), self.obstacles())
        # await self.gripper.open()
        # await helpers.show_box(self.robot, seq, x, y, z_tip - BOX_H / 2)
        # self.placed.append((x, y, z_tip - BOX_H / 2))

    async def run(self):
        """Pack the whole pallet: two layers of four."""
        await helpers.clear_boxes(self.robot)
        self.placed = []
        for _ in range(8):
            await self.place()
        print(f"packed {len(self.placed)} boxes")

        ## Depalletize
        await self.unpack()



    def obstacles(self, held=False, exclude_index=None):
        """Build the planner's obstacle set from the boxes already placed."""
        def cuboid(label, frame, x, y, z):
            return GeometriesInFrame(
                reference_frame=frame,
                geometries=[Geometry(
                    center=Pose(x=x, y=y, z=z, o_x=0, o_y=0, o_z=1, theta=0),
                    box=RectangularPrism(dims_mm=Vector3(x=BOX_W, y=BOX_L, z=BOX_H)),
                    label=label,
                )],
            )

        # obs = [cuboid(f"placed-{i}", "world", x, y, z)
        #     for i, (x, y, z) in enumerate(self.placed)]
        # if held:
        #     obs.append(cuboid("held", helpers.GRIPPER, 0, 0, BOX_H / 2))
        # return WorldState(obstacles=obs) if obs else None

        obs = [cuboid(f"placed-{i}", "world", x, y, z)
        for i, (x, y, z) in enumerate(self.placed)
        if i != exclude_index]

        # this is causing 
        '''
        grpclib.exceptions.GRPCError: (<Status.UNKNOWN: 2>, 'fatal early collision: obstacle constraint:
        violation between gripper-1:epick-bracket and held geometries', None)
        '''
        # if held:
        #     obs.append(cuboid("held", helpers.GRIPPER, 0, 0, BOX_H / 2))
        return WorldState(obstacles=obs) if obs else None

    async def remove(self):
        """Remove boxes backwards"""

        if not self.placed:
            print("No boxes left to remove!")
            return False

        # Always remove the most recently plcaed box
        seq = len(self.placed) - 1

        x,y,z_center = self.placed[seq]

        # top box
        z_top = z_center + BOX_H/2

        # vacuum cups position
        z_grasp = z_top - GRASP_DEPTH

        # good position above stack
        z_clear = self._clear_tip(z_top)

        # gripper is open
        await self.gripper.open()

        # Move above target box and delete target from obstacle
        await self.move_gripper(down_pose(x, y, z_clear), self.obstacles(exclude_index=seq),
        )

        await self.move_gripper(down_pose(x,y,z_grasp), self.obstacles(exclude_index=seq),)

        grabbed = await self.gripper.grab()


        # visually looks like the gripper grabs the box, but it actually fails
        if grabbed is False: 
            print(f"Failed to grab box {seq}")



        # Visualization: not sure if it will work
        await helpers.attach_box(self.robot, seq, BOX_H,)

        # no longer part of the pallet stack
        self.placed.pop()

        #lift straight up
        await self.move_gripper(down_pose(x,y,z_clear), self.obstacles(),)

        # return to pick station
        home = await helpers.pick_home_pose(self.robot, BOX_H,)

        drop = await helpers.grasp_pose(self.robot, BOX_H,)

        # good approach
        await self.move_gripper(home, self.obstacles(),)

        # Lower box back to station
        await self.move_gripper(down_pose(drop.x, drop.y, drop.z), self.obstacles(),)

        # release
        await self.gripper.open()
        await helpers.show_box(self.robot, seq, drop.x, drop.y, drop.z - BOX_H/2)
        # return up
        await self.move_gripper(home, self.obstacles(),)

        return True

    async def unpack(self):
        """Remove box from pallet (top to bottom)"""

        while self.placed:
            await self.remove()

# verb -> method. One entry per capability.
STEPS = {
    "resources": Palletizer.resources,
    "wave": Palletizer.wave,
    "zero": Palletizer.zero,
    "move": Palletizer.move,
    "pick": Palletizer.pick,
    "place": Palletizer.place,
    "remove": Palletizer.remove,
    "unpack": Palletizer.unpack,
    "run": Palletizer.run,
}

async def main(verb):
    robot = await connect()
    palletizer = Palletizer(robot)
    try:
        step = STEPS.get(verb)
        if step is None:
            print(f"unknown step '{verb}'. steps: {', '.join(STEPS)}")
            return
        await step(palletizer)
    finally:
        await robot.close()


# if __name__ == "__main__":
#     if len(sys.argv) < 2:
#         print(f"usage: python palletizer.py <step>   steps: {', '.join(STEPS)}")
#         sys.exit(1)
#     asyncio.run(main(sys.argv[1]))

if __name__ == "__main__":
    verb = sys.argv[1] if len(sys.argv) > 1 else "run"
    asyncio.run(main(verb))
