"""Provided plumbing for the palletizer tutorial.

These wrap the incidental parts (connecting and retrying, the pick-station and
pallet commands, and the box visuals) so the methods you write in palletizer.py
stay about motion and the frame system, not boilerplate. You write the motion
call and the obstacle set yourself in palletizer.py. You do not need to edit this
file.
"""

import asyncio
import json
import os

from viam.robot.client import RobotClient
from viam.services.worldstatestore import WorldStateStore
from viam.components.generic import Generic
from viam.proto.common import Pose

# Ships with placeholders; the scaffold exercise has you paste your machine's
# key id and key into it: {"key_id": "...", "key": "..."}.
KEY_FILE = os.path.expanduser("~/exercises/api-key.json")
# Your machine's address, from the app's CONNECT tab.
MACHINE_ADDRESS = "palletizer-101-main.hisrc4fnui.viam.cloud"

# Resource names, as configured in Parts 1-4.
GRIPPER = "gripper-1"
PICK_STATION = "pick-station"
PALLET = "pallet"
PACK_SEQUENCER = "pack-sequencer"
MOTION = "builtin"


async def connect() -> RobotClient:
    """Connect to your machine with your API key and return a client."""
    creds = json.load(open(KEY_FILE))
    opts = RobotClient.Options.with_api_key(
        api_key=creds["key"], api_key_id=creds["key_id"]
    )
    return await RobotClient.at_address(MACHINE_ADDRESS, opts)


async def retry(what, factory, attempts=6, delay=2.0):
    """Run an RPC, riding out the occasional cloud-connection blip."""
    last = None
    for k in range(attempts):
        try:
            return await factory()
        except Exception as e:  # noqa: BLE001
            last = e
            print(f"  [retry] {what}: attempt {k + 1}/{attempts} ({type(e).__name__})")
            await asyncio.sleep(delay)
    raise last


def _pose_from_map(m) -> Pose:
    return Pose(
        x=m["x"], y=m["y"], z=m["z"],
        o_x=m["o_x"], o_y=m["o_y"], o_z=m["o_z"], theta=m["theta"],
    )


async def grasp_pose(robot, box_h) -> Pose:
    """Ask the pick-station where to grasp a box of this height."""
    station = Generic.from_robot(robot, PICK_STATION)
    return _pose_from_map(
        await retry(
            "grasp_pose",
            lambda: station.do_command({"get_vacuum_pose": {"box_height_mm": box_h}}),
        )
    )


async def pick_home_pose(robot, box_h) -> Pose:
    """Ask the pick-station for the safe approach pose above the box."""
    station = Generic.from_robot(robot, PICK_STATION)
    return _pose_from_map(
        await retry(
            "pick_home_pose",
            lambda: station.do_command({"get_pick_home_pose": {"box_height_mm": box_h}}),
        )
    )


async def pallet_top(robot):
    """Return (cx, cy, z) of the center of the pallet's top face."""
    pallet = Generic.from_robot(robot, PALLET)
    pose = await retry("get_visual_pose", lambda: pallet.do_command({"get_visual_pose": True}))
    attrs = await retry("get_attributes", lambda: pallet.do_command({"get_attributes": True}))
    return (pose["x"], pose["y"], pose["z"] + attrs["thickness_mm"] / 2.0)


async def _set_box(robot, seq, parent, x, y, z):
    store = WorldStateStore.from_robot(robot, PACK_SEQUENCER)
    try:
        await retry(
            "set_box_transform",
            lambda: store.do_command(
                {"set_box_transform": {
                    "seq": seq, "parent": parent, "x": x, "y": y, "z": z,
                    "o_x": 0, "o_y": 0, "o_z": 1, "theta": 0}}
            ),
            attempts=2,
        )
    except Exception as e:  # viz is cosmetic; never let it block motion
        print(f"  [viz] set_box_transform failed (non-fatal): {type(e).__name__}")


async def show_box(robot, seq, x, y, z):
    """Render box `seq` resting at the world position (x, y, z) (its center)."""
    await _set_box(robot, seq, "world", x, y, z)


async def attach_box(robot, seq, box_h):
    """Render box `seq` riding the gripper."""
    await _set_box(robot, seq, GRIPPER, 0, 0, box_h / 2)


async def clear_boxes(robot, count=8):
    """Remove the rendered boxes (empty the pallet)."""
    store = WorldStateStore.from_robot(robot, PACK_SEQUENCER)
    for i in range(count):
        try:
            await store.do_command({"clear_box_transform": {"seq": i}})
        except Exception:  # noqa: BLE001
            pass
