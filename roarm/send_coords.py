"""Send a single x,y,z (and optional gripper angle) move command to the arm on the Pi,
through the SSH bridge (see bridge.py) — no need to have roarm_server.py already running
or an SSH tunnel already open, this sets both up and tears the tunnel down when done.

Usage:
    python send_coords.py 0.3 0.1 0.3
    python send_coords.py 0.3 0.1 0.3 --grip 1.7
    python send_coords.py 0.3 0.1 0.3 --speed 0.1        # smoothed/interpolated move
    python send_coords.py 0.3 0.1 0.3 --host pi@sdna-hdb  # override SSH target
    python send_coords.py 0.3 0.1 0.3 --no-auto-start     # assume roarm_server.py already running
"""

import argparse
import asyncio

from bridge import RoArmBridge


async def send_coords(x, y, z, grip_angle, speed, ssh_host, auto_start_server):
    with RoArmBridge(ssh_host=ssh_host, auto_start_server=auto_start_server) as bridge:
        arm = await bridge.connected_arm()
        try:
            if speed is None:
                result = await arm.move(x, y, z, grip_angle)
            else:
                result = await arm.move_interp(x, y, z, grip_angle, speed)
            print(result)
            return result
        finally:
            await arm.close()


def main():
    parser = argparse.ArgumentParser(description="Send a 3D coordinate move command to the RoArm over the Pi bridge.")
    parser.add_argument("x", type=float, help="x coordinate in meters")
    parser.add_argument("y", type=float, help="y coordinate in meters")
    parser.add_argument("z", type=float, help="z coordinate in meters")
    parser.add_argument("--grip", type=float, default=3.14, dest="grip_angle",
                         help="gripper angle in radians (default: 3.14)")
    parser.add_argument("--speed", type=float, default=None,
                         help="if given, use a smoothed/interpolated move at this speed instead of a direct move")
    parser.add_argument("--host", type=str, default="pi@sdna-hdb", dest="ssh_host",
                         help="SSH target for the Pi (default: pi@sdna-hdb)")
    parser.add_argument("--no-auto-start", action="store_false", dest="auto_start_server",
                         help="don't try to start roarm_server.py on the Pi; assume it's already running")
    args = parser.parse_args()

    asyncio.run(send_coords(
        args.x, args.y, args.z, args.grip_angle, args.speed, args.ssh_host, args.auto_start_server
    ))


if __name__ == "__main__":
    main()
