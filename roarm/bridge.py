"""Network bridge from this machine to the RoArm server running on the Raspberry Pi.

The arm's TCP API (RoArmAPI / RoArmServer, see roarm.py and roarm_server.py) listens
on port 8659 *on the Pi*. This module gets a local port 8659 tunneled to it over SSH
(so it works over Tailscale/LAN/wherever `ssh <host>` already works, with no need to
open port 8659 to the network), and can make sure roarm_server.py is actually running
on the Pi before you try to talk to it.

Usage:
    from bridge import RoArmBridge

    with RoArmBridge(ssh_host="pi@sdna-hdb") as bridge:
        arm = bridge.connect_arm()  # returns a connected RoArmAPI, using loop.run_until_complete
        ...

or from async code:
    async with RoArmBridge(ssh_host="pi@sdna-hdb") as bridge:
        arm = await bridge.connect_arm_async()
        result = await arm.move(0.3, 0.1, 0.3)
        await arm.close()
"""

import asyncio
import shlex
import socket
import subprocess
import sys
import time

from roarm import RoArmAPI

REMOTE_ROARM_DIR = "/home/pi/Documents/labspace/roarm"
REMOTE_SERVER_SCRIPT = "roarm_server.py"
REMOTE_LOG_FILE = "/tmp/roarm_server.log"
ROARM_PORT = 8659


class RoArmBridge:
    """Opens an SSH tunnel to the Pi and (optionally) ensures roarm_server.py is running there.

    Local code then talks to `localhost:<local_port>` exactly as if the arm server were
    running locally; the SSH tunnel forwards that to port 8659 on the Pi.
    """

    def __init__(self, ssh_host="pi@sdna-hdb", local_port=ROARM_PORT, remote_port=ROARM_PORT,
                 auto_start_server=True, connect_timeout=10):
        self.ssh_host = ssh_host
        self.local_port = local_port
        self.remote_port = remote_port
        self.auto_start_server = auto_start_server
        self.connect_timeout = connect_timeout
        self._tunnel_proc = None

    # ---------- context manager (sync) ----------
    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()

    # ---------- context manager (async) ----------
    async def __aenter__(self):
        self.open()
        return self

    async def __aexit__(self, exc_type, exc, tb):
        self.close()

    # ---------- lifecycle ----------
    def open(self):
        """Ensure the remote server is running (if requested), then open the SSH tunnel."""
        if self.auto_start_server:
            self._ensure_remote_server_running()
        self._open_tunnel()
        self._wait_for_local_port()

    def close(self):
        if self._tunnel_proc is not None:
            self._tunnel_proc.terminate()
            try:
                self._tunnel_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self._tunnel_proc.kill()
                self._tunnel_proc.wait(timeout=5)
            self._tunnel_proc = None

    # ---------- remote server management ----------
    def _run_ssh(self, remote_command, timeout=15):
        """Run a single command on the Pi over SSH and return CompletedProcess."""
        return subprocess.run(
            ["ssh", "-o", "BatchMode=yes", "-o", f"ConnectTimeout={self.connect_timeout}",
             self.ssh_host, remote_command],
            capture_output=True, text=True, timeout=timeout,
        )

    def _is_remote_server_running(self):
        # Check that something is actually listening on the arm's port, rather than
        # pgrep-matching the process list: `pgrep -f roarm_server.py` can false-positive
        # by matching its own SSH invocation's command line.
        result = self._run_ssh(f"ss -tln 2>/dev/null | grep -q ':{self.remote_port} '")
        return result.returncode == 0

    def _ensure_remote_server_running(self):
        if self._is_remote_server_running():
            print(f"roarm_server.py already running on {self.ssh_host}.")
            return

        print(f"roarm_server.py not running on {self.ssh_host}; starting it...")
        # Use setsid (not `& disown`) to fully detach the process from the SSH session:
        # `&` + `disown` depends on job control, which a non-interactive `bash -c` may not
        # reliably have, and SSH can then hang waiting on the backgrounded job anyway.
        # The `cd && setsid ...` must be grouped in `( )` with the redirects applied to the
        # whole group: redirecting only the final command in the list leaves the backgrounded
        # subshell holding the original (still-open) stdout/stderr until it execs into setsid,
        # and that gap is enough for the ssh client to hang waiting for the channel to close.
        remote_command = (
            f"(cd {shlex.quote(REMOTE_ROARM_DIR)} && "
            f"setsid python3 {shlex.quote(REMOTE_SERVER_SCRIPT)}) "
            f"> {shlex.quote(REMOTE_LOG_FILE)} 2>&1 < /dev/null &"
        )
        result = self._run_ssh(remote_command)
        if result.returncode != 0:
            raise RuntimeError(
                f"Failed to start roarm_server.py on {self.ssh_host}: "
                f"{result.stderr.strip() or result.stdout.strip()}"
            )

        # Give it a moment to bind its socket, and confirm it's actually up.
        for _ in range(20):
            if self._is_remote_server_running():
                print(f"roarm_server.py started on {self.ssh_host}.")
                return
            time.sleep(0.5)
        raise RuntimeError(
            f"roarm_server.py did not start on {self.ssh_host}. "
            f"Check {REMOTE_LOG_FILE} on the Pi for details."
        )

    # ---------- SSH tunnel ----------
    def _open_tunnel(self):
        print(f"Opening SSH tunnel: localhost:{self.local_port} -> "
              f"{self.ssh_host}:{self.remote_port}")
        self._tunnel_proc = subprocess.Popen(
            ["ssh", "-o", "BatchMode=yes", "-o", f"ConnectTimeout={self.connect_timeout}",
             "-o", "ExitOnForwardFailure=yes",
             "-N",  # no remote command, just forward
             "-L", f"{self.local_port}:localhost:{self.remote_port}",
             self.ssh_host],
        )
        # If SSH fails fast (e.g. port already in use, auth failure), surface that quickly.
        time.sleep(0.5)
        if self._tunnel_proc.poll() is not None:
            raise RuntimeError(
                f"SSH tunnel to {self.ssh_host} exited immediately "
                f"(code {self._tunnel_proc.returncode}). Is the host reachable?"
            )

    def _wait_for_local_port(self):
        deadline = time.time() + self.connect_timeout
        while time.time() < deadline:
            if self._tunnel_proc.poll() is not None:
                raise RuntimeError(
                    f"SSH tunnel to {self.ssh_host} died while waiting for port "
                    f"{self.local_port} to open (code {self._tunnel_proc.returncode})."
                )
            try:
                with socket.create_connection(("localhost", self.local_port), timeout=1):
                    return
            except OSError:
                time.sleep(0.2)
        raise TimeoutError(
            f"Timed out waiting for tunneled port {self.local_port} to become reachable."
        )

    # ---------- convenience ----------
    def connect_arm_async(self):
        """Return an *unconnected* RoArmAPI pointed at the tunnel; call `await arm.connect()`."""
        arm = RoArmAPI("localhost", self.local_port)
        return arm

    async def connected_arm(self):
        """Convenience: create and connect a RoArmAPI through the tunnel in one call."""
        arm = self.connect_arm_async()
        await arm.connect()
        return arm


def main():
    """Smoke-test the bridge from the command line: open it, report status, tear it down."""
    ssh_host = "pi@sdna-hdb" if "--host" not in sys.argv else sys.argv[sys.argv.index("--host") + 1]
    with RoArmBridge(ssh_host=ssh_host) as bridge:
        print(f"Bridge is up: localhost:{bridge.local_port} -> {ssh_host}:{bridge.remote_port}")
        input("Press Enter to close the tunnel...")


if __name__ == "__main__":
    main()
