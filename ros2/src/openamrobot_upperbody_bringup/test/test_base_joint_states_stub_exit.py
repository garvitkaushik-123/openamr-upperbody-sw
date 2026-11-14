# SPDX-License-Identifier: MIT
"""How base_joint_states_stub.py ends (issue #63): exit 0 on SIGINT and on SIGTERM, as every process of a launch must at teardown (P24).

Before #63 it exited 1 on every SIGTERM (ExternalShutdownException out of rclpy.spin was not caught: 3 of 3), and it could hang in Fast DDS
while destroying its node until the launch SIGKILLed it (M01, exit -9). The hang is rare and load-dependent, so it is not reproduced here;
the stress reproduction is on #63. This pins the exit codes of a real process given a real signal."""
import os
import random
import signal
import subprocess
import sys
import time

import pytest
from ament_index_python.packages import get_package_prefix

SCRIPT = os.path.join(get_package_prefix("openamrobot_upperbody_bringup"), "lib", "openamrobot_upperbody_bringup",
                      "base_joint_states_stub.py")


@pytest.fixture(scope="module", autouse=True)
def private_domain():
    saved = {k: os.environ.get(k) for k in ("ROS_DOMAIN_ID", "ROS_AUTOMATIC_DISCOVERY_RANGE")}
    os.environ["ROS_DOMAIN_ID"] = str(random.randint(101, 200))
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    yield
    for key, value in saved.items():
        if value is None:
            os.environ.pop(key, None)
        else:
            os.environ[key] = value


@pytest.mark.parametrize("sig", [signal.SIGINT, signal.SIGTERM], ids=["SIGINT", "SIGTERM"])
def test_a_signal_to_the_running_stub_is_exit_0(sig):
    for attempt in range(3):
        # launch gives its processes the default SIGINT disposition; a test runner may not
        process = subprocess.Popen([sys.executable, SCRIPT], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                                   preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_DFL))
        time.sleep(2.0 + random.random() * 0.5)
        process.send_signal(sig)
        output, _ = process.communicate(timeout=20)
        assert process.returncode == 0, f"attempt {attempt}: exit {process.returncode} on {sig.name}:\n{output[-2000:]}"
        assert "Traceback" not in output, output[-2000:]
