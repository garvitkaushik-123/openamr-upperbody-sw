#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Fault injection for the OpenArm motor emulators. Simulation only.

    ros2 run openamrobot_openarm_emulator inject_fault.py ARM FAULT [VALUE ...]

    ARM     left | right | both
    FAULT   can_loss on|off              the wire between adapter and motors is open: nothing reaches the motors, nothing
                                         comes back. The interface stays up and the motors keep their last setpoint.
            estop on|off                 E-stop power cut: every motor is silent and applies no torque. off is the reset:
                                         supply back, motors in the power-up state (disabled), positions kept.
            silent MOTOR on|off          single-motor timeout: the motor runs but does not answer. MOTOR is 1..8 (8: gripper).
            code MOTOR CODE              the motor reports fault CODE (8..14, or 0x8..0xE) and stops. CODE 0 removes the
                                         injection; the motor then needs a clear-error frame or a power cycle.
            over_voltage on|off          supply to 33 V (above the 32 V threshold: code 0x8 on every motor), or back to 25.6 V.
            over_temperature MOTOR on|off   the motor's MOSFET temperature to 130 C (code 0xB), or back to 25 C.
            supply VOLTS                 supply voltage. The robot's range is 25.6 to 29.2 V.
            clear                        remove every injected fault and set the supply to 25.6 V.

Each fault is a parameter of the emulator controllers /sim/left_arm_emulator and /sim/right_arm_emulator; this tool only
sets them (`ros2 param set` does the same). To take the interface itself down: sudo ip link set vcan_arm_l down.
"""
import sys

import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node

MOTORS = 8


def on(text):
    if text not in ("on", "off"):
        raise SystemExit(f"expected on or off, got {text!r}")
    return text == "on"


def motor_index(text):
    index = int(text)
    if not 1 <= index <= MOTORS:
        raise SystemExit(f"MOTOR is 1..{MOTORS}, got {text}")
    return index - 1


def value_of(python_value):
    if isinstance(python_value, bool):
        return ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=python_value)
    if isinstance(python_value, float):
        return ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=python_value)
    if all(isinstance(v, int) for v in python_value):
        return ParameterValue(type=ParameterType.PARAMETER_INTEGER_ARRAY, integer_array_value=python_value)
    return ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY, double_array_value=python_value)


class Injector(Node):
    def __init__(self, namespace):
        super().__init__("openarm_fault_injector")
        self.namespace = namespace.rstrip("/")

    def _call(self, service_type, name, request):
        client = self.create_client(service_type, name)
        if not client.wait_for_service(timeout_sec=10.0):
            raise SystemExit(f"{name} is not there: is the emulator running?")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if future.result() is None:
            raise SystemExit(f"{name} did not answer")
        return future.result()

    def get(self, arm, name):
        result = self._call(GetParameters, f"{self.namespace}/{arm}_arm_emulator/get_parameters", GetParameters.Request(names=[name]))
        value = result.values[0]
        return list(value.integer_array_value) if value.type == ParameterType.PARAMETER_INTEGER_ARRAY else list(value.double_array_value)

    def set(self, arm, values):
        request = SetParameters.Request(parameters=[Parameter(name=k, value=value_of(v)) for k, v in values.items()])
        result = self._call(SetParameters, f"{self.namespace}/{arm}_arm_emulator/set_parameters", request)
        for name, outcome in zip(values, result.results):
            if not outcome.successful:
                raise SystemExit(f"{arm} {name}: {outcome.reason}")
            self.get_logger().info(f"{arm}_arm_emulator {name} = {values[name]}")


def main():
    args = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    namespace = "/sim"
    if args and args[0].startswith("--namespace="):
        namespace = args.pop(0).split("=", 1)[1]
    if len(args) < 2 or args[0] not in ("left", "right", "both"):
        print(__doc__)
        return 2
    arms = ("left", "right") if args[0] == "both" else (args[0],)
    fault, rest = args[1], args[2:]
    rclpy.init()
    node = Injector(namespace)
    for arm in arms:
        if fault == "can_loss":
            node.set(arm, {"fault.can_loss": on(rest[0])})
        elif fault == "estop":
            node.set(arm, {"fault.estop_power_cut": on(rest[0])})
        elif fault == "silent":
            flags = node.get(arm, "fault.silent_motors")
            flags[motor_index(rest[0])] = int(on(rest[1]))
            node.set(arm, {"fault.silent_motors": flags})
        elif fault == "code":
            codes = node.get(arm, "fault.error_codes")
            codes[motor_index(rest[0])] = int(rest[1], 0)
            node.set(arm, {"fault.error_codes": codes})
        elif fault == "over_voltage":
            node.set(arm, {"supply_voltage": 33.0 if on(rest[0]) else 25.6})
        elif fault == "over_temperature":
            temperatures = node.get(arm, "fault.temperature_mos_c")
            temperatures[motor_index(rest[0])] = 130.0 if on(rest[1]) else 25.0
            node.set(arm, {"fault.temperature_mos_c": temperatures})
        elif fault == "supply":
            node.set(arm, {"supply_voltage": float(rest[0])})
        elif fault == "clear":
            node.set(arm, {"fault.can_loss": False, "fault.estop_power_cut": False, "fault.silent_motors": [0] * MOTORS,
                           "fault.error_codes": [0] * MOTORS, "fault.temperature_mos_c": [25.0] * MOTORS,
                           "fault.temperature_rotor_c": [25.0] * MOTORS, "supply_voltage": 25.6})
        else:
            print(__doc__)
            return 2
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
