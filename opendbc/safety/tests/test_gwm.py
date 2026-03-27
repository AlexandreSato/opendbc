#!/usr/bin/env python3
import unittest

from opendbc.car.structs import CarParams
import opendbc.safety.tests.common as common
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests.common import CANPackerSafety


class TestGwm(common.CarSafetyTest):
  TX_MSGS = [[0x12B, 0], [0x147, 2], [0xA1, 2]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x12B,), 2: (0x147,)}
  FWD_BLACKLISTED_ADDRS = {0: [0x147], 2: [0x12B]}

  MAX_RATE_UP = 3
  MAX_RATE_DOWN = 5
  MAX_TORQUE_LOOKUP = [0], [254]
  MAX_RT_DELTA = 100
  MAX_TORQUE_ERROR = 70

  MIN_GAS = -10
  MAX_GAS = 4577
  INACTIVE_GAS = 0
  MAX_BRAKE = 107

  def setUp(self):
    self.packer = CANPackerSafety("gwm_haval_h6_mk3_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.gwm, 0)
    self.safety.init_tests()

  def _user_gas_msg(self, gas):
    values = {"GAS_POSITION": gas}
    return self.packer.make_can_msg_safety("CAR_OVERALL_SIGNALS2", 0, values)

  def _user_brake_msg(self, brake):
    values = {"AP_CANCEL_COMMAND": brake}
    return self.packer.make_can_msg_safety("STEER_AND_AP_STALK", 0, values)

  def _speed_msg(self, speed):
    values = {f"{pos}_WHEEL_SPEED": speed * 1.0 for pos in ["FRONT_LEFT", "FRONT_RIGHT", "REAR_LEFT", "REAR_RIGHT"]}
    return self.packer.make_can_msg_safety("WHEEL_SPEEDS", 0, values)

  def _pcm_status_msg(self, enable):
    values = {"AP_ENABLE_COMMAND": enable, "AP_CANCEL_COMMAND": not enable}
    return self.packer.make_can_msg_safety("STEER_AND_AP_STALK", 0, values)

  def test_main_cancel_button(self):
    self.safety.set_controls_allowed(True)
    self._rx(self.packer.make_can_msg_safety("STEER_AND_AP_STALK", 0, {"AP_CANCEL_COMMAND": 1}))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_rx_hook(self):
    self.assertTrue(self._rx(self._speed_msg(0)))

  def _torque_meas_msg(self, torque):
    values = {"B_RX_EPS_TORQUE": torque}
    return self.packer.make_can_msg_safety("RX_STEER_RELATED", 0, values)

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"STEER_REQUEST": steer_req, "TORQUE_CMD": torque}
    return self.packer.make_can_msg_safety("STEER_CMD", 0, values)

  # MADS overrides: brake press (AP_CANCEL_COMMAND) always cancels cruise,
  # disengaging controls regardless of prior brake state or vehicle speed.
  # Re-engagement only happens via AP_ENABLE_COMMAND rising edge.

  def test_allow_user_brake_at_zero_speed(self):
    self._rx(self._vehicle_moving_msg(0))
    self._rx(self._user_brake_msg(1))
    self.safety.set_controls_allowed(1)
    self._rx(self._user_brake_msg(1))
    self.assertFalse(self.safety.get_controls_allowed())
    self._rx(self._user_brake_msg(0))
    self.assertFalse(self.safety.get_controls_allowed())
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_not_allow_user_brake_when_moving(self):
    self._rx(self._user_brake_msg(1))
    self.safety.set_controls_allowed(1)
    self._rx(self._vehicle_moving_msg(self.STANDSTILL_THRESHOLD))
    self._rx(self._user_brake_msg(1))
    self.assertFalse(self.safety.get_controls_allowed())
    self._rx(self._vehicle_moving_msg(0))

  def _send_brake_msg(self, brake):
    values = {"BRAKE_CMD": brake}
    return self.packer.make_can_msg_safety("ACC_CMD", 0, values)

  def _send_gas_msg(self, gas):
    values = {"GAS_CMD": gas}
    return self.packer.make_can_msg_safety("ACC_CMD", 0, values)


if __name__ == "__main__":
  unittest.main()
