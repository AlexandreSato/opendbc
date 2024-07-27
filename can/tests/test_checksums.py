#!/usr/bin/env python3
import unittest

from opendbc.can.parser import CANParser
from opendbc.can.packer import CANPacker
from opendbc.can.tests.test_packer_parser import can_list_to_can_capnp


class TestCanChecksums(unittest.TestCase):
  print('baba')

  def test_chrysler_checksum(self):
    dbc_file = "chrysler_jeep_commander_generated"
    msg = [("WHEEL_SPEEDS", 0)]
    parser = CANParser(dbc_file, msg, 0)
    packer = CANPacker(dbc_file)

    values = {
      'WHEEL_SPEED_FL':  989,
      'WHEEL_SPEED_FR': 1002,
      'WHEEL_SPEED_RL':  987,
      'WHEEL_SPEED_RR':  994,
    }

    # observed checksums of above values in cabana
    counter =          [  0,   1,   2,   3,   4]
    correct_checksum = [213, 200, 239, 242, 161]

    for id, counter_checksum in zip(counter, correct_checksum):
      # values.append('COUNTER': id,)
      msgs = [packer.make_can_msg('WHEEL_SPEEDS', 0, values),]
      can_strings = [can_list_to_can_capnp(msgs), ]
      parser.update_strings(can_strings)
      print(f'oh babi: {id:02d} | checksum from parser is: {int(parser.vl['WHEEL_SPEEDS']['CHECKSUM']):03d}')
      self.assertEqual(parser.vl['WHEEL_SPEEDS']['CHECKSUM'], counter_checksum)

if __name__ == "__main__":
  unittest.main()