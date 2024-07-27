from opendbc.can.parser import CANParser
from opendbc.can.packer import CANPacker
from opendbc.can.tests.test_packer_parser import can_list_to_can_capnp


class TestCanChecksums:
  print('baba')

  def test_chrysler_checksum(speed):
    dbc_file = "chrysler_pacifica_2017_hybrid_generated"
    msg = [("WHEEL_SPEEDS", 0)]
    parser = CANParser(dbc_file, msg, 0)
    packer = CANPacker(dbc_file)

    values = {
      'WHEEL_SPEED_LEFT': speed,
      'WHEEL_SPEED_RIGHT': speed,
    }

    # observed checksums of above values in cabana
    counter =          [ 0,   1,   2,  3,   4,   5,   6,  7,   8,  9, 10, 11, 12, 13,  14,  15]
    correct_checksum = [55, 129, 166, 47, 177, 172, 193, 20, 177, 25, 52, 28, 35, 86, 113, 108]

    for id, counter_checksum in zip(counter, correct_checksum):
      msgs = [packer.make_can_msg('WHEEL_SPEEDS', 0, values),]
      can_strings = [can_list_to_can_capnp(msgs), ]
      parser.update_strings(can_strings)
      # print(f'oh babi: {id:02d}')
      # assert parser.vl['WHEEL_SPEEDS']['CHECKSUM'] == counter_checksum
      if parser.vl['WHEEL_SPEEDS']['CHECKSUM'] == counter_checksum:
        print(f'match in checksum: {counter_checksum:03d} and speed: {speed:06d} and counter: {id:02d}')

  for speed in range(6218501, 6226698):
    test_chrysler_checksum(speed)