""" AUTO-FORMATTED USING opendbc/car/debug/format_fingerprints.py, EDIT STRUCTURE THERE."""
from opendbc.car.structs import CarParams
from opendbc.car.gwm.values import CAR

Ecu = CarParams.Ecu

FW_VERSIONS = {
  CAR.GWM_HAVAL_H6: {
    (Ecu.engine, 0x7a1, None): [
      b'\xf1\x873612100XEC56000\xf1\x89S013A01XKN17002',
    ],
    (Ecu.eps, 0x721, None): [
      b'\x10gs\x16\x01',
    ],
    (Ecu.fwdCamera, 0x733, None): [
      b'\x10y\x00 \x01',
    ],
    (Ecu.fwdRadar, 0x7ba, None): [
      b'\x10y\x000\x01',
    ],
    (Ecu.hvac, 0x7b3, None): [
      b'\xf1\x8b\x00\x00\x00\xff',
    ],
  },
}
