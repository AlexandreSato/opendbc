from opendbc.car.structs import CarParams
from opendbc.car.gwm.values import CAR

Ecu = CarParams.Ecu

FW_VERSIONS = {
  CAR.GWM_HAVAL_H6_PHEV_3RD_GEN: {
    (Ecu.engine, 0x7e0, None): [
      b'PLACEHOLDER',
    ],
  },
}