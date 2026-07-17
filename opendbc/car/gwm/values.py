from dataclasses import dataclass, field
from enum import IntFlag

from opendbc.car.structs import CarParams
from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms, uds
from opendbc.car.lateral import AngleSteeringLimits
from opendbc.car.docs_definitions import CarDocs, CarHarness, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, p16

Ecu = CarParams.Ecu


class CarControllerParams:
  STEER_STEP = 2
  STEER_MAX = 253        # MK3 torque limit
  ACCEL_MAX = 2
  ACCEL_MIN = -3.5

  # MK4 angle-based steering via STEER_REQUEST (14-bit) in STEER_CMD (factor 0.1, offset -779.6).
  ANGLE_LIMITS: AngleSteeringLimits = AngleSteeringLimits(
    360.,                  # STEER_ANGLE_MAX (deg) — hard safety cap on commanded wheel angle
    ([], []), ([], []),    # v1 rate-limit tables unused (vehicle-model path)
    MAX_LATERAL_ACCEL=3.0,  # m/s^2 (~ISO 11270 comfort; OEM route_7a peaked 3.57, so we're already conservative)
    MAX_LATERAL_JERK=2.5,   # m/s^3 — conservative/smooth
    MAX_ANGLE_RATE=2.0,     # deg per 20ms frame (= 100 deg/s) — low-speed backstop (jerk limit governs at speed)
  )

  # MK4: clamp the commanded angle to within this many deg of the MEASURED wheel. The EPS under-executes
  # angle offsets (~0.76x), so when the wheel trails, the model winds the command far past it (rails to
  # ±6 deg) during the under-hold — worsening the EPS handoff/"limp". Capping the lead trims that windup
  # without throttling normal steering: p99 of obeying command-vs-wheel error is 3.0 deg, so 4.0 caps
  # <0.2% of normal-driving frames (offline-validated on drives ce-5/6/8/10). NOT an override cure —
  # the limp's trigger isn't the error magnitude — just a windup limiter that complements the A_RX fault.
  MK4_ANGLE_ERROR_MAX = 4.0  # deg

  def __init__(self, CP: CarParams):
    self.STEER_DELTA_UP = 4
    self.STEER_DELTA_DOWN = 6
    self.STEER_ERROR_MAX = 80


class GwmSafetyFlags(IntFlag):
  LONG_CONTROL = 1
  # MK4 owns its own cruise loop (pcmCruise=False). The panda must arm controls on the gentle-DOWN stalk
  # gesture (msg 0xC7 GEAR_STALK), not the FURTHER_DOWN-only msg 161 bit47 the MK3 path uses.
  OP_CRUISE = 2


@dataclass
class GWMCarDocs(CarDocs):
  package: str = "Adaptive Cruise Control (ACC) & Lane Assist"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.gwm]))


@dataclass
class GWMPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {
    Bus.pt: 'gwm_haval_h6_mk3_generated',
  })


class CAR(Platforms):
  GWM_HAVAL_H6 = GWMPlatformConfig(
    [GWMCarDocs("Haval H6 2019-24")],
    CarSpecs(mass=2040, wheelbase=2.738, steerRatio=17.416),
  )
  GWM_HAVAL_H6_MK4 = GWMPlatformConfig(
    [GWMCarDocs("Haval H6 2024-26")],
    CarSpecs(mass=1915, wheelbase=2.738, steerRatio=18.0, centerToFrontRatio=0.420),
    dbc_dict={Bus.pt: 'gwm_haval_h6_mk4_generated'},
  )


GREATWALLMOTORS_VERSION_REQUEST_MULTI = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER]) + \
  p16(uds.DATA_IDENTIFIER_TYPE.VEHICLE_MANUFACTURER_SPARE_PART_NUMBER) + \
  p16(uds.DATA_IDENTIFIER_TYPE.VEHICLE_MANUFACTURER_ECU_SOFTWARE_VERSION_NUMBER) + \
  p16(uds.DATA_IDENTIFIER_TYPE.APPLICATION_DATA_IDENTIFICATION)
GREATWALLMOTORS_VERSION_RESPONSE = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER + 0x40])

GREATWALLMOTORS_RX_OFFSET = 0x6a

FW_QUERY_CONFIG = FwQueryConfig(
  requests=[request for bus, obd_multiplexing in [(1, True), (1, False), (0, False)] for request in [
    Request(
      [GREATWALLMOTORS_VERSION_REQUEST_MULTI],
      [GREATWALLMOTORS_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.engine],
      rx_offset=GREATWALLMOTORS_RX_OFFSET,
      bus=bus,
      obd_multiplexing=obd_multiplexing,
    ),
    Request(
      [GREATWALLMOTORS_VERSION_REQUEST_MULTI],
      [GREATWALLMOTORS_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.engine],
      bus=bus,
      obd_multiplexing=obd_multiplexing,
    ),
  ]],
)

DBC = CAR.create_dbc_map()
