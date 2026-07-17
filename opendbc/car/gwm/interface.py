from opendbc.car import structs, get_safety_config, CanBusBase, Bus, create_button_events
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.gwm.carcontroller import CarController
from opendbc.car.gwm.carstate import CarState
from opendbc.car.gwm.values import CAR, GwmSafetyFlags

SteerControlType = structs.CarParams.SteerControlType

ButtonType = structs.CarState.ButtonEvent.Type
TransmissionType = structs.CarParams.TransmissionType

# MK4 (angle control) hands-off latch for the steerTempUnavailable counter. The angle-path EPS flips
# A_RX_STEER_REQUESTED to 2 ("driver override") at modest driver torque, so A_RX!=1 alone is NOT a
# fault — it's a normal hands-on override. GWM exposes no dedicated EPS-inhibited signal (cf. Tesla
# EAC_INHIBITED / Ford EPAS_Failure), so fault only when the EPS stops AND the driver isn't the cause:
# the wheel hasn't been grabbed recently. A genuine hands-off limp keeps driver torque ~0.
MK4_GRAB_TORQUE = 80          # |driver torque| marking a deliberate hands-on override
MK4_GRAB_HOLD_FRAMES = 150    # ~1.5 s: keep treating the wheel as hands-on after the last grab


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  def __init__(self, CP):
      super().__init__(CP)
      self.lat_active = False
      self.isEPSobeying = True
      self.steer_fault_temporary_counter = 0
      self.recent_grab = 0    # MK4: frames remaining in the hands-on hold-off (see MK4_GRAB_* above)
      self.current_personality = 0
      self.pcm_follow_distance = 0
      self.prev_pcm_follow_distance = -1
      self.press_gac_button = False    # MK3 only
      # MK4 mirrors the car's OEM follow-distance dash straight onto openpilot's personality by writing
      # the LongitudinalPersonality param directly (selfdrived re-reads it every 0.1 s). Params lives in
      # openpilot, which isn't a hard dependency of opendbc, so import it lazily and degrade gracefully.
      self.params = None
      try:
        from openpilot.common.params import Params
        self.params = Params()
      except Exception:
        self.params = None

  def apply(self, CC, now_nanos):
    self.lat_active = CC.latActive
    hud_control = CC.hudControl
    self.current_personality = hud_control.leadDistanceBars
    return super().apply(CC, now_nanos)

  def update(self, can_packets):
    cp = self.can_parsers[Bus.main]
    self.isEPSobeying = cp.vl["RX_STEER_RELATED"]["A_RX_STEER_REQUESTED"] == 1

    # MK4: stash the raw EPS RX_STEER_RELATED (0x147) frame off the main bus so the carcontroller can re-transmit
    # it to the camera (hands-on keepalive). The DBC doesn't model the whole 64-byte frame, so we forward raw
    # bytes and patch only the driver-torque field there (see gwmcan.create_wheel_touch_mk4). can_packets is a
    # list of (nanos, [(address, dat, src), ...]) tuples (see can_capnp_to_list) -- index, don't attribute.
    if self.CP.carFingerprint == CAR.GWM_HAVAL_H6_MK4:
      for _, msgs in can_packets:
        for address, dat, src in msgs:
          if address == 0x147 and src == 0:
            self.CS.eps_stock_raw = bytes(dat)

    if self.CP.carFingerprint == CAR.GWM_HAVAL_H6_MK4:
      # Suppress the fault while the driver is overriding (recent grab) — see MK4_GRAB_* note above.
      driver_torque = cp.vl["RX_STEER_RELATED"]["B_RX_DRIVER_TORQUE"]
      self.recent_grab = MK4_GRAB_HOLD_FRAMES if abs(driver_torque) > MK4_GRAB_TORQUE else max(0, self.recent_grab - 1)
      hands_on = self.recent_grab > 0
      self.steer_fault_temporary_counter = (self.steer_fault_temporary_counter + 1) \
                                            if (self.lat_active and not self.isEPSobeying and not hands_on) else 0
    else:
      self.steer_fault_temporary_counter = (self.steer_fault_temporary_counter + 1) if (self.lat_active and not self.isEPSobeying) \
                                            else 0

    cp_cam = self.can_parsers[Bus.cam]
    self.pcm_follow_distance = cp_cam.vl["ACC"]["CAR_DISTANCE_SELECTION"]

    ret = super().update(can_packets)
    ret.steerFaultTemporary |= self.steer_fault_temporary_counter > 100

    # Driving personality:
    #  - MK4: cycled by the wheel follow-distance buttons -> gapAdjustCruise buttonEvents built in carstate
    #    (the OEM CAR_DISTANCE_SELECTION dash goes stale once openpilot owns the car, so we no longer mirror it).
    #  - MK3: mirror the OEM follow-distance dash via a gapAdjustCruise toggle.
    if self.CP.carFingerprint != CAR.GWM_HAVAL_H6_MK4:
      if self.pcm_follow_distance != self.prev_pcm_follow_distance:
        if (self.pcm_follow_distance == 4 and self.current_personality != 3) or \
           (self.pcm_follow_distance == 3 and self.current_personality != 3) or \
           (self.pcm_follow_distance == 2 and self.current_personality != 2) or \
           (self.pcm_follow_distance == 1 and self.current_personality != 1):
          self.press_gac_button = not self.press_gac_button
      self.prev_pcm_follow_distance = self.pcm_follow_distance
      ret.buttonEvents = create_button_events(self.press_gac_button, True, {1: ButtonType.gapAdjustCruise})

    return ret

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = 'gwm'

    cfgs = [get_safety_config(structs.CarParams.SafetyModel.gwm)]

    # If multipanda mapping is detected (offset >= 4), keep the first safety slot
    # as `noOutput` so an internal panda remains silent and the vehicle safety config
    # stays as the last entry (`-1`). This enables external panda to control the vehicle.
    CAN = CanBusBase(None, fingerprint)
    if CAN.offset >= 4:
      cfgs.insert(0, get_safety_config(structs.CarParams.SafetyModel.noOutput))

    ret.safetyConfigs = cfgs

    ret.dashcamOnly = False

    ret.steerActuatorDelay = 0.3
    ret.steerLimitTimer = 0.4
    ret.steerAtStandstill = False

    if candidate == CAR.GWM_HAVAL_H6_MK4:
      # Angle-based steering: do NOT call configure_torque_tune — controlsd would
      # call update_live_torque_params on LatControlAngle which doesn't implement it.
      ret.steerControlType = SteerControlType.angle
      ret.steerActuatorDelay = 0.08
      # MK4 owns its own cruise loop: the OEM ACC freezes its set speed once openpilot drives, so don't
      # follow it. openpilot manages engagement + set-speed from the wheel buttons (carstate buttonEvents).
      ret.pcmCruise = False
      # Tell the panda to arm controls off the gentle-DOWN stalk gesture (0xC7 GEAR_STALK) instead of the
      # FURTHER_DOWN-only msg 161 bit47 — matches the engage source in carstate so the two gates stay in sync.
      ret.safetyConfigs[-1].safetyParam |= GwmSafetyFlags.OP_CRUISE.value
    else:
      ret.steerControlType = structs.CarParams.SteerControlType.torque
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    ret.radarUnavailable = True

    ret.alphaLongitudinalAvailable = True
    if alpha_long:
      ret.openpilotLongitudinalControl = True
      ret.safetyConfigs[-1].safetyParam |= GwmSafetyFlags.LONG_CONTROL.value

      ret.longitudinalActuatorDelay = 0.25
      ret.vEgoStopping = 0.25
      ret.vEgoStarting = 0.25
      ret.stopAccel = -0.75
      ret.stoppingDecelRate = 0.75
      ret.longitudinalTuning.kiBP = [0.]
      ret.longitudinalTuning.kiV = [0.4]

    return ret
