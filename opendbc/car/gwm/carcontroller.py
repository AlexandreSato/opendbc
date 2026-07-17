import numpy as np
from opendbc.can.packer import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.lateral import apply_meas_steer_torque_limits, apply_steer_angle_limits_vm
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.vehicle_model import VehicleModel
from opendbc.car.gwm import gwmcan
from opendbc.car.gwm.values import CarControllerParams, CAR

SteerControlType = structs.CarParams.SteerControlType

LongCtrlState = structs.CarControl.Actuators.LongControlState

MAX_USER_TORQUE = 100  # 1.0 Nm

# MK4 hands-on keepalive: driver torque we spoof to the camera/ADAS (via 0x147 on the camera bus) so it never
# runs its hands-off "hold the wheel" warning + safe-stop escalation and the EPS keeps steering hands-off.
# comma's DM camera is the attention monitor. The MK3 torque path does this dynamically (ea_simulated_torque =
# apply_torque*2). The first MK4 try used a FIXED 65 and the EPS still limped at highway speed (routes 110-112,
# verified transmitting @50Hz) -- because the OEM only recognizes "hands on" around |torque| > ~102, so 65 STILL
# read as hands-off. Fix: sit the FLOOR clearly above ~102 and scale up with the commanded angle like MK3 (more
# steering effort -> more apparent driver torque), capped below a real hard grab so a genuine override still
# passes through. Follows the commanded steer direction. Tunable on-road: raise the floor if it still limps,
# lower the cap if the ADAS flags implausible torque.
MK4_HANDS_ON_TORQUE = 120      # floor (was 65): above the OEM hands-on recognition point (~102)
MK4_HANDS_ON_TORQUE_MAX = 170  # cap: strong hands-on, still below the driver's real hard grabs (125-214)
MK4_HANDS_ON_ANGLE_GAIN = 8    # extra spoofed torque per deg of |apply_angle| (mimics MK3's dynamic scaling)

# MK4 override thresholds. This driver NEVER rests a hand on the wheel (fully hands-off), so the old
# resting-hand-flap concern that pushed these up doesn't apply here — and it was HURTING take-control: on the
# highway limps (route 110) the driver's grabs to take over read 44-85 and even the real hard grabs 125-214
# didn't reliably register because the gate sat at 120. Match MK3's threshold (MAX_USER_TORQUE=100) so a genuine
# grab hands off cleanly, with a firm-grab instant release just above the OEM hands-on point (~102).
# Debounce still tolerates brief spikes.
OVERRIDE_TORQUE = 100          # sustained |driver torque| to hand off (was 120; = MK3 MAX_USER_TORQUE).
OVERRIDE_INSTANT_TORQUE = 150  # firm deliberate grab -> release within one frame for a clean takeover (was 250)
OVERRIDE_FRAMES = 7            # ~70 ms sustained @100 Hz before engaging override; tolerates brief spikes
OVERRIDE_HOLD_FRAMES = 100     # once overridden, stay handed-off ~1.0 s before re-asserting, like the OEM LKAS.
                               # The old latch re-asserted the instant torque dipped, so openpilot grabbed back
                               # "several times a second". The OEM waits ~1 s before trying again; this hold matches that and stops the flapping.


# MK4 regen-lead: every real braking demand recovers energy (regen state bit ON), not just deep braking. The old
# code only enabled regen below accel<-0.5 (Low)/-0.3 (Normal), so mild / hold-speed braking (incl. gentle
# downhills) was pure friction with the powertrain still in the gas/drive state -- wasting energy and burning pads
# (verified on the 8h30 drive: 49% of friction frames had regen OFF, readback msg283.b22 ~9 = 0%, while manual
# driving held b22 ~230 on the same descents). The regen state bit now follows the brake request directly: any
# accel<0 regenerates. Regen MAGNITUDE stays the powertrain's call (driver Low/Normal/Heavy selector, msg 726).
#
# CAUTION (drives 13e-146, 2026-07-16): an earlier version also pinned GAS_CMD=-192 (raw 0) and held brake mode
# through a lift-off deadband/hysteresis. That FAULTED the OEM ACC ECU ("Cruise Fault: Restart the Car" / TAKE
# CONTROL red screen -- accFaulted = ACC.CRUISE_STATE_2==0): under pcmCruise=False the dormant OEM ACC never sees
# GAS_CMD raw 0, nor a sustained brake request (req=13) held at the -41 brake-off baseline, so it rejected the
# frame. Those are reverted. Keep the frame otherwise byte-identical to the known-good 8h30 build. Lift-off-level
# regen (one-pedal feel) + gas/brake anti-dither are deferred until this base regen is confirmed fault-free.


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.params = CarControllerParams(self.CP)
    self.packer = CANPacker(dbc_names[Bus.main])
    self.apply_torque_last = 0
    self.apply_angle_last = 0.0
    self.CAN = gwmcan.CanBus(CP)
    self.accel = 0.0
    self.is_mk4 = CP.carFingerprint == CAR.GWM_HAVAL_H6_MK4
    self.VM = VehicleModel(CP)
    self.override_active = False   # MK4: debounced driver-override latch
    self.override_counter = 0
    self.override_hold = 0         # MK4: frames remaining in the post-override OEM-style hold-off
    self.regen_brake = False       # MK4: hysteretic braking/regen latch (regen-lead: braking == regen state)

  def update(self, CC, CS, now_nanos):
    can_sends = []
    actuators = CC.actuators
    if self.is_mk4:
      tq = abs(CS.out.steeringTorque)
      if tq > OVERRIDE_INSTANT_TORQUE:
        self.override_counter = OVERRIDE_FRAMES
      elif tq > OVERRIDE_TORQUE:
        self.override_counter = min(OVERRIDE_FRAMES, self.override_counter + 1)
      else:
        self.override_counter = max(0, self.override_counter - 1)
      if self.override_active:
        # OEM-style hold: keep control handed back for OVERRIDE_HOLD_FRAMES, then release only once the
        # driver torque has actually decayed.
        if self.override_hold > 0:
          self.override_hold -= 1
        if self.override_counter <= 0 and self.override_hold <= 0:
          self.override_active = False
      elif self.override_counter >= OVERRIDE_FRAMES:
        self.override_active = True
        self.override_hold = OVERRIDE_HOLD_FRAMES
      lat_active = CC.latActive and not self.override_active
    else:
      lat_active = CC.latActive and abs(CS.out.steeringTorque) < MAX_USER_TORQUE

    # Increment counter so cancel is prioritized even without openpilot longitudinal
    if CC.cruiseControl.cancel:
      counter = (CS.steer_and_ap_stalk_msg['COUNTER'] + 1) % 16
      can_sends.append(gwmcan.create_buttons_command(
        self.packer,
        self.CAN,
        counter,
        CS.steer_and_ap_stalk_msg,
        cancel_command=True,
      ))

    if self.frame % 2 == 0: # 50 Hz
      if self.CP.steerControlType == SteerControlType.angle:
        # MK4: 14-bit angle command (STEER_REQUEST) in STEER_CMD. Command the controller's wheel angle straight
        # through, then limit lateral jerk + accel via the vehicle model (Tesla-style). The helper holds the
        # current wheel angle when not lat_active, giving a bumpless hand-back on driver override.
        target_angle = actuators.steeringAngleDeg
        apply_angle = apply_steer_angle_limits_vm(target_angle, self.apply_angle_last,
                                                  CS.out.vEgoRaw, CS.out.steeringAngleDeg, lat_active,
                                                  CarControllerParams, self.VM)
        # MK4: stop the command from winding far past the measured wheel during EPS under-execution
        # (see MK4_ANGLE_ERROR_MAX). MK3 (torque path) is unaffected.
        if self.CP.carFingerprint == CAR.GWM_HAVAL_H6_MK4 and lat_active:
          apply_angle = float(np.clip(apply_angle,
                                      CS.out.steeringAngleDeg - CarControllerParams.MK4_ANGLE_ERROR_MAX,
                                      CS.out.steeringAngleDeg + CarControllerParams.MK4_ANGLE_ERROR_MAX))
        can_sends.append(gwmcan.create_steer_command_angle(
          self.packer,
          self.CAN,
          camera_stock_values=CS.camera_stock_values,
          apply_angle=apply_angle,
          lat_active=lat_active,
        ))
        self.apply_angle_last = apply_angle

        # MK4 hands-on keepalive: re-transmit the EPS 0x147 frame to the camera with a spoofed hands-on torque
        # (see MK4_HANDS_ON_TORQUE) so the ADAS never enters its hands-off warning/safe-stop escalation and the
        # EPS doesn't limp mid-drive. Follow the commanded steer direction; let a real driver grab pass through.
        # Only while engaged (relay closed): when disengaged the stock 0x147 reaches the camera directly, so a
        # spoofed copy would double it. gate on CC.enabled.
        if CC.enabled and CS.eps_stock_raw is not None:
          mag = min(MK4_HANDS_ON_TORQUE_MAX, MK4_HANDS_ON_TORQUE + MK4_HANDS_ON_ANGLE_GAIN * abs(apply_angle))
          spoof_torque = int(mag if apply_angle >= 0 else -mag)
          if abs(CS.out.steeringTorque) > mag:  # real grab is stronger -> forward the true torque/direction
            spoof_torque = int(CS.out.steeringTorque)
          can_sends.append(gwmcan.create_wheel_touch_mk4(self.CAN, CS.eps_stock_raw, spoof_torque))
      else:
        new_torque = int(round(actuators.torque * self.params.STEER_MAX))
        apply_torque = apply_meas_steer_torque_limits(new_torque, self.apply_torque_last, CS.out.steeringTorqueEps, self.params)
        # Prevent sending the same 'apply_torque = 1' torque repeatedly, as it can cause EPS faults.
        if abs(apply_torque) == 1:
          apply_torque = apply_torque * 2
        if not lat_active:
          apply_torque = 0
        can_sends.append(gwmcan.create_steer_command(
          self.packer,
          self.CAN,
          camera_stock_values=CS.camera_stock_values,
          steer=apply_torque,
          steer_req=lat_active,
        ))
        self.apply_torque_last = apply_torque

        ea_simulated_torque = float(np.clip(apply_torque * 2, -self.params.STEER_MAX, self.params.STEER_MAX))
        if abs(CS.out.steeringTorque) > abs(ea_simulated_torque):
          ea_simulated_torque = CS.out.steeringTorque
        can_sends.append(gwmcan.create_wheel_touch(
          self.packer,
          self.CAN,
          eps_stock_values=CS.eps_stock_values,
          ea_simulated_torque=ea_simulated_torque,
        ))

      # Longitudinal control
      if self.CP.openpilotLongitudinalControl:
        standstill = actuators.longControlState == LongCtrlState.stopping
        self.accel = float(np.clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
        if self.accel < 0:
          accel = - abs(self.accel / CarControllerParams.ACCEL_MIN)
        else:
          accel = self.accel / CarControllerParams.ACCEL_MAX
        # MK4 regen-lead (see note above): every real brake regenerates (regen state == braking). Frame otherwise
        # byte-identical to the known-good build -- braking is the plain sign of accel, no held-coast, no GAS floor.
        braking = self.accel < 0
        if self.is_mk4:
          self.regen_brake = CC.longActive and braking
        can_sends.append(gwmcan.create_longitudinal_command(
          self.packer,
          self.CAN,
          longitudinal_stock_values=CS.longitudinal_stock_values,
          accel=accel,
          active=CC.longActive,
          standstill=standstill,
          is_mk4=self.is_mk4,
          regen=self.regen_brake,
          braking=braking,
        ))

    if self.frame % 5 == 0: # 20 Hz
      # HUD updates
      can_sends.append(gwmcan.create_hud_command(
        self.packer,
        self.CAN,
        hud_stock_values=CS.hud_stock_values,
        steer_required=CC.latActive,
        is_mk4=self.is_mk4,
      ))

    new_actuators = actuators.as_builder()
    if self.CP.steerControlType == SteerControlType.angle:
      new_actuators.steeringAngleDeg = self.apply_angle_last
    else:
      new_actuators.torque = self.apply_torque_last / self.params.STEER_MAX
      new_actuators.torqueOutputCan = self.apply_torque_last
    new_actuators.accel = self.accel

    self.frame += 1
    return new_actuators, can_sends
