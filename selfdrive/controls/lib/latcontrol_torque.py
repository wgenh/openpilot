import math
import numpy as np
from collections import deque

from cereal import log
from openpilot.common.numpy_fast import interp
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.selfdrive.controls.lib.pid import PIDController
from openpilot.selfdrive.controls.lib.vehicle_model import ACCELERATION_DUE_TO_GRAVITY
from openpilot.selfdrive.controls.lib.drive_helpers import get_friction

# At higher speeds (25+mph) we can assume:
# Lateral acceleration achieved by a specific car correlates to
# torque applied to the steering rack. It does not correlate to
# wheel slip, or to speed.

# This controller applies torque to achieve desired lateral
# accelerations. To compensate for the low speed effects we
# use a LOW_SPEED_FACTOR in the error. Additionally, there is
# friction in the steering wheel that needs to be overcome to
# move it at all, this is compensated for too.

LOW_SPEED_X = [0, 10, 20, 30]
LOW_SPEED_Y = [15, 13, 10, 5]


class NanoFFModel:
  def __init__(self, temperature=1.0):
    self.w_1 = [[0.3670869767665863, -0.13670381903648376, -0.04756021499633789, -0.6537201404571533, 0.3472885191440582, 0.3419428765773773, -0.051871705800294876, -0.4816872775554657, -0.16528992354869843, 0.12323025614023209, 0.27154049277305603, -0.39521098136901855, -0.006314488127827644, 0.6651855111122131, 0.04361455515027046, 0.40486374497413635], [-0.09755722433328629, 0.4448520839214325, 0.03674095496535301, 0.2551441192626953, -0.06552024930715561, 0.012307964265346527, 0.01062088180333376, -0.14146888256072998, -0.12132205069065094, 0.020976416766643524, -0.27929607033729553, 0.24239453673362732, -0.03685691952705383, -0.3797924816608429, -0.18146292865276337, -0.0837688148021698], [-0.09720412641763687, -0.03286902979016304, -0.09091371297836304, 0.031427789479494095, -0.10913054645061493, 0.015439788810908794, 0.19777365028858185, 0.011424046009778976, -0.17703686654567719, 0.17143619060516357, 0.13303959369659424, 0.0631164088845253, 0.14098651707172394, 0.047740399837493896, -0.16221965849399567, -0.060127075761556625], [-0.02413320168852806, 0.14823046326637268, -0.047637540847063065, 0.02193460613489151, -0.008727051317691803, 0.28499743342399597, -0.20324677228927612, -0.024012122303247452, -0.20563958585262299, 0.22214773297309875, -0.07418151944875717, 0.01865888200700283, 0.2222577929496765, -0.034176528453826904, -0.12090824544429779, 0.2033114731311798]]  # noqa: E501
    self.b_1 = [0.3362099826335907, 0.10986226797103882, -0.016399161890149117, 0.20198550820350647, 0.3768672049045563, 0.43433839082717896, -0.19452157616615295, 0.31464865803718567, 0.07839246839284897, 0.39521074295043945, 0.42810776829719543, 0.24146676063537598, 0.5519666075706482, -0.076285719871521, 0.021531319245696068, 0.045368798077106476]  # noqa: E501
    self.w_2 = [[-0.08486070483922958, 0.01192394271492958, -0.07121425867080688, -0.05305226892232895, -0.10512731224298477, 0.37571147084236145, -0.023183071985840797, 0.3279595673084259], [0.24882152676582336, -0.010815223678946495, 0.10351767390966415, -0.11141183227300644, -0.058804046362638474, 0.11324000358581543, -0.06191949546337128, 0.13753317296504974], [0.08484940975904465, -0.1055818572640419, 0.08019820600748062, -0.029938261955976486, 0.07968451082706451, -0.08953803777694702, 0.09358040243387222, -0.06995416432619095], [1.2485003471374512, -0.0673857256770134, -0.09868630021810532, 0.025014957413077354, -0.0007586166029796004, -1.4332948923110962, -0.09456907957792282, -1.4142694473266602], [0.04119781404733658, 0.07675094902515411, -0.03177952393889427, -0.08969161659479141, -0.06676463782787323, 0.2508836090564728, 0.0028785387985408306, 0.21125593781471252], [0.0029771667905151844, 0.05020223557949066, 0.0025614865589886904, -0.020050542429089546, -0.09777888655662537, 0.20557016134262085, -0.034180644899606705, 0.25298601388931274], [-0.019257785752415657, -0.05102602392435074, 0.017074178904294968, 0.08377230167388916, -0.10951891541481018, -0.03904469311237335, 0.08356964588165283, -0.05567192658782005], [-1.0154119729995728, -0.04138084873557091, 0.025381365790963173, -0.013071980327367783, -0.09877105057239532, 1.0566171407699585, 0.03266756609082222, 1.0141639709472656], [0.04500339552760124, -0.11147982627153397, 0.04592466726899147, 0.0032524224370718002, -0.023737244307994843, -0.09188950061798096, 0.05623103678226471, 0.018191184848546982], [0.009710954502224922, -0.02949015609920025, 0.004294345621019602, -0.024654969573020935, 0.056295108050107956, 0.16392560303211212, 0.03165062516927719, 0.25089484453201294], [-0.02603774145245552, -0.1195405051112175, -0.05710092931985855, 0.00818283949047327, 0.05478727072477341, 0.4326222240924835, -0.0378209725022316, 0.2559981644153595], [0.2447129189968109, -0.11289433389902115, 0.05267596244812012, 0.05980170890688896, -0.11930987238883972, 0.11911609023809433, -0.02461097575724125, 0.08115680515766144], [0.1258135735988617, -0.033008232712745667, -0.11496032774448395, 0.09631512314081192, 0.06479545682668686, 0.29638996720314026, -0.10611703246831894, 0.12842239439487457], [-1.5275181531906128, -0.024188872426748276, -0.06936195492744446, 0.07431869953870773, -0.09157448261976242, 1.1211636066436768, 0.05373019352555275, 1.156968116760254], [-0.11582547426223755, -0.04792605713009834, -0.02929513156414032, 0.11110595613718033, -0.024740438908338547, -0.10561241954565048, 0.09291660785675049, -0.042067330330610275], [-0.08932635933160782, -0.01821572706103325, -0.05700545012950897, -0.00868851225823164, -0.04745101183652878, 0.27841466665267944, 0.07039258629083633, 0.3041419982910156]]  # noqa: E501
    self.b_2 = [0.07896982133388519, -0.012229854241013527, -0.0035900722723454237, -0.037341248244047165, 0.00706241512671113, 0.2521328926086426, -0.010501053184270859, 0.30783456563949585]  # noqa: E501
    self.w_3 = [[-1.3429830074310303, -0.136769637465477, 0.02066110074520111, 0.04597363993525505], [-0.06667837500572205, 0.06406445056200027, 0.14823555946350098, 0.11649730801582336], [0.13364139199256897, -0.11788422614336014, 0.10123910009860992, -0.051450371742248535], [-0.1186937540769577, -0.046979717910289764, -0.10463082045316696, -0.005663537420332432], [-0.0033075830433517694, -0.05552775785326958, 0.07422681152820587, 0.08520332723855972], [0.5206831097602844, -0.07504156976938248, -0.1169903576374054, -0.1668350100517273], [0.05011070892214775, 0.001810301560908556, -0.0701432004570961, 0.11965345591306686], [0.35074299573898315, 0.08597637712955475, 0.04867742583155632, -0.1446799486875534]]  # noqa: E501
    self.b_3 = [0.27658483386039734, -0.06846883893013, -0.08716510236263275, 0.014693153090775013]
    self.w_4 = [[0.5715762972831726, 0.08405161648988724], [-0.0510501004755497, 0.07599254697561264], [-0.13895413279533386, -0.11240525543689728], [-0.07673900574445724, -0.16612276434898376]]  # noqa: E501
    self.b_4 = [-0.14364436268806458, -2.827202320098877]

    self.input_norm_mat = np.array([[-3.0, 3.0], [-3.0, 3.0], [0.0, 40.0], [-3.0, 3.0]])
    self.output_norm_mat = np.array([-1.0, 1.0])
    self.temperature = temperature

  def sigmoid(self, x):
    return 1 / (1 + np.exp(-x))

  def relu(self, x):
    return np.maximum(0.0, x)

  def forward(self, x):
    assert x.ndim == 1
    x = (x - self.input_norm_mat[:, 0]) / (self.input_norm_mat[:, 1] - self.input_norm_mat[:, 0])
    x = self.relu(np.dot(x, self.w_1) + self.b_1)
    x = self.relu(np.dot(x, self.w_2) + self.b_2)
    x = self.relu(np.dot(x, self.w_3) + self.b_3)
    x = np.dot(x, self.w_4) + self.b_4
    return x

  def predict(self, x):
    x = self.forward(np.array(x))
    pred = np.random.laplace(x[0], np.exp(x[1]) / self.temperature)
    pred = pred * (self.output_norm_mat[1] - self.output_norm_mat[0]) + self.output_norm_mat[0]
    return float(pred)


class LatControlTorque(LatControl):
  def __init__(self, CP, CI):
    super().__init__(CP, CI)
    self.torque_params = CP.lateralTuning.torque
    self.pid = PIDController(self.torque_params.kp, self.torque_params.ki,
                             k_f=self.torque_params.kf, pos_limit=self.steer_max, neg_limit=-self.steer_max)
    self.torque_from_lateral_accel = CI.torque_from_lateral_accel()
    self.use_steering_angle = self.torque_params.useSteeringAngle
    self.steering_angle_deadzone_deg = self.torque_params.steeringAngleDeadzoneDeg
    self.ff_model = NanoFFModel(temperature=100.0)
    self.history = {key: deque([0, 0, 0], maxlen=3) for key in ["lataccel", "roll_compensation", "vego", "aego"]}
    self.history_counter = 0

  def update_live_torque_params(self, latAccelFactor, latAccelOffset, friction):
    self.torque_params.latAccelFactor = latAccelFactor
    self.torque_params.latAccelOffset = latAccelOffset
    self.torque_params.friction = friction

  def update(self, active, CS, VM, params, steer_limited, desired_curvature, llk):
    pid_log = log.ControlsState.LateralTorqueState.new_message()
    actual_curvature_vm = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
    roll_compensation = math.sin(params.roll) * ACCELERATION_DUE_TO_GRAVITY

    if not active:
      output_torque = 0.0
      pid_log.active = False
    else:
      if self.use_steering_angle:
        actual_curvature = actual_curvature_vm
        # curvature_deadzone = abs(VM.calc_curvature(math.radians(self.steering_angle_deadzone_deg), CS.vEgo, 0.0))
      else:
        actual_curvature_llk = llk.angularVelocityCalibrated.value[2] / CS.vEgo
        actual_curvature = interp(CS.vEgo, [2.0, 5.0], [actual_curvature_vm, actual_curvature_llk])
        # curvature_deadzone = 0.0
      desired_lateral_accel = desired_curvature * CS.vEgo ** 2

      # desired rate is the desired rate of change in the setpoint, not the absolute desired curvature
      # desired_lateral_jerk = desired_curvature_rate * CS.vEgo ** 2
      actual_lateral_accel = actual_curvature * CS.vEgo ** 2

      low_speed_factor = interp(CS.vEgo, LOW_SPEED_X, LOW_SPEED_Y)**2
      setpoint = desired_lateral_accel + low_speed_factor * desired_curvature
      measurement = actual_lateral_accel + low_speed_factor * actual_curvature

      # lateral_accel_deadzone = curvature_deadzone * CS.vEgo ** 2
      # gravity_adjusted_lateral_accel = desired_lateral_accel - roll_compensation
      # torque_from_setpoint = self.torque_from_lateral_accel(setpoint, self.torque_params, setpoint,
      #                                                lateral_accel_deadzone, friction_compensation=False)
      # torque_from_measurement = self.torque_from_lateral_accel(measurement, self.torque_params, measurement,
      #                                                lateral_accel_deadzone, friction_compensation=False)
      # pid_log.error = torque_from_setpoint - torque_from_measurement
      # ff = self.torque_from_lateral_accel(gravity_adjusted_lateral_accel, self.torque_params,
      #                                     desired_lateral_accel - actual_lateral_accel,
      #                                     lateral_accel_deadzone, friction_compensation=True)

      state_vector = [roll_compensation, CS.vEgo, CS.aEgo]
      # history_state_vector = list(self.history["lataccel"]) + list(self.history["roll_compensation"]) + list(self.history["vego"]) + list(self.history["aego"])  # noqa: E501
      # torque_from_setpoint = self.ff_model.predict([setpoint] + state_vector + history_state_vector)
      # torque_from_measurement = self.ff_model.predict([measurement] + state_vector + history_state_vector)

      torque_from_setpoint = self.ff_model.predict([setpoint] + state_vector)
      torque_from_measurement = self.ff_model.predict([measurement] + state_vector)

      pid_log.error = torque_from_setpoint - torque_from_measurement
      # ff = self.ff_model.predict([desired_lateral_accel] + state_vector + history_state_vector)
      ff = self.ff_model.predict([desired_lateral_accel] + state_vector)

      friction = get_friction(pid_log.error, 0.0, 0.3, self.torque_params, True)
      ff += friction

      freeze_integrator = steer_limited or CS.steeringPressed or CS.vEgo < 5
      output_torque = self.pid.update(pid_log.error,
                                      feedforward=ff,
                                      speed=CS.vEgo,
                                      freeze_integrator=freeze_integrator)

      pid_log.active = True
      pid_log.p = self.pid.p
      pid_log.i = self.pid.i
      pid_log.d = self.pid.d
      pid_log.f = self.pid.f
      pid_log.output = -output_torque
      pid_log.actualLateralAccel = actual_lateral_accel
      pid_log.desiredLateralAccel = desired_lateral_accel
      pid_log.saturated = self._check_saturation(self.steer_max - abs(output_torque) < 1e-3, CS, steer_limited)

    if self.history_counter % 10 == 0:
      self.history["lataccel"].append(actual_curvature_vm * CS.vEgo ** 2)
      self.history["roll_compensation"].append(roll_compensation)
      self.history["vego"].append(CS.vEgo)
      self.history["aego"].append(CS.aEgo)

    self.history_counter = (self.history_counter + 1) % 10

    # TODO left is positive in this convention
    return -output_torque, 0.0, pid_log
