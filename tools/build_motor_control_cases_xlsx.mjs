import fs from "node:fs/promises";
import { SpreadsheetFile, Workbook } from "@oai/artifact-tool";

const outputDir = "outputs/motor_control_cases";
const outputPath = `${outputDir}/motor_control_cases.xlsx`;

const C = {
  armMin: 0,
  armMax: 160,
  neutral: 80,
  deltaArmMax: 160,
  leftZero: 2480,
  rightZero: 636,
  pulsePerDeg: 2000 / 180,
  guidanceAttenuateS: 0.5,
  guidanceFailS: 1.5,
  gyroSpikeDegS: 250,
  integralDecayRate: 0.95,
  deltaMinEffective: 8,
  expo: 0.8,
  deadbandDegS: 3,
};

const policies = {
  NOMINAL_CLOSED_LOOP: {
    confidence: 1.0, cmdMax: 45, latAccMax: 4.0, deltaFfMax: 115,
    deltaPidMax: 45, deltaTotalMax: 160, maxArmRate: 100, pidEnabled: true,
  },
  NOMINAL_FEEDFORWARD: {
    confidence: 1.0, cmdMax: 40, latAccMax: 3.5, deltaFfMax: 105,
    deltaPidMax: 0, deltaTotalMax: 120, maxArmRate: 90, pidEnabled: false,
  },
  DEGRADED_CLOSED_LOOP: {
    confidence: 0.6, cmdMax: 30, latAccMax: 2.5, deltaFfMax: 80,
    deltaPidMax: 40, deltaTotalMax: 120, maxArmRate: 70, pidEnabled: true,
  },
  DEGRADED_FEEDFORWARD: {
    confidence: 0.45, cmdMax: 25, latAccMax: 2.0, deltaFfMax: 65,
    deltaPidMax: 0, deltaTotalMax: 80, maxArmRate: 60, pidEnabled: false,
  },
  TUMBLE_YAW_DOMINANT: {
    confidence: 0.0, cmdMax: 35, latAccMax: 0.0, deltaFfMax: 70,
    deltaPidMax: 0, deltaTotalMax: 80, maxArmRate: 80, pidEnabled: false,
  },
  NO_MOTION_TARGET_BEARING: {
    confidence: 0.45, cmdMax: 25, latAccMax: 0.0, deltaFfMax: 65,
    deltaPidMax: 0, deltaTotalMax: 80, maxArmRate: 60, pidEnabled: false,
  },
};

function clamp(v, lo, hi) {
  return Math.max(lo, Math.min(hi, v));
}

function round(v, digits = 3) {
  if (v === null || v === undefined || Number.isNaN(v)) return "";
  const f = 10 ** digits;
  return Math.round(v * f) / f;
}

function connectRoMo(deltaArmDeg) {
  const delta = clamp(deltaArmDeg, -C.deltaArmMax, C.deltaArmMax);
  const leftAngle = clamp(C.neutral - delta / 2, C.armMin, C.armMax);
  const rightAngle = clamp(C.neutral + delta / 2, C.armMin, C.armMax);
  const leftPw = Math.trunc(clamp(C.leftZero - leftAngle * C.pulsePerDeg, C.leftZero - C.armMax * C.pulsePerDeg, C.leftZero));
  const rightPw = Math.trunc(clamp(C.rightZero + rightAngle * C.pulsePerDeg, C.rightZero, C.rightZero + C.armMax * C.pulsePerDeg));
  return { delta, leftAngle, rightAngle, leftPw, rightPw };
}

function deltaFf(cmdDegS, policy) {
  const clamped = clamp(cmdDegS, -policy.cmdMax, policy.cmdMax);
  if (Math.abs(clamped) < C.deadbandDegS) return 0;
  const x = Math.abs(clamped) / policy.cmdMax;
  const delta = C.deltaMinEffective + (policy.deltaFfMax - C.deltaMinEffective) * (x ** C.expo);
  return Math.sign(clamped) * delta;
}

function simulateControl({
  policyName,
  rawCmdDegS,
  measuredDegS = null,
  guidanceAgeS = 0.05,
  gyroStatus = "valid",
  prevLeftDeg = C.neutral,
  prevRightDeg = C.neutral,
  dt = 0.1,
  previousIntegral = 0,
}) {
  const policy = policies[policyName];
  if (!policy) throw new Error(`Unknown policy ${policyName}`);

  if (guidanceAgeS > C.guidanceFailS) {
    return {
      policyName, controlMode: "GUIDANCE_TIMEOUT", fallback: "GUIDANCE_TIMEOUT",
      rawCmdDegS, usedCmdDegS: 0, measuredDegS: "", errorDegS: "",
      deltaFfDeg: 0, deltaPidDeg: 0, desiredDeltaArmDeg: 0, actualDeltaArmDeg: 0,
      leftAngleDeg: C.neutral, rightAngleDeg: C.neutral,
      leftPw: connectRoMo(0).leftPw, rightPw: connectRoMo(0).rightPw,
      saturated: false, sensorValid: false, slewLimited: false,
    };
  }

  let fallback = "NONE";
  let cmd = rawCmdDegS;
  if (guidanceAgeS > C.guidanceAttenuateS) {
    cmd *= 0.5;
    fallback = "GUIDANCE_ATTENUATED";
  }
  const usedCmdDegS = clamp(cmd, -policy.cmdMax, policy.cmdMax);
  const commandClamped = Math.abs(usedCmdDegS - rawCmdDegS) > 1e-9;
  const ff = deltaFf(usedCmdDegS, policy);

  const gyroSpike = gyroStatus === "spike" || (measuredDegS !== null && Math.abs(measuredDegS) > C.gyroSpikeDegS);
  const sensorValid = measuredDegS !== null && Number.isFinite(measuredDegS) && !gyroSpike;
  if (gyroSpike) fallback = "GYRO_SPIKE";

  let mode = "FEEDFORWARD_ONLY";
  let error = "";
  let pid = 0;
  if (policy.pidEnabled && policy.deltaPidMax > 0 && sensorValid) {
    mode = "CLOSED_LOOP";
    error = usedCmdDegS - measuredDegS;
    if (Math.abs(error) < 2) error = 0;
    const integralCandidate = clamp(previousIntegral + error * dt, -25, 25);
    pid = clamp(0.35 * error + 0.02 * integralCandidate, -policy.deltaPidMax, policy.deltaPidMax);
  }

  const deltaSum = ff + pid;
  const authoritySaturated = Math.abs(deltaSum) > policy.deltaTotalMax;
  const desiredDelta = clamp(deltaSum, -policy.deltaTotalMax, policy.deltaTotalMax);
  const desired = connectRoMo(desiredDelta);
  const maxStep = policy.maxArmRate * dt;
  const leftAngle = clamp(desired.leftAngle, prevLeftDeg - maxStep, prevLeftDeg + maxStep);
  const rightAngle = clamp(desired.rightAngle, prevRightDeg - maxStep, prevRightDeg + maxStep);
  const actualDelta = rightAngle - leftAngle;
  const actual = connectRoMo(actualDelta);
  const slewLimited = Math.abs(leftAngle - desired.leftAngle) > 1e-9 || Math.abs(rightAngle - desired.rightAngle) > 1e-9;

  return {
    policyName,
    controlMode: mode,
    fallback,
    rawCmdDegS,
    usedCmdDegS,
    measuredDegS: measuredDegS === null ? "" : measuredDegS,
    errorDegS: error,
    deltaFfDeg: ff,
    deltaPidDeg: pid,
    desiredDeltaArmDeg: desiredDelta,
    actualDeltaArmDeg: actualDelta,
    leftAngleDeg: leftAngle,
    rightAngleDeg: rightAngle,
    leftPw: actual.leftPw,
    rightPw: actual.rightPw,
    saturated: commandClamped || authoritySaturated,
    sensorValid,
    slewLimited,
  };
}

function movement(left, right) {
  if (left === "OFF" || right === "OFF") return "PWM off";
  if (Math.abs(left - C.neutral) < 0.01 && Math.abs(right - C.neutral) < 0.01) return "neutral";
  if (left > C.neutral && right < C.neutral) return "left turn: left brake down, right release up";
  if (left < C.neutral && right > C.neutral) return "right turn: right brake down, left release up";
  if (left === 0 && right === 0) return "both arms up";
  if (left === 160 && right === 160) return "both arms down";
  return "asymmetric / transient";
}

function caseRow(base, result) {
  return [
    base.id,
    base.category,
    base.inputCommand,
    base.state,
    base.motorEnabled,
    base.manualMode,
    base.originTarget,
    base.posQuality,
    base.motionQuality,
    base.gyrzQuality,
    base.altQuality,
    base.freefall,
    base.tumble,
    base.guidanceMode,
    base.failReason,
    base.controlValid,
    result.policyName || "",
    base.pidEnabled,
    base.guidanceAgeS,
    base.gyroStatus,
    round(result.rawCmdDegS),
    round(result.usedCmdDegS),
    result.controlMode,
    result.fallback,
    round(result.measuredDegS),
    round(result.errorDegS),
    round(result.deltaFfDeg),
    round(result.deltaPidDeg),
    round(result.desiredDeltaArmDeg),
    round(result.actualDeltaArmDeg),
    result.slewLimited ? "Y" : "N",
    round(result.leftAngleDeg),
    round(result.rightAngleDeg),
    movement(result.leftAngleDeg, result.rightAngleDeg),
    result.leftPw ?? "",
    result.rightPw ?? "",
    result.saturated ? "Y" : "N",
    result.sensorValid ? "Y" : "N",
    base.note,
  ];
}

function neutralResult(mode = "NEUTRAL", fallback = "GUIDANCE_INACTIVE") {
  const r = connectRoMo(0);
  return {
    policyName: "",
    rawCmdDegS: 0,
    usedCmdDegS: 0,
    controlMode: mode,
    fallback,
    measuredDegS: "",
    errorDegS: "",
    deltaFfDeg: 0,
    deltaPidDeg: 0,
    desiredDeltaArmDeg: 0,
    actualDeltaArmDeg: 0,
    leftAngleDeg: C.neutral,
    rightAngleDeg: C.neutral,
    leftPw: r.leftPw,
    rightPw: r.rightPw,
    saturated: false,
    sensorValid: false,
    slewLimited: false,
  };
}

function fixedDeltaResult(delta, mode, fallback = mode) {
  const r = connectRoMo(delta);
  return {
    policyName: "",
    rawCmdDegS: delta,
    usedCmdDegS: delta,
    controlMode: mode,
    fallback,
    measuredDegS: "",
    errorDegS: "",
    deltaFfDeg: 0,
    deltaPidDeg: 0,
    desiredDeltaArmDeg: delta,
    actualDeltaArmDeg: r.delta,
    leftAngleDeg: r.leftAngle,
    rightAngleDeg: r.rightAngle,
    leftPw: r.leftPw,
    rightPw: r.rightPw,
    saturated: Math.abs(delta) > C.deltaArmMax,
    sensorValid: false,
    slewLimited: false,
  };
}

function directAnglesResult(leftAngle, rightAngle, mode, fallback = mode) {
  const leftPw = Math.trunc(clamp(C.leftZero - leftAngle * C.pulsePerDeg, C.leftZero - C.armMax * C.pulsePerDeg, C.leftZero));
  const rightPw = Math.trunc(clamp(C.rightZero + rightAngle * C.pulsePerDeg, C.rightZero, C.rightZero + C.armMax * C.pulsePerDeg));
  return {
    policyName: "",
    rawCmdDegS: "",
    usedCmdDegS: "",
    controlMode: mode,
    fallback,
    measuredDegS: "",
    errorDegS: "",
    deltaFfDeg: "",
    deltaPidDeg: "",
    desiredDeltaArmDeg: rightAngle - leftAngle,
    actualDeltaArmDeg: rightAngle - leftAngle,
    leftAngleDeg: leftAngle,
    rightAngleDeg: rightAngle,
    leftPw,
    rightPw,
    saturated: false,
    sensorValid: false,
    slewLimited: false,
  };
}

const baseDefaults = {
  state: 3,
  motorEnabled: "ON",
  manualMode: "NEUTRAL",
  originTarget: "valid origin + valid target",
  posQuality: "FRESH",
  motionQuality: "FRESH",
  gyrzQuality: "FRESH",
  altQuality: "FRESH",
  freefall: 0,
  tumble: 0,
  guidanceMode: "",
  failReason: "NONE",
  controlValid: "Y",
  pidEnabled: "policy",
  guidanceAgeS: 0.05,
  gyroStatus: "valid",
  note: "",
};

let caseId = 1;
const rows = [];
function add(base, result) {
  rows.push(caseRow({ id: caseId++, ...baseDefaults, ...base }, result));
}

add({ category: "Lifecycle", inputCommand: "init_control()", state: "boot", motorEnabled: "N/A", guidanceMode: "N/A", controlValid: "N/A", note: "startup writes zero/up position on both arms" }, directAnglesResult(0, 0, "ZERO_UP", "INIT"));
add({ category: "Lifecycle", inputCommand: "MEC OFF", state: "any", motorEnabled: "OFF", guidanceMode: "N/A", controlValid: "N/A", note: "handle_mec OFF writes zero/up immediately on both arms" }, directAnglesResult(0, 0, "ZERO_UP", "DISABLED"));
add({ category: "Lifecycle", inputCommand: "STATE < 3", state: "0/1/2", guidanceMode: "N/A", controlValid: "N/A", note: "control loop writes zero/up before parafoil control starts" }, directAnglesResult(0, 0, "ZERO_UP", "IDLE"));
add({ category: "Lifecycle", inputCommand: "STATE == 5", state: 5, guidanceMode: "N/A", controlValid: "N/A", note: "landed state calls WriteOff; angle is not commanded because PWM is 0" }, {
  ...neutralResult("OFF", "LANDED"), leftAngleDeg: "OFF", rightAngleDeg: "OFF", leftPw: 0, rightPw: 0,
});
add({ category: "Direct helper", inputCommand: "Set180()", state: "test/helper", motorEnabled: "N/A", guidanceMode: "N/A", controlValid: "N/A", note: "sets both arms down to 180 deg request, clamped by 160 deg limit" }, directAnglesResult(160, 160, "SET180", "SET180"));
add({ category: "Lifecycle", inputCommand: "guidance invalid", posQuality: "STALE", motionQuality: "STALE", guidanceMode: "FAIL", failReason: "NO_POSITION", controlValid: "N", note: "L1 output not control_valid, motorapp writes neutral" }, neutralResult("NEUTRAL", "NO_POSITION"));

add({ category: "Manual", inputCommand: "MTR NEUTRAL", manualMode: "NEUTRAL", guidanceMode: "bypassed", controlValid: "N/A", note: "manual neutral keeps arms at 80 deg" }, neutralResult("MANUAL_NEUTRAL", "MANUAL_NEUTRAL"));
add({ category: "Manual", inputCommand: "MTR LEFT", manualMode: "LEFT", guidanceMode: "bypassed", controlValid: "Y", note: "manual command bypasses guidance/control and applies delta=-60 deg" }, fixedDeltaResult(-60, "MANUAL_LEFT"));
add({ category: "Manual", inputCommand: "MTR RIGHT", manualMode: "RIGHT", guidanceMode: "bypassed", controlValid: "Y", note: "manual command bypasses guidance/control and applies delta=+60 deg" }, fixedDeltaResult(60, "MANUAL_RIGHT"));

add({ category: "Autonomous nominal", inputCommand: "L1 cmd 0 deg/s", guidanceMode: "NOMINAL_CLOSED_LOOP", note: "zero yaw command" }, simulateControl({ policyName: "NOMINAL_CLOSED_LOOP", rawCmdDegS: 0, measuredDegS: 0 }));
add({ category: "Autonomous nominal", inputCommand: "L1 cmd +2 deg/s", guidanceMode: "NOMINAL_FEEDFORWARD", gyrzQuality: "STALE", gyroStatus: "none", note: "below 3 deg/s feedforward deadband" }, simulateControl({ policyName: "NOMINAL_FEEDFORWARD", rawCmdDegS: 2, measuredDegS: null, gyroStatus: "none" }));
add({ category: "Autonomous nominal", inputCommand: "L1 cmd +20 deg/s", guidanceMode: "NOMINAL_FEEDFORWARD", gyrzQuality: "STALE", gyroStatus: "none", note: "right turn, first cycle from neutral is slew-limited" }, simulateControl({ policyName: "NOMINAL_FEEDFORWARD", rawCmdDegS: 20, measuredDegS: null, gyroStatus: "none" }));
add({ category: "Autonomous nominal", inputCommand: "L1 cmd -20 deg/s", guidanceMode: "NOMINAL_FEEDFORWARD", gyrzQuality: "STALE", gyroStatus: "none", note: "left turn, first cycle from neutral is slew-limited" }, simulateControl({ policyName: "NOMINAL_FEEDFORWARD", rawCmdDegS: -20, measuredDegS: null, gyroStatus: "none" }));
add({ category: "Autonomous nominal", inputCommand: "L1 cmd +20, gyro +5", guidanceMode: "NOMINAL_CLOSED_LOOP", note: "gyro slower than command; PID adds right-turn trim" }, simulateControl({ policyName: "NOMINAL_CLOSED_LOOP", rawCmdDegS: 20, measuredDegS: 5 }));
add({ category: "Autonomous nominal", inputCommand: "L1 cmd +20, gyro +30", guidanceMode: "NOMINAL_CLOSED_LOOP", note: "gyro faster than command; PID subtracts right-turn trim" }, simulateControl({ policyName: "NOMINAL_CLOSED_LOOP", rawCmdDegS: 20, measuredDegS: 30 }));
add({ category: "Autonomous nominal", inputCommand: "L1 cmd +999 deg/s", guidanceMode: "NOMINAL_CLOSED_LOOP", note: "command is clamped to policy max" }, simulateControl({ policyName: "NOMINAL_CLOSED_LOOP", rawCmdDegS: 999, measuredDegS: 0 }));
add({ category: "Autonomous nominal", inputCommand: "L1 cmd -999 deg/s", guidanceMode: "NOMINAL_CLOSED_LOOP", note: "negative command is clamped to policy max" }, simulateControl({ policyName: "NOMINAL_CLOSED_LOOP", rawCmdDegS: -999, measuredDegS: 0 }));

add({ category: "Controller fallback", inputCommand: "guidance age 0.6s, cmd +20", guidanceMode: "NOMINAL_FEEDFORWARD", gyrzQuality: "STALE", gyroStatus: "none", guidanceAgeS: 0.6, note: "stale but not failed: command is halved" }, simulateControl({ policyName: "NOMINAL_FEEDFORWARD", rawCmdDegS: 20, measuredDegS: null, gyroStatus: "none", guidanceAgeS: 0.6 }));
add({ category: "Controller fallback", inputCommand: "guidance age 1.6s, cmd +20", guidanceMode: "NOMINAL_FEEDFORWARD", gyrzQuality: "STALE", gyroStatus: "none", guidanceAgeS: 1.6, note: "guidance timeout: neutral output" }, simulateControl({ policyName: "NOMINAL_FEEDFORWARD", rawCmdDegS: 20, measuredDegS: null, gyroStatus: "none", guidanceAgeS: 1.6 }));
add({ category: "Controller fallback", inputCommand: "gyro spike +300 deg/s", guidanceMode: "NOMINAL_CLOSED_LOOP", gyroStatus: "spike", note: "gyro sample rejected, feedforward only" }, simulateControl({ policyName: "NOMINAL_CLOSED_LOOP", rawCmdDegS: 20, measuredDegS: 300, gyroStatus: "spike" }));

add({ category: "Degraded", inputCommand: "history/DR position + gyro, cmd +15", posQuality: "FRESHED", motionQuality: "FRESHED", guidanceMode: "DEGRADED_CLOSED_LOOP", note: "degraded policy has lower authority and slower slew" }, simulateControl({ policyName: "DEGRADED_CLOSED_LOOP", rawCmdDegS: 15, measuredDegS: 5 }));
add({ category: "Degraded", inputCommand: "history/DR position, no gyro, cmd -15", posQuality: "FRESHED", motionQuality: "FRESHED", gyrzQuality: "STALE", guidanceMode: "DEGRADED_FEEDFORWARD", gyroStatus: "none", note: "degraded feedforward only" }, simulateControl({ policyName: "DEGRADED_FEEDFORWARD", rawCmdDegS: -15, measuredDegS: null, gyroStatus: "none" }));

add({ category: "Guidance fail fallback", inputCommand: "FREEFALL", freefall: 1, guidanceMode: "FAIL", failReason: "FREEFALL", controlValid: "N", note: "freefall disables control" }, neutralResult("NEUTRAL", "FREEFALL"));
add({ category: "Guidance fail fallback", inputCommand: "TUMBLE yaw dominant +120 deg/s", tumble: 1, guidanceMode: "FAIL", failReason: "TUMBLE_YAW_DOMINANT", note: "counter-yaw command is -0.5*gyrz, clamped to -35 deg/s" }, simulateControl({ policyName: "TUMBLE_YAW_DOMINANT", rawCmdDegS: -35, measuredDegS: null, gyroStatus: "none" }));
add({ category: "Guidance fail fallback", inputCommand: "TUMBLE yaw dominant -120 deg/s", tumble: 1, guidanceMode: "FAIL", failReason: "TUMBLE_YAW_DOMINANT", note: "counter-yaw command is +35 deg/s" }, simulateControl({ policyName: "TUMBLE_YAW_DOMINANT", rawCmdDegS: 35, measuredDegS: null, gyroStatus: "none" }));
add({ category: "Guidance fail fallback", inputCommand: "TUMBLE roll/pitch", tumble: 1, guidanceMode: "FAIL", failReason: "TUMBLE_ROLLPITCH", controlValid: "N", note: "roll/pitch tumble has no motor recovery command" }, neutralResult("NEUTRAL", "TUMBLE_ROLLPITCH"));
add({ category: "Guidance fail fallback", inputCommand: "UNSTABLE_BODY", tumble: 1, guidanceMode: "FAIL", failReason: "UNSTABLE_BODY", controlValid: "N", note: "tumble flag without yaw-dominant recovery goes neutral" }, neutralResult("NEUTRAL", "UNSTABLE_BODY"));
add({ category: "Guidance fail fallback", inputCommand: "NO_MOTION, target bearing +45 deg", motionQuality: "STALE", guidanceMode: "FAIL", failReason: "NO_MOTION", note: "target bearing fallback turns right without GPS course/speed" }, simulateControl({ policyName: "NO_MOTION_TARGET_BEARING", rawCmdDegS: 25, measuredDegS: null, gyroStatus: "none" }));
add({ category: "Guidance fail fallback", inputCommand: "NO_MOTION, target bearing -45 deg", motionQuality: "STALE", guidanceMode: "FAIL", failReason: "NO_MOTION", note: "target bearing fallback turns left without GPS course/speed" }, simulateControl({ policyName: "NO_MOTION_TARGET_BEARING", rawCmdDegS: -25, measuredDegS: null, gyroStatus: "none" }));
add({ category: "Guidance fail fallback", inputCommand: "NO_POSITION", posQuality: "STALE", guidanceMode: "FAIL", failReason: "NO_POSITION", controlValid: "N", note: "no position means no controllable path reference" }, neutralResult("NEUTRAL", "NO_POSITION"));
add({ category: "Guidance fail fallback", inputCommand: "DR_TIMEOUT", posQuality: "STALE", guidanceMode: "FAIL", failReason: "DR_TIMEOUT", controlValid: "N", note: "dead-reckoning position aged out" }, neutralResult("NEUTRAL", "DR_TIMEOUT"));
add({ category: "Guidance fail fallback", inputCommand: "SENSOR_BLACKOUT", posQuality: "STALE", motionQuality: "STALE", gyrzQuality: "STALE", altQuality: "STALE", guidanceMode: "FAIL", failReason: "SENSOR_BLACKOUT", controlValid: "N", note: "all major sensors stale" }, neutralResult("NEUTRAL", "SENSOR_BLACKOUT"));
add({ category: "Guidance fail fallback", inputCommand: "missing origin or target", originTarget: "missing origin or target", guidanceMode: "FAIL", failReason: "NO_POSITION", controlValid: "N", note: "L1 cannot form NE path/carrot" }, neutralResult("NEUTRAL", "NO_POSITION"));

const caseHeaders = [
  "case_id", "category", "input_command_or_condition", "STATE", "MOTOR_ENABLED", "MANUAL_STEER_MODE",
  "origin_target_status", "pos_quality", "motion_quality", "gyrz_quality", "alt_quality", "freefall",
  "tumble", "guidance_mode", "fail_reason", "control_valid", "control_policy", "pid_enabled",
  "guidance_age_s", "gyro_status", "raw_cmd_deg_s", "used_cmd_deg_s", "controller_mode",
  "fallback_mode", "gyro_meas_deg_s", "yaw_error_deg_s", "delta_ff_deg", "delta_pid_deg",
  "desired_delta_arm_deg", "actual_delta_arm_deg", "slew_limited", "left_angle_deg",
  "right_angle_deg", "motor_movement", "left_pwm_us", "right_pwm_us", "saturated", "sensor_valid",
  "notes",
];

const angleRows = [];
for (let delta = -160; delta <= 160; delta += 10) {
  const r = connectRoMo(delta);
  angleRows.push([
    delta,
    r.leftAngle,
    r.rightAngle,
    movement(r.leftAngle, r.rightAngle),
    r.leftPw,
    r.rightPw,
  ]);
}

const policyRows = Object.entries(policies).map(([name, p]) => [
  name,
  p.confidence,
  p.cmdMax,
  p.latAccMax,
  p.deltaFfMax,
  p.deltaPidMax,
  p.deltaTotalMax,
  p.maxArmRate,
  p.pidEnabled ? "Y" : "N",
]);

const summaryRows = [
  ["Workbook purpose", "One-cycle motor control case matrix for motorapp-guidance-control."],
  ["Angle convention", "0 deg = arm up, 160 deg = arm down, neutral = 80 deg."],
  ["Positive delta", "Right turn: right arm angle increases, left arm angle decreases."],
  ["Negative delta", "Left turn: left arm angle increases, right arm angle decreases."],
  ["Cycle_Cases", "Representative exhaustive branch cases: lifecycle, manual, nominal, degraded, fail fallback, timeout."],
  ["Angle_Map", "All possible differential arm motion in 10 deg increments from -160 to +160."],
  ["Slew note", "Autonomous rows show one 20 Hz cycle starting from neutral with code-equivalent slew-rate limits."],
];

const constantsRows = [
  ["ARM_MIN_DEG", C.armMin],
  ["ARM_MAX_DEG", C.armMax],
  ["NEUTRAL_ARM_DEG", C.neutral],
  ["DELTA_ARM_MAX_DEG", C.deltaArmMax],
  ["LEFT_ZERO_PWM_US", C.leftZero],
  ["RIGHT_ZERO_PWM_US", C.rightZero],
  ["PULSE_PER_DEG", C.pulsePerDeg],
  ["LEFT_NEUTRAL_PWM_US", connectRoMo(0).leftPw],
  ["RIGHT_NEUTRAL_PWM_US", connectRoMo(0).rightPw],
  ["GUIDANCE_TIMEOUT_ATTENUATE_S", C.guidanceAttenuateS],
  ["GUIDANCE_TIMEOUT_FAIL_S", C.guidanceFailS],
  ["GYRO_SPIKE_LIMIT_DEG_S", C.gyroSpikeDegS],
  ["ANGULAR_VELOCITY_DEADBAND_DEG_S", C.deadbandDegS],
  ["DELTA_MIN_EFFECTIVE_DEG", C.deltaMinEffective],
  ["EXPO", C.expo],
];

function writeTable(sheet, startCell, headers, data, tableName) {
  const startRow = Number(startCell.match(/\d+/)[0]);
  const startColLetters = startCell.match(/[A-Z]+/)[0];
  const colToNum = (letters) => [...letters].reduce((n, ch) => n * 26 + ch.charCodeAt(0) - 64, 0);
  const numToCol = (num) => {
    let s = "";
    while (num > 0) {
      const m = (num - 1) % 26;
      s = String.fromCharCode(65 + m) + s;
      num = Math.floor((num - m) / 26);
    }
    return s;
  };
  const startCol = colToNum(startColLetters);
  const endCol = numToCol(startCol + headers.length - 1);
  const endRow = startRow + data.length;
  const range = `${startCell}:${endCol}${endRow}`;
  sheet.getRange(range).values = [headers, ...data];
  const headerRange = `${startCell}:${endCol}${startRow}`;
  sheet.getRange(headerRange).format = {
    fill: "#1F4E79",
    font: { bold: true, color: "#FFFFFF" },
    wrapText: true,
  };
  sheet.getRange(range).format = { wrapText: true };
  const table = sheet.tables.add(range, true, tableName);
  table.style = "TableStyleMedium2";
  table.showFilterButton = true;
  return range;
}

function setColumnWidths(sheet, widths) {
  widths.forEach((px, idx) => {
    sheet.getRangeByIndexes(0, idx, 1, 1).format.columnWidthPx = px;
  });
}

await fs.mkdir(outputDir, { recursive: true });
const workbook = Workbook.create();
const summary = workbook.worksheets.add("Summary");
const cases = workbook.worksheets.add("Cycle_Cases");
const angles = workbook.worksheets.add("Angle_Map");
const policySheet = workbook.worksheets.add("Policies");
const constants = workbook.worksheets.add("Constants");

for (const sheet of [summary, cases, angles, policySheet, constants]) {
  sheet.showGridLines = false;
}

summary.getRange("A1:B1").values = [["Motor Control Case Workbook", ""]];
summary.getRange("A1:B1").merge();
summary.getRange("A1:B1").format = {
  fill: "#17365D",
  font: { bold: true, color: "#FFFFFF", size: 16 },
};
writeTable(summary, "A3", ["Item", "Description"], summaryRows, "SummaryTable");
setColumnWidths(summary, [220, 760]);

writeTable(cases, "A1", caseHeaders, rows, "CycleCasesTable");
cases.freezePanes.freezeRows(1);
setColumnWidths(cases, [
  60, 150, 240, 80, 110, 140, 180, 110, 120, 110, 100, 80, 80, 160, 160,
  100, 160, 100, 110, 120, 110, 110, 150, 150, 120, 120, 110, 110, 140,
  140, 100, 120, 120, 280, 100, 100, 90, 100, 360,
]);
cases.getRange("U:AF").format = { numberFormat: "0.000" };

writeTable(angles, "A1", ["delta_arm_deg", "left_angle_deg", "right_angle_deg", "motor_movement", "left_pwm_us", "right_pwm_us"], angleRows, "AngleMapTable");
angles.freezePanes.freezeRows(1);
setColumnWidths(angles, [120, 130, 130, 300, 110, 110]);

writeTable(policySheet, "A1", [
  "policy", "confidence_scale", "angular_velocity_cmd_max_deg_s", "lat_acc_max_mps2",
  "delta_ff_max_deg", "delta_pid_max_deg", "delta_total_max_deg", "max_arm_rate_deg_s", "pid_enabled",
], policyRows, "PoliciesTable");
policySheet.freezePanes.freezeRows(1);
setColumnWidths(policySheet, [230, 150, 240, 150, 150, 150, 160, 170, 120]);

writeTable(constants, "A1", ["constant", "value"], constantsRows, "ConstantsTable");
constants.freezePanes.freezeRows(1);
setColumnWidths(constants, [280, 160]);

const errorScan = await workbook.inspect({
  kind: "match",
  searchTerm: "#REF!|#DIV/0!|#VALUE!|#NAME\\?|#N/A",
  options: { useRegex: true, maxResults: 300 },
  summary: "final formula error scan",
});
console.log(errorScan.ndjson);

for (const sheetName of ["Summary", "Cycle_Cases", "Angle_Map", "Policies", "Constants"]) {
  const preview = await workbook.render({ sheetName, autoCrop: "all", scale: 1, format: "png" });
  await fs.writeFile(`${outputDir}/${sheetName}.png`, new Uint8Array(await preview.arrayBuffer()));
}

const xlsx = await SpreadsheetFile.exportXlsx(workbook);
await xlsx.save(outputPath);
console.log(`saved ${outputPath}`);
