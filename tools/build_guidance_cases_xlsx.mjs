import fs from "node:fs/promises";
import path from "node:path";
import { SpreadsheetFile, Workbook } from "@oai/artifact-tool";

const repoRoot = "C:/workspace/CANSAT_AAS_2026_FSW";
const outputDir = path.join(repoRoot, "outputs");
const outputPath = path.join(outputDir, "gps_dr_guidance_cases.xlsx");

const rows = [];

function modeFor(G, P, M, Y, A) {
  if (P && M) return G ? "GPS_TRACKING_CLOSED" : "GPS_TRACKING_OPEN";
  if (P && !M && Y && !A) return G ? "DR_TRACKING_CLOSED" : "DR_TRACKING_OPEN";
  if (A && G) return "DR_TRACKING_CLOSED";
  if (A && !G && Y) return "DR_TRACKING_OPEN";
  return "FAIL";
}

function controlType(mode) {
  if (mode.endsWith("_CLOSED")) return "Closed";
  if (mode.endsWith("_OPEN")) return "Open";
  return "None";
}

function posSource(mode, P, M, Y, A) {
  if (mode.startsWith("GPS")) return "GPS lat/lon -> local NE";
  if (P && !M && Y && !A) return "GPS position으로 pos-only DR anchor 생성 후 적분";
  if (mode.startsWith("DR")) return "DR 적분: E/N += V_dr*[sin(course), cos(course)]*dt";
  if (P) return "GPS position은 있으나 course/speed 부족";
  return "없음";
}

function courseSource(mode, G, P, M, Y, A) {
  if (mode.startsWith("GPS")) return "GPS course";
  if (P && !M && Y && !A) return "IMU yaw로 anchor_course 생성";
  if (mode === "DR_TRACKING_CLOSED") {
    if (Y) return "anchor_course + ∫gyrz dt, yaw delta와 비교/블렌드";
    return "anchor_course + ∫gyrz dt";
  }
  if (mode === "DR_TRACKING_OPEN") return "anchor_course + (yaw - yaw_at_anchor)";
  if (G && !A) return "gyrz만으로는 절대 heading 초기화 불가";
  if (!Y && A) return "anchor는 있으나 yaw/gyrz 없어 course 갱신 불가";
  return "없음";
}

function speedSource(mode, P, M, Y, A) {
  if (mode.startsWith("GPS")) return "GPS speed";
  if (P && !M && Y && !A) return "baro sink_rate 또는 V_MIN_MPS";
  if (mode.startsWith("DR")) return "baro sink_rate 유효 시 사용, 아니면 anchor_V";
  if (M) return "GPS speed는 있으나 position/anchor 부족으로 guidance 불가";
  return "없음";
}

function reason(mode, G, P, M, Y, A) {
  if (mode.startsWith("GPS")) {
    return "position/course/speed를 GPS에서 직접 확보";
  }
  if (P && !M && Y && !A) {
    return "motion이 없어 GPS position + IMU yaw로 pos-only DR anchor 생성";
  }
  if (mode === "DR_TRACKING_CLOSED") {
    return "valid DR anchor가 있고 gyro_z로 course 변화량 추정 가능";
  }
  if (mode === "DR_TRACKING_OPEN") {
    return "valid DR anchor가 있고 yaw delta로 course 추정 가능";
  }
  if (P && !M && !Y && !A) {
    return "position만 있고 course/speed 및 anchor 초기화용 yaw 없음";
  }
  if (!P && !A) {
    return "현재 position도 DR anchor도 없어 target bearing/거리 계산 불가";
  }
  if (A && !G && !Y) {
    return "DR anchor는 있으나 course 갱신 센서 없음";
  }
  return "position/course/speed 중 하나 이상 생성 불가";
}

let idx = 1;
for (const G of [1, 0]) {
  for (const P of [1, 0]) {
    for (const M of [1, 0]) {
      for (const Y of [1, 0]) {
        for (const A of [1, 0]) {
          const mode = modeFor(G, P, M, Y, A);
          rows.push([
            idx++, G, P, M, Y, A,
            mode,
            controlType(mode),
            posSource(mode, P, M, Y, A),
            courseSource(mode, G, P, M, Y, A),
            speedSource(mode, P, M, Y, A),
            reason(mode, G, P, M, Y, A),
          ]);
        }
      }
    }
  }
}

const workbook = Workbook.create();
const cases = workbook.worksheets.add("Cases_32");
const summary = workbook.worksheets.add("Summary");
const drRules = workbook.worksheets.add("DR_Rules");
const legend = workbook.worksheets.add("Legend");

const headers = [
  "#", "G gyro_z", "P GPS position", "M GPS motion", "Y IMU yaw", "A DR anchor",
  "Mode", "Control", "Position 생성", "Course 생성", "Speed 생성", "판단 근거",
];

cases.getRange("A1:L1").values = [headers];
cases.getRange(`A2:L${rows.length + 1}`).values = rows;
cases.tables.add(`A1:L${rows.length + 1}`, true, "GuidanceCases");
cases.freezePanes.freezeRows(1);
cases.showGridLines = false;
cases.getRange("A1:L1").format = {
  fill: "#1F4E79",
  font: { bold: true, color: "#FFFFFF" },
};
cases.getRange(`A1:L${rows.length + 1}`).format.borders = {
  preset: "all",
  style: "thin",
  color: "#D9E2F3",
};
cases.getRange("A:A").format.columnWidthPx = 44;
cases.getRange("B:F").format.columnWidthPx = 86;
cases.getRange("G:G").format.columnWidthPx = 178;
cases.getRange("H:H").format.columnWidthPx = 82;
cases.getRange("I:K").format.columnWidthPx = 275;
cases.getRange("L:L").format.columnWidthPx = 330;
cases.getRange(`I2:L${rows.length + 1}`).format.wrapText = true;
cases.getRange(`A2:H${rows.length + 1}`).format.horizontalAlignment = "center";

for (let r = 2; r <= rows.length + 1; r++) {
  const mode = rows[r - 2][6];
  const range = cases.getRange(`G${r}:H${r}`);
  if (mode.startsWith("GPS")) {
    range.format.fill = "#E2F0D9";
  } else if (mode.startsWith("DR")) {
    range.format.fill = "#FFF2CC";
  } else {
    range.format.fill = "#FCE4D6";
  }
}

const modeCounts = new Map();
for (const row of rows) modeCounts.set(row[6], (modeCounts.get(row[6]) ?? 0) + 1);
const summaryRows = [
  ["Mode", "Count", "의미"],
  ["GPS_TRACKING_CLOSED", modeCounts.get("GPS_TRACKING_CLOSED") ?? 0, "GPS position/course/speed + gyro_z feedback"],
  ["GPS_TRACKING_OPEN", modeCounts.get("GPS_TRACKING_OPEN") ?? 0, "GPS position/course/speed, gyro_z feedback 없음"],
  ["DR_TRACKING_CLOSED", modeCounts.get("DR_TRACKING_CLOSED") ?? 0, "DR anchor 기반 position/course/speed 추정 + gyro_z feedback"],
  ["DR_TRACKING_OPEN", modeCounts.get("DR_TRACKING_OPEN") ?? 0, "DR anchor/yaw 기반 추정, gyro_z feedback 없음"],
  ["FAIL", modeCounts.get("FAIL") ?? 0, "position/course/speed 중 하나 이상 생성 불가"],
];
summary.getRange("A1:C6").values = summaryRows;
summary.tables.add("A1:C6", true, "ModeSummary");
summary.getRange("A1:C1").format = {
  fill: "#1F4E79",
  font: { bold: true, color: "#FFFFFF" },
};
summary.getRange("A1:C6").format.borders = { preset: "all", style: "thin", color: "#D9E2F3" };
summary.getRange("A:A").format.columnWidthPx = 190;
summary.getRange("B:B").format.columnWidthPx = 80;
summary.getRange("C:C").format.columnWidthPx = 430;
summary.getRange("C:C").format.wrapText = true;
summary.showGridLines = false;

drRules.getRange("A1:D1").values = [["Nav 값", "정상 GPS", "DR 생성 방식", "사용 코드/요소"]];
drRules.getRange("A2:D5").values = [
  ["position E/N [m]", "GPS lat/lon -> local NE", "E/N += V_dr*[sin(course), cos(course)]*dt", "_update_state_from_dead_reckoning: nav.E/N 갱신, anchor_E/N"],
  ["course [rad]", "GPS course", "anchor_course + ∫gyrz dt 또는 anchor_course + yaw delta", "dr_estimate_course: gyrz, imu.yaw, yaw_at_anchor"],
  ["speed V [m/s]", "GPS speed", "baro sink_rate 유효 시 사용, 아니면 anchor_V", "_update_state_from_dead_reckoning: baro.sink_rate, anchor_V"],
  ["pos-only anchor", "GPS position만 fresh", "IMU yaw로 course, baro/V_MIN으로 speed 초기화", "TryInitStateFromPosOnly: gps.E/N, imu.yaw, baro.sink_rate"],
];
drRules.tables.add("A1:D5", true, "DRRules");
drRules.getRange("A1:D1").format = {
  fill: "#1F4E79",
  font: { bold: true, color: "#FFFFFF" },
};
drRules.getRange("A1:D5").format.borders = { preset: "all", style: "thin", color: "#D9E2F3" };
drRules.getRange("A:A").format.columnWidthPx = 155;
drRules.getRange("B:D").format.columnWidthPx = 300;
drRules.getRange("A1:D5").format.wrapText = true;
drRules.showGridLines = false;

legend.getRange("A1:C1").values = [["기호", "의미", "코드 기준"]];
legend.getRange("A2:C7").values = [
  ["G", "gyro_z fresh", "IMU fresh && gyrz_valid"],
  ["P", "GPS position fresh", "gps.pos_valid && pos_ts age <= GPS_FRESH_MAX_AGE_S"],
  ["M", "GPS motion fresh", "gps.motion_valid && motion_ts age <= GPS_FRESH_MAX_AGE_S; course + speed 묶음"],
  ["Y", "IMU yaw fresh", "IMU fresh && yaw_valid"],
  ["A", "DR anchor valid", "anchor_E/N, anchor_V, anchor_course, anchor_time finite"],
  ["전제", "origin/target ready, motor enabled, STATE>=3", "manual steer/detumbling 우선순위는 이 표에서 제외"],
];
legend.tables.add("A1:C7", true, "LegendTable");
legend.getRange("A1:C1").format = {
  fill: "#1F4E79",
  font: { bold: true, color: "#FFFFFF" },
};
legend.getRange("A1:C7").format.borders = { preset: "all", style: "thin", color: "#D9E2F3" };
legend.getRange("A:A").format.columnWidthPx = 80;
legend.getRange("B:B").format.columnWidthPx = 260;
legend.getRange("C:C").format.columnWidthPx = 480;
legend.getRange("B:C").format.wrapText = true;
legend.showGridLines = false;

const check = await workbook.inspect({
  kind: "table",
  range: "Cases_32!A1:L10",
  include: "values",
  tableMaxRows: 10,
  tableMaxCols: 12,
  maxChars: 5000,
});
console.log(check.ndjson);

const errors = await workbook.inspect({
  kind: "match",
  searchTerm: "#REF!|#DIV/0!|#VALUE!|#NAME\\?|#N/A",
  options: { useRegex: true, maxResults: 50 },
  summary: "formula error scan",
});
console.log(errors.ndjson);

const preview = await workbook.render({
  sheetName: "Cases_32",
  range: "A1:L18",
  scale: 1,
  format: "png",
});
await fs.mkdir(outputDir, { recursive: true });
await fs.writeFile(path.join(outputDir, "gps_dr_guidance_cases_preview.png"), new Uint8Array(await preview.arrayBuffer()));

const xlsx = await SpreadsheetFile.exportXlsx(workbook);
await xlsx.save(outputPath);
console.log(`saved=${outputPath}`);
