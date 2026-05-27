#include "ui/web_server.hpp"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "common/error_code.hpp"
#include "controller/gripper_types.hpp"

#if defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#endif

namespace gripper::ui {
namespace {

constexpr std::uint16_t kFallbackPortCount = 32;

std::mutex g_long_action_mutex;
bool g_self_check_running = false;

[[nodiscard]] bool isSelfCheckRunning() {
  std::lock_guard<std::mutex> lock{g_long_action_mutex};
  return g_self_check_running;
}

[[nodiscard]] std::string loadPrototypeHtml() {
  std::vector<std::filesystem::path> candidates = {
      "src/ui/prototype/admin_recovery_ui_preview.html",
      "../src/ui/prototype/admin_recovery_ui_preview.html",
      "../../src/ui/prototype/admin_recovery_ui_preview.html"};
#if defined(_WIN32)
  char module_path[MAX_PATH]{};
  const DWORD length = GetModuleFileNameA(nullptr, module_path, MAX_PATH);
  if (length > 0U && length < MAX_PATH) {
    const auto exe_dir = std::filesystem::path{module_path}.parent_path();
    candidates.push_back(
        exe_dir / "../../src/ui/prototype/admin_recovery_ui_preview.html");
    candidates.push_back(
        exe_dir / "../src/ui/prototype/admin_recovery_ui_preview.html");
  }
#endif
  for (const auto& path : candidates) {
    std::ifstream file(path, std::ios::binary);
    if (!file.is_open()) {
      continue;
    }
    std::ostringstream content;
    content << file.rdbuf();
    return content.str();
  }
  return {};
}

[[nodiscard]] std::string htmlPage() {
  const std::string prototype_html = loadPrototypeHtml();
  if (!prototype_html.empty()) {
    return prototype_html;
  }
  return R"HTML(<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>夹爪控制测试台</title>
  <style>
    :root {
      color-scheme: light;
      --bg: #f5f7f8;
      --panel: #ffffff;
      --line: #d9e0e3;
      --text: #172126;
      --muted: #64737b;
      --blue: #246b91;
      --green: #28714d;
      --red: #aa3a32;
      --amber: #9a6a1d;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      font: 14px/1.45 "Segoe UI", "Microsoft YaHei", Arial, sans-serif;
      background: var(--bg);
      color: var(--text);
      overflow: hidden;
    }
    .app {
      display: grid;
      grid-template-columns: 292px minmax(0, 1fr);
      height: 100vh;
      min-height: 0;
    }
    aside {
      border-right: 1px solid var(--line);
      background: #eef3f5;
      overflow-y: auto;
      padding: 14px;
      min-height: 0;
    }
    main {
      overflow: auto;
      min-width: 0;
      padding: 16px;
    }
    h1 {
      margin: 0 0 12px;
      font-size: 18px;
      font-weight: 650;
    }
    h2 {
      margin: 18px 0 8px;
      font-size: 13px;
      text-transform: uppercase;
      color: var(--muted);
      letter-spacing: 0;
    }
    .group {
      display: grid;
      gap: 8px;
      margin-bottom: 12px;
    }
    button {
      min-height: 34px;
      border: 1px solid #bac7cc;
      background: #fff;
      border-radius: 6px;
      color: var(--text);
      cursor: pointer;
      font: inherit;
      padding: 7px 10px;
      text-align: left;
    }
    button:hover { border-color: var(--blue); }
    button.primary { background: var(--blue); color: #fff; border-color: var(--blue); }
    button.safe { background: var(--green); color: #fff; border-color: var(--green); }
    button.warn { background: var(--amber); color: #fff; border-color: var(--amber); }
    button.danger { background: var(--red); color: #fff; border-color: var(--red); }
    button:disabled { opacity: .48; cursor: not-allowed; }
    label {
      display: grid;
      gap: 4px;
      color: var(--muted);
      font-size: 12px;
    }
    input[type="number"], input[type="text"] {
      width: 100%;
      min-height: 32px;
      border: 1px solid #c7d0d4;
      border-radius: 6px;
      padding: 6px 8px;
      font: inherit;
      background: #fff;
      color: var(--text);
    }
    .check {
      display: flex;
      align-items: center;
      gap: 8px;
      color: var(--text);
      font-size: 13px;
      margin: 6px 0 10px;
    }
    .statusbar {
      display: grid;
      grid-template-columns: repeat(6, minmax(120px, 1fr));
      gap: 10px;
      margin-bottom: 14px;
    }
    .metric, .panel {
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 8px;
    }
    .metric {
      padding: 10px 12px;
      min-width: 0;
    }
    .metric .name {
      color: var(--muted);
      font-size: 12px;
      white-space: nowrap;
      overflow: hidden;
      text-overflow: ellipsis;
    }
    .metric .value {
      margin-top: 4px;
      font-size: 18px;
      font-weight: 650;
      overflow-wrap: anywhere;
    }
    .grid {
      display: grid;
      grid-template-columns: repeat(2, minmax(280px, 1fr));
      gap: 14px;
    }
    .panel {
      padding: 12px;
      min-width: 0;
    }
    .panel h2 {
      margin-top: 0;
    }
    .kv {
      display: grid;
      grid-template-columns: 150px minmax(0, 1fr);
      gap: 7px 12px;
      align-items: baseline;
    }
    .kv span:nth-child(odd) { color: var(--muted); }
    .kv span:nth-child(even) { font-family: Consolas, monospace; overflow-wrap: anywhere; }
    .row {
      display: grid;
      grid-template-columns: repeat(4, minmax(0, 1fr));
      gap: 8px;
      align-items: end;
      margin-bottom: 8px;
    }
    .logbar {
      display: flex;
      gap: 12px;
      align-items: center;
      justify-content: space-between;
      margin-bottom: 8px;
    }
    .log {
      height: 320px;
      overflow: auto;
      white-space: pre-wrap;
      background: #11191d;
      color: #d6ecef;
      border-radius: 8px;
      padding: 10px;
      font: 12px/1.45 Consolas, "Cascadia Mono", monospace;
    }
    .pill {
      display: inline-flex;
      align-items: center;
      min-height: 24px;
      border-radius: 999px;
      padding: 2px 9px;
      background: #dfe8eb;
      color: #203038;
      font-size: 12px;
      font-weight: 600;
    }
    @media (max-width: 860px) {
      body { overflow: auto; }
      .app { grid-template-columns: 1fr; height: auto; min-height: 100vh; }
      aside { max-height: 58vh; border-right: 0; border-bottom: 1px solid var(--line); }
      .statusbar { grid-template-columns: repeat(2, minmax(120px, 1fr)); }
      .grid { grid-template-columns: 1fr; }
      .row { grid-template-columns: repeat(2, minmax(0, 1fr)); }
    }
  </style>
</head>
<body>
  <div class="app">
    <aside>
      <h1>夹爪控制测试台</h1>
      <span class="pill" id="connPill">Disconnected</span>

      <h2>连接</h2>
      <div class="group">
        <button class="primary" data-action="connect">连接硬件</button>
        <button data-action="disconnect">断开连接</button>
        <button class="danger" data-action="stop">主动停止</button>
        <button data-action="clear_fault">清除故障</button>
      </div>

      <h2>电机 Bring-up</h2>
      <label class="check"><input id="safeConfirm" type="checkbox">机构已安全/空载</label>
      <div class="group">
        <button data-action="enter_bringup">进入 Bring-up</button>
        <button data-action="can_probe">CAN 探测</button>
        <button data-action="feedback">读取反馈</button>
        <button class="safe" data-action="bringup_enable">Bring-up 使能</button>
        <button data-action="bringup_disable">Bring-up 失能</button>
      </div>
      <div class="row">
        <label>相对位置 rad<input id="jogRad" type="number" step="0.001" value="0.05"></label>
        <label>速度 rad/s<input id="jogVel" type="number" step="0.01" value="0.5"></label>
        <label>电流 A<input id="jogCur" type="number" step="0.01" value="0.2"></label>
        <label>时间 s<input id="jogSec" type="number" step="0.01" value="0.1"></label>
      </div>
      <div class="group">
        <button class="warn" data-action="jog_pos">正向短脉冲</button>
        <button class="warn" data-action="jog_neg">反向短脉冲</button>
      </div>
      <div class="row">
        <label>精确转动 rev<input id="turnRev" type="number" step="0.25" value="1.0"></label>
        <label>位置速度 rad/s<input id="turnVel" type="number" step="0.1" value="1.0"></label>
        <label>电流中止 A<input id="turnCur" type="number" step="0.1" value="1.5"></label>
        <label>超时 s<input id="turnTimeout" type="number" step="0.1" value="0.0"></label>
      </div>
      <div class="group">
        <button class="warn" data-action="turn_pos">正向精确转动</button>
        <button class="warn" data-action="turn_neg">反向精确转动</button>
        <button data-action="turn_custom">按输入圈数转动</button>
        <button data-action="turn_pos" data-turn-rev="2">正向 2 圈</button>
        <button data-action="turn_neg" data-turn-rev="2">反向 2 圈</button>
        <button data-action="exit_bringup">退出 Bring-up</button>
      </div>

      <h2>流程</h2>
      <div class="group">
        <button data-action="enable">常规使能</button>
        <button data-action="disable">常规失能</button>
        <button data-action="selfcheck">PreSelfCheck</button>
        <button data-action="home">回零</button>
        <button data-action="learn">行程学习</button>
        <button data-action="health">健康检查</button>
      </div>
      <div class="row">
        <label>目标力 N<input id="forceN" type="number" step="1" value="20"></label>
        <label>速度 mm/s<input id="speedMm" type="number" step="0.1" value="1.0"></label>
      </div>
      <div class="group">
        <button class="safe" data-action="clamp">目标力夹紧</button>
        <button data-action="release">释放</button>
      </div>

      <h2>服务</h2>
      <div class="group">
        <button data-action="refresh">刷新状态</button>
        <button class="danger" data-action="shutdown">关闭 Web UI</button>
      </div>
    </aside>

    <main>
      <div class="statusbar">
        <div class="metric"><div class="name">状态</div><div class="value" id="statusText">-</div></div>
        <div class="metric"><div class="name">连接</div><div class="value" id="connected">-</div></div>
        <div class="metric"><div class="name">使能</div><div class="value" id="enabled">-</div></div>
        <div class="metric"><div class="name">行程 mm</div><div class="value" id="stroke">-</div></div>
        <div class="metric"><div class="name">虚拟多圈电机位置 rad</div><div class="value" id="motorPos">-</div></div>
        <div class="metric"><div class="name">电流 A</div><div class="value" id="current">-</div></div>
      </div>

      <div class="grid">
        <section class="panel">
          <h2>实时反馈</h2>
          <div class="kv">
            <span>夹爪角度 rad</span><span id="angle">-</span>
            <span>速度 rad/s</span><span id="velocity">-</span>
            <span>力矩 Nm</span><span id="torque">-</span>
            <span>温度 °C</span><span id="temp">-</span>
            <span>估算夹紧力 N</span><span id="force">-</span>
            <span>故障码</span><span id="fault">-</span>
          </div>
        </section>
        <section class="panel">
          <h2>流程标志</h2>
          <div class="kv">
            <span>Bring-up</span><span id="bringup">-</span>
            <span>PreSelfCheck</span><span id="selfChecked">-</span>
            <span>Homed</span><span id="homed">-</span>
            <span>Limits</span><span id="limits">-</span>
            <span>Health</span><span id="health">-</span>
            <span>Contact</span><span id="contact">-</span>
          </div>
        </section>
      </div>

      <section class="panel" style="margin-top:14px;">
        <div class="logbar">
          <h2>运行日志</h2>
          <label class="check"><input id="pauseLog" type="checkbox">暂停滚动</label>
        </div>
        <div class="log" id="log"></div>
      </section>
    </main>
  </div>

  <script>
    const $ = (id) => document.getElementById(id);
    const fmt = (v, n = 3) => Number.isFinite(Number(v)) ? Number(v).toFixed(n) : "-";
    const yn = (v) => v ? "true" : "false";

    async function getJson(url) {
      const response = await fetch(url, { cache: "no-store" });
      return response.json();
    }

    function setText(id, value) {
      $(id).textContent = value;
    }

    function render(data) {
      const s = data.state || {};
      setText("statusText", data.status_text || "-");
      setText("connected", yn(s.connected));
      setText("enabled", yn(s.enabled));
      setText("stroke", fmt(s.stroke_mm));
      setText("motorPos", fmt(s.motor_virtual_pos_rad ?? s.motor_pos_rad));
      setText("current", fmt(s.motor_current_a, 4));
      setText("angle", fmt(s.gripper_angle_rad));
      setText("velocity", fmt(s.motor_vel_rad_s, 4));
      setText("torque", fmt(s.motor_torque_nm, 4));
      setText("temp", fmt(s.temperature_c, 1));
      setText("force", fmt(s.force_n, 2));
      setText("fault", `${s.fault_code ?? 0} / ${yn(s.motor_fault)}`);
      setText("bringup", yn(s.motor_bringup_active));
      setText("selfChecked", s.self_check_running ? "running" : yn(s.pre_self_check_completed));
      setText("homed", yn(s.homed));
      setText("limits", yn(s.travel_limits_learned));
      setText("health", yn(s.motion_health_checked));
      setText("contact", yn(s.contact_detected));

      const pill = $("connPill");
      pill.textContent = `${data.status_text || "-"} / ${s.connected ? "Connected" : "Disconnected"}`;
      pill.style.background = s.connected ? "#d7eadf" : "#dfe8eb";

      if (!$("pauseLog").checked) {
        const log = $("log");
        log.textContent = (data.logs || []).join("\n");
        log.scrollTop = log.scrollHeight;
      }
    }

    function queryFor(action) {
      const q = new URLSearchParams({ name: action });
      if (action === "enter_bringup") {
        q.set("confirmed", $("safeConfirm").checked ? "1" : "0");
      }
      if (action === "jog_pos" || action === "jog_neg") {
        q.set("rad", $("jogRad").value);
        q.set("vel", $("jogVel").value);
        q.set("cur", $("jogCur").value);
        q.set("sec", $("jogSec").value);
      }
      if (action === "turn_pos" || action === "turn_neg" || action === "turn_custom") {
        q.set("rev", $("turnRev")?.value || "1.0");
        q.set("vel", $("turnVel")?.value || "1.0");
        q.set("cur", $("turnCur")?.value || "1.5");
        q.set("timeout", $("turnTimeout")?.value || "0.0");
      }
      if (action === "clamp") {
        q.set("force", $("forceN").value);
        q.set("speed", $("speedMm").value);
      }
      return q.toString();
    }

    async function refresh() {
      try {
        render(await getJson("/api/view"));
      } catch (error) {
        $("log").textContent = String(error);
      }
    }

    async function runAction(action) {
      if (action === "refresh") {
        await refresh();
        return;
      }
      const data = await getJson(`/api/action?${queryFor(action)}`);
      render(data);
      if (action === "shutdown") {
        $("log").textContent += "\nWeb UI server is shutting down.";
      }
    }

    document.querySelectorAll("button[data-action]").forEach((button) => {
      button.addEventListener("click", () => {
        if (button.dataset.turnRev && $("turnRev")) {
          $("turnRev").value = button.dataset.turnRev;
        }
        runAction(button.dataset.action);
      });
    });

    refresh();
    setInterval(refresh, 800);
  </script>
</body>
</html>)HTML";
}

[[nodiscard]] std::string jsonEscape(std::string_view text) {
  std::ostringstream output;
  for (char ch : text) {
    switch (ch) {
      case '\\':
        output << "\\\\";
        break;
      case '"':
        output << "\\\"";
        break;
      case '\n':
        output << "\\n";
        break;
      case '\r':
        output << "\\r";
        break;
      case '\t':
        output << "\\t";
        break;
      default:
        if (static_cast<unsigned char>(ch) < 0x20U) {
          output << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                 << static_cast<int>(static_cast<unsigned char>(ch))
                 << std::dec << std::setfill(' ');
        } else {
          output << ch;
        }
        break;
    }
  }
  return output.str();
}

[[nodiscard]] std::string jsonString(std::string_view text) {
  return std::string{"\""} + jsonEscape(text) + "\"";
}

[[nodiscard]] std::string boolJson(bool value) {
  return value ? "true" : "false";
}

[[nodiscard]] std::string hexByte(std::uint8_t value) {
  std::ostringstream output;
  output << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
         << static_cast<int>(value);
  return output.str();
}

[[nodiscard]] std::string rawFrameBytes(
    const controller::MotorFeedbackSummary& motor) {
  if (!motor.raw_feedback_frame_valid) {
    return {};
  }
  std::ostringstream output;
  const std::uint8_t length =
      std::min<std::uint8_t>(motor.raw_feedback_frame_length, 64U);
  for (std::uint8_t i = 0; i < length; ++i) {
    if (i != 0U) {
      output << ' ';
    }
    output << hexByte(motor.raw_feedback_frame_data[i]);
  }
  return output.str();
}

[[nodiscard]] double parseDouble(const std::map<std::string, std::string>& query,
                                 const std::string& key, double fallback) {
  const auto iter = query.find(key);
  if (iter == query.end()) {
    return fallback;
  }
  try {
    return std::stod(iter->second);
  } catch (...) {
    return fallback;
  }
}

[[nodiscard]] bool parseBool(const std::map<std::string, std::string>& query,
                             const std::string& key) {
  const auto iter = query.find(key);
  if (iter == query.end()) {
    return false;
  }
  return iter->second == "1" || iter->second == "true" ||
         iter->second == "True";
}

[[nodiscard]] std::string urlDecode(std::string_view value) {
  std::string output;
  output.reserve(value.size());
  for (std::size_t i = 0; i < value.size(); ++i) {
    if (value[i] == '+') {
      output.push_back(' ');
    } else if (value[i] == '%' && i + 2 < value.size() &&
               std::isxdigit(static_cast<unsigned char>(value[i + 1])) &&
               std::isxdigit(static_cast<unsigned char>(value[i + 2]))) {
      const std::string hex{value.substr(i + 1, 2)};
      output.push_back(
          static_cast<char>(std::strtol(hex.c_str(), nullptr, 16)));
      i += 2;
    } else {
      output.push_back(value[i]);
    }
  }
  return output;
}

[[nodiscard]] std::string profileValidityName(
    controller::StructureProfileValidity validity) {
  using controller::StructureProfileValidity;
  switch (validity) {
    case StructureProfileValidity::Unknown:
      return "Unknown";
    case StructureProfileValidity::ConservativeDefaults:
      return "ConservativeDefaults";
    case StructureProfileValidity::PreSelfCheckCompleted:
      return "PreSelfCheckCompleted";
    case StructureProfileValidity::Homed:
      return "Homed";
    case StructureProfileValidity::TravelLimitsLearned:
      return "TravelLimitsLearned";
    case StructureProfileValidity::MotionHealthChecked:
      return "MotionHealthChecked";
  }
  return "Unknown";
}

[[nodiscard]] std::string frictionSummary(
    const controller::StructureLearnedParametersSummary& learned,
    bool dynamic) {
  const std::uint32_t opening_count =
      dynamic ? learned.opening.dynamic_friction_sample_count
              : learned.opening.static_friction_sample_count;
  const std::uint32_t closing_count =
      dynamic ? learned.closing.dynamic_friction_sample_count
              : learned.closing.static_friction_sample_count;
  if (opening_count == 0U || closing_count == 0U) {
    return "未学习";
  }
  std::ostringstream output;
  output << std::fixed << std::setprecision(3);
  if (dynamic) {
    output << "开 "
           << learned.opening.dynamic_friction_current_average.value
           << " A / 闭 "
           << learned.closing.dynamic_friction_current_average.value << " A";
  } else {
    output << "开 " << learned.opening.static_friction_current.value
           << " A / 闭 " << learned.closing.static_friction_current.value
           << " A";
  }
  return output.str();
}

[[nodiscard]] std::string safeZoneSummary(
    const controller::StructureLearnedParametersSummary& learned) {
  std::ostringstream output;
  output << std::fixed << std::setprecision(2)
         << learned.safe_zone_open_limit.value << " ~ "
         << learned.safe_zone_closed_limit.value << " mm";
  return output.str();
}

[[nodiscard]] std::string numberText(double value) {
  std::ostringstream output;
  output << std::setprecision(10) << value;
  return output.str();
}

[[nodiscard]] std::string boolText(bool value) {
  return value ? "true" : "false";
}

[[nodiscard]] std::string hexU32(std::uint32_t value) {
  std::ostringstream output;
  output << "0x" << std::uppercase << std::hex << value;
  return output.str();
}

[[nodiscard]] std::string adapterTypeName(config::AdapterType type) {
  switch (type) {
    case config::AdapterType::Simulated:
      return "simulated";
    case config::AdapterType::DmUsb2FdcanDual:
      return "dm_usb2fdcan_dual";
  }
  return "unknown";
}

[[nodiscard]] std::string motorControlModeName(
    config::MotorControlMode mode) {
  switch (mode) {
    case config::MotorControlMode::Unknown:
      return "unknown";
    case config::MotorControlMode::PositionForce:
      return "position_force";
  }
  return "unknown";
}

[[nodiscard]] std::string encoderUnwrapSourceName(
    config::EncoderUnwrapSource source) {
  switch (source) {
    case config::EncoderUnwrapSource::ProtocolPosition:
      return "protocol_position";
    case config::EncoderUnwrapSource::RawPositionCounts:
      return "raw_position_counts";
  }
  return "unknown";
}

[[nodiscard]] std::string motionDirectionName(config::MotionDirection dir) {
  switch (dir) {
    case config::MotionDirection::Open:
      return "open";
    case config::MotionDirection::Close:
      return "close";
  }
  return "unknown";
}

[[nodiscard]] std::string mmPerSList(
    const std::vector<common::MmPerS>& values) {
  std::ostringstream output;
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i != 0U) {
      output << ", ";
    }
    output << numberText(values[i].value);
  }
  return output.str();
}

void beginConfigGroup(std::ostringstream& json, bool& first_group,
                      std::string_view name) {
  if (!first_group) {
    json << ',';
  }
  first_group = false;
  json << "{\"name\":" << jsonString(name) << ",\"items\":[";
}

void endConfigGroup(std::ostringstream& json) {
  json << "]}";
}

void appendConfigItem(std::ostringstream& json, bool& first_item,
                      std::string_view key, std::string_view value,
                      std::string_view unit = {}) {
  if (!first_item) {
    json << ',';
  }
  first_item = false;
  json << "{\"key\":" << jsonString(key) << ",\"value\":"
       << jsonString(value) << ",\"unit\":" << jsonString(unit) << '}';
}

[[nodiscard]] std::string configSnapshotJson(const UiController& controller) {
  std::ostringstream json;
  json << "{";
  json << "\"valid\":" << boolJson(controller.hasConfig()) << ',';
  json << "\"common_note\":"
       << jsonString(
              "common 目录只定义单位、Result、错误码等基础类型，不作为运行参数组展示。")
       << ',';
  json << "\"groups\":[";
  if (controller.hasConfig()) {
    const auto& cfg = controller.config();
    bool first_group = true;
    bool first_item = true;

    beginConfigGroup(json, first_group, "adapter");
    first_item = true;
    appendConfigItem(json, first_item, "type",
                     adapterTypeName(cfg.adapter.type));
    appendConfigItem(json, first_item, "channel", cfg.adapter.channel);
    appendConfigItem(json, first_item, "driver_library_path",
                     cfg.adapter.driver_library_path);
    appendConfigItem(json, first_item, "device_index",
                     std::to_string(cfg.adapter.device_index));
    appendConfigItem(json, first_item, "channel_index",
                     std::to_string(cfg.adapter.channel_index));
    appendConfigItem(json, first_item, "fdcan_enabled",
                     boolText(cfg.adapter.fdcan_enabled));
    appendConfigItem(json, first_item, "brs_enabled",
                     boolText(cfg.adapter.brs_enabled));
    appendConfigItem(json, first_item, "nominal_bitrate",
                     std::to_string(cfg.adapter.nominal_bitrate_bps), "bps");
    appendConfigItem(json, first_item, "data_bitrate",
                     std::to_string(cfg.adapter.data_bitrate_bps), "bps");
    endConfigGroup(json);

    beginConfigGroup(json, first_group, "motor");
    first_item = true;
    appendConfigItem(json, first_item, "motor_id",
                     hexU32(cfg.motor.motor_id));
    appendConfigItem(json, first_item, "host_id", hexU32(cfg.motor.host_id));
    appendConfigItem(json, first_item, "control_mode",
                     motorControlModeName(cfg.motor.control_mode));
    appendConfigItem(json, first_item, "direction_sign",
                     std::to_string(cfg.motor.direction_sign));
    appendConfigItem(json, first_item, "position_command_sign",
                     std::to_string(cfg.motor.position_command_sign));
    appendConfigItem(json, first_item, "encoder_scale",
                     numberText(cfg.motor.encoder_scale.value));
    appendConfigItem(json, first_item, "encoder_unwrap_source",
                     encoderUnwrapSourceName(cfg.motor.encoder_unwrap_source));
    appendConfigItem(json, first_item, "encoder_wrap_range",
                     numberText(cfg.motor.encoder_wrap_range.value), "rad");
    appendConfigItem(json, first_item, "encoder_raw_count_range",
                     std::to_string(cfg.motor.encoder_raw_count_range));
    appendConfigItem(json, first_item, "P_MAX_config",
                     numberText(cfg.motor.max_position.value), "rad");
    appendConfigItem(json, first_item, "VMAX_config",
                     numberText(cfg.motor.max_velocity.value), "rad/s");
    appendConfigItem(json, first_item, "TMAX_config",
                     numberText(cfg.motor.max_torque.value), "Nm");
    appendConfigItem(json, first_item, "max_phase_current",
                     numberText(cfg.motor.max_phase_current.value), "A");
    appendConfigItem(json, first_item, "torque_per_amp",
                     numberText(cfg.motor.torque_per_amp.value), "Nm/A");
    appendConfigItem(json, first_item, "auto_switch_mode",
                     boolText(cfg.motor.auto_switch_mode));
    appendConfigItem(json, first_item, "motor_frames_canfd",
                     boolText(cfg.motor.motor_frames_canfd));
    appendConfigItem(json, first_item, "command_id_includes_mode_offset",
                     boolText(cfg.motor.command_id_includes_mode_offset));
    appendConfigItem(json, first_item, "continuous_encoder_enabled",
                     boolText(cfg.motor.continuous_encoder_enabled));
    appendConfigItem(json, first_item, "feedback_poll_period",
                     numberText(cfg.motor.feedback_poll_period.value), "s");
    appendConfigItem(json, first_item, "feedback_stale_timeout",
                     numberText(cfg.motor.feedback_stale_timeout.value), "s");
    endConfigGroup(json);

    beginConfigGroup(json, first_group, "mechanism");
    first_item = true;
    appendConfigItem(json, first_item, "lead_screw_pitch",
                     numberText(cfg.mechanism.lead_screw_pitch.value),
                     "mm/rev");
    appendConfigItem(json, first_item, "usable_travel",
                     numberText(cfg.mechanism.usable_travel.value), "mm");
    appendConfigItem(json, first_item, "theoretical_open_limit",
                     numberText(cfg.mechanism.theoretical_open_limit.value),
                     "mm");
    appendConfigItem(json, first_item, "theoretical_close_limit",
                     numberText(cfg.mechanism.theoretical_close_limit.value),
                     "mm");
    appendConfigItem(json, first_item, "zero_angle_stroke",
                     numberText(cfg.mechanism.zero_angle_stroke.value), "mm");
    appendConfigItem(json, first_item, "zero_stroke_gripper_angle",
                     numberText(
                         cfg.mechanism.zero_stroke_gripper_angle.value),
                     "rad");
    appendConfigItem(json, first_item, "gripper_angle_per_nut_stroke",
                     numberText(
                         cfg.mechanism.gripper_angle_per_nut_stroke.value),
                     "rad/mm");
    appendConfigItem(json, first_item, "gripper_angular_speed_per_nut_speed",
                     numberText(cfg.mechanism
                                    .gripper_angular_speed_per_nut_speed.value),
                     "(rad/s)/(mm/s)");
    endConfigGroup(json);

    beginConfigGroup(json, first_group, "nut_position_encoder");
    first_item = true;
    appendConfigItem(json, first_item, "direction_sign",
                     std::to_string(cfg.nut_position_encoder.direction_sign));
    endConfigGroup(json);

    beginConfigGroup(json, first_group, "self_check");
    first_item = true;
    appendConfigItem(json, first_item, "learned_profile_path",
                     cfg.self_check.learned_profile_path);
    appendConfigItem(json, first_item, "max_probe_window",
                     numberText(cfg.self_check.max_probe_window.value), "mm");
    appendConfigItem(json, first_item, "min_speed_scan_start",
                     numberText(cfg.self_check.min_speed_scan_start.value),
                     "mm/s");
    appendConfigItem(json, first_item, "min_speed_scan_stop",
                     numberText(cfg.self_check.min_speed_scan_stop.value),
                     "mm/s");
    appendConfigItem(json, first_item, "min_speed_scan_step",
                     numberText(cfg.self_check.min_speed_scan_step.value),
                     "mm/s");
    appendConfigItem(json, first_item, "static_friction_current_start",
                     numberText(
                         cfg.self_check.static_friction_current_start.value),
                     "A");
    appendConfigItem(json, first_item, "static_friction_current_stop",
                     numberText(
                         cfg.self_check.static_friction_current_stop.value),
                     "A");
    appendConfigItem(json, first_item, "static_friction_current_step",
                     numberText(
                         cfg.self_check.static_friction_current_step.value),
                     "A");
    appendConfigItem(json, first_item, "motion_start_distance",
                     numberText(cfg.self_check.motion_start_distance.value),
                     "mm");
    appendConfigItem(json, first_item, "low_confidence_motion_distance",
                     numberText(
                         cfg.self_check.low_confidence_motion_distance.value),
                     "mm");
    appendConfigItem(json, first_item, "stable_short_stroke_distance",
                     numberText(
                         cfg.self_check.stable_short_stroke_distance.value),
                     "mm");
    appendConfigItem(json, first_item, "dynamic_friction_speeds",
                     mmPerSList(cfg.self_check.dynamic_friction_speeds),
                     "mm/s");
    appendConfigItem(json, first_item, "feedback_noise_sample_time",
                     numberText(cfg.self_check.feedback_noise_sample_time.value),
                     "s");
    appendConfigItem(json, first_item, "motion_settle_timeout",
                     numberText(cfg.self_check.motion_settle_timeout.value),
                     "s");
    appendConfigItem(json, first_item, "motion_settle_stable_time",
                     numberText(cfg.self_check.motion_settle_stable_time.value),
                     "s");
    appendConfigItem(
        json, first_item, "motion_settle_speed_threshold",
        numberText(cfg.self_check.motion_settle_speed_threshold.value),
        "mm/s");
    appendConfigItem(json, first_item, "motion_hold_speed",
                     numberText(cfg.self_check.motion_hold_speed.value),
                     "mm/s");
    appendConfigItem(json, first_item, "motion_hold_current",
                     numberText(cfg.self_check.motion_hold_current.value), "A");
    appendConfigItem(json, first_item, "travel_learning_speed",
                     numberText(cfg.self_check.travel_learning_speed.value),
                     "mm/s");
    appendConfigItem(
        json, first_item, "travel_learning_search_distance",
        numberText(cfg.self_check.travel_learning_search_distance.value),
        "mm");
    appendConfigItem(json, first_item, "software_limit_margin",
                     numberText(cfg.self_check.software_limit_margin.value),
                     "mm");
    appendConfigItem(json, first_item, "motion_health_check_speeds",
                     mmPerSList(cfg.self_check.motion_health_check_speeds),
                     "mm/s");
    appendConfigItem(json, first_item, "min_measured_distance",
                     numberText(cfg.self_check.min_measured_distance.value),
                     "mm");
    appendConfigItem(json, first_item, "max_distance_error",
                     numberText(cfg.self_check.max_distance_error.value), "mm");
    appendConfigItem(json, first_item, "stable_speed_margin",
                     numberText(cfg.self_check.stable_speed_margin.value),
                     "mm/s");
    appendConfigItem(json, first_item, "max_theoretical_travel_error",
                     numberText(
                         cfg.self_check.max_theoretical_travel_error.value),
                     "mm");
    appendConfigItem(json, first_item, "safe_zone_margin",
                     numberText(cfg.self_check.safe_zone_margin.value), "mm");
    appendConfigItem(json, first_item, "pre_b_max_expansion_distance",
                     numberText(
                         cfg.self_check.pre_b_max_expansion_distance.value),
                     "mm");
    appendConfigItem(json, first_item, "pre_b_boundary_step",
                     numberText(cfg.self_check.pre_b_boundary_step.value),
                     "mm");
    appendConfigItem(json, first_item, "pre_b_expansion_speed",
                     numberText(cfg.self_check.pre_b_expansion_speed.value),
                     "mm/s");
    appendConfigItem(
        json, first_item, "pre_b_boundary_release_distance",
        numberText(cfg.self_check.pre_b_boundary_release_distance.value),
        "mm");
    appendConfigItem(json, first_item, "pre_b_boundary_release_speed",
                     numberText(
                         cfg.self_check.pre_b_boundary_release_speed.value),
                     "mm/s");
    appendConfigItem(
        json, first_item, "pre_b_boundary_release_current_limit",
        numberText(cfg.self_check.pre_b_boundary_release_current_limit.value),
        "A");
    appendConfigItem(json, first_item, "pre_b_soft_jam_retry_distance",
                     numberText(
                         cfg.self_check.pre_b_soft_jam_retry_distance.value),
                     "mm");
    appendConfigItem(json, first_item, "pre_b_soft_jam_retry_speed",
                     numberText(
                         cfg.self_check.pre_b_soft_jam_retry_speed.value),
                     "mm/s");
    appendConfigItem(json, first_item, "pre_b_soft_jam_progress_timeout",
                     numberText(
                         cfg.self_check.pre_b_soft_jam_progress_timeout.value),
                     "s");
    appendConfigItem(json, first_item, "pre_b_hard_current_confirm_time",
                     numberText(
                         cfg.self_check.pre_b_hard_current_confirm_time.value),
                     "s");
    appendConfigItem(json, first_item, "pre_b_min_learning_regions",
                     std::to_string(
                         cfg.self_check.pre_b_min_learning_regions));
    appendConfigItem(json, first_item, "pre_b_learning_anchor_count",
                     std::to_string(
                         cfg.self_check.pre_b_learning_anchor_count));
    appendConfigItem(json, first_item, "fallback_motor_position_noise",
                     numberText(
                         cfg.self_check.fallback_motor_position_noise.value),
                     "rad");
    appendConfigItem(json, first_item, "fallback_motor_velocity_noise",
                     numberText(
                         cfg.self_check.fallback_motor_velocity_noise.value),
                     "rad/s");
    appendConfigItem(json, first_item, "fallback_motor_current_noise",
                     numberText(
                         cfg.self_check.fallback_motor_current_noise.value),
                     "A");
    appendConfigItem(json, first_item, "fallback_motor_torque_noise",
                     numberText(
                         cfg.self_check.fallback_motor_torque_noise.value),
                     "Nm");
    appendConfigItem(json, first_item, "fallback_nut_stroke_noise",
                     numberText(cfg.self_check.fallback_nut_stroke_noise.value),
                     "mm");
    appendConfigItem(json, first_item, "max_velocity_tracking_error",
                     numberText(
                         cfg.self_check.max_velocity_tracking_error.value),
                     "mm/s");
    appendConfigItem(json, first_item, "max_current_ripple",
                     numberText(cfg.self_check.max_current_ripple.value), "A");
    appendConfigItem(json, first_item, "max_torque_ripple",
                     numberText(cfg.self_check.max_torque_ripple.value), "Nm");
    appendConfigItem(json, first_item, "max_motor_temperature",
                     numberText(cfg.self_check.max_motor_temperature.value),
                     "degC");
    appendConfigItem(json, first_item, "friction_anomaly_enabled",
                     cfg.self_check.friction_anomaly_enabled ? "true" : "false");
    appendConfigItem(
        json, first_item, "friction_anomaly_current_ratio_threshold",
        numberText(
            cfg.self_check.friction_anomaly_current_ratio_threshold.value));
    appendConfigItem(
        json, first_item, "friction_anomaly_sliding_window_distance",
        numberText(
            cfg.self_check.friction_anomaly_sliding_window_distance.value),
        "mm");
    appendConfigItem(json, first_item, "friction_anomaly_min_width",
                     numberText(cfg.self_check.friction_anomaly_min_width.value),
                     "mm");
    appendConfigItem(json, first_item, "friction_anomaly_min_confirmations",
                     std::to_string(
                         cfg.self_check.friction_anomaly_min_confirmations));
    appendConfigItem(
        json, first_item, "friction_anomaly_minor_ratio",
        numberText(cfg.self_check.friction_anomaly_minor_ratio.value));
    appendConfigItem(
        json, first_item, "friction_anomaly_moderate_ratio",
        numberText(cfg.self_check.friction_anomaly_moderate_ratio.value));
    appendConfigItem(json, first_item, "friction_anomaly_max_records",
                     std::to_string(
                         cfg.self_check.friction_anomaly_max_records));
    appendConfigItem(
        json, first_item, "friction_anomaly_min_baseline_current",
        numberText(
            cfg.self_check.friction_anomaly_min_baseline_current.value),
        "A");
    appendConfigItem(json, first_item, "friction_anomaly_avoid_margin",
                     numberText(
                         cfg.self_check.friction_anomaly_avoid_margin.value),
                     "mm");
    appendConfigItem(
        json, first_item, "friction_anomaly_learning_avoid_ratio",
        numberText(cfg.self_check.friction_anomaly_learning_avoid_ratio.value));
    endConfigGroup(json);

    beginConfigGroup(json, first_group, "safety");
    first_item = true;
    appendConfigItem(json, first_item, "max_motor_current",
                     numberText(cfg.safety.max_motor_current.value), "A");
    appendConfigItem(json, first_item, "self_check_current_limit",
                     numberText(cfg.safety.self_check_current_limit.value),
                     "A");
    appendConfigItem(
        json, first_item, "self_check_feedback_hard_current_limit",
        numberText(cfg.safety.self_check_feedback_hard_current_limit.value),
        "A");
    appendConfigItem(
        json, first_item, "self_check_feedback_emergency_current_limit",
        numberText(
            cfg.safety.self_check_feedback_emergency_current_limit.value),
        "A");
    appendConfigItem(json, first_item, "homing_current_limit",
                     numberText(cfg.safety.homing_current_limit.value), "A");
    appendConfigItem(json, first_item, "travel_learning_current_limit",
                     numberText(
                         cfg.safety.travel_learning_current_limit.value),
                     "A");
    appendConfigItem(json, first_item, "manual_positioning_current_limit",
                     numberText(
                         cfg.safety.manual_positioning_current_limit.value),
                     "A");
    appendConfigItem(json, first_item, "clamp_current_limit",
                     numberText(cfg.safety.clamp_current_limit.value), "A");
    appendConfigItem(json, first_item, "max_nut_speed",
                     numberText(cfg.safety.max_nut_speed.value), "mm/s");
    appendConfigItem(json, first_item, "max_nut_acceleration",
                     numberText(cfg.safety.max_nut_acceleration.value),
                     "mm/s^2");
    appendConfigItem(json, first_item, "contact_current_rise_threshold",
                     numberText(
                         cfg.safety.contact_current_rise_threshold.value),
                     "A");
    appendConfigItem(json, first_item, "jam_current_threshold",
                     numberText(cfg.safety.jam_current_threshold.value), "A");
    appendConfigItem(json, first_item, "jam_speed_threshold",
                     numberText(cfg.safety.jam_speed_threshold.value),
                     "mm/s");
    appendConfigItem(json, first_item, "stroke_limit_margin",
                     numberText(cfg.safety.stroke_limit_margin.value), "mm");
    appendConfigItem(json, first_item, "contact_detection_time",
                     numberText(cfg.safety.contact_detection_time.value), "s");
    appendConfigItem(json, first_item, "command_timeout",
                     numberText(cfg.safety.command_timeout.value), "s");
    endConfigGroup(json);

    beginConfigGroup(json, first_group, "motor_bringup");
    first_item = true;
    appendConfigItem(json, first_item, "default_relative_motor_position",
                     numberText(
                         cfg.motor_bringup.default_relative_motor_position.value),
                     "rad");
    appendConfigItem(json, first_item, "max_relative_motor_position",
                     numberText(
                         cfg.motor_bringup.max_relative_motor_position.value),
                     "rad");
    appendConfigItem(json, first_item, "default_relative_motor_revolutions",
                     numberText(cfg.motor_bringup
                                    .default_relative_motor_revolutions.value),
                     "rev");
    appendConfigItem(json, first_item, "max_relative_motor_revolutions",
                     numberText(
                         cfg.motor_bringup.max_relative_motor_revolutions.value),
                     "rev");
    appendConfigItem(json, first_item, "default_motor_velocity",
                     numberText(cfg.motor_bringup.default_motor_velocity.value),
                     "rad/s");
    appendConfigItem(json, first_item, "max_motor_velocity",
                     numberText(cfg.motor_bringup.max_motor_velocity.value),
                     "rad/s");
    appendConfigItem(json, first_item, "default_motor_current",
                     numberText(cfg.motor_bringup.default_motor_current.value),
                     "A");
    appendConfigItem(json, first_item, "max_motor_current",
                     numberText(cfg.motor_bringup.max_motor_current.value),
                     "A");
    appendConfigItem(json, first_item, "default_pulse_duration",
                     numberText(cfg.motor_bringup.default_pulse_duration.value),
                     "s");
    appendConfigItem(json, first_item, "max_pulse_duration",
                     numberText(cfg.motor_bringup.max_pulse_duration.value),
                     "s");
    appendConfigItem(
        json, first_item, "default_position_move_timeout",
        numberText(cfg.motor_bringup.default_position_move_timeout.value),
        "s");
    appendConfigItem(json, first_item, "max_position_move_timeout",
                     numberText(
                         cfg.motor_bringup.max_position_move_timeout.value),
                     "s");
    endConfigGroup(json);

    beginConfigGroup(json, first_group, "homing");
    first_item = true;
    appendConfigItem(json, first_item, "direction",
                     motionDirectionName(cfg.homing.direction));
    appendConfigItem(json, first_item, "homing_speed",
                     numberText(cfg.homing.homing_speed.value), "mm/s");
    appendConfigItem(json, first_item, "homing_current",
                     numberText(cfg.homing.homing_current.value), "A");
    appendConfigItem(json, first_item, "jam_confirm_time",
                     numberText(cfg.homing.jam_confirm_time.value), "s");
    appendConfigItem(json, first_item, "backoff_distance",
                     numberText(cfg.homing.backoff_distance.value), "mm");
    endConfigGroup(json);

    beginConfigGroup(json, first_group, "clamp");
    first_item = true;
    appendConfigItem(json, first_item, "target_force",
                     numberText(cfg.clamp.target_force.value), "N");
    appendConfigItem(json, first_item, "min_target_force",
                     numberText(cfg.clamp.min_target_force.value), "N");
    appendConfigItem(json, first_item, "max_target_force",
                     numberText(cfg.clamp.max_target_force.value), "N");
    appendConfigItem(json, first_item, "target_nut_speed",
                     numberText(cfg.clamp.target_nut_speed.value), "mm/s");
    appendConfigItem(json, first_item, "target_gripper_angular_speed",
                     numberText(
                         cfg.clamp.target_gripper_angular_speed.value),
                     "rad/s");
    appendConfigItem(json, first_item, "approach_distance",
                     numberText(cfg.clamp.approach_distance.value), "mm");
    appendConfigItem(json, first_item, "approach_nut_speed",
                     numberText(cfg.clamp.approach_nut_speed.value), "mm/s");
    appendConfigItem(json, first_item, "release_distance",
                     numberText(cfg.clamp.release_distance.value), "mm");
    appendConfigItem(json, first_item, "torque_per_force",
                     numberText(cfg.clamp.torque_per_force.value), "Nm/N");
    appendConfigItem(json, first_item, "current_per_torque",
                     numberText(cfg.clamp.current_per_torque.value), "A/Nm");
    appendConfigItem(json, first_item, "torque_offset",
                     numberText(cfg.clamp.torque_offset.value), "Nm");
    appendConfigItem(json, first_item, "current_offset",
                     numberText(cfg.clamp.current_offset.value), "A");
    appendConfigItem(json, first_item, "max_motor_torque",
                     numberText(cfg.clamp.max_motor_torque.value), "Nm");
    endConfigGroup(json);

    beginConfigGroup(json, first_group, "ui");
    first_item = true;
    appendConfigItem(json, first_item, "refresh_period",
                     numberText(cfg.ui.refresh_period.value), "s");
    appendConfigItem(json, first_item, "log_capacity",
                     std::to_string(cfg.ui.log_capacity));
    appendConfigItem(json, first_item, "default_target_force",
                     numberText(cfg.ui.default_target_force.value), "N");
    appendConfigItem(json, first_item, "default_clamp_speed",
                     numberText(cfg.ui.default_clamp_speed.value), "mm/s");
    endConfigGroup(json);
  }
  json << "]}";
  return json.str();
}

[[nodiscard]] std::map<std::string, std::string> parseQuery(
    std::string_view query_text) {
  std::map<std::string, std::string> query;
  while (!query_text.empty()) {
    const auto amp = query_text.find('&');
    const auto item = query_text.substr(0, amp);
    const auto eq = item.find('=');
    if (eq != std::string_view::npos) {
      query[urlDecode(item.substr(0, eq))] = urlDecode(item.substr(eq + 1));
    } else if (!item.empty()) {
      query[urlDecode(item)] = "";
    }
    if (amp == std::string_view::npos) {
      break;
    }
    query_text.remove_prefix(amp + 1);
  }
  return query;
}

[[nodiscard]] std::string makeViewJson(const UiController& controller) {
  const auto view = controller.viewModel();
  const auto& state = view.state;
  const auto& learned = state.learned_parameters;
  const auto& manual_range = state.manual_nut_stroke_range;
  const bool self_check_running = isSelfCheckRunning();
  const auto& pre_b_trace = state.pre_b_current_trace;
  bool pre_b_trace_has_points = false;
  double pre_b_trace_min_stroke = 0.0;
  double pre_b_trace_max_stroke = 0.0;
  double pre_b_trace_max_current = 0.0;
  for (const auto& point : pre_b_trace) {
    const double stroke = point.stroke.value;
    const double current_abs = point.motor_current.value < 0.0
                                   ? -point.motor_current.value
                                   : point.motor_current.value;
    if (!pre_b_trace_has_points) {
      pre_b_trace_min_stroke = stroke;
      pre_b_trace_max_stroke = stroke;
      pre_b_trace_max_current = current_abs;
      pre_b_trace_has_points = true;
    } else {
      pre_b_trace_min_stroke = std::min(pre_b_trace_min_stroke, stroke);
      pre_b_trace_max_stroke = std::max(pre_b_trace_max_stroke, stroke);
      pre_b_trace_max_current = std::max(pre_b_trace_max_current, current_abs);
    }
  }
  std::ostringstream json;
  json << "{";
  json << "\"status_text\":" << jsonString(view.status_text) << ',';
  json << "\"controls\":{";
  json << "\"can_connect\":" << boolJson(view.can_connect) << ',';
  json << "\"can_enable\":" << boolJson(view.can_enable && !self_check_running)
       << ',';
  json << "\"can_run_self_check\":"
       << boolJson(view.can_run_self_check && !self_check_running)
       << ',';
  json << "\"can_home\":" << boolJson(view.can_home && !self_check_running)
       << ',';
  json << "\"can_learn_limits\":"
       << boolJson(view.can_learn_limits && !self_check_running) << ',';
  json << "\"can_health_check\":"
       << boolJson(view.can_health_check && !self_check_running) << ',';
  json << "\"can_clamp\":" << boolJson(view.can_clamp && !self_check_running)
       << ',';
  json << "\"can_release\":"
       << boolJson(view.can_release && !self_check_running) << ',';
  json << "\"can_move_nut_stroke\":"
       << boolJson(view.can_move_nut_stroke && !self_check_running) << ',';
  json << "\"can_motor_bringup_move\":"
       << boolJson(view.can_motor_bringup_move && !self_check_running) << ',';
  json << "\"can_recover_active_stop\":"
       << boolJson(view.can_recover_active_stop && !self_check_running)
       << "},";
  json << "\"state\":{";
  json << "\"connected\":" << boolJson(state.connected) << ',';
  json << "\"enabled\":" << boolJson(state.enabled) << ',';
  json << "\"motor_bringup_active\":"
       << boolJson(state.motor_bringup_active) << ',';
  json << "\"pre_self_check_completed\":"
       << boolJson(state.pre_self_check_completed) << ',';
  json << "\"self_check_running\":" << boolJson(self_check_running) << ',';
  json << "\"pre_b_mechanism_anomaly\":"
       << boolJson(state.pre_b_mechanism_anomaly) << ',';
  json << "\"homed\":" << boolJson(state.homed) << ',';
  json << "\"travel_limits_learned\":"
       << boolJson(state.travel_limits_learned) << ',';
  json << "\"motion_health_checked\":"
       << boolJson(state.motion_health_checked) << ',';
  json << "\"contact_detected\":" << boolJson(state.contact_detected) << ',';
  json << "\"stroke_mm\":" << state.nut_stroke.value << ',';
  json << "\"gripper_angle_rad\":" << state.gripper_angle.value << ',';
  json << "\"force_n\":" << state.estimated_clamp_force.value << ',';
  json << "\"motor_virtual_pos_rad\":" << state.motor.position.value << ',';
  json << "\"motor_pos_rad\":" << state.motor.position.value << ',';
  json << "\"motor_wrapped_pos_rad\":";
  if (state.motor.wrapped_position_valid) {
    json << state.motor.wrapped_position.value;
  } else {
    json << "null";
  }
  json << ',';
  json << "\"motor_raw_pos_counts\":";
  if (state.motor.raw_position_counts_valid) {
    json << state.motor.raw_position_counts;
  } else {
    json << "null";
  }
  json << ',';
  json << "\"motor_raw_feedback_frame_valid\":"
       << boolJson(state.motor.raw_feedback_frame_valid) << ',';
  json << "\"motor_raw_feedback_frame_id\":";
  if (state.motor.raw_feedback_frame_valid) {
    json << state.motor.raw_feedback_frame_id;
  } else {
    json << "null";
  }
  json << ',';
  json << "\"motor_raw_feedback_frame_dlc\":";
  if (state.motor.raw_feedback_frame_valid) {
    json << static_cast<int>(state.motor.raw_feedback_frame_length);
  } else {
    json << "null";
  }
  json << ',';
  json << "\"motor_raw_feedback_frame_hex\":"
       << jsonString(rawFrameBytes(state.motor)) << ',';
  json << "\"motor_runtime_limits_valid\":"
       << boolJson(state.motor.runtime_limits_valid) << ',';
  json << "\"motor_runtime_pmax_rad\":";
  if (state.motor.runtime_limits_valid) {
    json << state.motor.runtime_position_limit.value;
  } else {
    json << "null";
  }
  json << ',';
  json << "\"motor_runtime_vmax_rad_s\":";
  if (state.motor.runtime_limits_valid) {
    json << state.motor.runtime_velocity_limit.value;
  } else {
    json << "null";
  }
  json << ',';
  json << "\"motor_runtime_tmax_nm\":";
  if (state.motor.runtime_limits_valid) {
    json << state.motor.runtime_torque_limit.value;
  } else {
    json << "null";
  }
  json << ',';
  json << "\"nut_encoder_position_mm\":"
       << state.nut_encoder.nut_position.value << ',';
  json << "\"nut_encoder_velocity_mm_s\":"
       << state.nut_encoder.nut_velocity.value << ',';
  json << "\"nut_encoder_zero_motor_rad\":"
       << state.nut_encoder.zero_motor_position.value << ',';
  json << "\"nut_encoder_zero_nut_mm\":"
       << state.nut_encoder.zero_nut_position.value << ',';
  json << "\"nut_encoder_motor_position_rad\":"
       << state.nut_encoder.motor_position.value << ',';
  json << "\"nut_encoder_motor_delta_rad\":"
       << state.nut_encoder.motor_delta.value << ',';
  json << "\"nut_encoder_motor_delta_rev\":"
       << state.nut_encoder.motor_delta_revolutions.value << ',';
  json << "\"nut_encoder_mm_per_rev_estimate\":"
       << state.nut_encoder.millimeters_per_revolution_estimate.value << ',';
  json << "\"nut_encoder_fresh\":"
       << boolJson(state.nut_encoder.fresh) << ',';
  json << "\"motor_vel_rad_s\":" << state.motor.velocity.value << ',';
  json << "\"motor_current_a\":" << state.motor.current.value << ',';
  json << "\"motor_torque_nm\":" << state.motor.torque.value << ',';
  json << "\"temperature_c\":" << state.motor.temperature.value << ',';
  json << "\"motor_enabled\":" << boolJson(state.motor.enabled) << ',';
  json << "\"motor_fault\":" << boolJson(state.motor.fault) << ',';
  json << "\"structure_profile_validity_text\":"
       << jsonString(profileValidityName(state.structure_profile_validity))
       << ',';
  json << "\"opening_breakaway_current_a\":"
       << learned.opening.breakaway_current.value << ',';
  json << "\"closing_breakaway_current_a\":"
       << learned.closing.breakaway_current.value << ',';
  json << "\"opening_breakaway_torque_nm\":"
       << learned.opening.breakaway_torque.value << ',';
  json << "\"closing_breakaway_torque_nm\":"
       << learned.closing.breakaway_torque.value << ',';
  json << "\"opening_static_friction_current_a\":"
       << learned.opening.static_friction_current.value << ',';
  json << "\"closing_static_friction_current_a\":"
       << learned.closing.static_friction_current.value << ',';
  json << "\"opening_static_friction_torque_nm\":"
       << learned.opening.static_friction_torque.value << ',';
  json << "\"closing_static_friction_torque_nm\":"
       << learned.closing.static_friction_torque.value << ',';
  json << "\"opening_minimum_stable_speed_mm_s\":"
       << learned.opening.minimum_stable_nut_speed.value << ',';
  json << "\"closing_minimum_stable_speed_mm_s\":"
       << learned.closing.minimum_stable_nut_speed.value << ',';
  json << "\"opening_dynamic_friction_current_average_a\":"
       << learned.opening.dynamic_friction_current_average.value << ',';
  json << "\"closing_dynamic_friction_current_average_a\":"
       << learned.closing.dynamic_friction_current_average.value << ',';
  json << "\"opening_dynamic_friction_current_max_a\":"
       << learned.opening.dynamic_friction_current_max.value << ',';
  json << "\"closing_dynamic_friction_current_max_a\":"
       << learned.closing.dynamic_friction_current_max.value << ',';
  json << "\"opening_dynamic_friction_torque_average_nm\":"
       << learned.opening.dynamic_friction_torque_average.value << ',';
  json << "\"closing_dynamic_friction_torque_average_nm\":"
       << learned.closing.dynamic_friction_torque_average.value << ',';
  json << "\"opening_breakaway_sample_count\":"
       << learned.opening.breakaway_sample_count << ',';
  json << "\"closing_breakaway_sample_count\":"
       << learned.closing.breakaway_sample_count << ',';
  json << "\"opening_static_friction_sample_count\":"
       << learned.opening.static_friction_sample_count << ',';
  json << "\"closing_static_friction_sample_count\":"
       << learned.closing.static_friction_sample_count << ',';
  json << "\"opening_stable_speed_sample_count\":"
       << learned.opening.stable_speed_sample_count << ',';
  json << "\"closing_stable_speed_sample_count\":"
       << learned.closing.stable_speed_sample_count << ',';
  json << "\"opening_dynamic_friction_sample_count\":"
       << learned.opening.dynamic_friction_sample_count << ',';
  json << "\"closing_dynamic_friction_sample_count\":"
       << learned.closing.dynamic_friction_sample_count << ',';
  json << "\"safe_zone_open_mm\":" << learned.safe_zone_open_limit.value
       << ',';
  json << "\"safe_zone_closed_mm\":"
       << learned.safe_zone_closed_limit.value << ',';
  json << "\"software_open_limit_mm\":"
       << learned.software_open_limit.value << ',';
  json << "\"software_closed_limit_mm\":"
       << learned.software_closed_limit.value << ',';
  json << "\"learned_travel_mm\":" << learned.learned_travel.value << ',';
  json << "\"manual_stroke_range_valid\":"
       << boolJson(manual_range.valid) << ',';
  json << "\"manual_stroke_min_mm\":" << manual_range.open_limit.value
       << ',';
  json << "\"manual_stroke_max_mm\":" << manual_range.closed_limit.value
       << ',';
  json << "\"manual_stroke_uses_software_limits\":"
       << boolJson(manual_range.use_software_limits) << ',';
  json << "\"manual_stroke_low_confidence_window\":"
       << boolJson(manual_range.low_confidence_window) << ',';
  json << "\"manual_stroke_confidence\":"
       << jsonString(manual_range.confidence) << ',';
  json << "\"friction_anomaly_record_count\":"
       << state.friction_anomaly.record_count << ',';
  json << "\"friction_anomaly_max_ratio\":"
       << state.friction_anomaly.max_current_excess_ratio.value << ',';
  json << "\"friction_anomaly_severe_center_mm\":"
       << state.friction_anomaly.severe_center_position.value << ',';
  json << "\"friction_anomaly_severe_width_mm\":"
       << state.friction_anomaly.severe_width.value << ',';
  json << "\"friction_anomaly_severe_peak_current_a\":"
       << state.friction_anomaly.severe_peak_current.value << ',';
  json << "\"friction_anomaly_severe_baseline_current_a\":"
       << state.friction_anomaly.severe_baseline_current.value << ',';
  json << "\"friction_anomaly_has_severe_record\":"
       << boolJson(state.friction_anomaly.has_severe_record) << ',';
  json << "\"pre_b_trace_sample_count\":" << pre_b_trace.size() << ',';
  json << "\"pre_b_trace_min_stroke_mm\":";
  if (pre_b_trace_has_points) {
    json << pre_b_trace_min_stroke;
  } else {
    json << "null";
  }
  json << ',';
  json << "\"pre_b_trace_max_stroke_mm\":";
  if (pre_b_trace_has_points) {
    json << pre_b_trace_max_stroke;
  } else {
    json << "null";
  }
  json << ',';
  json << "\"pre_b_trace_max_current_a\":";
  if (pre_b_trace_has_points) {
    json << pre_b_trace_max_current;
  } else {
    json << "null";
  }
  json << ',';
  json << "\"pre_b_current_trace\":[";
  for (std::size_t i = 0; i < pre_b_trace.size(); ++i) {
    if (i != 0U) {
      json << ',';
    }
    const auto& point = pre_b_trace[i];
    const double current_abs = point.motor_current.value < 0.0
                                   ? -point.motor_current.value
                                   : point.motor_current.value;
    json << "{";
    json << "\"stroke_mm\":" << point.stroke.value << ',';
    json << "\"current_a\":" << current_abs << ',';
    json << "\"abs_current_a\":" << current_abs << ',';
    json << "\"signed_current_a\":" << point.motor_current.value << ',';
    json << "\"speed_mm_s\":" << point.nut_speed.value << ',';
    json << "\"segment_id\":" << point.segment_id << ',';
    json << "\"phase\":" << static_cast<int>(point.phase) << ',';
    json << "\"direction\":" << static_cast<int>(point.direction);
    json << "}";
  }
  json << "],";
  json << "\"static_friction_summary\":"
       << jsonString(frictionSummary(learned, false)) << ',';
  json << "\"dynamic_friction_summary\":"
       << jsonString(frictionSummary(learned, true)) << ',';
  json << "\"safe_zone_summary\":" << jsonString(safeZoneSummary(learned))
       << ',';
  json << "\"fault_code\":" << state.fault.code << ',';
  json << "\"last_result_code\":" << state.last_result_code << "},";
  json << "\"config\":" << configSnapshotJson(controller) << ',';
  json << "\"logs\":[";
  const auto entries = controller.logModel().entries();
  for (std::size_t i = 0; i < entries.size(); ++i) {
    if (i != 0U) {
      json << ',';
    }
    json << jsonString(entries[i].message);
  }
  json << "]}";
  return json.str();
}

[[nodiscard]] std::string resultJson(const UiController& controller,
                                     const common::Result& result) {
  std::ostringstream json;
  json << "{";
  json << "\"ok\":" << boolJson(result.isOk()) << ',';
  json << "\"code\":" << jsonString(common::toString(result.code())) << ',';
  json << "\"message\":" << jsonString(result.message()) << ',';
  json << "\"view\":" << makeViewJson(controller);
  json << "}";
  return json.str();
}

[[nodiscard]] common::Result runNamedAction(
    UiController* controller, const std::map<std::string, std::string>& query,
    bool* request_shutdown) {
  if (controller == nullptr) {
    return common::Result::error(common::ErrorCode::ControlNotReady,
                                 "web controller is null");
  }
  const auto name_iter = query.find("name");
  const std::string name = name_iter == query.end() ? "" : name_iter->second;
  if (name == "connect") {
    return controller->connect();
  }
  if (name == "disconnect") {
    return controller->disconnect();
  }
  if (name == "enter_bringup") {
    return controller->enterMotorBringupMode(parseBool(query, "confirmed"));
  }
  if (name == "exit_bringup") {
    return controller->exitMotorBringupMode();
  }
  if (name == "can_probe") {
    return controller->runMotorBringupCommunicationProbe();
  }
  if (name == "feedback") {
    return controller->refreshMotorBringupFeedback();
  }
  if (name == "bringup_enable") {
    return controller->enableMotorBringupOutput();
  }
  if (name == "bringup_disable") {
    return controller->disableMotorBringupOutput();
  }
  if (name == "jog_pos" || name == "jog_neg") {
    const double sign = name == "jog_neg" ? -1.0 : 1.0;
    return controller->jogMotorBringup(
        common::Rad{sign * std::abs(parseDouble(query, "rad", 0.2))},
        common::RadPerS{parseDouble(query, "vel", 0.5)},
        common::A{parseDouble(query, "cur", 0.5)},
        common::S{parseDouble(query, "sec", 0.1)});
  }
  if (name == "turn_pos" || name == "turn_neg" || name == "turn_custom") {
    const double action_sign = name == "turn_neg" ? -1.0 : 1.0;
    const double requested_turns =
        name == "turn_custom" ? parseDouble(query, "rev", 1.0)
                              : action_sign *
                                    std::abs(parseDouble(query, "rev", 1.0));
    return controller->moveMotorBringupRelativeTurns(
        common::Ratio{requested_turns},
        common::RadPerS{parseDouble(query, "vel", 1.0)},
        common::A{parseDouble(query, "cur", 1.5)},
        common::S{parseDouble(query, "timeout", 0.0)});
  }
  if (name == "enable") {
    return controller->enable();
  }
  if (name == "disable") {
    return controller->disable();
  }
  if (name == "selfcheck") {
    {
      std::lock_guard<std::mutex> lock{g_long_action_mutex};
      if (g_self_check_running) {
        return common::Result::error(common::ErrorCode::InvalidOperation,
                                     "pre-self-check is already running");
      }
      g_self_check_running = true;
    }
    controller->appendLog("pre_self_check: started in background");
    std::thread([controller]() {
      const auto result = controller->runPreSelfCheck();
      controller->appendLog(std::string{"pre_self_check_background: "} +
                            std::string{common::toString(result.code())} +
                            (result.hasMessage()
                                 ? std::string{" | "} + result.message()
                                 : std::string{}));
      std::lock_guard<std::mutex> lock{g_long_action_mutex};
      g_self_check_running = false;
    }).detach();
    return common::Result{common::ErrorCode::Ok,
                          "pre-self-check started in background"};
  }
  if (name == "home") {
    return controller->home();
  }
  if (name == "learn") {
    return controller->learnTravelLimits();
  }
  if (name == "health") {
    return controller->runMotionHealthCheck();
  }
  if (name == "clamp") {
    return controller->clampForce(common::N{parseDouble(query, "force", 20.0)},
                                  common::MmPerS{
                                      parseDouble(query, "speed", 1.0)});
  }
  if (name == "release") {
    return controller->release();
  }
  if (name == "move_stroke") {
    return controller->moveNutStroke(
        common::Mm{parseDouble(query, "stroke", 0.0)},
        common::MmPerS{parseDouble(query, "speed", 1.0)},
        parseBool(query, "unloaded"));
  }
  if (name == "stop") {
    const auto result = controller->stop();
    controller->appendLog("stop: cancellation requested for background workflows");
    return result;
  }
  if (name == "clear_fault") {
    return controller->clearFault();
  }
  if (name == "shutdown") {
    if (request_shutdown != nullptr) {
      *request_shutdown = true;
    }
    return common::Ok();
  }
  return common::Result::error(common::ErrorCode::InvalidArgument,
                               "unknown web action: " + name);
}

#if defined(_WIN32)
void closeSocket(SOCKET socket) {
  if (socket != INVALID_SOCKET) {
    closesocket(socket);
  }
}

[[nodiscard]] bool sendAll(SOCKET client, const std::string& data) {
  const char* cursor = data.data();
  int remaining = static_cast<int>(data.size());
  while (remaining > 0) {
    const int sent = send(client, cursor, remaining, 0);
    if (sent == SOCKET_ERROR) {
      return false;
    }
    cursor += sent;
    remaining -= sent;
  }
  return true;
}

void sendResponse(SOCKET client, std::string_view status,
                  std::string_view content_type, const std::string& body) {
  std::ostringstream response;
  response << "HTTP/1.1 " << status << "\r\n"
           << "Content-Type: " << content_type << "\r\n"
           << "Content-Length: " << body.size() << "\r\n"
           << "Cache-Control: no-store\r\n"
           << "Connection: close\r\n\r\n"
           << body;
  (void)sendAll(client, response.str());
}

[[nodiscard]] std::string receiveRequest(SOCKET client) {
  std::string request;
  char buffer[2048]{};
  while (request.find("\r\n\r\n") == std::string::npos &&
         request.size() < 32768U) {
    const int received = recv(client, buffer, sizeof(buffer), 0);
    if (received <= 0) {
      break;
    }
    request.append(buffer, buffer + received);
  }
  return request;
}

[[nodiscard]] std::string requestTarget(const std::string& request) {
  const auto line_end = request.find("\r\n");
  const std::string_view line{
      request.data(), line_end == std::string::npos ? request.size() : line_end};
  const auto first_space = line.find(' ');
  if (first_space == std::string_view::npos) {
    return "/";
  }
  const auto second_space = line.find(' ', first_space + 1);
  const auto target = line.substr(first_space + 1,
                                  second_space == std::string_view::npos
                                      ? std::string_view::npos
                                      : second_space - first_space - 1);
  return std::string{target};
}

[[nodiscard]] SOCKET openListenSocket(std::uint16_t preferred_port,
                                      std::uint16_t* bound_port) {
  for (std::uint16_t offset = 0; offset < kFallbackPortCount; ++offset) {
    const std::uint16_t port =
        static_cast<std::uint16_t>(preferred_port + offset);
    SOCKET listen_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listen_socket == INVALID_SOCKET) {
      return INVALID_SOCKET;
    }
    BOOL reuse = TRUE;
    setsockopt(listen_socket, SOL_SOCKET, SO_REUSEADDR,
               reinterpret_cast<const char*>(&reuse), sizeof(reuse));

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = htons(port);
    if (bind(listen_socket, reinterpret_cast<sockaddr*>(&address),
             sizeof(address)) == 0 &&
        listen(listen_socket, SOMAXCONN) == 0) {
      if (bound_port != nullptr) {
        *bound_port = port;
      }
      return listen_socket;
    }
    closeSocket(listen_socket);
  }
  return INVALID_SOCKET;
}
#endif

}  // namespace

WebServer::WebServer(UiController* controller, std::uint16_t preferred_port)
    : controller_(controller), preferred_port_(preferred_port) {}

int WebServer::run(std::ostream& output) {
  if (controller_ == nullptr) {
    output << "web ui failed: controller is null\n";
    return 1;
  }

#if !defined(_WIN32)
  output << "web ui failed: built-in web UI is currently Windows-only\n";
  return 1;
#else
  WSADATA wsa_data{};
  if (WSAStartup(MAKEWORD(2, 2), &wsa_data) != 0) {
    output << "web ui failed: WSAStartup failed\n";
    return 1;
  }

  std::uint16_t bound_port = 0;
  SOCKET listen_socket = openListenSocket(preferred_port_, &bound_port);
  if (listen_socket == INVALID_SOCKET) {
    WSACleanup();
    output << "web ui failed: no localhost port available near "
           << preferred_port_ << '\n';
    return 1;
  }

  output << "web ui: http://127.0.0.1:" << bound_port << "/\n";
  output << "web ui: close from the browser or press Ctrl+C in this console\n";

  bool running = true;
  while (running) {
    SOCKET client = accept(listen_socket, nullptr, nullptr);
    if (client == INVALID_SOCKET) {
      continue;
    }

    const std::string request = receiveRequest(client);
    const std::string target = requestTarget(request);
    const auto query_start = target.find('?');
    const std::string path = target.substr(0, query_start);
    const auto query = query_start == std::string::npos
                           ? std::map<std::string, std::string>{}
                           : parseQuery(std::string_view{target}.substr(
                                 query_start + 1));

    if (path == "/" || path == "/index.html") {
      sendResponse(client, "200 OK", "text/html; charset=utf-8", htmlPage());
    } else if (path == "/api/view") {
      if (!isSelfCheckRunning()) {
        (void)controller_->update();
      }
      sendResponse(client, "200 OK", "application/json; charset=utf-8",
                   makeViewJson(*controller_));
    } else if (path == "/api/action") {
      bool request_shutdown = false;
      const auto result = runNamedAction(controller_, query, &request_shutdown);
      sendResponse(client, "200 OK", "application/json; charset=utf-8",
                   resultJson(*controller_, result));
      if (request_shutdown) {
        running = false;
      }
    } else if (path == "/favicon.ico") {
      sendResponse(client, "204 No Content", "text/plain; charset=utf-8", "");
    } else {
      sendResponse(client, "404 Not Found", "application/json; charset=utf-8",
                   "{\"ok\":false,\"message\":\"not found\"}");
    }
    closeSocket(client);
  }

  (void)controller_->disconnect();
  closeSocket(listen_socket);
  WSACleanup();
  return 0;
#endif
}

}  // namespace gripper::ui
