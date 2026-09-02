const AXIS_NAMES = ["J1/X", "J2/Y", "J3/Z", "J4/Rx", "J5/Ry", "J6/Rz"];

function toast(msg, isError) {
  const el = document.getElementById("toast");
  el.textContent = msg;
  el.className = "toast show" + (isError ? " error" : "");
  clearTimeout(toast._t);
  toast._t = setTimeout(() => { el.className = "toast"; }, 2500);
}

async function api(path, body) {
  try {
    const res = await fetch(path, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body || {}),
    });
    const json = await res.json();
    if (!json.ok) {
      toast("오류: " + json.error, true);
      return null;
    }
    return json.data;
  } catch (e) {
    toast("통신 실패: " + e, true);
    return null;
  }
}

function getVel() {
  return Number(document.getElementById("vel-slider").value);
}

// ------------------------------------------------------------------
// 연결 / 상태 폴링
// ------------------------------------------------------------------
let connected = false;

document.getElementById("btn-connect").addEventListener("click", async () => {
  const ip = document.getElementById("ip-input").value.trim();
  const data = await api("/api/connect", { ip });
  if (data) {
    toast(`연결 성공: SDK ${data.sdk_version}, 로봇 ${data.robot_version}`);
    connected = true;
  }
});

function fmtPose(p) {
  if (!p) return "-";
  return p.map(v => Number(v).toFixed(2)).join(", ");
}

async function pollStatus() {
  try {
    const res = await fetch("/api/status");
    const json = await res.json();
    const indicator = document.getElementById("conn-indicator");
    const box = document.getElementById("status-box");
    if (!json.ok || !json.data || !json.data.connected) {
      indicator.textContent = "연결 안됨";
      indicator.className = "badge badge-off";
      box.textContent = "아직 연결하지 않았습니다.";
      connected = false;
    } else {
      const d = json.data;
      indicator.textContent = "연결됨 (" + d.ip + ")";
      indicator.className = "badge badge-on";
      connected = true;
      box.textContent =
        `에러코드      : ${JSON.stringify(d.error_code)}\n` +
        `TCP 위치(mm/°) : ${fmtPose(d.tcp_pose_mm_deg)}\n` +
        `관절 각도(°)   : ${fmtPose(d.joint_pos_deg)}\n` +
        `드래그 티칭    : ${d.is_drag_teach ? "예" : "아니오"}\n` +
        `프로그램 상태  : ${ {1:"정지/없음",2:"실행중",3:"일시정지"}[d.program_state] || d.program_state }`;
    }
  } catch (e) {
    // 서버 자체가 아직 안 떠 있는 경우 등 - 조용히 무시
  }
}
setInterval(pollStatus, 1200);
pollStatus();

document.getElementById("vel-slider").addEventListener("input", (e) => {
  document.getElementById("vel-value").textContent = e.target.value;
});

// ------------------------------------------------------------------
// 정지 / 일시정지 / 재개
// ------------------------------------------------------------------
document.getElementById("btn-stop").addEventListener("click", async () => {
  await api("/api/stop", {});
  await api("/api/jog/stop_immediate", {});
  toast("정지 명령 전송");
});
document.getElementById("btn-pause").addEventListener("click", () => api("/api/pause", {}));
document.getElementById("btn-resume").addEventListener("click", () => api("/api/resume", {}));

// ------------------------------------------------------------------
// 활성화 / 모드 / 드래그티칭 / 에러초기화 (공통 data-api 버튼)
// ------------------------------------------------------------------
document.querySelectorAll("button[data-api]").forEach(btn => {
  btn.addEventListener("click", async () => {
    const path = btn.getAttribute("data-api");
    const body = JSON.parse(btn.getAttribute("data-body") || "{}");
    const data = await api(path, body);
    if (data !== null) toast(path + " 완료 (반환값: " + JSON.stringify(data.error) + ")");
  });
});

// ------------------------------------------------------------------
// 조그 UI 생성
// ------------------------------------------------------------------
const jogGrid = document.getElementById("jog-grid");
for (let axis = 1; axis <= 6; axis++) {
  const col = document.createElement("div");
  col.className = "jog-axis";
  col.innerHTML = `
    <span>${AXIS_NAMES[axis - 1]}</span>
    <button class="jog-btn" data-dir="1" data-nb="${axis}">+</button>
    <button class="jog-btn" data-dir="0" data-nb="${axis}">−</button>
  `;
  jogGrid.appendChild(col);
}

function refToStopRef(ref) {
  // SDK 규칙: 시작 ref(0,2,4,8) -> 정지 ref(1,3,5,9) = 시작값+1
  return ref + 1;
}

jogGrid.querySelectorAll(".jog-btn").forEach(btn => {
  const start = async () => {
    const ref = Number(document.getElementById("jog-ref").value);
    await api("/api/jog/start", {
      ref, nb: Number(btn.dataset.nb), dir: Number(btn.dataset.dir),
      max_dis: 90, vel: getVel(),
    });
  };
  const stop = async () => {
    const ref = Number(document.getElementById("jog-ref").value);
    await api("/api/jog/stop", { ref: refToStopRef(ref) });
  };
  btn.addEventListener("mousedown", start);
  btn.addEventListener("touchstart", (e) => { e.preventDefault(); start(); });
  btn.addEventListener("mouseup", stop);
  btn.addEventListener("mouseleave", stop);
  btn.addEventListener("touchend", stop);
});

// ------------------------------------------------------------------
// 정밀 직선/회전 이동
// ------------------------------------------------------------------
function bindLinear(btnId, sign) {
  document.getElementById(btnId).addEventListener("click", () => {
    api("/api/move/linear", {
      axis: document.getElementById("lin-axis").value,
      distance_cm: Number(document.getElementById("lin-distance").value),
      sign, vel: getVel(), frame: "base",
    });
  });
}
bindLinear("btn-lin-plus", "+");
bindLinear("btn-lin-minus", "-");

function bindToolLinear(btnId, sign) {
  document.getElementById(btnId).addEventListener("click", () => {
    api("/api/move/linear", {
      axis: document.getElementById("tool-axis").value,
      distance_cm: Number(document.getElementById("tool-distance").value),
      sign, vel: getVel(), frame: "tool",
    });
  });
}
bindToolLinear("btn-tool-plus", "+");
bindToolLinear("btn-tool-minus", "-");

function bindRotate(btnId, sign) {
  document.getElementById(btnId).addEventListener("click", () => {
    api("/api/move/rotate", {
      axis: document.getElementById("rot-axis").value,
      angle_deg: Number(document.getElementById("rot-angle").value),
      sign, vel: getVel(),
    });
  });
}
bindRotate("btn-rot-plus", "+");
bindRotate("btn-rot-minus", "-");

// ------------------------------------------------------------------
// 도형 그리기
// ------------------------------------------------------------------
document.querySelectorAll(".shape-btn").forEach(btn => {
  btn.addEventListener("click", async () => {
    const plane = document.getElementById("shape-plane").value;
    const direction = document.getElementById("shape-direction").value;
    const vel = getVel();
    const shape = btn.dataset.shape;
    let path, body;
    if (shape === "circle") {
      path = "/api/shape/circle";
      body = { radius_cm: Number(document.getElementById("circle-radius").value), plane, direction, vel };
    } else if (shape === "arc") {
      path = "/api/shape/arc";
      body = {
        radius_cm: Number(document.getElementById("arc-radius").value),
        sweep_deg: Number(document.getElementById("arc-sweep").value),
        plane, direction, vel,
      };
    } else if (shape === "polygon") {
      path = "/api/shape/polygon";
      body = {
        sides: Number(document.getElementById("poly-sides").value),
        side_length_cm: Number(document.getElementById("poly-side").value),
        plane, direction, vel,
      };
    } else if (shape === "ellipse") {
      path = "/api/shape/ellipse";
      body = {
        a_cm: Number(document.getElementById("ellipse-a").value),
        b_cm: Number(document.getElementById("ellipse-b").value),
        segments: Number(document.getElementById("ellipse-seg").value),
        plane, direction, vel,
      };
    } else if (shape === "spiral") {
      path = "/api/shape/spiral";
      body = {
        turns: Number(document.getElementById("spiral-turns").value),
        tilt_deg: Number(document.getElementById("spiral-tilt").value),
        radius_init_cm: Number(document.getElementById("spiral-r0").value),
        radius_add_cm: Number(document.getElementById("spiral-radd").value),
        axis_add_cm: Number(document.getElementById("spiral-aadd").value),
        direction: direction === "ccw" ? "ccw" : "cw",
        vel,
      };
    }
    btn.disabled = true;
    const data = await api(path, body);
    btn.disabled = false;
    if (data !== null) toast(shape + " 실행 완료 (반환값: " + JSON.stringify(data.error) + ")");
  });
});

// ------------------------------------------------------------------
// 카메라 얼굴 트래킹
// ------------------------------------------------------------------
let cameraPollTimer = null;

function syncCameraConfigInputs(d) {
  document.getElementById("invert-x").checked = !!d.invert.x;
  document.getElementById("invert-y").checked = !!d.invert.y;
  document.getElementById("invert-z").checked = !!d.invert.z;
  document.getElementById("max-step").value = d.max_step_mm;
}

async function pollCameraState() {
  const res = await fetch("/api/camera/state");
  const json = await res.json();
  if (!json.ok) return;
  const d = json.data;
  const box = document.getElementById("camera-status");
  if (!d.opened) {
    box.textContent = "카메라를 아직 열지 않았습니다.";
    return;
  }
  box.textContent =
    `트래킹 상태    : ${d.tracking_enabled ? "실행 중" : "정지"}\n` +
    `얼굴 인식됨    : ${d.face_found ? "예" : "아니오 (사람 탐색 중)"}\n` +
    `중심 오차(px)  : x=${d.error_x_px}, y=${d.error_y_px}\n` +
    `크기비율/목표  : ${d.size_ratio} / ${d.target_size_ratio}\n` +
    `반전 x/y/z     : ${d.invert.x}/${d.invert.y}/${d.invert.z}\n` +
    `최대 이동폭    : ${d.max_step_mm} mm`;
}

document.getElementById("btn-camera-open").addEventListener("click", async () => {
  const index = Number(document.getElementById("camera-index").value);
  const data = await api("/api/camera/open", { index });
  if (data === null) return;
  const img = document.getElementById("camera-img");
  img.src = "/api/camera/stream?t=" + Date.now();
  toast("카메라 열림");
  clearInterval(cameraPollTimer);
  cameraPollTimer = setInterval(pollCameraState, 700);
  pollCameraState();
  const stateRes = await fetch("/api/camera/state");
  const stateJson = await stateRes.json();
  if (stateJson.ok) syncCameraConfigInputs(stateJson.data);
});

document.getElementById("btn-camera-close").addEventListener("click", async () => {
  await api("/api/camera/close", {});
  document.getElementById("camera-img").src = "";
  clearInterval(cameraPollTimer);
  document.getElementById("camera-status").textContent = "카메라를 닫았습니다.";
  toast("카메라 닫힘");
});

document.getElementById("btn-camera-calibrate").addEventListener("click", async () => {
  const data = await api("/api/camera/calibrate", {});
  if (data !== null) toast("거리 기준 설정됨 (목표 크기비율: " + data.target_size_ratio.toFixed(3) + ")");
});

document.getElementById("btn-track-start").addEventListener("click", async () => {
  const data = await api("/api/camera/track/start", {});
  if (data !== null) toast("트래킹 시작");
});

document.getElementById("btn-track-stop").addEventListener("click", async () => {
  await api("/api/camera/track/stop", {});
  toast("트래킹 정지");
});

document.getElementById("btn-camera-config").addEventListener("click", async () => {
  const data = await api("/api/camera/config", {
    invert_x: document.getElementById("invert-x").checked,
    invert_y: document.getElementById("invert-y").checked,
    invert_z: document.getElementById("invert-z").checked,
    max_step_mm: Number(document.getElementById("max-step").value),
  });
  if (data !== null) toast("설정 적용됨");
});

// 상단 정지 버튼은 카메라 트래킹도 같이 멈춤 (서버 /api/stop 안에서 처리되지만,
// 화면 상태도 바로 갱신해준다)
document.getElementById("btn-stop").addEventListener("click", () => {
  pollCameraState();
});
