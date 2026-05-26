let mapConfig = null;
let plannedRoute = null;
let sequencePlan = null;
let dragging = null;
let mouseWorld = { x: 0, y: 0 };
let robotState = null;
let backgroundImage = new Image();
let backgroundImageLoaded = false;

let roiConfig = null;
let draggingROI = false;
let resizingROI = false;
let roiDragOffset = { x: 0, y: 0 };

const canvas = document.getElementById("mapCanvas");
const ctx = canvas.getContext("2d");

const roiCanvas = document.getElementById("roiCanvas");
const roiCtx = roiCanvas.getContext("2d");

const margin = 35;

// ============================================================
// API
// ============================================================

async function apiGet(url) {
  const res = await fetch(url);
  return await res.json();
}

async function apiPost(url, data) {
  const res = await fetch(url, {
    method: "POST",
    headers: {"Content-Type": "application/json"},
    body: JSON.stringify(data)
  });

  return await res.json();
}

// ============================================================
// STATUS
// ============================================================

function logStatus(text) {
  document.getElementById("statusBox").textContent = text;
}

function setTopStatus(text) {
  document.getElementById("topStatus").textContent = text;
}

// ============================================================
// VELOCIDADES MISIÓN
// ============================================================

function getMissionSpeeds() {
  const linearInput = document.getElementById("missionLinearSpeed");
  const angularInput = document.getElementById("missionAngularSpeed");
  const slowPickInput = document.getElementById("missionSlowPickSpeed");

  return {
    linearSpeed: parseFloat((linearInput && linearInput.value) || "0.03"),
    angularSpeed: parseFloat((angularInput && angularInput.value) || "0.15"),
    slowPickSpeed: parseFloat((slowPickInput && slowPickInput.value) || "0.015")
  };
}

// ============================================================
// COORDENADAS
// ============================================================

function getCanvasMousePosition(event) {
  const rect = canvas.getBoundingClientRect();

  const scaleX = canvas.width / rect.width;
  const scaleY = canvas.height / rect.height;

  return {
    x: (event.clientX - rect.left) * scaleX,
    y: (event.clientY - rect.top) * scaleY
  };
}

function boardVisualW() {
  return mapConfig.board.width_y_mm;
}

function boardVisualH() {
  return mapConfig.board.height_x_mm;
}

function scale() {
  const sx = (canvas.width - 2 * margin) / boardVisualW();
  const sy = (canvas.height - 2 * margin) / boardVisualH();
  return Math.min(sx, sy);
}

function worldToCanvas(x, y) {
  const s = scale();

  const cx = margin + (boardVisualW() - y) * s;
  const cy = margin + (boardVisualH() - x) * s;

  return { x: cx, y: cy };
}

function canvasToWorld(cx, cy) {
  const s = scale();

  const y = boardVisualW() - ((cx - margin) / s);
  const x = boardVisualH() - ((cy - margin) / s);

  return {
    x: Math.round(x),
    y: Math.round(y)
  };
}

function clampWorldPoint(p) {
  return {
    x: Math.max(0, Math.min(mapConfig.board.height_x_mm, p.x)),
    y: Math.max(0, Math.min(mapConfig.board.width_y_mm, p.y))
  };
}

function loadBackgroundFromMap() {
  if (!mapConfig || !mapConfig.background_image) {
    return;
  }

  const bg = mapConfig.background_image;

  const enabledInput = document.getElementById("bgEnabled");
  const urlInput = document.getElementById("bgUrl");
  const offsetXInput = document.getElementById("bgOffsetX");
  const offsetYInput = document.getElementById("bgOffsetY");
  const scaleInput = document.getElementById("bgScale");
  const opacityInput = document.getElementById("bgOpacity");

  if (enabledInput) enabledInput.checked = !!bg.enabled;
  if (urlInput) urlInput.value = bg.url || "/web/map_background.png";
  if (offsetXInput) offsetXInput.value = bg.offset_x || 0;
  if (offsetYInput) offsetYInput.value = bg.offset_y || 0;
  if (scaleInput) scaleInput.value = bg.scale || 1.0;
  if (opacityInput) opacityInput.value = bg.opacity || 0.35;

  backgroundImageLoaded = false;
  backgroundImage = new Image();

  backgroundImage.onload = function() {
    backgroundImageLoaded = true;
    drawMap();
  };

  backgroundImage.src = bg.url || "/web/map_background.png";
}


async function saveBackgroundConfig() {
  const payload = {
    enabled: document.getElementById("bgEnabled").checked,
    url: document.getElementById("bgUrl").value.trim() || "/web/map_background.png",
    offset_x: parseFloat(document.getElementById("bgOffsetX").value || "0"),
    offset_y: parseFloat(document.getElementById("bgOffsetY").value || "0"),
    scale: parseFloat(document.getElementById("bgScale").value || "1"),
    opacity: parseFloat(document.getElementById("bgOpacity").value || "0.35")
  };

  const result = await apiPost("/api/background/save", payload);

  if (!result.ok) {
    logStatus("ERROR guardando fondo:\n" + result.error);
    return;
  }

  await reloadMap();
  logStatus(result.message);
}


function drawBackgroundImage() {
  if (!mapConfig || !mapConfig.background_image) {
    return;
  }

  const bg = mapConfig.background_image;

  if (!bg.enabled || !backgroundImageLoaded) {
    return;
  }

  const boardBottomRight = worldToCanvas(0, 0);
  const boardTopLeft = worldToCanvas(mapConfig.board.height_x_mm, mapConfig.board.width_y_mm);

  const left = Math.min(boardBottomRight.x, boardTopLeft.x);
  const top = Math.min(boardBottomRight.y, boardTopLeft.y);
  const w = Math.abs(boardBottomRight.x - boardTopLeft.x);
  const h = Math.abs(boardBottomRight.y - boardTopLeft.y);

  const bgScaleValue = parseFloat(bg.scale || 1.0);
  const drawW = w * bgScaleValue;
  const drawH = h * bgScaleValue;

  const offsetX = parseFloat(bg.offset_x || 0);
  const offsetY = parseFloat(bg.offset_y || 0);

  ctx.save();
  ctx.globalAlpha = parseFloat(bg.opacity || 0.35);

  ctx.drawImage(
    backgroundImage,
    left + offsetX,
    top + offsetY,
    drawW,
    drawH
  );

  ctx.restore();
}

// ============================================================
// DIBUJO MAPA
// ============================================================

function drawText(text, x, y, color = "black") {
  ctx.fillStyle = color;
  ctx.font = "12px Arial";
  ctx.fillText(text, x, y);
}

function drawGrid() {
  ctx.clearRect(0, 0, canvas.width, canvas.height);

  ctx.fillStyle = "white";
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  drawBackgroundImage();

  const bottomRight = worldToCanvas(0, 0);
  const topLeft = worldToCanvas(mapConfig.board.height_x_mm, mapConfig.board.width_y_mm);

  const left = Math.min(bottomRight.x, topLeft.x);
  const top = Math.min(bottomRight.y, topLeft.y);
  const w = Math.abs(bottomRight.x - topLeft.x);
  const h = Math.abs(bottomRight.y - topLeft.y);

  ctx.strokeStyle = "black";
  ctx.lineWidth = 3;
  ctx.strokeRect(left, top, w, h);

  ctx.strokeStyle = "#ddd";
  ctx.lineWidth = 1;

  for (let y = 0; y <= mapConfig.board.width_y_mm; y += 100) {
    const p1 = worldToCanvas(0, y);
    const p2 = worldToCanvas(mapConfig.board.height_x_mm, y);

    ctx.beginPath();
    ctx.moveTo(p1.x, p1.y);
    ctx.lineTo(p2.x, p2.y);
    ctx.stroke();
  }

  for (let x = 0; x <= mapConfig.board.height_x_mm; x += 100) {
    const p1 = worldToCanvas(x, 0);
    const p2 = worldToCanvas(x, mapConfig.board.width_y_mm);

    ctx.beginPath();
    ctx.moveTo(p1.x, p1.y);
    ctx.lineTo(p2.x, p2.y);
    ctx.stroke();
  }

  drawText("X=0,Y=0", bottomRight.x - 70, bottomRight.y + 20, "red");
  drawText("Y=3000", topLeft.x + 10, bottomRight.y + 20, "green");
  drawText("X=2000", bottomRight.x - 75, topLeft.y - 10, "red");

  drawText(`Cursor X=${mouseWorld.x} Y=${mouseWorld.y}`, 15, 20, "black");
}

function drawRectWorld(rect, color, label) {
  const p1 = worldToCanvas(rect.x, rect.y);
  const p2 = worldToCanvas(rect.x + rect.w, rect.y + rect.h);

  const left = Math.min(p1.x, p2.x);
  const top = Math.min(p1.y, p2.y);
  const w = Math.abs(p2.x - p1.x);
  const h = Math.abs(p2.y - p1.y);

  ctx.strokeStyle = color;
  ctx.lineWidth = 3;
  ctx.strokeRect(left, top, w, h);

  ctx.fillStyle = "rgba(0,0,0,0.04)";
  ctx.fillRect(left, top, w, h);

  drawText(label, left + w / 2 - 18, top + h / 2 + 4, color);
}

function drawFilledRectWorld(rect, fillColor, strokeColor, label) {
  const p1 = worldToCanvas(rect.x, rect.y);
  const p2 = worldToCanvas(rect.x + rect.w, rect.y + rect.h);

  const left = Math.min(p1.x, p2.x);
  const top = Math.min(p1.y, p2.y);
  const w = Math.abs(p2.x - p1.x);
  const h = Math.abs(p2.y - p1.y);

  ctx.fillStyle = fillColor;
  ctx.fillRect(left, top, w, h);

  ctx.strokeStyle = strokeColor;
  ctx.lineWidth = 3;
  ctx.strokeRect(left, top, w, h);

  drawText(label, left + 8, top + 16, strokeColor);
}


function getRobotInflationRadius() {
  if (!mapConfig || !mapConfig.robot) {
    return 220;
  }

  const robot = mapConfig.robot;

  const lengthX = parseFloat(robot.length_x_mm || 230);
  const widthY = parseFloat(robot.width_y_mm || 250);
  const safety = parseFloat(robot.safety_margin_mm || 50);

  return Math.sqrt(
    Math.pow(lengthX / 2.0, 2) + Math.pow(widthY / 2.0, 2)
  ) + safety;
}


function drawForbiddenZones() {
  if (!mapConfig || !mapConfig.forbidden_zones) {
    return;
  }

  for (const [name, zone] of Object.entries(mapConfig.forbidden_zones)) {
    drawFilledRectWorld(
      zone,
      "rgba(255, 0, 0, 0.35)",
      "rgba(180, 0, 0, 0.95)",
      name
    );
  }
}

function drawTriangleWorld(point, color, label, size = 10) {
  const p = worldToCanvas(point.x, point.y);

  ctx.beginPath();
  ctx.moveTo(p.x, p.y - size);
  ctx.lineTo(p.x - size, p.y + size);
  ctx.lineTo(p.x + size, p.y + size);
  ctx.closePath();

  ctx.fillStyle = color;
  ctx.fill();
  ctx.strokeStyle = "black";
  ctx.stroke();

  drawText(label, p.x + 9, p.y - 9, color);

  if (point.theta_deg !== undefined) {
    drawHeadingArrow(point, color);
  }
}

function drawCircleWorld(point, color, label, radius = 6) {
  const p = worldToCanvas(point.x, point.y);

  ctx.beginPath();
  ctx.arc(p.x, p.y, radius, 0, Math.PI * 2);
  ctx.fillStyle = color;
  ctx.fill();
  ctx.strokeStyle = "black";
  ctx.stroke();

  drawText(label, p.x + 8, p.y - 8, color);
}

function drawHeadingArrow(point, color) {
  const p = worldToCanvas(point.x, point.y);

  const theta = point.theta_deg * Math.PI / 180.0;
  const len = 45;

  const x2 = point.x + Math.cos(theta) * len;
  const y2 = point.y + Math.sin(theta) * len;

  const p2 = worldToCanvas(x2, y2);

  ctx.strokeStyle = color;
  ctx.lineWidth = 3;
  ctx.beginPath();
  ctx.moveTo(p.x, p.y);
  ctx.lineTo(p2.x, p2.y);
  ctx.stroke();
}

function drawRobotPose(pose, color = "orange", label = "ROBOT") {
  if (!pose) return;

  const robot = mapConfig.robot;

  const rect = {
    x: pose.x - robot.length_x_mm / 2,
    y: pose.y - robot.width_y_mm / 2,
    w: robot.length_x_mm,
    h: robot.width_y_mm
  };

  drawRectWorld(rect, color, label);
  drawCircleWorld(pose, color, "R", 6);
  drawHeadingArrow(pose, color);
}

function drawRoute() {
  if (!plannedRoute || !plannedRoute.route || plannedRoute.route.length < 2) {
    return;
  }

  ctx.strokeStyle = "#2563eb";
  ctx.lineWidth = 4;
  ctx.beginPath();

  for (let i = 0; i < plannedRoute.route.length; i++) {
    const p = worldToCanvas(plannedRoute.route[i].x, plannedRoute.route[i].y);

    if (i === 0) {
      ctx.moveTo(p.x, p.y);
    } else {
      ctx.lineTo(p.x, p.y);
    }
  }

  ctx.stroke();

  for (const node of plannedRoute.route) {
    drawCircleWorld(node, "#2563eb", node.name, 5);
  }
}

function drawMap() {
  if (!mapConfig) return;

  drawGrid();

  for (const [name, rect] of Object.entries(mapConfig.almacenes || {})) {
    drawRectWorld(rect, "black", "A" + name);
  }

  for (const [name, rect] of Object.entries(mapConfig.despensas || {})) {
    drawRectWorld(rect, "green", "D" + name);
  }

  for (const [name, point] of Object.entries(mapConfig.approach_almacenes || {})) {
    drawTriangleWorld(point, "blue", "AA" + name);
  }

  for (const [name, point] of Object.entries(mapConfig.approach_despensas || {})) {
    drawTriangleWorld(point, "limegreen", "AD" + name);
  }

  for (const [name, point] of Object.entries(mapConfig.transit || {})) {
    drawTriangleWorld(point, "red", "T" + name);
  }

  drawForbiddenZones();

  drawRoute();

  const start = mapConfig.start;
  drawRobotPose(start, "orange", "START");

  if (robotState && robotState.pose) {
    drawRobotPose(robotState.pose, "#ff8800", "ROBOT");
  }
}

// ============================================================
// MISSION SEQUENCE UI
// ============================================================

function getMissionSequence() {
  if (!mapConfig || !Array.isArray(mapConfig.mission_sequence)) {
    return [];
  }

  return mapConfig.mission_sequence;
}

function missionItemLabel(item, index) {
  const storage = item.storage || `Palm${index + 1}`;
  const pantry = item.pantry || `Desp${index + 1}`;
  return `${index + 1}. ${storage} → ${pantry}`;
}

function getSelectedMissionStorages() {
  const checks = document.querySelectorAll(".mission-storage-check");
  const selected = [];

  checks.forEach(check => {
    if (check.checked) {
      selected.push(check.value);
    }
  });

  return selected;
}

function renderMissionCheckboxes() {
  const container = document.getElementById("missionSequenceList");
  if (!container) return;

  container.innerHTML = "";

  const sequence = getMissionSequence();

  if (sequence.length === 0) {
    container.innerHTML =
      "<p class='hint'>No hay mission_sequence configurada en map_config.json.</p>";
    return;
  }

  sequence.forEach((item, index) => {
    const storage = item.storage || "";
    const pantry = item.pantry || "";

    const row = document.createElement("label");
    row.className = "mission-check-row";

    const check = document.createElement("input");
    check.type = "checkbox";
    check.className = "mission-storage-check";
    check.value = storage;
    check.checked = index === 0;

    const text = document.createElement("span");
    text.textContent = missionItemLabel(item, index);

    row.appendChild(check);
    row.appendChild(text);

    container.appendChild(row);
  });
}

function selectAllMissionStorages() {
  document.querySelectorAll(".mission-storage-check").forEach(check => {
    check.checked = true;
  });
}

function clearMissionStorages() {
  document.querySelectorAll(".mission-storage-check").forEach(check => {
    check.checked = false;
  });
}

function getMissionStorageOptions() {
  return getMissionSequence().map(item => item.storage).filter(Boolean);
}

function refreshTransitMissionControls() {
  const missionStorageSelect = document.getElementById("transitMissionStorage");
  if (!missionStorageSelect) return;

  const sequence = getMissionSequence();
  const values = sequence.map(item => item.storage).filter(Boolean);

  const labels = {};
  sequence.forEach((item, index) => {
    if (item.storage) {
      labels[item.storage] = missionItemLabel(item, index);
    }
  });

  setSelectOptions(missionStorageSelect, values, labels);
}

function setTransitControlsVisible(visible) {
  const box = document.getElementById("transitAssignmentBox");
  if (!box) return;

  box.style.display = visible ? "block" : "none";
}

function getTransitAssignmentPayload() {
  const type = document.getElementById("itemType").value;

  if (type !== "transit") {
    return {};
  }

  const storageSelect = document.getElementById("transitMissionStorage");
  const roleSelect = document.getElementById("transitRole");
  const orderInput = document.getElementById("transitOrder");

  if (!storageSelect || !roleSelect || !orderInput) {
    return {};
  }

  return {
    mission_storage: storageSelect.value,
    transit_role: roleSelect.value,
    transit_order: parseInt(orderInput.value || "999")
  };
}

function formatSequencePlan(plan) {
  if (!plan || !Array.isArray(plan.legs)) {
    return "Misión sin legs.";
  }

  const lines = [];

  lines.push("Misión secuencial preparada");
  lines.push("");
  lines.push("Almacenes seleccionados: " + plan.selected_storages.join(", "));
  lines.push(`Distancia total: ${plan.distance_mm.toFixed(1)} mm`);
  lines.push("");
  lines.push("Rutas:");

  for (const leg of plan.legs) {
    lines.push(`${leg.label}: ${leg.node_names.join(" → ")}`);
  }

  return lines.join("\n");
}

// ============================================================
// SELECTORES
// ============================================================

function getAllNodes() {
  const nodes = ["START"];

  for (const name of Object.keys(mapConfig.approach_almacenes || {})) {
    nodes.push("AA_" + name);
  }

  for (const name of Object.keys(mapConfig.approach_despensas || {})) {
    nodes.push("AD_" + name);
  }

  for (const name of Object.keys(mapConfig.transit || {})) {
    nodes.push("T_" + name);
  }

  return nodes;
}

function setSelectOptions(select, values, labels = null, keepValue = true) {
  if (!select) return;

  const oldValue = select.value;
  select.innerHTML = "";

  for (const value of values) {
    const option = document.createElement("option");
    option.value = value;
    option.textContent = labels && labels[value] ? labels[value] : value;
    select.appendChild(option);
  }

  if (keepValue && values.includes(oldValue)) {
    select.value = oldValue;
  }
}

function refreshSelects() {
  const origin = document.getElementById("originSelect");
  const dest = document.getElementById("destSelect");
  const missionOrigin = document.getElementById("missionOriginSelect");
  const storageSelect = document.getElementById("storageSelect");
  const pantrySelect = document.getElementById("pantrySelect");

  const nodes = getAllNodes();

  setSelectOptions(origin, nodes);
  setSelectOptions(dest, nodes);
  setSelectOptions(missionOrigin, nodes);

  const storages = Object.keys(mapConfig.approach_almacenes || {});
  const pantries = Object.keys(mapConfig.approach_despensas || {});

  const storageLabels = {};
  storages.forEach((name, index) => {
    storageLabels[name] = `Almacén ${index + 1} (${name})`;
  });

  const pantryLabels = {};
  pantries.forEach((name, index) => {
    pantryLabels[name] = `Despensa ${index + 1} (${name})`;
  });

  setSelectOptions(storageSelect, storages, storageLabels);
  setSelectOptions(pantrySelect, pantries, pantryLabels);
  renderMissionCheckboxes();
  refreshTransitMissionControls();
}

// ============================================================
// FORM
// ============================================================

function onTypeChanged() {
  if (!mapConfig) return;

  const type = document.getElementById("itemType").value;
  const wInput = document.getElementById("itemW");
  const hInput = document.getElementById("itemH");
  const thetaInput = document.getElementById("itemTheta");
  setTransitControlsVisible(type === "transit");

  wInput.readOnly = true;
  hInput.readOnly = true;

  if (type === "almacenes") {
    wInput.value = mapConfig.fixed_sizes.almacen.x_size_mm;
    hInput.value = mapConfig.fixed_sizes.almacen.y_size_mm;
    thetaInput.value = "";
    thetaInput.disabled = true;

  } else if (type === "despensas") {
    wInput.value = mapConfig.fixed_sizes.despensa.x_size_mm;
    hInput.value = mapConfig.fixed_sizes.despensa.y_size_mm;
    thetaInput.value = "";
    thetaInput.disabled = true;

  } else if (type === "forbidden_zones") {
    wInput.readOnly = false;
    hInput.readOnly = false;

    if (!wInput.value) {
      wInput.value = 500;
    }

    if (!hInput.value) {
      hInput.value = 700;
    }

    thetaInput.value = "";
    thetaInput.disabled = true;

  } else if (type === "transit") {
    wInput.value = "";
    hInput.value = "";
    thetaInput.value = "";
    thetaInput.disabled = true;

  } else {
    wInput.value = "";
    hInput.value = "";
    thetaInput.disabled = false;
  }
}
// ============================================================
// MAP API ACTIONS
// ============================================================

async function reloadMap() {
  mapConfig = await apiGet("/api/map");
  plannedRoute = null;
  refreshSelects();
  onTypeChanged();
  loadBackgroundFromMap();
  await refreshState();
  drawMap();
  logStatus("Mapa cargado.");
}

async function refreshState() {
  const data = await apiGet("/api/state");

  if (!data.ok) {
    document.getElementById("robotState").textContent = "Sin estado.";
    return;
  }

  robotState = data.state;

  const pose = robotState.pose;

  let text = "";

  if (pose) {
    text += `Pose:\nX=${pose.x}\nY=${pose.y}\nTheta=${pose.theta_deg}\n`;
  } else {
    text += "Pose: sin datos\n";
  }

  text += `\nAcción: ${robotState.last_action || "N/A"}`;

  document.getElementById("robotState").textContent = text;

  if (robotState.roi) {
    document.getElementById("roiState").textContent =
      `Topic: ${robotState.roi.topic}\n` +
      `Pieza en ROI: ${robotState.roi.piece_in_roi}`;
  }

  drawMap();
}

async function saveItem() {
  const type = document.getElementById("itemType").value;
  const name = document.getElementById("itemName").value.trim();

  if (!name) {
    alert("Nombre no válido");
    return;
  }

  const x = parseFloat(document.getElementById("itemX").value);
  const y = parseFloat(document.getElementById("itemY").value);

  if (Number.isNaN(x) || Number.isNaN(y)) {
    alert("X/Y no válidos");
    return;
  }

  let data = { x, y };

  if (type === "almacenes") {
    data.w = mapConfig.fixed_sizes.almacen.x_size_mm;
    data.h = mapConfig.fixed_sizes.almacen.y_size_mm;

  } else if (type === "despensas") {
    data.w = mapConfig.fixed_sizes.despensa.x_size_mm;
    data.h = mapConfig.fixed_sizes.despensa.y_size_mm;

    } else if (type === "forbidden_zones") {
    const w = parseFloat(document.getElementById("itemW").value);
    const h = parseFloat(document.getElementById("itemH").value);

    if (Number.isNaN(w) || Number.isNaN(h) || w <= 0 || h <= 0) {
      alert("Tamaño W/H de zona prohibida no válido");
      return;
    }

    data.w = w;
    data.h = h;

  } else if (type === "transit") {
    // Transit no tiene theta.
  } else {
    const thetaText = document.getElementById("itemTheta").value;
    if (thetaText !== "") {
      data.theta_deg = parseFloat(thetaText);
    }
  }

  const payload = {
    type,
    name,
    data,
    ...getTransitAssignmentPayload()
  };

  const result = await apiPost("/api/item/save", payload);

  if (!result.ok) {
    logStatus("ERROR: " + result.error);
    return;
  }

  await reloadMap();
  logStatus(`Guardado: ${type} / ${name}`);
}

async function deleteItem() {
  const type = document.getElementById("itemType").value;
  const name = document.getElementById("itemName").value.trim();

  if (!name) {
    alert("Nombre no válido");
    return;
  }

  const result = await apiPost("/api/item/delete", { type, name });

  if (!result.ok) {
    logStatus("ERROR: " + result.error);
    return;
  }

  await reloadMap();
  logStatus(`Borrado: ${type} / ${name}`);
}

async function saveForbiddenZone() {
  const name = document.getElementById("forbiddenName").value.trim();

  if (!name) {
    alert("Nombre de zona prohibida no válido");
    return;
  }

  const x = parseFloat(document.getElementById("forbiddenX").value);
  const y = parseFloat(document.getElementById("forbiddenY").value);
  const w = parseFloat(document.getElementById("forbiddenW").value);
  const h = parseFloat(document.getElementById("forbiddenH").value);

  if ([x, y, w, h].some(Number.isNaN)) {
    alert("X/Y/W/H no válidos");
    return;
  }

  if (w <= 0 || h <= 0) {
    alert("La zona prohibida debe tener tamaño positivo");
    return;
  }

  const result = await apiPost("/api/forbidden/save", {
    name,
    x,
    y,
    w,
    h
  });

  if (!result.ok) {
    logStatus("ERROR guardando zona prohibida:\n" + result.error);
    return;
  }

  await reloadMap();
  logStatus(result.message);
}


async function deleteForbiddenZone() {
  const name = document.getElementById("forbiddenName").value.trim();

  if (!name) {
    alert("Nombre de zona prohibida no válido");
    return;
  }

  const result = await apiPost("/api/forbidden/delete", {
    name
  });

  if (!result.ok) {
    logStatus("ERROR borrando zona prohibida:\n" + result.error);
    return;
  }

  await reloadMap();
  logStatus(result.message);
}

async function savePointPosition(type, name, x, y) {
  if (!mapConfig[type] || !mapConfig[type][name]) return;

  const old = mapConfig[type][name];
  old.x = x;
  old.y = y;

  const result = await apiPost("/api/item/save", {
    type,
    name,
    data: old
  });

  if (!result.ok) {
    logStatus("ERROR guardando punto: " + result.error);
    return;
  }

  await reloadMap();
  logStatus(`Punto movido: ${type}/${name}\nX=${x} Y=${y}`);
}

// ============================================================
// PLANIFICACIÓN
// ============================================================

async function planRoute() {
  const origin = document.getElementById("originSelect").value;
  const destination = document.getElementById("destSelect").value;

  const result = await apiPost("/api/plan", { origin, destination });

  if (!result.ok) {
    plannedRoute = null;
    drawMap();
    logStatus("ERROR PLANIFICANDO:\n" + result.error);
    return;
  }

  plannedRoute = result.plan;
  drawMap();

  logStatus(
    "Ruta planificada\n" +
    `Origen: ${origin}\n` +
    `Destino: ${destination}\n` +
    `Distancia: ${plannedRoute.distance_mm.toFixed(1)} mm\n` +
    "Nodos: " + plannedRoute.route.map(p => p.name).join(" → ")
  );
}

async function executeRoute() {
  const origin = document.getElementById("originSelect").value;
  const destination = document.getElementById("destSelect").value;
  const { linearSpeed, angularSpeed } = getMissionSpeeds();

  const result = await apiPost("/api/execute_route", {
    origin,
    destination,
    linear_speed: linearSpeed,
    angular_speed: angularSpeed
  });

  if (!result.ok) {
    logStatus("ERROR EJECUTANDO:\n" + result.error);
    return;
  }

  logStatus(result.message);
}

async function resetRobotStart() {
  const confirmReset = confirm(
    "¿Seguro que el robot físico está colocado en START?\n\n" +
    "Esto solo resetea la pose lógica del mapa."
  );

  if (!confirmReset) {
    return;
  }

  const result = await apiPost("/api/reset_start", {});

  if (!result.ok) {
    logStatus("ERROR RESET START:\n" + result.error);
    return;
  }

  await refreshState();
  drawMap();

  logStatus(result.message);
}

// ============================================================
// MISIÓN AUTOMÁTICA
// ============================================================

async function preparePickPlace() {
  const selectedStorages = getSelectedMissionStorages();
  const { linearSpeed, angularSpeed, slowPickSpeed } = getMissionSpeeds();

  if (selectedStorages.length === 0) {
    alert("Selecciona al menos un almacén para la misión.");
    return;
  }

  const result = await apiPost("/api/sequence_plan", {
    selected_storages: selectedStorages,
    linear_speed: linearSpeed,
    angular_speed: angularSpeed,
    slow_pick_speed: slowPickSpeed
  });

  if (!result.ok) {
    sequencePlan = null;
    plannedRoute = null;
    drawMap();
    logStatus("ERROR PREPARANDO MISIÓN:\n" + result.error);
    return;
  }

  sequencePlan = result.plan;

  // Para pintar algo en azul en el mapa, juntamos todas las legs.
  const fullRoute = [];

  for (const leg of sequencePlan.legs) {
    for (let i = 0; i < leg.route.length; i++) {
      if (fullRoute.length > 0 && i === 0) {
        continue;
      }

      fullRoute.push(leg.route[i]);
    }
  }

  plannedRoute = {
    route: fullRoute,
    distance_mm: sequencePlan.distance_mm
  };

  drawMap();

  logStatus(
    formatSequencePlan(sequencePlan) +
    "\n\n" +
    `Vel. lineal ruta: ${linearSpeed} m/s\n` +
    `Vel. angular: ${angularSpeed} rad/s\n` +
    `Vel. slow pick: ${slowPickSpeed} m/s`
  );
}

async function runPickPlace() {
  const selectedStorages = getSelectedMissionStorages();
  const { linearSpeed, angularSpeed, slowPickSpeed } = getMissionSpeeds();

  if (selectedStorages.length === 0) {
    alert("Selecciona al menos un almacén para la misión.");
    return;
  }

  const confirmRun = confirm(
    "¿Ejecutar misión secuencial completa?\n\n" +
    "Almacenes: " + selectedStorages.join(", ") + "\n" +
    "Vuelta final: START\n" +
    "Vel. lineal ruta: " + linearSpeed + " m/s\n" +
    "Vel. angular: " + angularSpeed + " rad/s\n" +
    "Vel. slow pick: " + slowPickSpeed + " m/s\n\n" +
    "Asegúrate de que:\n" +
    "- El robot está en START si la misión empieza desde START.\n" +
    "- El ROI está bien ajustado.\n" +
    "- Hay pieza con ArUco visible.\n" +
    "- La misión ha sido preparada/validada."
  );

  if (!confirmRun) {
    return;
  }

  const result = await apiPost("/api/start_sequence_mission", {
    selected_storages: selectedStorages,
    linear_speed: linearSpeed,
    angular_speed: angularSpeed,
    slow_pick_speed: slowPickSpeed
  });

  if (!result.ok) {
    logStatus("ERROR MISIÓN SECUENCIAL:\n" + result.error);
    return;
  }

  sequencePlan = result.plan;

  if (sequencePlan) {
    const fullRoute = [];

    for (const leg of sequencePlan.legs) {
      for (let i = 0; i < leg.route.length; i++) {
        if (fullRoute.length > 0 && i === 0) {
          continue;
        }

        fullRoute.push(leg.route[i]);
      }
    }

    plannedRoute = {
      route: fullRoute,
      distance_mm: sequencePlan.distance_mm
    };

    drawMap();
  }

  logStatus(result.message);
}

// ============================================================
// BRAZO / VACUUM
// ============================================================

async function armPlaceholder(action) {
  const result = await apiPost("/api/arm_action", {
    action: action,
    vacuum_mode: "none"
  });

  if (!result.ok) {
    logStatus("ERROR BRAZO:\n" + result.error);
    return;
  }

  logStatus(result.message);
}

async function vacuumAction(action) {
  const result = await apiPost("/api/vacuum_action", {
    action: action
  });

  if (!result.ok) {
    logStatus("ERROR VACUUM:\n" + result.error);
    return;
  }

  document.getElementById("vacuumState").textContent = result.message;
  logStatus(result.message);
}

function downloadMap() {
  logStatus(JSON.stringify(mapConfig, null, 2));
}

// ============================================================
// DRAG MAPA
// ============================================================

function findDraggablePoint(world) {
  const sections = [
    "approach_almacenes",
    "approach_despensas",
    "transit"
  ];

  let best = null;
  let bestDist = Infinity;

  for (const section of sections) {
    for (const [name, point] of Object.entries(mapConfig[section] || {})) {
      const d = Math.hypot(point.x - world.x, point.y - world.y);

      if (d < bestDist && d < 80) {
        bestDist = d;
        best = { type: section, name, point };
      }
    }
  }

  return best;
}

canvas.addEventListener("mousemove", function(event) {
  if (!mapConfig) return;

  const mouse = getCanvasMousePosition(event);
  mouseWorld = clampWorldPoint(canvasToWorld(mouse.x, mouse.y));

  if (dragging) {
    mapConfig[dragging.type][dragging.name].x = mouseWorld.x;
    mapConfig[dragging.type][dragging.name].y = mouseWorld.y;

    document.getElementById("itemX").value = mouseWorld.x;
    document.getElementById("itemY").value = mouseWorld.y;
  }

  drawMap();
});

canvas.addEventListener("mousedown", function(event) {
  if (!mapConfig) return;

  const mouse = getCanvasMousePosition(event);
  const world = clampWorldPoint(canvasToWorld(mouse.x, mouse.y));

  const candidate = findDraggablePoint(world);

  if (!candidate) return;

  dragging = candidate;
  canvas.style.cursor = "grabbing";

  document.getElementById("itemType").value = dragging.type;
  document.getElementById("itemName").value = dragging.name;
  document.getElementById("itemX").value = dragging.point.x;
  document.getElementById("itemY").value = dragging.point.y;

  if (dragging.point.theta_deg !== undefined) {
    document.getElementById("itemTheta").value = dragging.point.theta_deg;
  } else {
    document.getElementById("itemTheta").value = "";
  }

  onTypeChanged();
});

async function finishDrag() {
  if (!dragging) return;

  const p = mapConfig[dragging.type][dragging.name];

  await savePointPosition(
    dragging.type,
    dragging.name,
    Math.round(p.x),
    Math.round(p.y)
  );

  dragging = null;
  canvas.style.cursor = "crosshair";
}

canvas.addEventListener("mouseup", finishDrag);
canvas.addEventListener("mouseleave", finishDrag);

canvas.addEventListener("click", function(event) {
  if (!mapConfig || dragging) return;

  const mouse = getCanvasMousePosition(event);
  const world = clampWorldPoint(canvasToWorld(mouse.x, mouse.y));

  document.getElementById("itemX").value = world.x;
  document.getElementById("itemY").value = world.y;

  const forbiddenX = document.getElementById("forbiddenX");
  const forbiddenY = document.getElementById("forbiddenY");

  if (forbiddenX && forbiddenY) {
    forbiddenX.value = world.x;
    forbiddenY.value = world.y;
  }
});

// ============================================================
// ROI CAMERA
// ============================================================

async function loadROI() {
  const data = await apiGet("/api/roi");

  if (!data.ok) {
    logStatus("ERROR cargando ROI");
    return;
  }

  roiConfig = data.roi_config;

  roiCanvas.width = roiConfig.image_width;
  roiCanvas.height = roiConfig.image_height;

  syncROIInputs();
  drawROI();
}

function syncROIInputs() {
  if (!roiConfig) return;

  const roi = roiConfig.roi;

  document.getElementById("roiX").value = roi.x;
  document.getElementById("roiY").value = roi.y;
  document.getElementById("roiW").value = roi.w;
  document.getElementById("roiH").value = roi.h;

  document.getElementById("roiFooter").textContent =
    `ROI x=${roi.x} y=${roi.y} w=${roi.w} h=${roi.h}`;
}

function clampROI() {
  const roi = roiConfig.roi;
  const maxW = roiConfig.image_width;
  const maxH = roiConfig.image_height;

  roi.x = Math.max(0, Math.min(roi.x, maxW - 1));
  roi.y = Math.max(0, Math.min(roi.y, maxH - 1));
  roi.w = Math.max(5, Math.min(roi.w, maxW - roi.x));
  roi.h = Math.max(5, Math.min(roi.h, maxH - roi.y));
}

function drawROI() {
  if (!roiConfig) return;

  const roi = roiConfig.roi;

  roiCtx.clearRect(0, 0, roiCanvas.width, roiCanvas.height);

  roiCtx.strokeStyle = "lime";
  roiCtx.lineWidth = 3;
  roiCtx.strokeRect(roi.x, roi.y, roi.w, roi.h);

  roiCtx.fillStyle = "rgba(0, 255, 0, 0.12)";
  roiCtx.fillRect(roi.x, roi.y, roi.w, roi.h);

  roiCtx.strokeStyle = "yellow";
  roiCtx.lineWidth = 2;
  roiCtx.beginPath();
  roiCtx.moveTo(roi.x + roi.w / 2 - 10, roi.y + roi.h / 2);
  roiCtx.lineTo(roi.x + roi.w / 2 + 10, roi.y + roi.h / 2);
  roiCtx.moveTo(roi.x + roi.w / 2, roi.y + roi.h / 2 - 10);
  roiCtx.lineTo(roi.x + roi.w / 2, roi.y + roi.h / 2 + 10);
  roiCtx.stroke();

  roiCtx.fillStyle = "lime";
  roiCtx.fillRect(roi.x + roi.w - 10, roi.y + roi.h - 10, 10, 10);

  roiCtx.fillStyle = "lime";
  roiCtx.font = "14px Arial";
  roiCtx.fillText(
    `ROI ${roi.x},${roi.y},${roi.w},${roi.h}`,
    roi.x,
    Math.max(14, roi.y - 6)
  );
}

function getROIMouse(event) {
  const rect = roiCanvas.getBoundingClientRect();

  const scaleX = roiCanvas.width / rect.width;
  const scaleY = roiCanvas.height / rect.height;

  return {
    x: Math.round((event.clientX - rect.left) * scaleX),
    y: Math.round((event.clientY - rect.top) * scaleY)
  };
}

function pointInsideROI(p) {
  const roi = roiConfig.roi;

  return (
    p.x >= roi.x &&
    p.x <= roi.x + roi.w &&
    p.y >= roi.y &&
    p.y <= roi.y + roi.h
  );
}

function pointInsideResizeHandle(p) {
  const roi = roiConfig.roi;

  return (
    p.x >= roi.x + roi.w - 18 &&
    p.x <= roi.x + roi.w + 8 &&
    p.y >= roi.y + roi.h - 18 &&
    p.y <= roi.y + roi.h + 8
  );
}

function updateROIFromInputs() {
  if (!roiConfig) return;

  roiConfig.roi.x = parseInt(document.getElementById("roiX").value || "0");
  roiConfig.roi.y = parseInt(document.getElementById("roiY").value || "0");
  roiConfig.roi.w = parseInt(document.getElementById("roiW").value || "10");
  roiConfig.roi.h = parseInt(document.getElementById("roiH").value || "10");

  clampROI();
  syncROIInputs();
  drawROI();
}

function centerROI() {
  if (!roiConfig) return;

  const roi = roiConfig.roi;

  roi.x = Math.round((roiConfig.image_width - roi.w) / 2);
  roi.y = Math.round((roiConfig.image_height - roi.h) / 2);

  clampROI();
  syncROIInputs();
  drawROI();
}

async function saveROI() {
  if (!roiConfig) return;

  clampROI();

  const result = await apiPost("/api/roi/save", roiConfig);

  if (!result.ok) {
    logStatus("ERROR guardando ROI:\n" + result.error);
    return;
  }

  roiConfig = result.roi_config;
  syncROIInputs();
  drawROI();

  logStatus("ROI guardado correctamente.");
}

roiCanvas.addEventListener("mousedown", function(event) {
  if (!roiConfig) return;

  const p = getROIMouse(event);
  const roi = roiConfig.roi;

  if (pointInsideResizeHandle(p)) {
    resizingROI = true;
    return;
  }

  if (pointInsideROI(p)) {
    draggingROI = true;
    roiDragOffset.x = p.x - roi.x;
    roiDragOffset.y = p.y - roi.y;
  }
});

roiCanvas.addEventListener("mousemove", function(event) {
  if (!roiConfig) return;

  const p = getROIMouse(event);
  const roi = roiConfig.roi;

  if (resizingROI) {
    roi.w = p.x - roi.x;
    roi.h = p.y - roi.y;

    clampROI();
    syncROIInputs();
    drawROI();
    return;
  }

  if (draggingROI) {
    roi.x = p.x - roiDragOffset.x;
    roi.y = p.y - roiDragOffset.y;

    clampROI();
    syncROIInputs();
    drawROI();
    return;
  }

  if (pointInsideResizeHandle(p)) {
    roiCanvas.style.cursor = "nwse-resize";
  } else if (pointInsideROI(p)) {
    roiCanvas.style.cursor = "move";
  } else {
    roiCanvas.style.cursor = "crosshair";
  }
});

roiCanvas.addEventListener("mouseup", function() {
  draggingROI = false;
  resizingROI = false;
});

roiCanvas.addEventListener("mouseleave", function() {
  draggingROI = false;
  resizingROI = false;
});

// ============================================================
// IMU CONFIG
// ============================================================

async function loadIMUConfig() {
  const data = await apiGet("/api/imu_config");

  if (!data.ok) {
    logStatus("ERROR cargando configuración IMU");
    return;
  }

  const cfg = data.imu_config;

  document.getElementById("imuUseTurning").checked = !!cfg.use_imu_turning;
  document.getElementById("imuYawOffset").value = cfg.imu_yaw_offset_deg;
  document.getElementById("imuYawTolerance").value = cfg.imu_yaw_tolerance_deg;
  document.getElementById("imuMinAngularSpeed").value = cfg.min_angular_speed;
  document.getElementById("imuMaxAngularSpeed").value = cfg.max_angular_speed;
  document.getElementById("imuAngularKp").value = cfg.angular_kp;
  document.getElementById("imuAngularInverted").checked = !!cfg.angular_command_inverted;

  document.getElementById("imuState").textContent =
    "Offset=" + cfg.imu_yaw_offset_deg +
    " | Tol=" + cfg.imu_yaw_tolerance_deg +
    " | IMU=" + (cfg.use_imu_turning ? "ON" : "OFF");
}

async function saveIMUConfig() {
  const cfg = {
    use_imu_turning: document.getElementById("imuUseTurning").checked,
    imu_yaw_offset_deg: parseFloat(document.getElementById("imuYawOffset").value || "-11.81"),
    imu_yaw_tolerance_deg: parseFloat(document.getElementById("imuYawTolerance").value || "2.0"),
    min_angular_speed: parseFloat(document.getElementById("imuMinAngularSpeed").value || "0.06"),
    max_angular_speed: parseFloat(document.getElementById("imuMaxAngularSpeed").value || "0.15"),
    angular_kp: parseFloat(document.getElementById("imuAngularKp").value || "0.012"),
    angular_command_inverted: document.getElementById("imuAngularInverted").checked
  };

  const result = await apiPost("/api/imu_config/save", cfg);

  if (!result.ok) {
    logStatus("ERROR guardando IMU:\n" + result.error);
    return;
  }

  document.getElementById("imuState").textContent =
    "IMU guardada. Offset=" + result.imu_config.imu_yaw_offset_deg +
    " | Tol=" + result.imu_config.imu_yaw_tolerance_deg;

  logStatus("Configuración IMU guardada. Se aplicará en la próxima ruta/base_robot.py.");
}

// ============================================================
// INIT
// ============================================================

async function init() {
  await reloadMap();
  await loadROI();
  await loadIMUConfig();
  setTopStatus("Panel listo");
  setInterval(refreshState, 500);
}

init();