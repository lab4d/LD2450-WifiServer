#include <Arduino.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <WiFiManager.h>

namespace {
constexpr uint32_t SensorBaud = 256000;
constexpr uint8_t FrameHeader[] = {0xAA, 0xFF, 0x03, 0x00};
constexpr uint8_t FrameTail[] = {0x55, 0xCC};
constexpr size_t FrameSize = 30;
constexpr uint32_t TargetTimeoutMilliseconds = 500;
constexpr uint8_t MaxTargets = 3;

uint8_t frame[FrameSize];
size_t frameSize = 0;
struct Target {
  int16_t x;
  int16_t y;
  uint16_t distance;
  uint16_t resolution;
};
Target targets[MaxTargets] = {};
uint8_t targetCount = 0;
uint32_t lastTargetTime = 0;
ESP8266WebServer webServer(80);
WiFiManager wifiManager;

bool hasBytes(const uint8_t *bytes, const uint8_t *expected, size_t count) {
  for (size_t index = 0; index < count; ++index) {
    if (bytes[index] != expected[index]) {
      return false;
    }
  }
  return true;
}

int16_t decodeCoordinate(uint8_t lowByte, uint8_t highByte) {
  const uint16_t encoded = static_cast<uint16_t>(lowByte | (highByte << 8));
  const int16_t magnitude = static_cast<int16_t>(encoded & 0x7FFF);
  return (encoded & 0x8000) ? -magnitude : magnitude;
}

void updateTarget() {
  if (!hasBytes(frame, FrameHeader, sizeof(FrameHeader)) ||
      !hasBytes(frame + FrameSize - sizeof(FrameTail), FrameTail, sizeof(FrameTail))) {
    return;
  }
  targetCount = 0;
  for (uint8_t index = 0; index < MaxTargets; ++index) {
    const uint8_t offset = 4 + index * 8;
    targets[index].x = decodeCoordinate(frame[offset], frame[offset + 1]);
    targets[index].y = decodeCoordinate(frame[offset + 2], frame[offset + 3]);
    targets[index].resolution = static_cast<uint16_t>(frame[offset + 6] |
                                                       (frame[offset + 7] << 8));
    targets[index].distance = static_cast<uint16_t>(
        hypotf(static_cast<float>(targets[index].x), static_cast<float>(targets[index].y)));
    if (targets[index].resolution != 0) {
      ++targetCount;
    }
  }
  lastTargetTime = millis();
}

void readSensorFrames() {
  while (Serial.available()) {
    const uint8_t incomingByte = static_cast<uint8_t>(Serial.read());

    if (frameSize < sizeof(FrameHeader)) {
      if (incomingByte == FrameHeader[frameSize]) {
        frame[frameSize++] = incomingByte;
      } else {
        frameSize = incomingByte == FrameHeader[0] ? 1 : 0;
        frame[0] = incomingByte;
      }
    } else if (frameSize < FrameSize) {
      frame[frameSize++] = incomingByte;
    }

    if (frameSize == FrameSize) {
      updateTarget();
      frameSize = 0;
    }
  }

  yield();
}

void handleRoot() {
  static const char page[] PROGMEM = R"HTML(
<!doctype html><html><head><meta name="viewport" content="width=device-width,initial-scale=1">
<title>LD2450 Visualizer</title><style>
 :root{color-scheme:dark}*{box-sizing:border-box}body{margin:0;background:#050607;color:#e9eef0;font:15px system-ui,sans-serif;display:flex;justify-content:center}main{width:min(100%,760px);padding:20px 16px 28px}header{display:flex;align-items:baseline;justify-content:space-between;border-bottom:1px solid #263035;padding:0 2px 14px;margin-bottom:16px}h1{font-size:18px;letter-spacing:.08em;margin:0;color:#9be7c4}.readouts{display:flex;gap:18px;align-items:baseline}#distance,#lateral{font-size:22px;font-variant-numeric:tabular-nums;color:#f4c95d}#lateral{color:#63d7ff}.panel{border:1px solid #263035;background:#080b0c;border-radius:6px;padding:12px;margin-bottom:14px}canvas{display:block;width:100%;height:auto}.label{color:#829197;font-size:12px;letter-spacing:.08em;text-transform:uppercase;margin:0 0 8px}.empty{color:#657177;text-align:center;font-size:13px}.setup{display:block;width:max-content;margin:18px auto 0;padding:9px 16px;border:1px solid #9be7c4;border-radius:4px;color:#9be7c4;text-decoration:none;font-size:13px;letter-spacing:.04em}.setup:hover{background:#10231e}
 </style></head><body><main><header><h1>LD2450 / LIVE RANGE</h1><div class="readouts"><div id="distance">--.-- m</div><div id="lateral">L/R --.-- m</div></div></header>
<section class="panel"><p class="label">Position</p><canvas id="radar" width="720" height="500"></canvas><div id="state" class="empty">Waiting for target</div></section>
<section class="panel"><p class="label">Distance history</p><canvas id="graph" width="720" height="190"></canvas></section>
<a class="setup" href="/setup">Wi-Fi Setup</a>
<script>
const radar=document.getElementById('radar'),graph=document.getElementById('graph'),distance=document.getElementById('distance'),lateral=document.getElementById('lateral'),state=document.getElementById('state'),history=[];
function resize(canvas){const scale=window.devicePixelRatio||1,w=canvas.clientWidth;canvas.width=w*scale;canvas.height=canvas.height/ (canvas.width/(w*scale));return canvas.getContext('2d')}
function drawRadar(d){const c=radar.getContext('2d'),s=devicePixelRatio||1,w=radar.width/s,h=radar.height/s;c.setTransform(s,0,0,s,0,0);c.fillStyle='#080b0c';c.fillRect(0,0,w,h);const cx=w/2,sy=36,max=6,range=Math.min(w*.43,330);c.strokeStyle='#1b3430';c.lineWidth=1;for(let r=1;r<=5;r++){c.beginPath();c.arc(cx,sy,range*r/5,0,Math.PI*2);c.stroke()}c.strokeStyle='#172427';c.beginPath();c.moveTo(cx,sy);c.lineTo(cx,h-18);c.stroke();c.fillStyle='#9be7c4';c.fillRect(cx-25,sy-8,50,8);c.fillRect(cx-4,sy-18,8,10);if(d.detected){const colors=['#f4c95d','#63d7ff','#ff7e9b'];d.targets.forEach((t,i)=>{if(!t.detected)return;const x=cx+(t.x/3)*range,y=sy+Math.min(t.distance,max)/max*(h-70);c.fillStyle=colors[i];c.beginPath();c.arc(x,y,8,0,Math.PI*2);c.fill();c.shadowColor=colors[i];c.shadowBlur=18;c.fill();c.shadowBlur=0})}else{c.fillStyle='#657177';c.beginPath();c.arc(cx,sy+12,3,0,Math.PI*2);c.fill()}}
function drawGraph(){const c=graph.getContext('2d'),s=devicePixelRatio||1,w=graph.width/s,h=graph.height/s;c.setTransform(s,0,0,s,0,0);c.fillStyle='#080b0c';c.fillRect(0,0,w,h);c.strokeStyle='#172427';for(let i=1;i<4;i++){const y=i*h/4;c.beginPath();c.moveTo(0,y);c.lineTo(w,y);c.stroke()}c.strokeStyle='#9be7c4';c.lineWidth=2;c.beginPath();history.forEach((v,i)=>{const x=i/(history.length-1||1)*w,y=h-8-v/6*(h-16);i?c.lineTo(x,y):c.moveTo(x,y)});c.stroke()}
async function update(){try{const d=await (await fetch('/data',{cache:'no-store'})).json();distance.textContent=d.detected?d.distance.toFixed(2)+' m':'--.-- m';lateral.textContent=d.detected?'L/R '+d.x.toFixed(2)+' m':'L/R --.-- m';state.textContent=d.detected?d.count+' target'+(d.count===1?'':'s')+' detected':'Waiting for target';history.push(d.detected?d.distance:0);if(history.length>120)history.shift();drawRadar(d);drawGraph()}catch(e){state.textContent='Connection lost'}}
addEventListener('resize',()=>{drawRadar({detected:false});drawGraph()});drawRadar({detected:false});drawGraph();update();setInterval(update,100);
</script></main></body></html>)HTML";
  webServer.send_P(200, "text/html", page);
}

void handleData() {
  const bool fresh = targetCount > 0 && millis() - lastTargetTime <= TargetTimeoutMilliseconds;
  uint16_t nearestDistance = 0;
  if (fresh) {
    nearestDistance = 65535;
    for (uint8_t index = 0; index < MaxTargets; ++index) {
      if (targets[index].resolution != 0 && targets[index].distance < nearestDistance) {
        nearestDistance = targets[index].distance;
      }
    }
  }
  int16_t nearestX = 0;
  if (fresh) {
    for (uint8_t index = 0; index < MaxTargets; ++index) {
      if (targets[index].resolution != 0 && targets[index].distance == nearestDistance) {
        nearestX = targets[index].x;
        break;
      }
    }
  }
  String response = "{\"detected\":" + String(fresh ? "true" : "false") +
                    ",\"count\":" + String(fresh ? targetCount : 0) +
                    ",\"distance\":" + String(nearestDistance / 1000.0f, 3) +
                    ",\"x\":" + String(nearestX / 1000.0f, 3) +
                    ",\"targets\":[";
  for (uint8_t index = 0; index < MaxTargets; ++index) {
    if (index > 0) {
      response += ",";
    }
    const bool detected = fresh && targets[index].resolution != 0;
    response += "{\"detected\":" + String(detected ? "true" : "false") +
                ",\"distance\":" + String(targets[index].distance / 1000.0f, 3) +
                ",\"x\":" + String(targets[index].x / 1000.0f, 3) +
                ",\"y\":" + String(targets[index].y / 1000.0f, 3) + "}";
  }
  response += "]}";
  webServer.send(200, "application/json", response);
}

void startWebServer() {
  webServer.on("/", handleRoot);
  webServer.on("/data", handleData);
  webServer.on("/setup", []() {
    webServer.stop();
    wifiManager.startConfigPortal("LD2450-Setup");
    ESP.restart();
  });
  webServer.begin();
}
}  // namespace

void setup() {
  Serial.begin(SensorBaud);
  Serial.swap();
  wifiManager.setEnableConfigPortal(false);
  wifiManager.setConnectTimeout(5);
  wifiManager.setWiFiAutoReconnect(true);
  if (!wifiManager.autoConnect()) {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_AP);
    WiFi.softAP("LD2450-Visualizer");
  }
  if (WiFi.status() == WL_CONNECTED) {
    MDNS.begin("ld2450");
  }
  startWebServer();
}

void loop() {
  readSensorFrames();
  webServer.handleClient();
  MDNS.update();
}