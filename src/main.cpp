#include <Arduino.h>
#include <ArduinoJson.h>
#include <DNSServer.h>
#include <EEPROM.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <SoftwareSerial.h>
#include <WiFiManager.h>
#include <WiFiUdp.h>

namespace {
constexpr uint32_t SensorBaud = 256000;
constexpr size_t SensorRxBufferSize = 1024;
// The hardware UART is swapped to the sensor, so logs are bit-banged out of the USB TX pin.
constexpr uint32_t LogBaud = 115200;
constexpr int8_t LogTxPin = 1;
constexpr uint8_t FrameHeader[] = {0xAA, 0xFF, 0x03, 0x00};
constexpr uint8_t FrameTail[] = {0x55, 0xCC};
constexpr size_t FrameSize = 30;
constexpr uint32_t TargetTimeoutMilliseconds = 500;
constexpr uint8_t MaxTargets = 3;

// WLED realtime UDP, DNRGB protocol: [4, timeout, start high, start low, r, g, b, ...].
constexpr uint16_t WledPort = 21324;
constexpr uint8_t WledProtocolDnrgb = 4;
constexpr uint16_t WledMaxLedsPerPacket = 489;
constexpr uint16_t WledMaxLeds = 2048;
constexpr uint8_t WledReleaseTimeout = 0;
constexpr uint8_t WledHandoverTimeoutSeconds = 2;
constexpr uint8_t WledKeepControlTimeout = 255;
constexpr uint32_t WledFrameIntervalMilliseconds = 33;
constexpr uint32_t WledKeepaliveMilliseconds = 1000;
constexpr uint32_t WledIdleRepeatMilliseconds = 200;
constexpr uint8_t WledIdleRepeatPackets = 3;
constexpr uint16_t WledInfoTimeoutMilliseconds = 1500;
constexpr uint32_t FadeMilliseconds = 300;
constexpr uint16_t PreviewLedCount = 100;
constexpr uint16_t PreviewMaxPixels = 300;
constexpr uint32_t ConfigMagic = 0x4C443235;
constexpr uint8_t ConfigVersion = 1;

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
// In access point mode every DNS name resolves to the device, so phones open it as a captive portal.
DNSServer dnsServer;
constexpr uint16_t DnsPort = 53;
bool accessPointMode = false;
SoftwareSerial logSerial;
// Serial is swapped to the sensor, so WiFiManager's debug output goes to the USB log instead.
WiFiManager wifiManager(logSerial);
// The Wi-Fi setup pages are served on the current network in place of the visualizer.
bool wifiSetupRequested = false;
bool wifiSetupActive = false;
bool wifiSetupSaved = false;

struct Color {
  uint8_t r;
  uint8_t g;
  uint8_t b;
};

struct WledConfig {
  uint32_t magic;
  uint8_t version;
  bool enabled;
  bool singleUser;
  bool keepControl;
  bool reverse;
  uint32_t ip;
  uint16_t ledCount;
  char name[33];
  uint16_t corridorStartMm;
  uint16_t corridorEndMm;
  uint8_t stripStartPercent;
  uint8_t stripEndPercent;
  uint8_t widthPercent;
  uint8_t softnessPercent;
  uint8_t brightnessPercent;
  uint8_t smoothing;
  uint16_t holdMilliseconds;
  Color colors[MaxTargets];
  Color background;
};

// Position is the fraction along the configured strip section, already reversed if needed.
struct Track {
  float position;
  float target;
  float level;
  uint32_t lastSeen;
  bool seen;
};

struct FrameGeometry {
  float centers[MaxTargets];
  float levels[MaxTargets];
  float backgroundLevel;
  float halfWidth;
  float edge;
};

WledConfig wledConfig;
Track tracks[MaxTargets] = {};
WiFiUDP wledUdp;
uint8_t wledPacket[4 + WledMaxLedsPerPacket * 3];
uint32_t lastWledTick = 0;
uint32_t lastWledSend = 0;
uint32_t lastWledSignature = 0;
bool wledStreaming = false;
uint8_t wledIdlePackets = WledIdleRepeatPackets;

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

WledConfig defaultWledConfig() {
  WledConfig config = {};
  config.magic = ConfigMagic;
  config.version = ConfigVersion;
  config.singleUser = true;
  config.corridorStartMm = 300;
  config.corridorEndMm = 5000;
  config.stripEndPercent = 100;
  config.widthPercent = 20;
  config.softnessPercent = 50;
  config.brightnessPercent = 100;
  config.smoothing = 30;
  config.holdMilliseconds = 1500;
  config.colors[0] = {255, 154, 60};
  config.colors[1] = {60, 180, 255};
  config.colors[2] = {255, 60, 140};
  return config;
}

void loadWledConfig() {
  EEPROM.begin(sizeof(WledConfig));
  EEPROM.get(0, wledConfig);
  if (wledConfig.magic != ConfigMagic || wledConfig.version != ConfigVersion) {
    wledConfig = defaultWledConfig();
  }
  wledConfig.name[sizeof(wledConfig.name) - 1] = '\0';
}

void saveWledConfig() {
  EEPROM.put(0, wledConfig);
  EEPROM.commit();
}

bool fetchWledInfo(const IPAddress &ip, String &name, uint16_t &ledCount) {
  WiFiClient client;
  HTTPClient http;
  http.setTimeout(WledInfoTimeoutMilliseconds);
  http.useHTTP10(true);
  if (!http.begin(client, "http://" + ip.toString() + "/json/info")) {
    return false;
  }
  if (http.GET() != HTTP_CODE_OK) {
    http.end();
    return false;
  }
  JsonDocument filter;
  filter["name"] = true;
  filter["leds"]["count"] = true;
  JsonDocument info;
  const DeserializationError error =
      deserializeJson(info, http.getStream(), DeserializationOption::Filter(filter));
  http.end();
  if (error) {
    return false;
  }
  name = info["name"] | "WLED";
  ledCount = min<uint16_t>(info["leds"]["count"] | 0, WledMaxLeds);
  return ledCount > 0;
}

// The LD2450 reports the forward axis with its sign bit set, so the magnitude is used.
float corridorFraction(const Target &target) {
  const float along = fabsf(static_cast<float>(target.y));
  const float span = static_cast<float>(wledConfig.corridorEndMm - wledConfig.corridorStartMm);
  const float fraction = constrain((along - wledConfig.corridorStartMm) / span, 0.0f, 1.0f);
  return wledConfig.reverse ? 1.0f - fraction : fraction;
}

void observeTrack(Track &track, const Target &target, uint32_t now) {
  track.target = corridorFraction(target);
  if (track.level <= 0.0f) {
    track.position = track.target;
  }
  track.lastSeen = now;
  track.seen = true;
}

void updateTracks(uint32_t now, uint32_t elapsed) {
  const bool fresh = targetCount > 0 && now - lastTargetTime <= TargetTimeoutMilliseconds;
  if (fresh && wledConfig.singleUser) {
    int8_t nearest = -1;
    for (uint8_t index = 0; index < MaxTargets; ++index) {
      if (targets[index].resolution != 0 &&
          (nearest < 0 || targets[index].distance < targets[nearest].distance)) {
        nearest = static_cast<int8_t>(index);
      }
    }
    if (nearest >= 0) {
      observeTrack(tracks[0], targets[nearest], now);
    }
  } else if (fresh) {
    for (uint8_t index = 0; index < MaxTargets; ++index) {
      if (targets[index].resolution != 0) {
        observeTrack(tracks[index], targets[index], now);
      }
    }
  }

  const float fadeStep = static_cast<float>(elapsed) / FadeMilliseconds;
  const float smoothingMilliseconds = wledConfig.smoothing * 10.0f;
  const float follow = smoothingMilliseconds > 0.0f
                           ? 1.0f - expf(-static_cast<float>(elapsed) / smoothingMilliseconds)
                           : 1.0f;
  for (Track &track : tracks) {
    const bool present = track.seen && now - track.lastSeen <= wledConfig.holdMilliseconds;
    track.level = present ? min(1.0f, track.level + fadeStep) : max(0.0f, track.level - fadeStep);
    if (!present && track.level <= 0.0f) {
      track.seen = false;
    }
    track.position += (track.target - track.position) * follow;
  }
}

bool tracksLit() {
  for (const Track &track : tracks) {
    if (track.level > 0.0f) {
      return true;
    }
  }
  return false;
}

uint32_t trackSignature() {
  uint32_t hash = 2166136261u;
  for (const Track &track : tracks) {
    const uint32_t values[] = {static_cast<uint32_t>(lroundf(track.position * 4096.0f)),
                               static_cast<uint32_t>(lroundf(track.level * 255.0f))};
    for (const uint32_t value : values) {
      hash = (hash ^ value) * 16777619u;
    }
  }
  return hash;
}

FrameGeometry frameGeometry(uint16_t ledCount) {
  FrameGeometry geometry = {};
  const float sectionStart = wledConfig.stripStartPercent * ledCount / 100.0f;
  const float sectionLength =
      (wledConfig.stripEndPercent - wledConfig.stripStartPercent) * ledCount / 100.0f;
  geometry.halfWidth = max(0.5f, wledConfig.widthPercent * ledCount / 200.0f);
  geometry.edge = geometry.halfWidth * wledConfig.softnessPercent / 100.0f;
  for (uint8_t index = 0; index < MaxTargets; ++index) {
    geometry.centers[index] = sectionStart + tracks[index].position * sectionLength;
    geometry.levels[index] = tracks[index].level;
    geometry.backgroundLevel = max(geometry.backgroundLevel, tracks[index].level);
  }
  return geometry;
}

Color renderPixel(const FrameGeometry &geometry, uint16_t index) {
  const Color &background = wledConfig.background;
  float red = background.r * geometry.backgroundLevel;
  float green = background.g * geometry.backgroundLevel;
  float blue = background.b * geometry.backgroundLevel;
  const float led = index + 0.5f;
  for (uint8_t track = 0; track < MaxTargets; ++track) {
    if (geometry.levels[track] <= 0.0f) {
      continue;
    }
    const float distance = fabsf(led - geometry.centers[track]);
    float coverage = 0.0f;
    if (distance <= geometry.halfWidth - geometry.edge) {
      coverage = 1.0f;
    } else if (distance < geometry.halfWidth) {
      coverage = (geometry.halfWidth - distance) / geometry.edge;
    }
    const float amount = coverage * geometry.levels[track];
    const Color &color = wledConfig.colors[track];
    red = max(red, color.r * amount);
    green = max(green, color.g * amount);
    blue = max(blue, color.b * amount);
  }
  const float scale = wledConfig.brightnessPercent / 100.0f;
  return {static_cast<uint8_t>(red * scale + 0.5f), static_cast<uint8_t>(green * scale + 0.5f),
          static_cast<uint8_t>(blue * scale + 0.5f)};
}

void sendWledFrame(bool blank) {
  const IPAddress ip(wledConfig.ip);
  const uint8_t timeout =
      wledConfig.keepControl ? WledKeepControlTimeout : WledHandoverTimeoutSeconds;
  const FrameGeometry geometry = frameGeometry(wledConfig.ledCount);
  for (uint16_t start = 0; start < wledConfig.ledCount; start += WledMaxLedsPerPacket) {
    const uint16_t count = min<uint16_t>(WledMaxLedsPerPacket, wledConfig.ledCount - start);
    wledPacket[0] = WledProtocolDnrgb;
    wledPacket[1] = timeout;
    wledPacket[2] = static_cast<uint8_t>(start >> 8);
    wledPacket[3] = static_cast<uint8_t>(start & 0xFF);
    for (uint16_t offset = 0; offset < count; ++offset) {
      const Color color = blank ? Color{0, 0, 0} : renderPixel(geometry, start + offset);
      wledPacket[4 + offset * 3] = color.r;
      wledPacket[5 + offset * 3] = color.g;
      wledPacket[6 + offset * 3] = color.b;
    }
    wledUdp.beginPacket(ip, WledPort);
    wledUdp.write(wledPacket, 4 + count * 3);
    wledUdp.endPacket();
  }
}

// A zero timeout makes WLED leave realtime mode immediately and resume its own preset.
void sendWledRelease(uint32_t ip) {
  const uint8_t packet[] = {WledProtocolDnrgb, WledReleaseTimeout, 0, 0};
  wledUdp.beginPacket(IPAddress(ip), WledPort);
  wledUdp.write(packet, sizeof(packet));
  wledUdp.endPacket();
}

void serviceWled() {
  const uint32_t now = millis();
  const uint32_t elapsed = now - lastWledTick;
  if (elapsed < WledFrameIntervalMilliseconds) {
    return;
  }
  lastWledTick = now;
  updateTracks(now, elapsed);
  if (!wledConfig.enabled || wledConfig.ip == 0 || wledConfig.ledCount == 0 ||
      WiFi.status() != WL_CONNECTED) {
    return;
  }

  if (tracksLit()) {
    const uint32_t signature = trackSignature();
    if (!wledStreaming || signature != lastWledSignature ||
        now - lastWledSend >= WledKeepaliveMilliseconds) {
      sendWledFrame(false);
      wledStreaming = true;
      lastWledSignature = signature;
      lastWledSend = now;
    }
    wledIdlePackets = WledIdleRepeatPackets;
  } else if (wledIdlePackets > 0 && now - lastWledSend >= WledIdleRepeatMilliseconds) {
    // Nobody present: blank the strip and hold it, or hand it back to WLED. Then go quiet.
    if (wledConfig.keepControl) {
      sendWledFrame(true);
    } else {
      sendWledRelease(wledConfig.ip);
    }
    --wledIdlePackets;
    wledStreaming = false;
    lastWledSend = now;
  }
}

void applyWledConfig(const WledConfig &previous) {
  if (previous.enabled && previous.ip != 0 &&
      (!wledConfig.enabled || previous.ip != wledConfig.ip)) {
    sendWledRelease(previous.ip);
  }
  if (previous.singleUser != wledConfig.singleUser || previous.reverse != wledConfig.reverse) {
    memset(tracks, 0, sizeof(tracks));
  }
  wledStreaming = false;
  wledIdlePackets = WledIdleRepeatPackets;
}

String formatColor(const Color &color) {
  char text[8];
  snprintf(text, sizeof(text), "#%02x%02x%02x", color.r, color.g, color.b);
  return text;
}

void sendJson(int code, const JsonDocument &document) {
  String response;
  serializeJson(document, response);
  webServer.send(code, "application/json", response);
}

long argNumber(const char *name, long minimum, long maximum, long fallback) {
  if (!webServer.hasArg(name)) {
    return fallback;
  }
  return constrain(webServer.arg(name).toInt(), minimum, maximum);
}

// Converts a value entered in metres or seconds to thousandths.
long argThousandths(const char *name, long minimum, long maximum, long fallback) {
  if (!webServer.hasArg(name)) {
    return fallback;
  }
  return constrain(lroundf(webServer.arg(name).toFloat() * 1000.0f), minimum, maximum);
}

bool argFlag(const char *name, bool fallback) {
  return webServer.hasArg(name) ? webServer.arg(name) == "1" : fallback;
}

Color argColor(const char *name, const Color &fallback) {
  const String text = webServer.arg(name);
  if (text.length() != 7 || text[0] != '#') {
    return fallback;
  }
  const uint32_t value = strtoul(text.c_str() + 1, nullptr, 16);
  return {static_cast<uint8_t>(value >> 16), static_cast<uint8_t>(value >> 8),
          static_cast<uint8_t>(value)};
}

void handleRoot() {
  static const char page[] PROGMEM = R"HTML(
<!doctype html><html><head><meta name="viewport" content="width=device-width,initial-scale=1">
<title>LD2450 Visualizer</title><style>
 :root{color-scheme:dark}*{box-sizing:border-box}body{margin:0;background:#050607;color:#e9eef0;font:15px system-ui,sans-serif;display:flex;justify-content:center}main{width:min(100%,760px);padding:20px 16px 28px}header{display:flex;align-items:baseline;justify-content:space-between;border-bottom:1px solid #263035;padding:0 2px 14px;margin-bottom:16px}h1{font-size:18px;letter-spacing:.08em;margin:0;color:#9be7c4}.readouts{display:flex;gap:18px;align-items:baseline}#distance,#lateral{font-size:22px;font-variant-numeric:tabular-nums;color:#f4c95d}#lateral{color:#63d7ff}.panel{border:1px solid #263035;background:#080b0c;border-radius:6px;padding:12px;margin-bottom:14px}canvas{display:block;width:100%;height:auto}.label{color:#829197;font-size:12px;letter-spacing:.08em;text-transform:uppercase;margin:0 0 8px}.empty{color:#657177;text-align:center;font-size:13px}.links{display:flex;justify-content:center;gap:10px;margin-top:18px}.setup{display:block;padding:9px 16px;border:1px solid #9be7c4;border-radius:4px;color:#9be7c4;text-decoration:none;font-size:13px;letter-spacing:.04em}.setup:hover{background:#10231e}
 </style></head><body><main><header><h1>LD2450 / LIVE RANGE</h1><div class="readouts"><div id="distance">--.-- m</div><div id="lateral">L/R --.-- m</div></div></header>
<section class="panel"><p class="label">Position</p><canvas id="radar" width="720" height="500"></canvas><div id="state" class="empty">Waiting for target</div></section>
<section class="panel"><p class="label">Distance history</p><canvas id="graph" width="720" height="190"></canvas></section>
<nav class="links"><a class="setup" href="/wled">WLED Control</a><a class="setup" href="/setup">Wi-Fi Setup</a></nav>
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

void handleWledPage() {
  static const char page[] PROGMEM = R"HTML(
<!doctype html><html><head><meta name="viewport" content="width=device-width,initial-scale=1">
<title>LD2450 WLED Control</title><style>
 :root{color-scheme:dark}*{box-sizing:border-box}[hidden]{display:none!important}body{margin:0;background:#050607;color:#e9eef0;font:15px system-ui,sans-serif;display:flex;justify-content:center}main{width:min(100%,760px);padding:20px 16px 28px}header{display:flex;align-items:baseline;justify-content:space-between;border-bottom:1px solid #263035;padding:0 2px 14px;margin-bottom:16px}h1{font-size:18px;letter-spacing:.08em;margin:0;color:#9be7c4}header a{color:#9be7c4;font-size:13px}.panel{border:1px solid #263035;background:#080b0c;border-radius:6px;padding:12px;margin-bottom:14px}.label{color:#829197;font-size:12px;letter-spacing:.08em;text-transform:uppercase;margin:0 0 10px}.row{display:flex;flex-wrap:wrap;gap:8px;align-items:center;margin-bottom:10px}.grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(190px,1fr));gap:10px 14px}label{display:flex;flex-direction:column;gap:4px;color:#b7c3c7;font-size:13px}label.check{flex-direction:row;align-items:center;gap:8px}input,select,button{font:inherit;color:#e9eef0;background:#0d1214;border:1px solid #263035;border-radius:4px;padding:7px 9px}input[type=color]{padding:2px;height:36px;width:100%}input[type=checkbox]{width:18px;height:18px;margin:0;accent-color:#9be7c4}select,#ip{flex:1;min-width:160px}button{cursor:pointer;border-color:#9be7c4;color:#9be7c4}button:hover{background:#10231e}button.primary{background:#9be7c4;color:#050607}.hint{color:#657177;font-size:12px;margin:6px 0 0}#device{color:#b7c3c7;font-size:13px}canvas{display:block;width:100%;height:28px;border-radius:3px;background:#000}#status{color:#f4c95d;font-size:13px}
 </style></head><body><main><header><h1>LD2450 / WLED</h1><a href="/">Visualizer</a></header>
<section class="panel"><p class="label">Output</p>
<div class="row"><label class="check"><input type="checkbox" id="enabled">Enable WLED control</label></div>
<div class="row"><label class="check"><input type="checkbox" id="singleUser">Single User (follow the nearest person only)</label></div>
<div class="row"><label class="check"><input type="checkbox" id="keepControl">Keep control when nobody is present</label></div>
<p class="hint">Off: WLED returns to its own preset when nobody is present. On: the strip stays dark until someone appears.</p></section>
<section class="panel"><p class="label">Device</p>
<div class="row"><select id="devices"><option value="">Press Scan to find WLED devices</option></select><button id="scan" type="button">Scan</button></div>
<div class="row"><input id="ip" placeholder="WLED IP address"><button id="check" type="button">Check</button></div>
<div id="device">No device selected</div></section>
<section class="panel"><p class="label">Mapping</p><div class="grid">
<label>Corridor start (m)<input type="number" id="corridorStart" min="0" max="8" step="0.05"></label>
<label>Corridor end (m)<input type="number" id="corridorEnd" min="0.1" max="8" step="0.05"></label>
<label>Strip start (%)<input type="number" id="stripStart" min="0" max="99"></label>
<label>Strip end (%)<input type="number" id="stripEnd" min="1" max="100"></label>
<label class="check"><input type="checkbox" id="reverse">Reverse direction</label></div>
<p class="hint">Distances from the sensor between corridor start and end are mapped onto the strip section between strip start and end.</p></section>
<section class="panel"><p class="label">Light</p><div class="grid">
<label>Lit width (% of strip)<input type="number" id="width" min="1" max="100"></label>
<label>Edge softness (%)<input type="number" id="softness" min="0" max="100"></label>
<label>Brightness (%)<input type="number" id="brightness" min="1" max="100"></label>
<label>Smoothing (0-100)<input type="number" id="smoothing" min="0" max="100"></label>
<label>Hold time (s)<input type="number" id="hold" min="0.2" max="60" step="0.1"></label>
<label>Person 1 color<input type="color" id="color0"></label>
<label class="multi">Person 2 color<input type="color" id="color1"></label>
<label class="multi">Person 3 color<input type="color" id="color2"></label>
<label>Background<input type="color" id="background"></label></div>
<p class="hint">The background lights the rest of the strip while someone is present. Black turns it off.</p></section>
<section class="panel"><p class="label">Preview</p><canvas id="preview" width="720" height="28"></canvas></section>
<div class="row"><button class="primary" id="save" type="button">Save</button><span id="status"></span></div>
<script>
const $=id=>document.getElementById(id),fields=['corridorStart','corridorEnd','stripStart','stripEnd','width','softness','brightness','smoothing','hold','color0','color1','color2','background'],checks=['enabled','singleUser','keepControl','reverse'];let device={ip:'',name:'',leds:0},found=[];
function status(t){$('status').textContent=t}
function showDevice(){$('device').textContent=device.ip?(device.name||'WLED')+' at '+device.ip+', '+device.leds+' LEDs':'No device selected';$('ip').value=device.ip}
function multi(){document.querySelectorAll('.multi').forEach(e=>e.hidden=$('singleUser').checked)}
async function load(){const c=await (await fetch('/wled/config',{cache:'no-store'})).json();fields.forEach(f=>$(f).value=c[f]);checks.forEach(f=>$(f).checked=c[f]);device={ip:c.ip,name:c.name,leds:c.leds};showDevice();multi()}
$('singleUser').onchange=multi;
$('scan').onclick=async()=>{status('Scanning...');try{found=(await (await fetch('/wled/scan',{cache:'no-store'})).json()).devices;const s=$('devices');s.innerHTML='';if(!found.length)s.add(new Option('No WLED devices found',''));found.forEach((x,i)=>s.add(new Option(x.name+' ('+x.ip+', '+x.leds+' LEDs)',i)));if(found.length){device=found[0];showDevice()}status(found.length+' found')}catch(e){status('Scan failed')}};
$('devices').onchange=e=>{const x=found[e.target.value];if(x){device=x;showDevice()}};
$('check').onclick=async()=>{status('Checking...');try{const r=await fetch('/wled/info?ip='+encodeURIComponent($('ip').value.trim()),{cache:'no-store'});if(!r.ok)throw 0;device=await r.json();showDevice();status('Device found')}catch(e){status('No WLED device at that address')}};
$('save').onclick=async()=>{const p=new URLSearchParams(),ip=$('ip').value.trim();fields.forEach(f=>p.set(f,$(f).value));checks.forEach(f=>p.set(f,$(f).checked?'1':'0'));p.set('ip',ip);p.set('name',ip===device.ip?device.name:'');p.set('leds',ip===device.ip?device.leds:0);status('Saving...');try{const r=await fetch('/wled/config',{method:'POST',body:p});if(!r.ok)throw 0;await load();status(device.ip&&!device.leds?'Saved, but the device did not answer':'Saved')}catch(e){status('Save failed')}};
async function preview(){try{const d=await (await fetch('/wled/preview',{cache:'no-store'})).json(),c=$('preview'),x=c.getContext('2d'),n=d.px.length/6,w=c.width/n;for(let i=0;i<n;i++){x.fillStyle='#'+d.px.substr(i*6,6);x.fillRect(Math.floor(i*w),0,Math.ceil(w),c.height)}}catch(e){}setTimeout(preview,100)}
load().catch(()=>status('Connection lost'));preview();
</script></main></body></html>)HTML";
  webServer.send_P(200, "text/html", page);
}

void handleWledConfigGet() {
  JsonDocument document;
  document["enabled"] = wledConfig.enabled;
  document["singleUser"] = wledConfig.singleUser;
  document["keepControl"] = wledConfig.keepControl;
  document["reverse"] = wledConfig.reverse;
  document["ip"] = wledConfig.ip != 0 ? IPAddress(wledConfig.ip).toString() : String();
  document["name"] = wledConfig.name;
  document["leds"] = wledConfig.ledCount;
  document["corridorStart"] = serialized(String(wledConfig.corridorStartMm / 1000.0f, 2));
  document["corridorEnd"] = serialized(String(wledConfig.corridorEndMm / 1000.0f, 2));
  document["stripStart"] = wledConfig.stripStartPercent;
  document["stripEnd"] = wledConfig.stripEndPercent;
  document["width"] = wledConfig.widthPercent;
  document["softness"] = wledConfig.softnessPercent;
  document["brightness"] = wledConfig.brightnessPercent;
  document["smoothing"] = wledConfig.smoothing;
  document["hold"] = serialized(String(wledConfig.holdMilliseconds / 1000.0f, 1));
  for (uint8_t index = 0; index < MaxTargets; ++index) {
    document["color" + String(index)] = formatColor(wledConfig.colors[index]);
  }
  document["background"] = formatColor(wledConfig.background);
  sendJson(200, document);
}

void handleWledConfigPost() {
  const WledConfig previous = wledConfig;
  WledConfig next = wledConfig;
  next.enabled = argFlag("enabled", next.enabled);
  next.singleUser = argFlag("singleUser", next.singleUser);
  next.keepControl = argFlag("keepControl", next.keepControl);
  next.reverse = argFlag("reverse", next.reverse);
  next.corridorStartMm = argThousandths("corridorStart", 0, 8000, next.corridorStartMm);
  next.corridorEndMm = argThousandths("corridorEnd", 100, 8000, next.corridorEndMm);
  if (next.corridorEndMm < next.corridorStartMm + 100) {
    next.corridorEndMm = next.corridorStartMm + 100;
  }
  next.stripStartPercent = argNumber("stripStart", 0, 99, next.stripStartPercent);
  next.stripEndPercent = argNumber("stripEnd", 1, 100, next.stripEndPercent);
  if (next.stripEndPercent <= next.stripStartPercent) {
    next.stripEndPercent = next.stripStartPercent + 1;
  }
  next.widthPercent = argNumber("width", 1, 100, next.widthPercent);
  next.softnessPercent = argNumber("softness", 0, 100, next.softnessPercent);
  next.brightnessPercent = argNumber("brightness", 1, 100, next.brightnessPercent);
  next.smoothing = argNumber("smoothing", 0, 100, next.smoothing);
  next.holdMilliseconds = argThousandths("hold", 200, 60000, next.holdMilliseconds);
  for (uint8_t index = 0; index < MaxTargets; ++index) {
    next.colors[index] = argColor(("color" + String(index)).c_str(), next.colors[index]);
  }
  next.background = argColor("background", next.background);

  IPAddress ip;
  if (ip.fromString(webServer.arg("ip"))) {
    String name = webServer.arg("name");
    uint16_t ledCount = argNumber("leds", 0, WledMaxLeds, 0);
    fetchWledInfo(ip, name, ledCount);
    next.ip = static_cast<uint32_t>(ip);
    next.ledCount = ledCount;
    strlcpy(next.name, name.c_str(), sizeof(next.name));
  } else {
    next.ip = 0;
    next.ledCount = 0;
    next.name[0] = '\0';
  }

  wledConfig = next;
  saveWledConfig();
  logSerial.printf("WLED config saved: enabled=%d ip=%s leds=%u\n",wledConfig.enabled,
                   IPAddress(wledConfig.ip).toString().c_str(), wledConfig.ledCount);
  applyWledConfig(previous);
  handleWledConfigGet();
}

void handleWledInfo() {
  IPAddress ip;
  String name;
  uint16_t ledCount = 0;
  if (!ip.fromString(webServer.arg("ip")) || !fetchWledInfo(ip, name, ledCount)) {
    webServer.send(404, "application/json", "{}");
    return;
  }
  JsonDocument document;
  document["ip"] = ip.toString();
  document["name"] = name;
  document["leds"] = ledCount;
  sendJson(200, document);
}

// WLED advertises itself over mDNS as _wled._tcp. The query blocks for about a second.
void handleWledScan() {
  JsonDocument document;
  JsonArray devices = document["devices"].to<JsonArray>();
  if (WiFi.status() == WL_CONNECTED) {
    const uint32_t answers = MDNS.queryService("wled", "tcp");
    for (uint32_t index = 0; index < answers; ++index) {
      const IPAddress ip = MDNS.answerIP(index);
      String name;
      uint16_t ledCount = 0;
      if (fetchWledInfo(ip, name, ledCount)) {
        JsonObject device = devices.add<JsonObject>();
        device["ip"] = ip.toString();
        device["name"] = name;
        device["leds"] = ledCount;
      }
    }
    MDNS.removeQuery();
    logSerial.printf("WLED scan: %u answers, %u devices\n",answers, devices.size());
  }
  sendJson(200, document);
}

void handleWledPreview() {
  const uint16_t ledCount = wledConfig.ledCount != 0 ? wledConfig.ledCount : PreviewLedCount;
  const uint16_t pixels = min(ledCount, PreviewMaxPixels);
  const FrameGeometry geometry = frameGeometry(ledCount);
  String response = "{\"leds\":" + String(ledCount) + ",\"px\":\"";
  response.reserve(response.length() + pixels * 6 + 2);
  char hex[7];
  for (uint16_t index = 0; index < pixels; ++index) {
    const Color color =
        renderPixel(geometry, static_cast<uint32_t>(index) * ledCount / pixels);
    snprintf(hex, sizeof(hex), "%02x%02x%02x", color.r, color.g, color.b);
    response += hex;
  }
  response += "\"}";
  webServer.send(200, "application/json", response);
}

void startWebServer() {
  webServer.on("/", handleRoot);
  webServer.on("/data", handleData);
  webServer.on("/wled", handleWledPage);
  webServer.on("/wled/config", HTTP_GET, handleWledConfigGet);
  webServer.on("/wled/config", HTTP_POST, handleWledConfigPost);
  webServer.on("/wled/info", handleWledInfo);
  webServer.on("/wled/scan", handleWledScan);
  webServer.on("/wled/preview", handleWledPreview);
  // The page reloads once the setup pages have taken over port 80.
  webServer.on("/setup", []() {
    webServer.send(200, "text/html",
                   "<!doctype html><meta http-equiv=\"refresh\" content=\"2;url=/\">"
                   "<body style=\"background:#050607;color:#e9eef0;font:15px system-ui,sans-serif\">"
                   "Opening Wi-Fi setup...");
    wifiSetupRequested = true;
  });
  // Captive portal probes (Android /generate_204 and similar) land here and get sent to the page.
  webServer.onNotFound([]() {
    if (!accessPointMode) {
      webServer.send(404, "text/plain", "Not found");
      return;
    }
    webServer.sendHeader("Location", "http://" + WiFi.softAPIP().toString() + "/", true);
    webServer.send(302, "text/plain", "");
  });
  // Logs every request except the pages' frequent polling.
  webServer.addHook([](const String &method, const String &url, WiFiClient *client,
                       ESP8266WebServer::ContentTypeFunction) {
    if (url != "/data" && url != "/wled/preview") {
      logSerial.printf("HTTP %s %s from %s\n", method.c_str(), url.c_str(),
                       client->remoteIP().toString().c_str());
    }
    return ESP8266WebServer::CLIENT_REQUEST_CAN_CONTINUE;
  });
  webServer.begin();
  logSerial.println("Web server started");
}

void startWifiSetup() {
  wifiSetupRequested = false;
  webServer.stop();
  wifiManager.startWebPortal();
  wifiSetupActive = true;
  const IPAddress address = accessPointMode ? WiFi.softAPIP() : WiFi.localIP();
  logSerial.printf("Wi-Fi setup at http://%s\n", address.toString().c_str());
}

// Restarts after credentials are saved, whether or not they connected, or when setup is exited.
// A failed connection falls back to the access point on the next boot.
void serviceWifiSetup() {
  wifiManager.process();
  if (wifiSetupSaved || !wifiManager.getWebPortalActive()) {
    logSerial.println("Wi-Fi setup finished, restarting");
    ESP.restart();
  }
}
}  // namespace

void setup() {
  Serial.setRxBufferSize(SensorRxBufferSize);
  Serial.begin(SensorBaud);
  Serial.swap();
  logSerial.begin(LogBaud, SWSERIAL_8N1, -1, LogTxPin);
  // Wi-Fi interrupts otherwise break the bit timing and garble the log.
  logSerial.enableIntTx(false);
  logSerial.printf("\nLD2450 boot, reset reason: %s\n",ESP.getResetReason().c_str());
  loadWledConfig();
  // Modem sleep makes the device miss ARP requests, so new clients such as phones cannot reach it.
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  wifiManager.setEnableConfigPortal(false);
  wifiManager.setConnectTimeout(5);
  wifiManager.setWiFiAutoReconnect(true);
  wifiManager.setBreakAfterConfig(true);
  wifiManager.setSaveConfigCallback([]() { wifiSetupSaved = true; });
  if (!wifiManager.autoConnect()) {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_AP);
    WiFi.softAP("LD2450-Visualizer");
    dnsServer.start(DnsPort, "*", WiFi.softAPIP());
    accessPointMode = true;
    logSerial.printf("No saved Wi-Fi, access point LD2450-Visualizer at http://%s\n",
                     WiFi.softAPIP().toString().c_str());
  }
  if (WiFi.status() == WL_CONNECTED) {
    logSerial.printf("Connected to %s, http://%s or http://ld2450.local\n", WiFi.SSID().c_str(),
                     WiFi.localIP().toString().c_str());
    MDNS.begin("ld2450");
    String name;
    uint16_t ledCount = 0;
    if (wledConfig.enabled && wledConfig.ip != 0 &&
        fetchWledInfo(IPAddress(wledConfig.ip), name, ledCount) &&
        ledCount != wledConfig.ledCount) {
      wledConfig.ledCount = ledCount;
      saveWledConfig();
    }
    if (wledConfig.enabled) {
      logSerial.printf("WLED %s, %u LEDs\n", IPAddress(wledConfig.ip).toString().c_str(),
                       wledConfig.ledCount);
    }
  }
  startWebServer();
}

void loop() {
  readSensorFrames();
  if (accessPointMode) {
    dnsServer.processNextRequest();
  }
  if (wifiSetupRequested) {
    startWifiSetup();
  }
  if (wifiSetupActive) {
    serviceWifiSetup();
  } else {
    webServer.handleClient();
  }
  MDNS.update();
  serviceWled();
}
