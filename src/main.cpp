#include <Arduino.h>
#include <ArduinoJson.h>
#include <DNSServer.h>
#include <EEPROM.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <NeoPixelBus.h>
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
constexpr char AccessPointName[] = "LD2450-Visualizer";
constexpr char DefaultHostname[] = "presence";

// WLED realtime UDP, DNRGB protocol: [4, timeout, start high, start low, r, g, b, ...].
constexpr uint16_t WledPort = 21324;
constexpr uint8_t WledProtocolDnrgb = 4;
constexpr uint16_t WledMaxLedsPerPacket = 489;
constexpr uint16_t WledMaxLeds = 2048;
constexpr uint8_t WledReleaseTimeout = 0;
constexpr uint8_t WledHandoverTimeoutSeconds = 2;
constexpr uint8_t WledKeepControlTimeout = 255;
constexpr uint32_t WledKeepaliveMilliseconds = 1000;
constexpr uint32_t WledIdleRepeatMilliseconds = 200;
constexpr uint8_t WledIdleRepeatPackets = 3;
constexpr uint16_t WledInfoTimeoutMilliseconds = 1500;

// The connected strip can only use the ESP8266's hardware-timed outputs, so sending a frame
// never blocks interrupts and the sensor UART keeps receiving.
constexpr uint8_t StripPinRx = 0;  // GPIO3, I2S DMA
constexpr uint8_t StripPinD4 = 1;  // GPIO2, UART1
constexpr uint16_t StripMaxLeds = 600;
// Wire order per colour order setting (GRB, RGB, BRG, RBG, GBR, BGR), as indexes into {r, g, b}.
constexpr uint8_t ColorOrders[][3] = {{1, 0, 2}, {0, 1, 2}, {2, 0, 1},
                                      {0, 2, 1}, {1, 2, 0}, {2, 1, 0}};
constexpr uint8_t ColorOrderCount = sizeof(ColorOrders) / sizeof(ColorOrders[0]);

constexpr uint32_t OutputFrameIntervalMilliseconds = 33;
// LEDs are linear while colours are picked on a gamma-encoded screen; 2.8 matches WLED.
constexpr float LedGamma = 2.8f;
constexpr uint16_t PreviewLedCount = 100;
constexpr uint16_t PreviewMaxPixels = 300;
constexpr uint32_t RestartDelayMilliseconds = 300;
// The configuration page keeps the test light on while it is open and checked.
constexpr uint32_t TestLightTimeoutMilliseconds = 30000;
constexpr uint32_t ConfigMagic = 0x4C443235;
constexpr uint8_t ConfigVersion = 2;

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
uint16_t sensorFrameCount = 0;
uint16_t sensorFrameRate = 0;
uint32_t lastSensorRateTime = 0;
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
bool restartRequested = false;
uint32_t restartRequestTime = 0;

struct Color {
  uint8_t r;
  uint8_t g;
  uint8_t b;
};

// Mapping and light settings of one strip output.
struct OutputConfig {
  uint16_t corridorStartMm;
  uint16_t corridorEndMm;
  uint8_t stripStartPercent;
  uint8_t stripEndPercent;
  uint8_t widthPercent;
  uint8_t softnessPercent;
  uint8_t brightnessPercent;
  bool reverse;
  Color colors[MaxTargets];
  Color background;
};

struct Settings {
  uint32_t magic;
  uint8_t version;
  bool singleUser;
  uint8_t smoothing;
  uint16_t holdMilliseconds;
  uint16_t fadeMilliseconds;
  char hostname[32];
  bool stripEnabled;
  uint8_t stripPin;
  uint8_t stripColorOrder;
  uint16_t stripLedCount;
  OutputConfig strip;
  bool wledEnabled;
  bool wledKeepControl;
  bool wledSync;
  uint32_t wledIp;
  uint16_t wledLedCount;
  char wledName[33];
  OutputConfig wled;
};

// Distance is the smoothed distance from the sensor along the corridor, in millimetres.
struct Track {
  float distance;
  float target;
  float level;
  uint32_t lastSeen;
  bool seen;
};

struct FrameGeometry {
  const OutputConfig *output;
  float centers[MaxTargets];
  float levels[MaxTargets];
  float backgroundLevel;
  float halfWidth;
  float edge;
};

Settings settings;
// Set when the configuration page applied changes live that are not stored in flash yet.
bool settingsUnsaved = false;
bool testLight = false;
uint32_t testLightTime = 0;
Track tracks[MaxTargets] = {};
uint32_t lastOutputTick = 0;
uint8_t gammaTable[256];

WiFiUDP wledUdp;
uint8_t wledPacket[4 + WledMaxLedsPerPacket * 3];
uint32_t lastWledSend = 0;
uint32_t lastWledSignature = 0;
bool wledStreaming = false;
uint8_t wledIdlePackets = WledIdleRepeatPackets;

using DmaStrip = NeoPixelBus<NeoRgbFeature, NeoEsp8266DmaWs2812xMethod>;
using Uart1Strip = NeoPixelBus<NeoRgbFeature, NeoEsp8266Uart1Ws2812xMethod>;
DmaStrip *dmaStrip = nullptr;
Uart1Strip *uart1Strip = nullptr;
uint32_t lastStripSignature = 0;
bool stripShowingLight = false;
bool stripDirty = true;

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
  ++sensorFrameCount;
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

void updateSensorRate() {
  const uint32_t now = millis();
  if (now - lastSensorRateTime >= 1000) {
    sensorFrameRate = sensorFrameCount;
    sensorFrameCount = 0;
    lastSensorRateTime = now;
  }
}

bool targetsFresh() {
  return targetCount > 0 && millis() - lastTargetTime <= TargetTimeoutMilliseconds;
}

OutputConfig defaultOutputConfig() {
  OutputConfig output = {};
  output.corridorStartMm = 300;
  output.corridorEndMm = 5000;
  output.stripEndPercent = 100;
  output.widthPercent = 20;
  output.softnessPercent = 50;
  output.brightnessPercent = 100;
  output.colors[0] = {255, 154, 60};
  output.colors[1] = {60, 180, 255};
  output.colors[2] = {255, 60, 140};
  return output;
}

Settings defaultSettings() {
  Settings defaults = {};
  defaults.magic = ConfigMagic;
  defaults.version = ConfigVersion;
  defaults.singleUser = true;
  defaults.smoothing = 30;
  defaults.holdMilliseconds = 1500;
  defaults.fadeMilliseconds = 300;
  strlcpy(defaults.hostname, DefaultHostname, sizeof(defaults.hostname));
  defaults.stripPin = StripPinRx;
  defaults.stripLedCount = 60;
  defaults.strip = defaultOutputConfig();
  defaults.wledSync = true;
  defaults.wled = defaultOutputConfig();
  return defaults;
}

void loadSettings() {
  EEPROM.get(0, settings);
  if (settings.magic != ConfigMagic || settings.version != ConfigVersion) {
    settings = defaultSettings();
  }
  settings.hostname[sizeof(settings.hostname) - 1] = '\0';
  settings.wledName[sizeof(settings.wledName) - 1] = '\0';
  settings.stripPin = min(settings.stripPin, StripPinD4);
  settings.stripColorOrder = min<uint8_t>(settings.stripColorOrder, ColorOrderCount - 1);
  settings.stripLedCount = constrain(settings.stripLedCount, 1, StripMaxLeds);
  settings.fadeMilliseconds = max<uint16_t>(settings.fadeMilliseconds, 100);
}

void saveSettings() {
  EEPROM.put(0, settings);
  EEPROM.commit();
}

// With Sync on, WLED mirrors the connected strip's settings but keeps its own direction,
// because strips on opposite walls are often wired from opposite ends.
OutputConfig wledOutput() {
  if (!settings.wledSync) {
    return settings.wled;
  }
  OutputConfig output = settings.strip;
  output.reverse = settings.wled.reverse;
  return output;
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

float corridorFraction(const OutputConfig &output, float distance) {
  const float span = static_cast<float>(output.corridorEndMm - output.corridorStartMm);
  const float fraction = constrain((distance - output.corridorStartMm) / span, 0.0f, 1.0f);
  return output.reverse ? 1.0f - fraction : fraction;
}

void observeTrack(Track &track, float distance, uint32_t now) {
  track.target = distance;
  if (track.level <= 0.0f) {
    track.distance = track.target;
  }
  track.lastSeen = now;
  track.seen = true;
}

// The LD2450 reports the forward axis with its sign bit set, so the magnitude is used.
float targetDistance(const Target &target) {
  return fabsf(static_cast<float>(target.y));
}

void updateTracks(uint32_t now, uint32_t elapsed) {
  if (testLight && now - testLightTime > TestLightTimeoutMilliseconds) {
    testLight = false;
  }
  const bool fresh = targetsFresh();
  if (testLight) {
    // Stands in for a person in the middle of the corridor while settings are adjusted.
    const OutputConfig &reference =
        settings.stripEnabled || settings.wledSync ? settings.strip : settings.wled;
    observeTrack(tracks[0], (reference.corridorStartMm + reference.corridorEndMm) / 2.0f, now);
  } else if (fresh && settings.singleUser) {
    int8_t nearest = -1;
    for (uint8_t index = 0; index < MaxTargets; ++index) {
      if (targets[index].resolution != 0 &&
          (nearest < 0 || targets[index].distance < targets[nearest].distance)) {
        nearest = static_cast<int8_t>(index);
      }
    }
    if (nearest >= 0) {
      observeTrack(tracks[0], targetDistance(targets[nearest]), now);
    }
  } else if (fresh) {
    for (uint8_t index = 0; index < MaxTargets; ++index) {
      if (targets[index].resolution != 0) {
        observeTrack(tracks[index], targetDistance(targets[index]), now);
      }
    }
  }

  const float fadeStep = static_cast<float>(elapsed) / settings.fadeMilliseconds;
  const float smoothingMilliseconds = settings.smoothing * 10.0f;
  const float follow = smoothingMilliseconds > 0.0f
                           ? 1.0f - expf(-static_cast<float>(elapsed) / smoothingMilliseconds)
                           : 1.0f;
  for (Track &track : tracks) {
    const bool present = track.seen && now - track.lastSeen <= settings.holdMilliseconds;
    track.level = present ? min(1.0f, track.level + fadeStep) : max(0.0f, track.level - fadeStep);
    if (!present && track.level <= 0.0f) {
      track.seen = false;
    }
    track.distance += (track.target - track.distance) * follow;
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
    const uint32_t values[] = {static_cast<uint32_t>(lroundf(track.distance * 0.5f)),
                               static_cast<uint32_t>(lroundf(track.level * 255.0f))};
    for (const uint32_t value : values) {
      hash = (hash ^ value) * 16777619u;
    }
  }
  return hash;
}

FrameGeometry frameGeometry(const OutputConfig &output, uint16_t ledCount) {
  FrameGeometry geometry = {};
  geometry.output = &output;
  const float sectionStart = output.stripStartPercent * ledCount / 100.0f;
  const float sectionLength =
      (output.stripEndPercent - output.stripStartPercent) * ledCount / 100.0f;
  geometry.halfWidth = max(0.5f, output.widthPercent * ledCount / 200.0f);
  geometry.edge = geometry.halfWidth * output.softnessPercent / 100.0f;
  for (uint8_t index = 0; index < MaxTargets; ++index) {
    geometry.centers[index] =
        sectionStart + corridorFraction(output, tracks[index].distance) * sectionLength;
    geometry.levels[index] = tracks[index].level;
    geometry.backgroundLevel = max(geometry.backgroundLevel, tracks[index].level);
  }
  return geometry;
}

void buildGammaTable() {
  for (uint16_t value = 0; value < 256; ++value) {
    gammaTable[value] = static_cast<uint8_t>(powf(value / 255.0f, LedGamma) * 255.0f + 0.5f);
  }
}

// Previews show the colour as picked; the strips get it gamma corrected so they look the same.
Color ledColor(const Color &color) {
  return {gammaTable[color.r], gammaTable[color.g], gammaTable[color.b]};
}

Color renderPixel(const FrameGeometry &geometry, uint16_t index) {
  const OutputConfig &output = *geometry.output;
  float red = output.background.r * geometry.backgroundLevel;
  float green = output.background.g * geometry.backgroundLevel;
  float blue = output.background.b * geometry.backgroundLevel;
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
    const Color &color = output.colors[track];
    red = max(red, color.r * amount);
    green = max(green, color.g * amount);
    blue = max(blue, color.b * amount);
  }
  const float scale = output.brightnessPercent / 100.0f;
  return {static_cast<uint8_t>(red * scale + 0.5f), static_cast<uint8_t>(green * scale + 0.5f),
          static_cast<uint8_t>(blue * scale + 0.5f)};
}

void endLocalStrip() {
  if (dmaStrip != nullptr) {
    dmaStrip->ClearTo(RgbColor(0));
    dmaStrip->Show();
    delete dmaStrip;
    dmaStrip = nullptr;
  }
  if (uart1Strip != nullptr) {
    uart1Strip->ClearTo(RgbColor(0));
    uart1Strip->Show();
    while (!uart1Strip->CanShow()) {
      yield();
    }
    delete uart1Strip;
    uart1Strip = nullptr;
  }
}

void beginLocalStrip() {
  endLocalStrip();
  if (!settings.stripEnabled) {
    return;
  }
  if (settings.stripPin == StripPinD4) {
    uart1Strip = new Uart1Strip(settings.stripLedCount);
    uart1Strip->Begin();
  } else {
    dmaStrip = new DmaStrip(settings.stripLedCount);
    dmaStrip->Begin();
  }
  stripDirty = true;
  stripShowingLight = false;
  logSerial.printf("Local strip: %u LEDs on %s\n", settings.stripLedCount,
                   settings.stripPin == StripPinD4 ? "D4" : "RX");
}

bool localStripCanShow() {
  if (dmaStrip != nullptr) {
    return dmaStrip->CanShow();
  }
  return uart1Strip != nullptr && uart1Strip->CanShow();
}

void renderLocalStrip(bool lit) {
  const FrameGeometry geometry = frameGeometry(settings.strip, settings.stripLedCount);
  const uint8_t *order = ColorOrders[settings.stripColorOrder];
  for (uint16_t index = 0; index < settings.stripLedCount; ++index) {
    const Color color = lit ? ledColor(renderPixel(geometry, index)) : Color{0, 0, 0};
    const uint8_t channels[] = {color.r, color.g, color.b};
    const RgbColor wire(channels[order[0]], channels[order[1]], channels[order[2]]);
    if (dmaStrip != nullptr) {
      dmaStrip->SetPixelColor(index, wire);
    } else {
      uart1Strip->SetPixelColor(index, wire);
    }
  }
  if (dmaStrip != nullptr) {
    dmaStrip->Show();
  } else {
    uart1Strip->Show();
  }
}

// Only redraws when the picture changed, and once more to go dark when nobody is present.
void serviceLocalStrip(bool lit, uint32_t signature) {
  if (dmaStrip == nullptr && uart1Strip == nullptr) {
    return;
  }
  if (!lit && !stripShowingLight && !stripDirty) {
    return;
  }
  if (lit && !stripDirty && signature == lastStripSignature) {
    return;
  }
  if (!localStripCanShow()) {
    return;
  }
  renderLocalStrip(lit);
  stripShowingLight = lit;
  stripDirty = false;
  lastStripSignature = signature;
}

void sendWledFrame(bool blank) {
  const IPAddress ip(settings.wledIp);
  const uint8_t timeout =
      settings.wledKeepControl ? WledKeepControlTimeout : WledHandoverTimeoutSeconds;
  const OutputConfig output = wledOutput();
  const FrameGeometry geometry = frameGeometry(output, settings.wledLedCount);
  for (uint16_t start = 0; start < settings.wledLedCount; start += WledMaxLedsPerPacket) {
    const uint16_t count = min<uint16_t>(WledMaxLedsPerPacket, settings.wledLedCount - start);
    wledPacket[0] = WledProtocolDnrgb;
    wledPacket[1] = timeout;
    wledPacket[2] = static_cast<uint8_t>(start >> 8);
    wledPacket[3] = static_cast<uint8_t>(start & 0xFF);
    for (uint16_t offset = 0; offset < count; ++offset) {
      const Color color =
          blank ? Color{0, 0, 0} : ledColor(renderPixel(geometry, start + offset));
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

void serviceWled(uint32_t now, bool lit, uint32_t signature) {
  if (!settings.wledEnabled || settings.wledIp == 0 || settings.wledLedCount == 0 ||
      WiFi.status() != WL_CONNECTED) {
    return;
  }

  if (lit) {
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
    if (settings.wledKeepControl) {
      sendWledFrame(true);
    } else {
      sendWledRelease(settings.wledIp);
    }
    --wledIdlePackets;
    wledStreaming = false;
    lastWledSend = now;
  }
}

void serviceOutputs() {
  const uint32_t now = millis();
  const uint32_t elapsed = now - lastOutputTick;
  if (elapsed < OutputFrameIntervalMilliseconds) {
    return;
  }
  lastOutputTick = now;
  updateTracks(now, elapsed);
  const bool lit = tracksLit();
  const uint32_t signature = trackSignature();
  serviceLocalStrip(lit, signature);
  serviceWled(now, lit, signature);
}

void applySettings(const Settings &previous) {
  if (previous.wledEnabled && previous.wledIp != 0 &&
      (!settings.wledEnabled || previous.wledIp != settings.wledIp)) {
    sendWledRelease(previous.wledIp);
  }
  if (previous.singleUser != settings.singleUser) {
    memset(tracks, 0, sizeof(tracks));
  }
  if (previous.stripEnabled != settings.stripEnabled || previous.stripPin != settings.stripPin ||
      previous.stripLedCount != settings.stripLedCount) {
    beginLocalStrip();
  }
  stripDirty = true;
  wledStreaming = false;
  wledIdlePackets = WledIdleRepeatPackets;
}

String formatColor(const Color &color) {
  char text[8];
  snprintf(text, sizeof(text), "#%02x%02x%02x", color.r, color.g, color.b);
  return text;
}

// Keeps lowercase letters, digits and hyphens, as allowed in a .local name.
String sanitizeHostname(const String &text) {
  String hostname;
  for (const char character : text) {
    const char lower = static_cast<char>(tolower(character));
    if (isalnum(lower) || (lower == '-' && hostname.length() > 0)) {
      hostname += lower;
    }
    if (hostname.length() >= sizeof(settings.hostname) - 1) {
      break;
    }
  }
  while (hostname.endsWith("-")) {
    hostname.remove(hostname.length() - 1);
  }
  return hostname.length() > 0 ? hostname : String(DefaultHostname);
}

void sendJson(int code, const JsonDocument &document) {
  String response;
  serializeJson(document, response);
  webServer.send(code, "application/json", response);
}

long argNumber(const String &name, long minimum, long maximum, long fallback) {
  if (!webServer.hasArg(name)) {
    return fallback;
  }
  return constrain(webServer.arg(name).toInt(), minimum, maximum);
}

// Converts a value entered in metres or seconds to thousandths.
long argThousandths(const String &name, long minimum, long maximum, long fallback) {
  if (!webServer.hasArg(name)) {
    return fallback;
  }
  return constrain(lroundf(webServer.arg(name).toFloat() * 1000.0f), minimum, maximum);
}

bool argFlag(const String &name, bool fallback) {
  return webServer.hasArg(name) ? webServer.arg(name) == "1" : fallback;
}

Color argColor(const String &name, const Color &fallback) {
  const String text = webServer.arg(name);
  if (text.length() != 7 || text[0] != '#') {
    return fallback;
  }
  const uint32_t value = strtoul(text.c_str() + 1, nullptr, 16);
  return {static_cast<uint8_t>(value >> 16), static_cast<uint8_t>(value >> 8),
          static_cast<uint8_t>(value)};
}

void writeOutputConfig(JsonObject json, const OutputConfig &output) {
  json["corridorStart"] = serialized(String(output.corridorStartMm / 1000.0f, 2));
  json["corridorEnd"] = serialized(String(output.corridorEndMm / 1000.0f, 2));
  json["stripStart"] = output.stripStartPercent;
  json["stripEnd"] = output.stripEndPercent;
  json["width"] = output.widthPercent;
  json["softness"] = output.softnessPercent;
  json["brightness"] = output.brightnessPercent;
  json["reverse"] = output.reverse;
  for (uint8_t index = 0; index < MaxTargets; ++index) {
    json["color" + String(index)] = formatColor(output.colors[index]);
  }
  json["background"] = formatColor(output.background);
}

void readOutputConfig(const String &prefix, OutputConfig &output) {
  output.reverse = argFlag(prefix + "reverse", output.reverse);
  output.corridorStartMm = argThousandths(prefix + "corridorStart", 0, 8000, output.corridorStartMm);
  output.corridorEndMm = argThousandths(prefix + "corridorEnd", 100, 8000, output.corridorEndMm);
  if (output.corridorEndMm < output.corridorStartMm + 100) {
    output.corridorEndMm = output.corridorStartMm + 100;
  }
  output.stripStartPercent = argNumber(prefix + "stripStart", 0, 99, output.stripStartPercent);
  output.stripEndPercent = argNumber(prefix + "stripEnd", 1, 100, output.stripEndPercent);
  if (output.stripEndPercent <= output.stripStartPercent) {
    output.stripEndPercent = output.stripStartPercent + 1;
  }
  output.widthPercent = argNumber(prefix + "width", 1, 100, output.widthPercent);
  output.softnessPercent = argNumber(prefix + "softness", 0, 100, output.softnessPercent);
  output.brightnessPercent = argNumber(prefix + "brightness", 1, 100, output.brightnessPercent);
  for (uint8_t index = 0; index < MaxTargets; ++index) {
    output.colors[index] = argColor(prefix + "color" + String(index), output.colors[index]);
  }
  output.background = argColor(prefix + "background", output.background);
}

// Appends "key":{"enabled","leds","px"} where px holds hex colours, downsampled to at most
// PreviewMaxPixels.
void appendOutputPreview(String &response, const char *key, bool enabled,
                         const OutputConfig &output, uint16_t ledCount) {
  if (ledCount == 0) {
    ledCount = PreviewLedCount;
  }
  const uint16_t pixels = min(ledCount, PreviewMaxPixels);
  const FrameGeometry geometry = frameGeometry(output, ledCount);
  response.reserve(response.length() + pixels * 6 + 64);
  response += "\"" + String(key) + "\":{\"enabled\":" + String(enabled ? "true" : "false") +
              ",\"leds\":" + String(ledCount) + ",\"px\":\"";
  char hex[7];
  for (uint16_t index = 0; index < pixels; ++index) {
    const Color color =
        renderPixel(geometry, static_cast<uint32_t>(index) * ledCount / pixels);
    snprintf(hex, sizeof(hex), "%02x%02x%02x", color.r, color.g, color.b);
    response += hex;
  }
  response += "\"}";
}

void appendOutputPreviews(String &response) {
  appendOutputPreview(response, "strip", settings.stripEnabled, settings.strip,
                      settings.stripLedCount);
  response += ",";
  appendOutputPreview(response, "wled", settings.wledEnabled, wledOutput(),
                      settings.wledLedCount);
}

void handleRoot() {
  static const char page[] PROGMEM = R"HTML(
<!doctype html><html><head><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Presence Lights</title><style>
 :root{color-scheme:dark}*{box-sizing:border-box}body{margin:0;background:#050607;color:#e9eef0;font:15px system-ui,sans-serif;display:flex;justify-content:center}main{width:min(100%,760px);padding:20px 16px 28px}header{display:flex;align-items:baseline;justify-content:space-between;border-bottom:1px solid #263035;padding:0 2px 14px;margin-bottom:16px}h1{font-size:clamp(14px,4vw,18px);letter-spacing:.08em;margin:0;color:#9be7c4;white-space:nowrap}.readouts{display:flex;gap:clamp(10px,3vw,18px);align-items:baseline}#distance,#lateral{font-size:clamp(15px,4.4vw,22px);font-variant-numeric:tabular-nums;white-space:nowrap;color:#e9eef0}#lateral{color:#8fa3aa}.panel{border:1px solid #263035;background:#080b0c;border-radius:6px;padding:12px;margin-bottom:14px}canvas{display:block;width:100%;height:auto}.label{color:#829197;font-size:12px;letter-spacing:.08em;text-transform:uppercase;margin:0 0 8px}.empty{color:#657177;text-align:center;font-size:13px}.links{display:flex;justify-content:center;gap:10px;margin-top:18px}.setup{display:block;padding:9px 16px;border:1px solid #9be7c4;border-radius:4px;color:#9be7c4;text-decoration:none;font-size:13px;letter-spacing:.04em}.setup:hover{background:#10231e}.strip{height:28px;border-radius:3px;background:#000}.sub{color:#b7c3c7;font-size:13px;margin:10px 0 6px}.sub:first-of-type{margin-top:0}
 </style></head><body><main><header><h1>PRESENCE LIGHTS</h1><div class="readouts"><div id="distance">--.-- m</div><div id="lateral">L/R --.-- m</div></div></header>
<section class="panel"><p class="label">Position</p><canvas id="radar" width="720" height="500"></canvas><div id="state" class="empty">Waiting for target</div></section>
<section class="panel"><p class="label">Distance history</p><canvas id="graph" width="720" height="190"></canvas></section>
<section class="panel"><p class="label">LED strips</p><p class="sub" id="stripLabel">Local strip</p><canvas class="strip" id="strip" width="720" height="28"></canvas><p class="sub" id="wledLabel">WLED</p><canvas class="strip" id="wled" width="720" height="28"></canvas></section>
<nav class="links"><a class="setup" href="/config">Configuration</a><a class="setup" href="/setup">Wi-Fi Setup</a></nav>
<script>
const radar=document.getElementById('radar'),graph=document.getElementById('graph'),distance=document.getElementById('distance'),lateral=document.getElementById('lateral'),state=document.getElementById('state'),strip=document.getElementById('strip'),wled=document.getElementById('wled'),stripLabel=document.getElementById('stripLabel'),wledLabel=document.getElementById('wledLabel'),history=[];
function drawStrip(canvas,label,name,o){const c=canvas.getContext('2d'),n=o.px.length/6,w=canvas.width/n;for(let i=0;i<n;i++){c.fillStyle='#'+o.px.substr(i*6,6);c.fillRect(Math.floor(i*w),0,Math.ceil(w),canvas.height)}label.textContent=name+(o.enabled?', '+o.leds+' LEDs':', off (preview only)')}
function resize(canvas){const scale=window.devicePixelRatio||1,w=canvas.clientWidth;canvas.width=w*scale;canvas.height=canvas.height/ (canvas.width/(w*scale));return canvas.getContext('2d')}
function drawRadar(d){const c=radar.getContext('2d'),s=devicePixelRatio||1,w=radar.width/s,h=radar.height/s;c.setTransform(s,0,0,s,0,0);c.fillStyle='#080b0c';c.fillRect(0,0,w,h);const cx=w/2,sy=36,max=6,range=Math.min(w*.43,330);c.strokeStyle='#1b3430';c.lineWidth=1;for(let r=1;r<=5;r++){c.beginPath();c.arc(cx,sy,range*r/5,0,Math.PI*2);c.stroke()}c.strokeStyle='#172427';c.beginPath();c.moveTo(cx,sy);c.lineTo(cx,h-18);c.stroke();c.fillStyle='#9be7c4';c.fillRect(cx-25,sy-8,50,8);c.fillRect(cx-4,sy-18,8,10);if(d.detected){const colors=['#f4c95d','#63d7ff','#ff7e9b'];d.targets.forEach((t,i)=>{if(!t.detected)return;const x=cx+(t.x/3)*range,y=sy+Math.min(t.distance,max)/max*(h-70);c.fillStyle=colors[i];c.beginPath();c.arc(x,y,8,0,Math.PI*2);c.fill();c.shadowColor=colors[i];c.shadowBlur=18;c.fill();c.shadowBlur=0})}else{c.fillStyle='#657177';c.beginPath();c.arc(cx,sy+12,3,0,Math.PI*2);c.fill()}}
function drawGraph(){const c=graph.getContext('2d'),s=devicePixelRatio||1,w=graph.width/s,h=graph.height/s;c.setTransform(s,0,0,s,0,0);c.fillStyle='#080b0c';c.fillRect(0,0,w,h);c.strokeStyle='#172427';for(let i=1;i<4;i++){const y=i*h/4;c.beginPath();c.moveTo(0,y);c.lineTo(w,y);c.stroke()}c.strokeStyle='#9be7c4';c.lineWidth=2;c.beginPath();history.forEach((v,i)=>{const x=i/(history.length-1||1)*w,y=h-8-v/6*(h-16);i?c.lineTo(x,y):c.moveTo(x,y)});c.stroke()}
async function update(){try{const d=await (await fetch('/data',{cache:'no-store'})).json();distance.textContent=d.detected?d.distance.toFixed(2)+' m':'--.-- m';lateral.textContent=d.detected?'L/R '+d.x.toFixed(2)+' m':'L/R --.-- m';state.textContent=d.detected?d.count+' target'+(d.count===1?'':'s')+' detected':'Waiting for target';history.push(d.detected?d.distance:0);if(history.length>120)history.shift();drawRadar(d);drawGraph();drawStrip(strip,stripLabel,'Local strip',d.strip);drawStrip(wled,wledLabel,'WLED',d.wled)}catch(e){state.textContent='Connection lost'}}
addEventListener('resize',()=>{drawRadar({detected:false});drawGraph()});drawRadar({detected:false});drawGraph();update();setInterval(update,100);
</script></main></body></html>)HTML";
  webServer.send_P(200, "text/html", page);
}

void handleData() {
  const bool fresh = targetsFresh();
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
  response += "],";
  appendOutputPreviews(response);
  response += "}";
  webServer.send(200, "application/json", response);
}

void handleConfigPage() {
  static const char page[] PROGMEM = R"HTML(
<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Presence Lights Configuration</title><style>
 :root{color-scheme:dark}*{box-sizing:border-box}[hidden]{display:none!important}body{margin:0;background:#050607;color:#e9eef0;font:15px system-ui,sans-serif;display:flex;justify-content:center}main{width:min(100%,760px);padding:20px 16px 0}header{display:flex;align-items:baseline;justify-content:space-between;border-bottom:1px solid #263035;padding:0 2px 14px;margin-bottom:16px}h1{font-size:clamp(14px,4vw,18px);letter-spacing:.08em;margin:0;color:#9be7c4}header a{color:#9be7c4;font-size:13px}.panel{border:1px solid #263035;background:#080b0c;border-radius:6px;padding:12px;margin-bottom:14px}.head{display:flex;justify-content:space-between;align-items:center;margin-bottom:10px}.label{color:#829197;font-size:12px;letter-spacing:.08em;text-transform:uppercase;margin:0}.row{display:flex;flex-wrap:wrap;gap:8px;align-items:center;margin:10px 0}.grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(140px,1fr));gap:10px 14px;margin-top:10px}label{display:flex;flex-direction:column;gap:4px;color:#b7c3c7;font-size:13px}label.check{flex-direction:row;align-items:center;gap:8px}input,select,button{font:inherit;color:#e9eef0;background:#0d1214;border:1px solid #263035;border-radius:4px;padding:7px 9px}input[type=checkbox]{width:18px;height:18px;margin:0;accent-color:#9be7c4}.switch{appearance:none;-webkit-appearance:none;position:relative;width:38px!important;height:22px!important;padding:0;border:0;border-radius:11px;background:#263035;cursor:pointer;flex:none}.switch::after{content:'';position:absolute;top:3px;left:3px;width:16px;height:16px;border-radius:50%;background:#829197;transition:left .15s}.switch:checked{background:#2f6b57}.switch:checked::after{left:19px;background:#9be7c4}.toggles{display:flex;flex-wrap:wrap;gap:10px 24px;margin-bottom:6px}.ends{display:flex;justify-content:space-between;align-items:center;gap:8px;margin:8px 0}.end{font-size:11px;color:#829197;letter-spacing:.04em;gap:3px}.end.right{align-items:flex-end}.end span{display:flex;align-items:center;gap:4px;font-size:13px;color:#b7c3c7}.small{width:4.8em;padding:4px 6px}canvas{display:block;width:100%;height:28px;border-radius:3px;background:#000}.sliders{display:grid;grid-template-columns:repeat(auto-fill,minmax(250px,1fr));gap:2px 20px;margin-top:6px}.slide{display:grid;grid-template-columns:6.2em 1fr 4.6em;align-items:center;gap:8px;min-height:34px;font-size:13px;color:#b7c3c7}input[type=range]{padding:0;border:0;background:none;accent-color:#9be7c4;width:100%;min-width:0}.value{display:flex;align-items:center;justify-content:flex-end;gap:3px;color:#829197}.value i{font-style:normal;width:.8em}.val{width:3.6em;padding:3px 5px;text-align:right;font-size:13px;font-variant-numeric:tabular-nums;-moz-appearance:textfield;appearance:textfield}.val::-webkit-inner-spin-button,.val::-webkit-outer-spin-button{-webkit-appearance:none;margin:0}.colors{display:flex;flex-wrap:wrap;gap:8px 12px;margin-top:8px}.colors label{font-size:11px;color:#829197;gap:3px}.colors input{width:52px;height:30px;padding:2px}details summary{display:flex;align-items:center;gap:6px;cursor:pointer;list-style:none;color:#b7c3c7;font-size:13px;margin-top:10px}details summary::-webkit-details-marker{display:none}details summary::before{content:'\25B8';color:#829197}details[open] summary::before{content:'\25BE'}.panel>summary{margin:0}.sum{margin-left:auto;color:#657177;font-size:12px;text-align:right}#devices,#wled_ip{flex:1;min-width:150px}button{cursor:pointer;border-color:#9be7c4;color:#9be7c4}button:hover{background:#10231e}button.primary{background:#9be7c4;color:#050607}.hint{color:#657177;font-size:12px;margin:6px 0 0}#device{color:#b7c3c7;font-size:13px}.synced .own{display:none!important}.status{display:grid;grid-template-columns:repeat(auto-fill,minmax(160px,1fr));gap:10px 14px;margin:12px 0}.status div{display:flex;flex-direction:column;gap:2px;min-width:0}.status span{color:#829197;font-size:11px;letter-spacing:.06em;text-transform:uppercase}.status b{font-weight:500;font-size:13px;font-variant-numeric:tabular-nums;overflow-wrap:anywhere}.savebar{position:sticky;bottom:0;display:flex;align-items:center;gap:10px;padding:10px 0 calc(10px + env(safe-area-inset-bottom));background:#050607;border-top:1px solid #263035}#message{font-size:13px;color:#829197}#message.unsaved{color:#f4c95d}
 </style></head><body><main><header><h1>CONFIGURATION</h1><a href="/">Visualizer</a></header>
<section class="panel"><div class="head"><p class="label">Tracking</p></div>
<div class="toggles"><label class="check"><input type="checkbox" class="switch" id="singleUser" name="singleUser">Single User</label><label class="check"><input type="checkbox" class="switch" id="testLight">Test light</label></div>
<div class="sliders" id="trackingSliders"></div>
<p class="hint">Single User follows only the nearest person. Test light shows a light mid-corridor while you adjust. Hold time keeps the light on after someone is lost.</p></section>
<section class="panel"><div class="head"><p class="label">Local strip</p><input type="checkbox" class="switch" id="strip_enabled" name="strip_enabled" aria-label="Enable the local strip"></div>
<div id="stripFields"></div>
<details><summary>Hardware<span class="sum" id="stripSummary"></span></summary><div class="grid">
<label>Data pin<select id="strip_pin" name="strip_pin"><option value="0">RX (GPIO3)</option><option value="1">D4 (GPIO2)</option></select></label>
<label>LED count<input type="number" id="strip_leds" name="strip_leds" min="1" max="600"></label>
<label>Color order<select id="strip_order" name="strip_order"><option value="0">GRB (most WS2812)</option><option value="1">RGB</option><option value="2">BRG</option><option value="3">RBG</option><option value="4">GBR</option><option value="5">BGR</option></select></label></div>
<p class="hint">WS2812 only. These pins are driven by hardware, so the strip does not disturb the sensor or Wi-Fi.</p></details></section>
<section class="panel" id="wledCard"><div class="head"><p class="label">WLED</p><input type="checkbox" class="switch" id="wled_enabled" name="wled_enabled" aria-label="Enable WLED control"></div>
<div class="row"><label class="check"><input type="checkbox" class="switch" id="wled_sync" name="wled_sync">Sync with local strip</label></div>
<div id="wledFields"></div>
<details><summary>Device<span class="sum" id="wledSummary"></span></summary>
<div class="row"><select id="devices"><option value="">Press Scan to find WLED devices</option></select><button id="scan" type="button">Scan</button></div>
<div class="row"><input id="wled_ip" name="wled_ip" placeholder="WLED IP address"><button id="check" type="button">Check</button></div>
<div id="device">No device selected</div>
<div class="row"><label class="check"><input type="checkbox" class="switch" id="wled_keepControl" name="wled_keepControl">Keep control when nobody is present</label></div>
<p class="hint">Off: WLED returns to its own preset when nobody is present. On: the strip stays dark. A new device is used after Save.</p></details></section>
<details class="panel"><summary><span class="label">Device Info</span><span class="sum" id="deviceSummary"></span></summary>
<div class="status" id="status"></div>
<div class="grid"><label>Hostname<input id="hostname" name="hostname" maxlength="31"></label></div>
<p class="hint">Reachable at http://hostname.local. A new hostname applies after a restart.</p>
<div class="row"><button id="restart" type="button">Restart device</button></div></details>
<datalist id="palette"><option value="#ff0000"><option value="#00ffff"><option value="#0000ff"><option value="#00ff00"><option value="#ff00ff"><option value="#ff9a3c"><option value="#000000"><option value="#ffffff"></datalist>
<div class="savebar"><button class="primary" id="save" type="button">Save</button><button id="discard" type="button">Discard</button><span id="message"></span></div>
<script>
const $=id=>document.getElementById(id),pins=['RX','D4'],orders=['GRB','RGB','BRG','RBG','GBR','BGR'],dot=' · ';let device={ip:'',name:'',leds:0},found=[],unsaved=false,busy=false,again=false,timer,testTimer;
function slider(id,label,min,max,step,unit){const a=' min="'+min+'" max="'+max+'" step="'+step+'"';return '<div class="slide"><span>'+label+'</span><input type="range" id="'+id+'" name="'+id+'"'+a+'><span class="value"><input type="number" class="val" inputmode="decimal"'+a+' aria-label="'+label+'"><i>'+unit+'</i></span></div>'}
function num(id,min,max,step){return '<input type="number" class="small" id="'+id+'" name="'+id+'" min="'+min+'" max="'+max+'" step="'+step+'">'}
function color(id,label,cls){return '<label'+(cls?' class="'+cls+'"':'')+'>'+label+'<input type="color" list="palette" id="'+id+'" name="'+id+'"></label>'}
function outputFields(p){return '<div class="ends own"><label class="end">Corridor start<span>'+num(p+'corridorStart',0,8,0.05)+'m</span></label><label class="end right">Corridor end<span>'+num(p+'corridorEnd',0.1,8,0.05)+'m</span></label></div>'+
'<canvas id="'+p+'preview" width="720" height="28"></canvas>'+
'<div class="ends"><label class="end own">Strip start<span>'+num(p+'stripStart',0,99,1)+'%</span></label><label class="check"><input type="checkbox" id="'+p+'reverse" name="'+p+'reverse">Reverse &#8644;</label><label class="end right own">Strip end<span>'+num(p+'stripEnd',1,100,1)+'%</span></label></div>'+
'<div class="sliders own">'+slider(p+'width','Lit width',1,100,1,'%')+slider(p+'softness','Softness',0,100,1,'%')+slider(p+'brightness','Brightness',1,100,1,'%')+'</div>'+
'<div class="colors own">'+color(p+'color0','Person 1')+color(p+'color1','Person 2','multi')+color(p+'color2','Person 3','multi')+color(p+'background','Background')+'</div>'}
$('trackingSliders').innerHTML=slider('smoothing','Smoothing',0,100,1,'')+slider('hold','Hold time',0.2,30,0.1,'s')+slider('fade','Fade time',0.1,5,0.1,'s');
$('stripFields').innerHTML=outputFields('strip_');$('wledFields').innerHTML=outputFields('wled_');
function message(t){const m=$('message');m.className='';m.textContent=t}
function setUnsaved(v){unsaved=v;const m=$('message');m.className=v?'unsaved':'';m.textContent=v?'Unsaved changes':''}
function summaries(){$('stripSummary').textContent=pins[$('strip_pin').value]+dot+$('strip_leds').value+' LEDs'+dot+orders[$('strip_order').value];$('wledSummary').textContent=device.ip?(device.name||'WLED')+dot+device.leds+' LEDs':'No device'}
function showDevice(){$('device').textContent=device.ip?(device.name||'WLED')+' at '+device.ip+', '+device.leds+' LEDs':'No device selected';$('wled_ip').value=device.ip;summaries()}
function refresh(){document.querySelectorAll('.val').forEach(e=>{if(e!==document.activeElement)e.value=e.parentNode.previousElementSibling.value});document.querySelectorAll('.multi').forEach(e=>e.hidden=$('singleUser').checked);$('wledCard').classList.toggle('synced',$('wled_sync').checked);summaries()}
function fill(o,p){for(const k in o){const e=$(p+k);if(!e||typeof o[k]==='object')continue;if(e.type==='checkbox')e.checked=o[k];else e.value=o[k]}}
function apply(c){fill(c,'');fill(c.strip,'strip_');fill(c.wled,'wled_');device={ip:c.wled.ip,name:c.wled.name,leds:c.wled.leds};showDevice();refresh();setUnsaved(c.unsaved)}
function collect(){const p=new URLSearchParams();document.querySelectorAll('main [name]').forEach(e=>p.set(e.name,e.type==='checkbox'?(e.checked?'1':'0'):e.value.trim()));return p}
async function live(){if(busy){again=true;return}busy=true;try{do{again=false;const p=collect();p.set('live','1');await fetch('/api/config',{method:'POST',body:p})}while(again)}catch(e){message('Connection lost')}finally{busy=false}}
document.querySelector('main').addEventListener('input',e=>{let t=e.target;if(t.classList.contains('val')){if(t.value==='')return;t=t.parentNode.previousElementSibling;t.value=e.target.value}if(!t.name)return;refresh();setUnsaved(true);if(t.id==='wled_ip')return;clearTimeout(timer);timer=setTimeout(live,150)});
document.querySelector('main').addEventListener('change',e=>{if(e.target.classList.contains('val'))e.target.value=e.target.parentNode.previousElementSibling.value});
async function settle(){clearTimeout(timer);while(busy)await new Promise(r=>setTimeout(r,50))}
$('save').onclick=async()=>{await settle();const p=collect(),ip=p.get('wled_ip');p.set('wled_name',ip===device.ip?device.name:'');p.set('wled_leds',ip===device.ip?device.leds:0);message('Saving...');try{const r=await fetch('/api/config',{method:'POST',body:p});if(!r.ok)throw 0;apply(await r.json());message(device.ip&&!device.leds?'Saved, but the WLED device did not answer':'Saved')}catch(e){message('Save failed')}};
$('discard').onclick=async()=>{await settle();try{const r=await fetch('/api/discard',{method:'POST'});if(!r.ok)throw 0;apply(await r.json());message('Changes discarded')}catch(e){message('Connection lost')}};
async function test(on){try{await fetch('/api/test',{method:'POST',body:new URLSearchParams({on:on?'1':'0'})})}catch(e){}}
$('testLight').onchange=e=>{const on=e.target.checked;clearInterval(testTimer);test(on);if(on)testTimer=setInterval(()=>test(true),10000)};
addEventListener('pagehide',()=>{if($('testLight').checked)navigator.sendBeacon('/api/test',new URLSearchParams({on:'0'}))});
addEventListener('beforeunload',e=>{if(unsaved){e.preventDefault();e.returnValue=''}});
$('scan').onclick=async()=>{message('Scanning...');try{found=(await (await fetch('/api/scan',{cache:'no-store'})).json()).devices;const s=$('devices');s.innerHTML='';if(!found.length)s.add(new Option('No WLED devices found',''));found.forEach((x,i)=>s.add(new Option(x.name+' ('+x.ip+', '+x.leds+' LEDs)',i)));if(found.length){device=found[0];showDevice();setUnsaved(true)}message(found.length?found.length+' found, press Save to use it':'None found')}catch(e){message('Scan failed')}};
$('devices').onchange=e=>{const x=found[e.target.value];if(x){device=x;showDevice();setUnsaved(true)}};
$('check').onclick=async()=>{message('Checking...');try{const r=await fetch('/api/wled-info?ip='+encodeURIComponent($('wled_ip').value.trim()),{cache:'no-store'});if(!r.ok)throw 0;device=await r.json();showDevice();setUnsaved(true);message('Device found, press Save to use it')}catch(e){message('No WLED device at that address')}};
$('restart').onclick=async()=>{if(unsaved&&!confirm('Restarting drops unsaved changes. Restart anyway?'))return;unsaved=false;message('Restarting...');try{await fetch('/api/restart',{method:'POST'})}catch(e){}setTimeout(()=>location.reload(),8000)};
function uptime(s){const d=Math.floor(s/86400),h=Math.floor(s%86400/3600),m=Math.floor(s%3600/60);return(d?d+'d ':'')+h+'h '+m+'m'}
async function status(){try{const s=await (await fetch('/api/status',{cache:'no-store'})).json(),items=[['Network',s.network+(s.ssid?', '+s.ssid:'')],['IP address',s.ip],['Address',s.network==='Wi-Fi'?'http://'+s.hostname+'.local':'-'],['Signal',s.rssi==null?'-':s.rssi+' dBm'],['MAC address',s.mac],['Uptime',uptime(s.uptime)],['Free memory',(s.heap/1024).toFixed(1)+' KB'],['Sensor',s.sensorRate+' frames/s, '+s.targets+' target'+(s.targets===1?'':'s')],['WLED output',s.wled],['Last reset',s.reset],['Firmware',s.firmware]];$('deviceSummary').textContent=s.ip;$('status').replaceChildren(...items.map(([k,v])=>{const d=document.createElement('div'),a=document.createElement('span'),b=document.createElement('b');a.textContent=k;b.textContent=v;d.append(a,b);return d}))}catch(e){}}
function draw(canvas,o){const c=canvas.getContext('2d'),n=o.px.length/6,w=canvas.width/n;for(let i=0;i<n;i++){c.fillStyle='#'+o.px.substr(i*6,6);c.fillRect(Math.floor(i*w),0,Math.ceil(w),canvas.height)}}
async function preview(){try{const d=await (await fetch('/api/preview',{cache:'no-store'})).json();draw($('strip_preview'),d.strip);draw($('wled_preview'),d.wled)}catch(e){}setTimeout(preview,100)}
fetch('/api/config',{cache:'no-store'}).then(r=>r.json()).then(apply).catch(()=>message('Connection lost'));status();setInterval(status,2000);preview();
</script></main></body></html>)HTML";
  webServer.send_P(200, "text/html", page);
}

void handleConfigGet() {
  JsonDocument document;
  document["singleUser"] = settings.singleUser;
  document["smoothing"] = settings.smoothing;
  document["hold"] = serialized(String(settings.holdMilliseconds / 1000.0f, 1));
  document["fade"] = serialized(String(settings.fadeMilliseconds / 1000.0f, 1));
  document["hostname"] = settings.hostname;
  JsonObject strip = document["strip"].to<JsonObject>();
  strip["enabled"] = settings.stripEnabled;
  strip["pin"] = settings.stripPin;
  strip["leds"] = settings.stripLedCount;
  strip["order"] = settings.stripColorOrder;
  writeOutputConfig(strip, settings.strip);
  JsonObject wled = document["wled"].to<JsonObject>();
  writeOutputConfig(wled, settings.wled);
  wled["enabled"] = settings.wledEnabled;
  wled["keepControl"] = settings.wledKeepControl;
  wled["sync"] = settings.wledSync;
  wled["ip"] = settings.wledIp != 0 ? IPAddress(settings.wledIp).toString() : String();
  wled["name"] = settings.wledName;
  wled["leds"] = settings.wledLedCount;
  document["unsaved"] = settingsUnsaved;
  sendJson(200, document);
}

// live=1 applies the settings without storing them, so the strips follow the page while it is
// edited. The WLED device only changes on save, because checking it blocks for a moment.
void handleConfigPost() {
  const bool live = webServer.arg("live") == "1";
  const Settings previous = settings;
  Settings next = settings;
  next.singleUser = argFlag("singleUser", next.singleUser);
  next.smoothing = argNumber("smoothing", 0, 100, next.smoothing);
  next.holdMilliseconds = argThousandths("hold", 200, 60000, next.holdMilliseconds);
  next.fadeMilliseconds = argThousandths("fade", 100, 5000, next.fadeMilliseconds);
  if (webServer.hasArg("hostname")) {
    strlcpy(next.hostname, sanitizeHostname(webServer.arg("hostname")).c_str(),
            sizeof(next.hostname));
  }

  next.stripEnabled = argFlag("strip_enabled", next.stripEnabled);
  next.stripPin = argNumber("strip_pin", StripPinRx, StripPinD4, next.stripPin);
  next.stripLedCount = argNumber("strip_leds", 1, StripMaxLeds, next.stripLedCount);
  next.stripColorOrder = argNumber("strip_order", 0, ColorOrderCount - 1, next.stripColorOrder);
  readOutputConfig("strip_", next.strip);

  next.wledEnabled = argFlag("wled_enabled", next.wledEnabled);
  next.wledKeepControl = argFlag("wled_keepControl", next.wledKeepControl);
  next.wledSync = argFlag("wled_sync", next.wledSync);
  readOutputConfig("wled_", next.wled);
  IPAddress ip;
  if (live) {
    // Keeps the current WLED device.
  } else if (ip.fromString(webServer.arg("wled_ip"))) {
    String name = webServer.arg("wled_name");
    uint16_t ledCount = argNumber("wled_leds", 0, WledMaxLeds, 0);
    fetchWledInfo(ip, name, ledCount);
    next.wledIp = static_cast<uint32_t>(ip);
    next.wledLedCount = ledCount;
    strlcpy(next.wledName, name.c_str(), sizeof(next.wledName));
  } else {
    next.wledIp = 0;
    next.wledLedCount = 0;
    next.wledName[0] = '\0';
  }

  settings = next;
  applySettings(previous);
  if (live) {
    settingsUnsaved = true;
    webServer.send(200, "application/json", "{}");
    return;
  }
  saveSettings();
  settingsUnsaved = false;
  logSerial.printf("Settings saved: strip=%d wled=%d ip=%s leds=%u\n", settings.stripEnabled,
                   settings.wledEnabled, IPAddress(settings.wledIp).toString().c_str(),
                   settings.wledLedCount);
  handleConfigGet();
}

// Drops live changes and goes back to the settings stored in flash.
void handleConfigDiscard() {
  const Settings previous = settings;
  loadSettings();
  applySettings(previous);
  settingsUnsaved = false;
  handleConfigGet();
}

void handleTestLight() {
  testLight = webServer.arg("on") == "1";
  testLightTime = millis();
  webServer.send(200, "application/json", "{}");
}

void handleStatus() {
  JsonDocument document;
  document["network"] = accessPointMode ? "Access point" : "Wi-Fi";
  document["ssid"] = accessPointMode ? String(AccessPointName) : WiFi.SSID();
  document["ip"] = (accessPointMode ? WiFi.softAPIP() : WiFi.localIP()).toString();
  if (accessPointMode) {
    document["rssi"] = nullptr;
  } else {
    document["rssi"] = WiFi.RSSI();
  }
  document["mac"] = WiFi.macAddress();
  document["hostname"] = settings.hostname;
  document["uptime"] = millis() / 1000;
  document["heap"] = ESP.getFreeHeap();
  document["reset"] = ESP.getResetReason();
  document["sensorRate"] = sensorFrameRate;
  document["targets"] = targetsFresh() ? targetCount : 0;
  document["wled"] = !settings.wledEnabled ? "Off" : wledStreaming ? "Sending" : "Idle";
  document["firmware"] = __DATE__ " " __TIME__;
  sendJson(200, document);
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
    logSerial.printf("WLED scan: %u answers, %u devices\n", answers, devices.size());
  }
  sendJson(200, document);
}

void handlePreview() {
  String response = "{";
  appendOutputPreviews(response);
  response += "}";
  webServer.send(200, "application/json", response);
}

void startWebServer() {
  webServer.on("/", handleRoot);
  webServer.on("/data", handleData);
  webServer.on("/config", handleConfigPage);
  // The configuration page used to be the WLED page.
  webServer.on("/wled", []() {
    webServer.sendHeader("Location", "/config", true);
    webServer.send(301, "text/plain", "");
  });
  webServer.on("/api/config", HTTP_GET, handleConfigGet);
  webServer.on("/api/config", HTTP_POST, handleConfigPost);
  webServer.on("/api/discard", HTTP_POST, handleConfigDiscard);
  webServer.on("/api/test", HTTP_POST, handleTestLight);
  webServer.on("/api/status", handleStatus);
  webServer.on("/api/wled-info", handleWledInfo);
  webServer.on("/api/scan", handleWledScan);
  webServer.on("/api/preview", handlePreview);
  webServer.on("/api/restart", HTTP_POST, []() {
    webServer.send(200, "application/json", "{}");
    restartRequested = true;
    restartRequestTime = millis();
  });
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
  // Logs every request except the pages' frequent polling and live updates.
  webServer.addHook([](const String &method, const String &url, WiFiClient *client,
                       ESP8266WebServer::ContentTypeFunction) {
    if (url != "/data" && url != "/api/preview" && url != "/api/status" &&
        url != "/api/config" && url != "/api/test") {
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
  logSerial.printf("\nLD2450 boot, reset reason: %s\n", ESP.getResetReason().c_str());
  EEPROM.begin(sizeof(Settings));
  loadSettings();
  buildGammaTable();
  beginLocalStrip();
  // Modem sleep makes the device miss ARP requests, so new clients such as phones cannot reach it.
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  wifiManager.setHostname(settings.hostname);
  wifiManager.setEnableConfigPortal(false);
  wifiManager.setConnectTimeout(5);
  wifiManager.setWiFiAutoReconnect(true);
  wifiManager.setBreakAfterConfig(true);
  wifiManager.setSaveConfigCallback([]() { wifiSetupSaved = true; });
  if (!wifiManager.autoConnect()) {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AccessPointName);
    dnsServer.start(DnsPort, "*", WiFi.softAPIP());
    accessPointMode = true;
    logSerial.printf("No saved Wi-Fi, access point %s at http://%s\n", AccessPointName,
                     WiFi.softAPIP().toString().c_str());
  }
  if (WiFi.status() == WL_CONNECTED) {
    logSerial.printf("Connected to %s, http://%s or http://%s.local\n", WiFi.SSID().c_str(),
                     WiFi.localIP().toString().c_str(), settings.hostname);
    MDNS.begin(settings.hostname);
    String name;
    uint16_t ledCount = 0;
    if (settings.wledEnabled && settings.wledIp != 0 &&
        fetchWledInfo(IPAddress(settings.wledIp), name, ledCount) &&
        ledCount != settings.wledLedCount) {
      settings.wledLedCount = ledCount;
      saveSettings();
    }
    if (settings.wledEnabled) {
      logSerial.printf("WLED %s, %u LEDs\n", IPAddress(settings.wledIp).toString().c_str(),
                       settings.wledLedCount);
    }
  }
  startWebServer();
}

void loop() {
  readSensorFrames();
  updateSensorRate();
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
  serviceOutputs();
  if (restartRequested && millis() - restartRequestTime >= RestartDelayMilliseconds) {
    ESP.restart();
  }
}
