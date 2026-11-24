// XIAO ESP32-C6 + BMP180 altimeter (built-in BLE library version) + REST API
// - Configure Wi-Fi and sea-level pressure over Bluetooth LE (nRF Connect / LightBlue)
// - Joins your router as a Wi-Fi client; open http://altimeter.local (or the IP) on your phone
// - REST API under /api/v1/... for other devices (see the table below)
//
// Needs: Arduino-ESP32 core 3.x and the Adafruit BMP085 library.
// Do NOT have the NimBLE-Arduino library installed for this sketch (it fails to link on the C6).
// BLE callbacks only set flags; all real work (flash writes, Wi-Fi) happens in loop().
// Status is read-only: tap "Read" on the Status characteristic to refresh it.
//
// REST API
//   GET  /api/v1/altitude    full reading (JSON)
//   GET  /api/v1/status      Wi-Fi / uptime info (JSON)
//   GET  /api/v1/sealevel    current sea-level pressure in hPa
//   PUT  /api/v1/sealevel    set it:  hpa=1013.2  (form/query) or {"hpa":1013.2}
//   POST /api/v1/calibrate   calibrate to a known elevation: m=123 or ft=400 (form/query) or JSON
//
// Legacy routes used by the built-in web page still work: /data, /sl, /cal

#include <WiFi.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <Wire.h>
#include <Preferences.h>
#include <time.h>
#include <Adafruit_BMP085.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

// ---------- BLE identifiers ----------
#define BLE_NAME        "AltimeterC6"
#define SVC_UUID        "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define CHR_SSID_UUID   "6e400002-b5a3-f393-e0a9-e50e24dcca9e"   // write: Wi-Fi SSID (text)
#define CHR_PASS_UUID   "6e400003-b5a3-f393-e0a9-e50e24dcca9e"   // write: Wi-Fi password (text)
#define CHR_CMD_UUID    "6e400004-b5a3-f393-e0a9-e50e24dcca9e"   // write: "connect" | "forget" | "reboot"
#define CHR_STATUS_UUID "6e400005-b5a3-f393-e0a9-e50e24dcca9e"   // read: status text + IP
#define CHR_SL_UUID     "6e400006-b5a3-f393-e0a9-e50e24dcca9e"   // read/write: sea-level pressure in hPa

const char* HOSTNAME = "altimeter";      // -> http://altimeter.local

// Optional API key for the WRITE endpoints (PUT /sealevel, POST /calibrate).
// Leave empty ("") to disable. Clients then send the header:  X-API-Key: <key>
// Note: plain HTTP, so this is only light protection on a trusted home network.
// The legacy /sl and /cal routes used by the built-in web page are NOT protected by this key.
const char* API_KEY = "";

Adafruit_BMP085 bmp;
bool bmpOk = false;
WebServer server(80);
Preferences prefs;

float  seaLevelPa = 101325.0;
String wifiSsid, wifiPass;

BLECharacteristic *chStatus = nullptr;
BLECharacteristic *chSL     = nullptr;

// Flags set by BLE callbacks; handled in loop()
volatile bool doConnect  = false;
volatile bool doReboot   = false;
volatile bool doForget   = false;
volatile bool saveCreds  = false;
volatile bool saveSL     = false;
String lastStatus;

// Cached sensor reading (refreshed once per second in loop())
float    cachedTempC     = 0;
float    cachedPressurePa = 0;
uint32_t cachedAtMs      = 0;
bool     haveReading     = false;

const char PAGE[] PROGMEM = R"HTML(
<!doctype html><html><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
 body{ 
    font-family: -apple-system, sans-serif; 
    text-align: center; 
    margin-top: 6vh; 
    padding: 0 16px;
  }
  #m{
    font-size: 4.5rem;
    font-weight: 700;
  } 
  #ft{ font-size:1.6rem;
    color: #666;
  }
  input{ 
    font-size:1.1rem;
    padding: .4em;
    width: 7em;
  } 
  button{
    font-size: 1.1rem;
    padding: .4em .9em;
  }
  .box{
    margin-top: 1.5em;
  } 
  small{
    color: #888;
  }
</style></head>

  <body>
    <div id="m">--</div><div>meters above sea level</div>
    <div id="ft">--</div>
    <p id="p">--</p><p id="t">--</p>
    <div class="box">
      <input id="sl" type="number" step="0.1" placeholder="hPa">
      <button onclick="setSL()">Set sea-level pressure</button><br>
      <small id="cur"></small>
    </div>

    <div class="box">
      <input id="el" type="number" step="1" placeholder="feet">
      <button onclick="calAlt()">Calibrate to known elevation</button>
    </div>

  <script>
    const $=id=>document.getElementById(id);
    async function tick() {
      try {
        const d=await (await fetch('/data')).json();
        $('m').textContent=d.m.toFixed(1)+' m';
        $('ft').textContent=d.ft.toFixed(0)+' ft';
        $('p').textContent=d.pressure.toFixed(2)+' hPa';
        $('t').textContent=d.tc.toFixed(1)+'°C  /  '+d.tf.toFixed(1)+'°F';
        $('cur').textContent='Sea-level pressure in use: '+d.sl.toFixed(1)+' hPa';
      } catch(e) {}

      setTimeout(tick,500);
    }
async function setSL(){ if($('sl').value) await fetch('/sl?hpa='+$('sl').value); }
async function calAlt(){ if($('el').value) await fetch('/cal?m='+($('el').value/3.28084)); }
tick();
</script></body></html>
)HTML";

// ---------- helpers (call from loop context only) ----------
void setStatus(const String &s) {
  if (s == lastStatus) return;
  lastStatus = s;
  Serial.println("Status: " + s);
  if (chStatus) chStatus->setValue(s);
}

void persistSeaLevel() {
  prefs.putFloat("sl", seaLevelPa);
  if (chSL) chSL->setValue(String(seaLevelPa / 100.0, 1));
}

void connectWiFi() {
  if (wifiSsid.length() == 0) {
    setStatus("No Wi-Fi credentials saved");
    return;
  }
  setStatus("Connecting to " + wifiSsid + "...");
  WiFi.disconnect(true);
  delay(100);
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(HOSTNAME);
  WiFi.setAutoReconnect(true);
  WiFi.begin(wifiSsid.c_str(), wifiPass.c_str());

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    delay(250);
  }

  if (WiFi.status() == WL_CONNECTED) {
    MDNS.end();
    MDNS.begin(HOSTNAME);
    MDNS.addService("http", "tcp", 80);
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");   // real timestamps in the API (UTC)
    setStatus("Connected " + WiFi.localIP().toString());
  } else {
    setStatus("Wi-Fi connect failed (check SSID/password)");
  }
}

// Read the sensor at most once per second and cache the result, so many
// API clients polling quickly don't hammer the I2C bus.
void updateSensor() {
  if (!bmpOk) return;
  if (haveReading && millis() - cachedAtMs < 1000) return;
  cachedTempC      = bmp.readTemperature();
  cachedPressurePa = bmp.readPressure();
  cachedAtMs       = millis();
  haveReading      = true;
}

float altitudeFor(float pressurePa, float slPa) {
  return 44330.0f * (1.0f - pow(pressurePa / slPa, 0.1903f));
}

// ---------- BLE callbacks: set flags only ----------
class ServerCB : public BLEServerCallbacks {
  void onDisconnect(BLEServer*) override {
    BLEDevice::startAdvertising();         // stay discoverable
  }
};

class SsidCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    String v = c->getValue();
    v.trim();
    wifiSsid = v;
    saveCreds = true;
  }
};

class PassCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    wifiPass = c->getValue();
    saveCreds = true;
  }
};

class CmdCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    String cmd = c->getValue();
    cmd.trim();
    cmd.toLowerCase();
    if (cmd == "connect")      doConnect = true;
    else if (cmd == "forget")  doForget  = true;
    else if (cmd == "reboot")  doReboot  = true;
  }
};

class SlCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    float hpa = String(c->getValue()).toFloat();
    if (hpa > 900 && hpa < 1100) {
      seaLevelPa = hpa * 100.0;
      saveSL = true;
    }
  }
};

void startBLE() {
  BLEDevice::init(BLE_NAME);
  BLEServer *srv = BLEDevice::createServer();
  srv->setCallbacks(new ServerCB());

  BLEService *svc = srv->createService(SVC_UUID);

  BLECharacteristic *c;

  c = svc->createCharacteristic(CHR_SSID_UUID, BLECharacteristic::PROPERTY_WRITE);
  c->setCallbacks(new SsidCB());

  c = svc->createCharacteristic(CHR_PASS_UUID, BLECharacteristic::PROPERTY_WRITE);
  c->setCallbacks(new PassCB());

  c = svc->createCharacteristic(CHR_CMD_UUID, BLECharacteristic::PROPERTY_WRITE);
  c->setCallbacks(new CmdCB());

  chStatus = svc->createCharacteristic(CHR_STATUS_UUID, BLECharacteristic::PROPERTY_READ);
  chStatus->setValue("Booting");

  chSL = svc->createCharacteristic(CHR_SL_UUID,
           BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  chSL->setCallbacks(new SlCB());
  chSL->setValue(String(seaLevelPa / 100.0, 1));

  svc->start();

  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SVC_UUID);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();
}

// ---------- API helpers ----------
void addCors() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET, POST, PUT, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type, X-API-Key");
}

void sendJson(int code, const String &body) {
  addCors();
  server.sendHeader("Cache-Control", "no-store");
  server.send(code, "application/json", body);
}

void sendError(int code, const String &msg) {
  sendJson(code, "{\"error\":\"" + msg + "\"}");
}

// Answers CORS preflight requests. Returns true if the request was an OPTIONS call.
bool handlePreflight() {
  if (server.method() == HTTP_OPTIONS) {
    addCors();
    server.send(204);
    return true;
  }
  return false;
}

// Returns true if the caller may proceed with a write request.
bool authorized() {
  if (strlen(API_KEY) == 0) return true;
  if (server.header("X-API-Key") == API_KEY) return true;
  sendError(401, "unauthorized");
  return false;
}

String jsonEscape(const String &s) {
  String o;
  for (size_t i = 0; i < s.length(); i++) {
    char ch = s[i];
    if (ch == '"' || ch == '\\') { o += '\\'; o += ch; }
    else if ((uint8_t)ch < 0x20) { o += ' '; }
    else o += ch;
  }
  return o;
}

// Looks for a numeric parameter in form/query args first, then in a JSON body
// such as {"hpa":1013.2}. Returns true and fills 'out' if found.
bool getNumber(const char *name, float &out) {
  if (server.hasArg(name)) {
    out = server.arg(name).toFloat();
    return true;
  }
  if (server.hasArg("plain")) {
    String body = server.arg("plain");
    String key = String("\"") + name + "\"";
    int k = body.indexOf(key);
    if (k >= 0) {
      int colon = body.indexOf(':', k + key.length());
      if (colon >= 0) {
        out = body.substring(colon + 1).toFloat();
        return true;
      }
    }
  }
  return false;
}

bool applySeaLevel(float hpa) {
  if (!(hpa > 900 && hpa < 1100)) return false;
  seaLevelPa = hpa * 100.0;
  persistSeaLevel();
  return true;
}

bool applyCalibration(float meters) {
  if (!bmpOk) return false;
  if (!(meters > -500 && meters < 9000)) return false;
  float p = bmp.readPressure();
  seaLevelPa = p / pow(1.0 - meters / 44330.0, 5.255);
  persistSeaLevel();
  return true;
}

// ---------- API handlers ----------
// GET /api/v1/altitude  (also served at the legacy /data)
void handleData() {
  if (handlePreflight()) return;
  if (server.method() != HTTP_GET) { sendError(405, "method not allowed"); return; }
  if (!bmpOk || !haveReading) { sendError(503, "sensor not found"); return; }

  float tc = cachedTempC;
  float tf = tc * 9.0 / 5.0 + 32.0;
  float m  = altitudeFor(cachedPressurePa, seaLevelPa);

  time_t now = time(nullptr);
  bool timeOk = now > 1700000000;          // false until NTP has synced

  String json = "{\"m\":" + String(m, 2) +
                ",\"ft\":" + String(m * 3.28084, 1) +
                ",\"pressure\":" + String(cachedPressurePa / 100.0, 2) +
                ",\"tc\":" + String(tc, 2) +
                ",\"tf\":" + String(tf, 2) +
                ",\"sl\":" + String(seaLevelPa / 100.0, 2) +
                ",\"age_ms\":" + String(millis() - cachedAtMs) +
                ",\"uptime_s\":" + String(millis() / 1000) +
                ",\"time\":" + (timeOk ? String((unsigned long)now) : String("null")) + "}";
  sendJson(200, json);
}

// GET /api/v1/status
void handleStatus() {
  if (handlePreflight()) return;
  if (server.method() != HTTP_GET) { sendError(405, "method not allowed"); return; }
  bool conn = WiFi.status() == WL_CONNECTED;
  String json = String("{\"sensor_ok\":") + (bmpOk ? "true" : "false") +
                ",\"wifi_connected\":" + (conn ? "true" : "false") +
                ",\"ssid\":\"" + jsonEscape(conn ? WiFi.SSID() : String("")) + "\"" +
                ",\"ip\":\"" + (conn ? WiFi.localIP().toString() : String("")) + "\"" +
                ",\"rssi\":" + String(conn ? WiFi.RSSI() : 0) +
                ",\"hostname\":\"" + HOSTNAME + "\"" +
                ",\"uptime_s\":" + String(millis() / 1000) + "}";
  sendJson(200, json);
}

// GET / PUT / POST /api/v1/sealevel
void handleSeaLevel() {
  if (handlePreflight()) return;

  if (server.method() == HTTP_GET) {
    sendJson(200, "{\"hpa\":" + String(seaLevelPa / 100.0, 2) + "}");
    return;
  }
  if (server.method() == HTTP_PUT || server.method() == HTTP_POST) {
    if (!authorized()) return;
    float hpa;
    if (!getNumber("hpa", hpa)) { sendError(400, "missing hpa"); return; }
    if (!applySeaLevel(hpa))    { sendError(400, "hpa must be between 900 and 1100"); return; }
    sendJson(200, "{\"hpa\":" + String(seaLevelPa / 100.0, 2) + "}");
    return;
  }
  sendError(405, "method not allowed");
}

// POST /api/v1/calibrate   (m=<meters> or ft=<feet>)
void handleCalibrate() {
  if (handlePreflight()) return;
  if (server.method() != HTTP_POST) { sendError(405, "method not allowed"); return; }
  if (!authorized()) return;
  if (!bmpOk) { sendError(503, "sensor not found"); return; }

  float meters;
  float feet;
  if (getNumber("m", meters)) {
    // meters given directly
  } else if (getNumber("ft", feet)) {
    meters = feet / 3.28084;
  } else {
    sendError(400, "provide m (meters) or ft (feet)");
    return;
  }
  if (!applyCalibration(meters)) { sendError(400, "elevation out of range"); return; }
  sendJson(200, "{\"hpa\":" + String(seaLevelPa / 100.0, 2) + "}");
}

void handleNotFound() {
  if (handlePreflight()) return;
  sendError(404, "not found");
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Wire.begin(22, 23);                               // SDA = D4, SCL = D5

  prefs.begin("alt", false);
  seaLevelPa = prefs.getFloat("sl", 101325.0);
  wifiSsid   = prefs.getString("ssid", "");
  wifiPass   = prefs.getString("pass", "");

  WiFi.persistent(false);                           // we store credentials ourselves
  WiFi.mode(WIFI_STA);                              // start the network stack BEFORE the web server,
                                                    // even when no credentials are saved yet
  Serial.println("Setup: Wi-Fi stack started");

  startBLE();                                       // BLE first, so setup works even if the sensor is missing
  Serial.println("Setup: BLE started");

  bmpOk = bmp.begin();
  if (!bmpOk) {
    Serial.println("BMP180 not found, check wiring");
  } else {
    updateSensor();                                 // prime the cache
  }

  // Web page
  server.on("/", []() { server.send_P(200, "text/html", PAGE); });

  // REST API (all methods are routed here; handlers check the method themselves)
  server.on("/api/v1/altitude",  handleData);
  server.on("/api/v1/status",    handleStatus);
  server.on("/api/v1/sealevel",  handleSeaLevel);
  server.on("/api/v1/calibrate", handleCalibrate);

  // Legacy routes used by the built-in page
  server.on("/data", handleData);
  server.on("/sl", []() {
    float hpa = server.arg("hpa").toFloat();
    applySeaLevel(hpa);
    server.send(200, "text/plain", "ok");
  });
  server.on("/cal", []() {
    if (!bmpOk) { server.send(503, "text/plain", "sensor not found"); return; }
    applyCalibration(server.arg("m").toFloat());
    server.send(200, "text/plain", "ok");
  });

  server.onNotFound(handleNotFound);

  // Needed so server.header("X-API-Key") works. Must be called before server.begin().
  const char *hdrs[] = {"X-API-Key"};
  server.collectHeaders(hdrs, 1);

  server.begin();                                   // fine to start before Wi-Fi is up

  if (wifiSsid.length() > 0) {
    connectWiFi();
  } else {
    setStatus("Waiting for Wi-Fi setup over BLE");
  }
}

void loop() {
  server.handleClient();
  updateSensor();

  if (saveCreds) {
    saveCreds = false;
    prefs.putString("ssid", wifiSsid);
    prefs.putString("pass", wifiPass);
    setStatus("Credentials saved");
  }
  if (saveSL) {
    saveSL = false;
    persistSeaLevel();
  }
  if (doForget) {
    doForget = false;
    wifiSsid = ""; wifiPass = "";
    prefs.remove("ssid");
    prefs.remove("pass");
    WiFi.disconnect(true);
    lastStatus = "";
    setStatus("Wi-Fi credentials cleared");
  }
  if (doConnect) {
    doConnect = false;
    connectWiFi();
  }
  if (doReboot) {
    delay(200);
    ESP.restart();
  }

  // Keep the status text current if Wi-Fi drops or comes back
  static uint32_t lastCheck = 0;
  if (millis() - lastCheck > 2000) {
    lastCheck = millis();
    if (wifiSsid.length() > 0) {
      if (WiFi.status() == WL_CONNECTED) {
        setStatus("Connected " + WiFi.localIP().toString());
      } else if (lastStatus.startsWith("Connected")) {
        setStatus("Wi-Fi disconnected, retrying...");
      }
    }
  }
}
