/*
 * GPS Tracker - Phase 3 firmware
 * ESP32-S3-N16R8 + A7608E-H
 *
 * Pipeline: GNSS fix -> compact JSON -> AES-256-GCM -> base64 -> MQTT over LTE.
 * Payload is ciphertext, so even a public broker is a zero-knowledge relay.
 *
 * Phase 3 adds (for real-world / rural use):
 *  - ADAPTIVE REPORTING: fast cadence while the vehicle moves, slow while parked
 *    (+ an immediate report on every moving<->parked transition). Saves data.
 *  - STORE-AND-FORWARD: if a publish fails (no coverage), the reading is queued
 *    in a RAM ring buffer with its own GNSS timestamp; the queue is flushed
 *    (oldest first) once the link is back. No positions lost in rural gaps.
 *  - AUTO-RECOVER: re-power GNSS if it drops (CGNSSINFO -> ERROR), and
 *    re-establish MQTT if a publish fails.
 *
 * Working without USB:
 *  - Firmware goes over Wi-Fi (ArduinoOTA) once the board has joined a network,
 *    so the board only needs power. Wi-Fi is a bench convenience; the tracker
 *    still works with it switched off.
 *  - Logs can be mirrored to an MQTT topic (encrypted like everything else), so
 *    the console is readable from anywhere. Off by default to save airtime.
 *
 * Board settings (upload via right "COM" port = CH343 = COM3):
 *   FQBN esp32:esp32:esp32s3 PSRAM=opi FlashSize=16M CDCOnBoot=default
 *        USBMode=hwcdc UploadSpeed=921600
 * Blob published = base64( IV(12) || ciphertext || GCM-tag(16) )  (Web-Crypto compatible)
 */

#include <math.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
#include "esp_system.h"
#include "mbedtls/gcm.h"
#include "mbedtls/base64.h"

// ---------------- config ----------------
#define MODEM_BAUD   115200
#define MODEM_RX_PIN 16
#define MODEM_TX_PIN 15

// Private settings live in config.h (APN, broker, client id, topics, DEVICE_ID and
// the 32-byte AES_KEY). That file is untracked — copy config.example.h to config.h
// and fill in your own values before building.
#include "config.h"

// Adaptive cadence
// NOTE: TEST values (frequent, so the web-app trail/trajectory is visible while
// walking). For data-saving production use: MOVING_MS 20000 / PARKED_MS 300000 / MOVE_DIST_M 12.
#define MOVE_DIST_M    5.0         // >5 m between 3 s fixes => moving
#define GNSS_POLL_MS   3000UL

// Store-and-forward
#define BUF_CAP        240         // queued readings when offline (~240 * ~130 B)

HardwareSerial &Modem = Serial1;

// state
bool     gnssPowered = false;
bool     mqttUp      = false;
uint32_t lastGnssPoll = 0;
uint32_t lastReport   = 0;
uint32_t lastMqttTry  = 0;
// Modem supply voltage. Below ~3.7 V the LTE transmit bursts start failing and
// the modem silently drops off the network, so this is worth watching.
float    vbat = 0;
uint32_t lastVbat = 0;
uint32_t movMs = 1000, parkMs = 1000;  // report interval (ms); adjustable via MQTT {"interval":sec}
String   rxAccum;                       // accumulates modem bytes for downlink parsing

/* ---- ACCURACY FEATURES: every one runtime-toggleable for debugging ----
   Downlink: {"agps":0|1,"hdopgate":0|1,"statlock":0|1,"jumprej":0|1,"hdopmax":3.0}
   Also {"cmd":"agpsnow"|"lbs"|"report"}                                        */
bool  F_AGPS     = true;    // AT+CAGPS assisted GNSS (ephemeris over LTE, tiny data)
bool  F_HDOPGATE = true;    // drop fixes whose HDOP is too poor
bool  F_STATLOCK = true;    // heavy smoothing while parked (kills wander)
bool  F_JUMPREJ  = true;    // reject physically impossible jumps
bool  F_LBS      = false;   // cell-tower fallback (off until verified on Azercell)
bool  F_KALMAN   = false;   // 1-D Kalman per axis, HDOP-adaptive (test outdoors before enabling)
bool  F_VERBOSE  = false;   // echo raw AT traffic to USB. OFF keeps the console readable at 1 Hz.
/* An active GNSS antenna is powered by DC on the coax (the module's VDD_AUX pin).
   The voltage also sets the antenna's LNA gain, so it is the knob that matches the
   antenna to the module's recommended input level (< 18 dB total, after cable loss).
   Turn the bias OFF for a passive antenna — SIMCom explicitly recommends that. */
bool  F_ANTBIAS  = true;
int   ANT_MV     = 3000;    // allowed: 1200,1250,1700,1800,1850,1900,2500..3300
/* Bench conveniences so the board never has to be tethered to USB:
   F_WIFI  — join Wi-Fi at boot and accept OTA firmware uploads
   F_RLOG  — mirror the console to an MQTT topic (encrypted, costs airtime) */
bool  F_WIFI     = true;
bool  F_RLOG     = false;
bool  otaBusy    = false;   // pause the modem loop while flashing
bool  otaReady   = false;   // true once the OTA port is listening (station OR hosted network)
String logBuf;
uint32_t lastLogPub = 0;
double kfLat = 0, kfLon = 0, kfP = 100;   // filter state + covariance
float HDOP_MAX   = 3.0;     // reject above this
float JUMP_MAX_MPS = 60.0;  // 216 km/h — anything faster is noise
uint32_t rejCount = 0, lastFixMs = 0;

/* Replay protection. Both directions carry a strictly increasing sequence number.
   Counters survive reboots via NVS, so an attacker cannot replay an old frame
   after power-cycling the device. Uplink counters are reserved in blocks to keep
   flash wear low at 1 Hz reporting. */
Preferences prefs;
uint32_t upSeq = 0, upSeqPersisted = 0, cmdRejected = 0;
// Command sequence numbers are wall-clock milliseconds, which overflow 32 bits.
uint64_t lastCmdSeq = 0;
#define SEQ_BLOCK 100
uint32_t gnssMs = 1000;     // GNSS fix rate (ms). Commercial trackers run 1-10 Hz; we were at 0.33 Hz.

bool   haveFix = false;
double fixLat = 0, fixLon = 0;
String fixAlt, fixSpd, fixHdop, fixUtc, fixDate, fixSats;

bool   moving = false;              // current motion state
bool   havePrevFix = false;
double prevLat = 0, prevLon = 0;    // previous fix, for motion detection
bool   reportedOnce = false;        // force one report on the very first fix

// ring buffer of plaintext JSON payloads (encrypted fresh at send time)
String  qbuf[BUF_CAP];
int     qHead = 0, qCount = 0;

// Print a console line and, when remote logging is on, queue it for the log
// topic so the same message is readable without a USB cable.
void say(const String &s) {
  Serial.println(s);
  if (!F_RLOG) return;
  logBuf += s; logBuf += '\n';
  if (logBuf.length() > 1400) logBuf.remove(0, logBuf.length() - 1400);
}

// ---------------- AT helpers ----------------
bool waitFor(const char *token, uint32_t timeoutMs, String &out) {
  uint32_t start = millis(); out = "";
  while (millis() - start < timeoutMs) {
    while (Modem.available()) {
      char c = (char)Modem.read(); out += c; if (F_VERBOSE) Serial.write(c);
      if (out.indexOf(token) >= 0) return true;
    }
    delay(3);
  }
  return false;
}

bool sendAT(const char *cmd, const char *expect, uint32_t timeoutMs) {
  while (Modem.available()) Modem.read();
  if (F_VERBOSE) { Serial.print("\r\n>> "); Serial.println(cmd); }
  Modem.print(cmd); Modem.print("\r\n");
  String out; bool matched = (expect == nullptr || expect[0] == '\0');
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    while (Modem.available()) {
      char c = (char)Modem.read(); out += c; if (F_VERBOSE) Serial.write(c);
      if (!matched && out.indexOf(expect) >= 0) matched = true;
    }
    if ((out.indexOf("OK\r\n") >= 0 || out.indexOf("ERROR") >= 0) && matched) break;
    delay(3);
  }
  return matched;
}

// Send a command and hand back the modem's raw reply, for answers that need parsing.
String atReply(const char *cmd, uint32_t timeoutMs) {
  while (Modem.available()) Modem.read();
  if (F_VERBOSE) { Serial.print("\r\n>> "); Serial.println(cmd); }
  Modem.print(cmd); Modem.print("\r\n");
  String out; uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    while (Modem.available()) { char c = (char)Modem.read(); out += c; if (F_VERBOSE) Serial.write(c); }
    if (out.indexOf("OK\r\n") >= 0 || out.indexOf("ERROR") >= 0) break;
    delay(3);
  }
  return out;
}

bool sendPrompted(const String &cmd, const uint8_t *data, size_t len) {
  while (Modem.available()) Modem.read();
  if (F_VERBOSE) { Serial.print("\r\n>> "); Serial.println(cmd); }
  Modem.print(cmd); Modem.print("\r\n");
  String r;
  if (!waitFor(">", 3000, r)) return false;
  Modem.write(data, len);
  return waitFor("OK", 5000, r);
}

// ---------------- crypto ----------------
size_t encryptPayload(const String &plain, uint8_t *out, size_t outCap) {
  const size_t ivLen = 12, tagLen = 16;
  size_t ptLen = plain.length();
  if (ivLen + ptLen + tagLen > outCap) return 0;
  uint8_t *iv = out, *ct = out + ivLen, *tag = out + ivLen + ptLen;
  for (size_t i = 0; i < ivLen; i++) iv[i] = (uint8_t)(esp_random() & 0xFF);
  mbedtls_gcm_context gcm; mbedtls_gcm_init(&gcm);
  if (mbedtls_gcm_setkey(&gcm, MBEDTLS_CIPHER_ID_AES, AES_KEY, 256) != 0) { mbedtls_gcm_free(&gcm); return 0; }
  int rc = mbedtls_gcm_crypt_and_tag(&gcm, MBEDTLS_GCM_ENCRYPT, ptLen, iv, ivLen, nullptr, 0,
                                     (const uint8_t*)plain.c_str(), ct, tagLen, tag);
  mbedtls_gcm_free(&gcm);
  return rc == 0 ? ivLen + ptLen + tagLen : 0;
}

String base64Of(const uint8_t *data, size_t len) {
  size_t need = 0; mbedtls_base64_encode(nullptr, 0, &need, data, len);
  uint8_t *buf = (uint8_t*)malloc(need + 1); if (!buf) return String();
  size_t wrote = 0; mbedtls_base64_encode(buf, need + 1, &wrote, data, len); buf[wrote] = 0;
  String s = String((char*)buf); free(buf); return s;
}

// Decrypt a base64( IV(12) || ciphertext || tag(16) ) blob. Returns "" if the
// GCM tag does not verify — i.e. wrong key or tampered/forged command.
String decryptPayload(const String &b64) {
  size_t inLen = b64.length();
  uint8_t *raw = (uint8_t*)malloc(inLen); if (!raw) return String();
  size_t rawLen = 0;
  if (mbedtls_base64_decode(raw, inLen, &rawLen, (const uint8_t*)b64.c_str(), inLen) != 0 ||
      rawLen < 12 + 16 + 1) { free(raw); return String(); }
  const size_t ivLen = 12, tagLen = 16, ptLen = rawLen - ivLen - tagLen;
  uint8_t *iv = raw, *ct = raw + ivLen, *tag = raw + ivLen + ptLen;
  uint8_t *pt = (uint8_t*)malloc(ptLen + 1); if (!pt) { free(raw); return String(); }
  mbedtls_gcm_context gcm; mbedtls_gcm_init(&gcm);
  int rc = -1;
  if (mbedtls_gcm_setkey(&gcm, MBEDTLS_CIPHER_ID_AES, AES_KEY, 256) == 0)
    rc = mbedtls_gcm_auth_decrypt(&gcm, ptLen, iv, ivLen, nullptr, 0, tag, tagLen, ct, pt);
  mbedtls_gcm_free(&gcm);
  String out;
  if (rc == 0) { pt[ptLen] = 0; out = String((char*)pt); }
  free(raw); free(pt);
  return out;                                   // empty on auth failure
}

// ---------------- geo ----------------
double distMeters(double la1, double lo1, double la2, double lo2) {
  const double R = 6371000.0, d = 0.017453292519943295;
  double dla = (la2 - la1) * d, dlo = (lo2 - lo1) * d;
  double a = sin(dla/2)*sin(dla/2) + cos(la1*d)*cos(la2*d)*sin(dlo/2)*sin(dlo/2);
  return 2 * R * asin(sqrt(a));
}

// ---------------- LTE + GNSS ----------------
int splitFields(const String &s, String f[], int maxF) {
  int n = 0, start = 0;
  while (n < maxF) {
    int c = s.indexOf(',', start);
    if (c < 0) { f[n++] = s.substring(start); break; }
    f[n++] = s.substring(start, c); start = c + 1;
  }
  return n;
}

bool lteUp() {
  bool alive = false;
  for (int i = 0; i < 10 && !alive; i++) { alive = sendAT("AT", "OK", 1500); if (!alive) delay(800); }
  if (!alive) return false;
  sendAT("ATE0", "OK", 1500);
  sendAT("AT+CPIN?", "READY", 3000);
  sendAT("AT+CMGD=1,4", nullptr, 5000);   // clear SMS storage; a full store blocks the modem
  sendAT("AT+CGDCONT=1,\"IP\",\"" APN "\"", "OK", 3000);
  // Wait for real packet registration. MQTT cannot connect before the network
  // is attached, and after a cold boot that takes tens of seconds.
  for (int i = 0; i < 40; i++) {
    String r = atReply("AT+CGREG?", 2000);
    int k = r.indexOf("+CGREG:");
    if (k >= 0) {
      int c = r.indexOf(',', k);
      if (c > 0) {
        int st = r.substring(c + 1, c + 2).toInt();     // 1 = home, 5 = roaming
        if (st == 1 || st == 5) { say(String("[lte] registered after ") + i + " s"); return true; }
      }
    }
    delay(1000);
  }
  say("[lte] no packet registration yet");
  return false;
}

void gnssOn() {
  sendAT("AT+CGNSSPWR=1", "OK", 5000);
  String r; waitFor("READY!", 15000, r);
  gnssPowered = true;
}

// Poll one CGNSSINFO; updates fix + motion state. Auto-recovers GNSS if it dropped.
void pollGnss() {
  String resp;
  while (Modem.available()) Modem.read();
  if (F_VERBOSE) Serial.print("\r\n>> AT+CGNSSINFO\r\n");
  Modem.print("AT+CGNSSINFO\r\n");
  waitFor("OK", 3000, resp);

  int idx = resp.indexOf("+CGNSSINFO:");
  if (idx < 0) {
    if (resp.indexOf("ERROR") >= 0) {          // GNSS powered off (e.g. modem reset)
      Serial.println("   [GNSS] dropped -> re-powering");
      gnssOn();
    }
    return;
  }
  String data = resp.substring(idx + 11); data.trim();
  String f[20]; int n = splitFields(data, f, 20);
  for (int i = 0; i < n; i++) f[i].trim();

  int li = -1;
  for (int i = 1; i + 1 < n; i++)
    if ((f[i+1] == "N" || f[i+1] == "S") && f[i].length() && isDigit(f[i][0])) { li = i; break; }
  if (f[0].length() == 0 || li < 0) { Serial.println("   [GNSS] no fix yet"); return; }

  double lat = f[li].toDouble();   if (f[li+1] == "S") lat = -lat;
  double lon = f[li+2].toDouble(); if (f[li+3] == "W") lon = -lon;
  double hd  = (li+10 < n) ? f[li+10].toDouble() : 0.0;

  /* ---------- quality filters (toggleable) ---------- */
  if (F_HDOPGATE && hd > HDOP_MAX) {
    rejCount++; Serial.printf("   [GNSS] REJECT hdop %.1f > %.1f\r\n", hd, HDOP_MAX); return; }
  if (F_JUMPREJ && haveFix) {
    double dt = (millis() - lastFixMs) / 1000.0; if (dt < 0.2) dt = 0.2;
    double d  = distMeters(fixLat, fixLon, lat, lon);
    if (d / dt > JUMP_MAX_MPS) {
      rejCount++; Serial.printf("   [GNSS] REJECT jump %.0f m in %.1f s\r\n", d, dt); return; }
  }
  /* motion detection on RAW position (stays responsive) */
  bool wasMoving = moving;
  /* threshold must scale with the fix rate: at 1 Hz a walk is only ~1.4 m per fix,
     so a fixed 5 m would read as "parked". Speed field is used as a second opinion. */
  double effDist = MOVE_DIST_M * (double)gnssMs / 3000.0; if (effDist < 1.5) effDist = 1.5;
  double spdKmh  = (li + 7 < n) ? f[li + 7].toDouble() : 0.0;
  if (havePrevFix) moving = (distMeters(prevLat, prevLon, lat, lon) >= effDist) || (spdKmh > 2.0);
  prevLat = lat; prevLon = lon; havePrevFix = true;
  /* parked -> heavy EMA so the marker stops wandering; moving -> raw for responsiveness */
  if (F_STATLOCK && haveFix && !moving) { lat = fixLat*0.9 + lat*0.1; lon = fixLon*0.9 + lon*0.1; }
  /* optional Kalman: measurement noise scales with HDOP, process noise with motion */
  if (F_KALMAN) {
    if (!haveFix) { kfLat = lat; kfLon = lon; kfP = 100; }
    else {
      double R = (hd > 0 ? hd : 1.0) * 5.0; R *= R;          // metres^2 from HDOP
      double Q = moving ? 9.0 : 0.25;                        // trust motion less when parked
      kfP += Q;
      double K = kfP / (kfP + R);
      kfLat += K * (lat - kfLat); kfLon += K * (lon - kfLon);
      kfP *= (1 - K);
      lat = kfLat; lon = kfLon;
    }
  }

  fixLat = lat; fixLon = lon; lastFixMs = millis();
  fixDate = (li+4 < n) ? f[li+4] : "";
  fixUtc  = (li+5 < n) ? f[li+5] : "";
  fixAlt  = (li+6 < n) ? f[li+6] : "";
  fixSpd  = (li+7 < n) ? f[li+7] : "";
  fixHdop = (li+10 < n) ? f[li+10] : "";
  { long t = 0; for (int i = 1; i < li; i++) t += f[i].toInt(); fixSats = String(t); }
  haveFix = true;

  Serial.printf("   [GNSS] lat=%.6f lon=%.6f hdop=%s sat=%s %s\r\n",
                lat, lon, fixHdop.c_str(), fixSats.c_str(), moving ? "MOVING" : "parked");

  if (!reportedOnce) {                  // first fix -> report right away
    reportedOnce = true;
    reportPosition();
    return;
  }
  if (moving != wasMoving) {            // report immediately on state change
    Serial.println(moving ? "   >> started moving" : "   >> stopped");
    reportPosition();
  }
}

// ---------------- MQTT ----------------
bool mqttConnect() {
  // Clean slate: MQTT state persists on the modem across ESP32 resets, so a
  // stale acquired/connected client (error 19) must be torn down first.
  sendAT("AT+CMQTTSTART", nullptr, 8000);       // ok whether or not already started
  delay(1500);                                  // the stack needs a moment before ACCQ
  sendAT("AT+CMQTTDISC=0,120", nullptr, 5000);  // drop any stale connection (ignore error)
  sendAT("AT+CMQTTREL=0", nullptr, 3000);       // release stale client (ignore error)
  String accq = String("AT+CMQTTACCQ=0,\"") + MQTT_CLIENTID + "\",0";
  sendAT(accq.c_str(), "OK", 3000);
  String conn = String("AT+CMQTTCONNECT=0,\"") + MQTT_BROKER + "\",60,1";
  if (!sendAT(conn.c_str(), "+CMQTTCONNECT: 0,0", 20000)) { say("!! MQTT connect failed"); mqttUp = false; return false; }
  mqttUp = true; say("*** MQTT connected ***");
  String st = String("AT+CMQTTSUBTOPIC=0,") + strlen(CMD_TOPIC) + ",1";
  sendPrompted(st, (const uint8_t*)CMD_TOPIC, strlen(CMD_TOPIC));
  sendAT("AT+CMQTTSUB=0", "OK", 5000);
  Serial.println("[downlink] subscribed to cmd topic");
  return true;
}

// Encrypt a plaintext JSON and publish it to any topic. Returns success.
bool publishTo(const char *topic, const String &json) {
  uint8_t blob[768];
  size_t blobLen = encryptPayload(json, blob, sizeof(blob));
  if (blobLen == 0) { Serial.println("!! encrypt failed (payload too big?)"); return false; }
  String b64 = base64Of(blob, blobLen);
  String tCmd = String("AT+CMQTTTOPIC=0,") + strlen(topic);
  if (!sendPrompted(tCmd, (const uint8_t*)topic, strlen(topic))) return false;
  String pCmd = String("AT+CMQTTPAYLOAD=0,") + b64.length();
  if (!sendPrompted(pCmd, (const uint8_t*)b64.c_str(), b64.length())) return false;
  return sendAT("AT+CMQTTPUB=0,1,60", "+CMQTTPUB: 0,0", 10000);
}
bool publishJson(const String &json) { return publishTo(TOPIC, json); }

// Ship whatever the console has accumulated to the log topic. Encrypted like
// telemetry, because log lines can contain positions.
void publishLog() {
  if (!F_RLOG || !mqttUp || logBuf.length() == 0) return;
  String chunk = logBuf.substring(0, 480);
  logBuf.remove(0, chunk.length());
  chunk.replace("\\", "");  chunk.replace("\"", "'");
  chunk.replace("\r", "");  chunk.replace("\n", " | ");
  publishTo(LOG_TOPIC, String("{\"id\":\"") + DEVICE_ID + "\",\"log\":\"" + chunk + "\"}");
}

// ---------------- store & forward ----------------
void qPush(const String &json) {
  if (qCount < BUF_CAP) { qbuf[(qHead + qCount) % BUF_CAP] = json; qCount++; }
  else { qbuf[qHead] = json; qHead = (qHead + 1) % BUF_CAP; }   // overwrite oldest
  Serial.printf("   [buffer] queued (%d/%d)\r\n", qCount, BUF_CAP);
}

void flushQueue() {
  while (qCount > 0) {
    String &oldest = qbuf[qHead];
    if (!publishJson(oldest)) { Serial.println("   [buffer] link down, keep queued"); mqttUp = false; return; }
    qHead = (qHead + 1) % BUF_CAP; qCount--;
    Serial.printf("   [buffer] flushed one (%d left)\r\n", qCount);
  }
}

// Monotonic uplink sequence, persisted in blocks so it survives reboots without
// wearing flash at 1 Hz. The viewer rejects any frame whose seq is not increasing.
uint32_t nextUpSeq() {
  upSeq++;
  if (upSeq > upSeqPersisted) { upSeqPersisted = upSeq + SEQ_BLOCK; prefs.putUInt("upseq", upSeqPersisted); }
  return upSeq;
}

String buildJson() {
  return String("{\"id\":\"") + DEVICE_ID + "\",\"seq\":" + String(nextUpSeq()) +
         ",\"lat\":" + String(fixLat, 6) +
         ",\"lon\":" + String(fixLon, 6) +
         ",\"alt\":" + (fixAlt.length() ? fixAlt : "0") +
         ",\"spd\":" + (fixSpd.length() ? fixSpd : "0") +
         ",\"hdop\":" + (fixHdop.length() ? fixHdop : "0") +
         ",\"sat\":" + fixSats + ",\"mov\":" + (moving ? "1" : "0") + ",\"rej\":" + String(rejCount) +
         ",\"iv\":" + String(parkMs / 1000) +          // echoes live config back = downlink confirmation
         ",\"kf\":" + String(F_KALMAN ? 1 : 0) +
         ",\"vbat\":" + String(vbat, 2) +
         ",\"utc\":\"" + fixUtc + "\",\"date\":\"" + fixDate + "\"}";
}

void reportPosition() {
  if (!haveFix) { static uint32_t lastMsg=0; if (millis()-lastMsg>15000){lastMsg=millis();Serial.println("   [report] no fix yet, waiting...");} return; }
  lastReport = millis();
  String json = buildJson();
  Serial.print("\r\n---- REPORT ----\r\nplain: "); Serial.println(json);

  if (!mqttUp && !mqttConnect()) { qPush(json); return; }   // offline -> queue
  flushQueue();                                             // drain backlog first (oldest first)
  if (qCount > 0) { qPush(json); return; }                  // still offline
  if (publishJson(json)) Serial.println("*** PUBLISHED (encrypted) ***");
  else { Serial.println("!! publish failed -> queue + reconnect next time"); mqttUp = false; qPush(json); }
  Serial.println("----------------\r\n");
}

// ---------------- setup / loop ----------------
// Join Wi-Fi and open the OTA port. Bench convenience only: if the network is
// not reachable the tracker carries on over LTE exactly as before.
void wifiOtaBegin() {
  otaReady = false;
  if (!F_WIFI) { WiFi.mode(WIFI_OFF); return; }
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 6000) delay(200);

  if (WiFi.status() == WL_CONNECTED) {
    say(String("[wifi] joined \"") + WIFI_SSID + "\", OTA at " + WiFi.localIP().toString());
  } else {
    // No such network to join, so host one. Either way the board is reachable
    // for OTA without a cable; joining an existing network is just more
    // convenient because the laptop keeps its internet connection.
    WiFi.mode(WIFI_AP);
    if (!WiFi.softAP(WIFI_SSID, WIFI_PASS)) {
      say("[wifi] could not start — continuing without OTA");
      WiFi.mode(WIFI_OFF);
      return;
    }
    say(String("[wifi] hosting \"") + WIFI_SSID + "\", OTA at " + WiFi.softAPIP().toString());
  }

  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);
  // Flashing must own the CPU: stop touching the modem until it finishes.
  ArduinoOTA.onStart([]() { otaBusy = true; Serial.println("\r\n[ota] upload started"); });
  ArduinoOTA.onEnd([]()   { Serial.println("[ota] done, rebooting"); });
  ArduinoOTA.onError([](ota_error_t e) { otaBusy = false; Serial.printf("[ota] error %u\r\n", e); });
  ArduinoOTA.onProgress([](unsigned int done, unsigned int total) {
    static int last = -1; int pct = total ? (done * 100 / total) : 0;
    if (pct != last && pct % 10 == 0) { last = pct; Serial.printf("[ota] %d%%\r\n", pct); }
  });
  ArduinoOTA.begin();
  otaReady = true;
}

void readVbat() {
  String r = atReply("AT+CBC", 3000);
  int i = r.indexOf("+CBC:");
  if (i < 0) return;
  vbat = r.substring(i + 5).toFloat();
  if (vbat > 0 && vbat < 3.70)
    say(String("[power] VBAT ") + String(vbat, 3) + " V is low — modem wants 3.8 V, "
        "registration and publishing will fail before anything else does");
}

// Apply the antenna bias setting. Voltage trades LNA gain against front-end
// overload: a 28 dB antenna on a 3 m coax already exceeds what the module wants,
// so dropping to 1800 mV is often the better match.
void applyAntennaBias() {
  if (!F_ANTBIAS) {
    sendAT("AT+CVAUXS=0", "OK", 3000);
    say("[ant] bias OFF (passive antenna)");
    return;
  }
  String v = String("AT+CVAUXV=") + ANT_MV;
  sendAT(v.c_str(), "OK", 3000);
  sendAT("AT+CVAUXS=1", "OK", 3000);
  say(String("[ant] bias ON @ ") + ANT_MV + " mV");
}

// ---------------- downlink (remote commands over MQTT) ----------------
static long jnum(const String &p, const char *key, long def) {
  int i = p.indexOf(key); if (i < 0) return def;
  int c = p.indexOf(':', i); if (c < 0) return def;
  return p.substring(c + 1).toInt();
}
// 64-bit variant: sequence numbers are millisecond timestamps and do not fit in a long.
static uint64_t jnum64(const String &p, const char *key, uint64_t def) {
  int i = p.indexOf(key); if (i < 0) return def;
  int c = p.indexOf(':', i); if (c < 0) return def;
  return strtoull(p.c_str() + c + 1, nullptr, 10);
}
static float jflt(const String &p, const char *key, float def) {
  int i = p.indexOf(key); if (i < 0) return def;
  int c = p.indexOf(':', i); if (c < 0) return def;
  return p.substring(c + 1).toFloat();
}
void handleCommand(const String &p, bool requireSeq) {
  // Replay guard for the radio channel: commands must carry an increasing seq.
  // USB is skipped — holding the cable already means physical access, and this
  // is the channel we need when the radio link is the broken thing.
  if (requireSeq) {
    uint64_t seq = jnum64(p, "\"seq\"", 0);
    if (seq == 0) { cmdRejected++; say("[CMD] rejected: no seq"); return; }
    if (seq <= lastCmdSeq) {
      cmdRejected++;
      say(String("[CMD] REPLAY rejected: seq ") + (unsigned long)(seq % 1000000) + " not newer");
      return;
    }
    lastCmdSeq = seq; prefs.putULong64("cmdseq", lastCmdSeq);
  }

  Serial.print("\r\n[CMD] "); Serial.println(p);
  long n = jnum(p, "\"interval\"", -1);
  if (n >= 1 && n <= 3600) { movMs = parkMs = (uint32_t)n * 1000UL; Serial.printf("[CMD] interval -> %ld s\r\n", n); }
  if (p.indexOf("\"agps\"")     >= 0) F_AGPS     = jnum(p, "\"agps\"", 1);
  if (p.indexOf("\"hdopgate\"") >= 0) F_HDOPGATE = jnum(p, "\"hdopgate\"", 1);
  if (p.indexOf("\"statlock\"") >= 0) F_STATLOCK = jnum(p, "\"statlock\"", 1);
  if (p.indexOf("\"jumprej\"")  >= 0) F_JUMPREJ  = jnum(p, "\"jumprej\"", 1);
  if (p.indexOf("\"lbs\"")      >= 0) F_LBS      = jnum(p, "\"lbs\"", 0);
  if (p.indexOf("\"kalman\"")   >= 0) { F_KALMAN = jnum(p, "\"kalman\"", 0); kfP = 100; }
  if (p.indexOf("\"verbose\"")  >= 0) F_VERBOSE  = jnum(p, "\"verbose\"", 0);
  if (p.indexOf("\"antbias\"")  >= 0) { F_ANTBIAS = jnum(p, "\"antbias\"", 1); applyAntennaBias(); }
  if (p.indexOf("\"antmv\"")    >= 0) { ANT_MV = jnum(p, "\"antmv\"", 3000); applyAntennaBias(); }
  if (p.indexOf("\"rlog\"")     >= 0) F_RLOG = jnum(p, "\"rlog\"", 0);
  if (p.indexOf("\"wifi\"")     >= 0) { F_WIFI = jnum(p, "\"wifi\"", 1); wifiOtaBegin(); }
  if (p.indexOf("\"hdopmax\"")  >= 0) HDOP_MAX   = jflt(p, "\"hdopmax\"", 3.0);
  { long g = jnum(p, "\"gnssms\"", -1); if (g >= 200 && g <= 10000) gnssMs = (uint32_t)g; }
  if (p.indexOf("agpsnow") >= 0) sendAT("AT+CAGPS", "OK", 25000);
  if (p.indexOf("\"lbs\"") >= 0 || p.indexOf("lbsnow") >= 0) sendAT("AT+CLBS=1", "+CLBS", 25000);
  if (p.indexOf("report")  >= 0) lastReport = 0;             // force an immediate report
  char cfg[200];
  snprintf(cfg, sizeof(cfg),
           "[CFG] agps=%d hdopgate=%d(max %.1f) statlock=%d jumprej=%d kalman=%d lbs=%d ant=%d@%dmV "
           "wifi=%d rlog=%d int=%lus gnss=%lums rej=%lu",
           F_AGPS, F_HDOPGATE, HDOP_MAX, F_STATLOCK, F_JUMPREJ, F_KALMAN, F_LBS, F_ANTBIAS, ANT_MV,
           F_WIFI, F_RLOG, (unsigned long)(parkMs/1000), (unsigned long)gnssMs, (unsigned long)rejCount);
  say(cfg);
}
// Same JSON commands, typed straight into the USB console. Also accepts a raw
// AT command so the modem can be probed when nothing else works.
void checkUsbCommand() {
  static String line;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c != '\n' && c != '\r') { if (line.length() < 220) line += c; continue; }
    line.trim();
    if (line.length()) {
      if (line.startsWith("{")) handleCommand(line, false);
      else if (line.startsWith("AT")) { String r = atReply(line.c_str(), 8000); Serial.println(r); }
      else Serial.println("[usb] send {\"key\":value} or an AT command");
    }
    line = "";
  }
}

void checkDownlink() {
  while (Modem.available()) { char c = (char)Modem.read(); rxAccum += c; if (F_VERBOSE) Serial.write(c); }
  int ps = rxAccum.indexOf("+CMQTTRXPAYLOAD:");
  int pe = rxAccum.indexOf("+CMQTTRXEND");
  if (ps >= 0 && pe > ps) {
    int nl = rxAccum.indexOf('\n', ps);
    if (nl >= 0 && nl < pe) {
      String enc = rxAccum.substring(nl + 1, pe); enc.trim();
      String pl = decryptPayload(enc);            // GCM verifies authenticity
      if (pl.length() == 0) { cmdRejected++; Serial.println("[CMD] rejected: decrypt/auth failed"); }
      else handleCommand(pl, true);
    }
    rxAccum = rxAccum.substring(pe + 11);
  }
  if (rxAccum.length() > 600) rxAccum = rxAccum.substring(rxAccum.length() - 600);
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis(); while (!Serial && millis() - t0 < 2000) delay(10);
  delay(300);
  Serial.println("\r\n#### GPS Tracker (Phase 3: adaptive + store-and-forward) ####");
  Modem.begin(MODEM_BAUD, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);
  delay(200);

  prefs.begin("trk", false);                       // replay counters (survive reboot)
  lastCmdSeq     = prefs.getULong64("cmdseq", 0);
  upSeqPersisted = prefs.getUInt("upseq", 0);
  upSeq          = upSeqPersisted;                 // resume above the last reserved block
  Serial.printf("[seq] uplink>=%lu  lastCmd=%lu\r\n", (unsigned long)upSeq, (unsigned long)lastCmdSeq);

  wifiOtaBegin();                                  // OTA first, so a bad build is still recoverable
  if (!lteUp()) say("!! LTE bringup problem");
  mqttConnect();                                   // command channel before the slow GNSS steps,
                                                   // so the device is reachable as early as possible
  sendAT("AT+CGNSSMODE?", nullptr, 3000);          // log which constellations are active
  applyAntennaBias();                              // power the antenna before it has to acquire
  if (F_AGPS) sendAT("AT+CAGPS", "OK", 25000);     // assisted GNSS: ephemeris over LTE (~tens of KB)
  gnssOn();
  Serial.println("Ready. Adaptive reporting: 20s moving / 5min parked, +transitions. Offline -> buffered.\r\n");
}

void loop() {
  if (otaReady) ArduinoOTA.handle();
  if (otaBusy) return;                       // flashing owns the CPU

  checkUsbCommand();
  checkDownlink();
  // Keep trying to reconnect even with no fix — otherwise a device that boots
  // before the network is up stays silent and unreachable by command.
  if (!mqttUp && millis() - lastMqttTry >= 20000) { lastMqttTry = millis(); mqttConnect(); }
  if (gnssPowered && millis() - lastGnssPoll >= gnssMs) { lastGnssPoll = millis(); pollGnss(); }
  uint32_t due = moving ? movMs : parkMs;
  if (millis() - lastReport >= due) reportPosition();
  if (millis() - lastVbat >= 30000) { lastVbat = millis(); readVbat(); }
  if (F_RLOG && millis() - lastLogPub >= 10000) { lastLogPub = millis(); publishLog(); }
}
