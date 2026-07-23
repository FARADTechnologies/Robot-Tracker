/*
 * A7608E-H (BK-A7608 breakout) AT + GNSS test
 * Board: ESP32-S3-N16R8 (16MB flash, 8MB OPI PSRAM)
 *
 * Wiring:
 *   ESP32-S3 GPIO15 (TX) --> A7608 "R" (RXD)
 *   ESP32-S3 GPIO16 (RX) --> A7608 "T" (TXD)
 *   GND ------------------- common
 *   Antennas: LTE on J104, GNSS on J101 (both required).
 *
 * Serial  = UART0 (GPIO43/44) -> CH343 serial chip -> COM3 -> PC monitor.
 *           (Right "COM" port, USB CDC On Boot = Disabled.)
 * Serial1 = modem UART, 115200, RX=16 TX=15
 *
 * Sequence:
 *   1) LTE boot test  (AT, ATI, CPIN?, CSQ, CREG?, CGREG?, CPSI?, COPS?)
 *   2) GNSS power-on  (CGNSSPWR=1, wait for "+CGNSSPWR: READY!")
 *   3) Poll CGNSSINFO every 3 s, parse + print position until a fix is found,
 *      then keep printing periodically.
 *   4) Transparent bridge stays active the whole time: type AT commands in
 *      the serial monitor and they go straight to the modem.
 *
 * NOTE (power): bulk cap (1000uF) not yet fitted. GNSS + LTE together draw
 * more; if the board resets, check for BROWNOUT in the reset reason below.
 */

#include "esp_system.h"

#define MODEM_BAUD   115200
#define MODEM_RX_PIN 16   // ESP32 RX  <- modem T (TXD)
#define MODEM_TX_PIN 15   // ESP32 TX  -> modem R (RXD)

#define GNSS_POLL_MS 3000 // how often to ask CGNSSINFO

// --- Step 2: LTE upload ---
#define APN         "internet"                                        // Azercell
// Put your own test endpoint here (e.g. a fresh https://webhook.site URL).
#define WEBHOOK_URL "http://example.com/your-test-endpoint"
// To save mobile data the position is POSTed only ONCE, on the first GNSS fix.

HardwareSerial &Modem = Serial1;

bool     gnssPowered = false;
bool     gnssFirstFix = false;
uint32_t lastGnssPoll = 0;

bool     httpBusy   = false;   // while true, loop() leaves the modem to httpPost()
bool     gnssPosted = false;   // ensure we upload only once
double   fixLat = 0, fixLon = 0;
String   fixAlt = "", fixUtc = "", fixDate = "", fixSats = "";

void httpPost(const String &body);   // fwd decl (called from pollGnss)

static const char *resetReasonStr(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:   return "POWERON (normal cold boot)";
    case ESP_RST_EXT:       return "EXT (external pin)";
    case ESP_RST_SW:        return "SW (software reset)";
    case ESP_RST_PANIC:     return "PANIC (exception/panic)";
    case ESP_RST_INT_WDT:   return "INT_WDT (interrupt watchdog)";
    case ESP_RST_TASK_WDT:  return "TASK_WDT (task watchdog)";
    case ESP_RST_WDT:       return "WDT (other watchdog)";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:  return "BROWNOUT  <-- power dip! check 5V rail / add bulk cap";
    default:                return "UNKNOWN";
  }
}

// Send an AT command; echo the reply live to the monitor and also return it
// in `out`. Returns true if `expect` was seen (empty expect => always true).
bool sendATCapture(const char *cmd, const char *expect, uint32_t timeoutMs, String &out) {
  while (Modem.available()) Modem.read();       // drain leftovers
  out = "";

  Serial.print("\r\n>> ");
  Serial.println(cmd);
  Modem.print(cmd);
  Modem.print("\r\n");

  uint32_t start = millis();
  bool matched = (expect == nullptr || expect[0] == '\0');
  while (millis() - start < timeoutMs) {
    while (Modem.available()) {
      char c = (char)Modem.read();
      out += c;
      Serial.write(c);
      if (!matched && out.indexOf(expect) >= 0) matched = true;
    }
    if (out.indexOf("OK\r\n") >= 0 || out.indexOf("ERROR") >= 0) {
      if (matched) break;
    }
    delay(5);
  }

  Serial.print("   [");
  Serial.print(cmd);
  Serial.print("] -> ");
  Serial.println(matched ? "OK / matched" : "TIMEOUT or no match");
  return matched;
}

bool sendAT(const char *cmd, const char *expect, uint32_t timeoutMs) {
  String dummy;
  return sendATCapture(cmd, expect, timeoutMs, dummy);
}

void runLteBootTest() {
  Serial.println(F("\r\n================ A7608 LTE BOOT TEST ================"));

  bool alive = false;
  for (int i = 1; i <= 10 && !alive; i++) {
    Serial.printf("AT probe attempt %d/10...\r\n", i);
    alive = sendAT("AT", "OK", 1500);
    if (!alive) delay(1000);
  }
  if (!alive) {
    Serial.println(F("!! Modem not responding to AT. Check power / PWRKEY / wiring."));
    return;
  }

  sendAT("ATE0",      "OK",    1500);
  sendAT("ATI",       "OK",    2000);
  sendAT("AT+CPIN?",  "READY", 3000);   // SIM ready?
  sendAT("AT+CSQ",    "+CSQ",  2000);   // signal quality
  sendAT("AT+CREG?",  "+CREG", 3000);   // circuit-switched registration
  sendAT("AT+CGREG?", "+CGREG",3000);   // packet registration
  sendAT("AT+CPSI?",  "+CPSI", 5000);   // serving cell: tech + band
  sendAT("AT+COPS?",  "+COPS", 5000);   // registered operator (expect Azercell)

  Serial.println(F("================ LTE BOOT TEST DONE ================\r\n"));
}

// Split a String on ',' into fields[] (up to maxF). Returns count.
int splitFields(const String &s, String fields[], int maxF) {
  int count = 0, start = 0;
  while (count < maxF) {
    int comma = s.indexOf(',', start);
    if (comma < 0) { fields[count++] = s.substring(start); break; }
    fields[count++] = s.substring(start, comma);
    start = comma + 1;
  }
  return count;
}

void powerOnGnss() {
  Serial.println(F("---------------- GNSS POWER-ON ----------------"));
  String resp;
  sendATCapture("AT+CGNSSPWR=1", "OK", 5000, resp);

  // Wait for the "+CGNSSPWR: READY!" URC (GNSS chip warmed up).
  Serial.println(F("Waiting for +CGNSSPWR: READY! ..."));
  uint32_t start = millis();
  String buf;
  while (millis() - start < 15000) {
    while (Modem.available()) {
      char c = (char)Modem.read();
      buf += c;
      Serial.write(c);
    }
    if (buf.indexOf("READY!") >= 0) { Serial.println(F("\r\n[GNSS READY]")); break; }
    delay(10);
  }
  gnssPowered = true;
  Serial.println(F("---- GNSS on. Polling CGNSSINFO every 3s ----\r\n"));
}

void pollGnss() {
  String resp;
  sendATCapture("AT+CGNSSINFO", "+CGNSSINFO", 3000, resp);

  int idx = resp.indexOf("+CGNSSINFO:");
  if (idx < 0) return;
  String data = resp.substring(idx + 11);
  data.trim();

  // A7608E-H format (decimal degrees, Galileo count included -> lat at idx 5):
  // <mode>,<GPS>,<GLO>,<BDS>,<GAL>,<lat>,<N/S>,<lon>,<E/W>,<date>,<UTC>,<alt>,...
  // No fix looks like "+CGNSSINFO: ,,,,,,,,".  Locate the latitude field
  // robustly: a numeric field immediately followed by "N" or "S".
  String f[20];
  int n = splitFields(data, f, 20);
  for (int i = 0; i < n; i++) f[i].trim();

  int li = -1;
  for (int i = 1; i + 1 < n; i++) {
    if ((f[i + 1] == "N" || f[i + 1] == "S") &&
        f[i].length() > 0 && isDigit(f[i][0])) { li = i; break; }
  }
  if (f[0].length() == 0 || li < 0) {
    Serial.println(F("   [GNSS] no fix yet (searching satellites)..."));
    return;
  }

  double lat = f[li].toDouble();       if (f[li + 1] == "S") lat = -lat;
  double lon = f[li + 2].toDouble();   if (f[li + 3] == "W") lon = -lon;

  if (!gnssFirstFix) {
    gnssFirstFix = true;
    Serial.println(F("\r\n*** GNSS FIX ACQUIRED ***"));
  }
  // stash latest fix for the LTE upload
  fixLat  = lat;  fixLon = lon;
  fixSats = f[1] + "/" + f[2] + "/" + f[3];
  fixAlt  = (li + 6 < n) ? f[li + 6] : "";
  fixUtc  = (li + 5 < n) ? f[li + 5] : "";
  fixDate = (li + 4 < n) ? f[li + 4] : "";

  Serial.printf("   [GNSS] mode=%s sats(GPS/GLO/BDS)=%s  lat=%.6f lon=%.6f",
                f[0].c_str(), fixSats.c_str(), lat, lon);
  if (fixAlt.length()) Serial.printf("  alt=%sm", fixAlt.c_str());
  if (fixUtc.length()) Serial.printf("  UTC=%s date=%s", fixUtc.c_str(), fixDate.c_str());
  Serial.printf("  maps: https://maps.google.com/?q=%.6f,%.6f\r\n", lat, lon);

  // Upload once, on the first valid fix (data-saving).
  if (!gnssPosted) {
    gnssPosted = true;
    String json = String("{\"lat\":") + String(fixLat, 6) +
                  ",\"lon\":" + String(fixLon, 6) +
                  ",\"alt\":" + (fixAlt.length() ? fixAlt : "0") +
                  ",\"utc\":\"" + fixUtc + "\",\"date\":\"" + fixDate +
                  "\",\"sats\":\"" + fixSats + "\"}";
    httpPost(json);
  }
}

// Read the modem until `token` appears (or timeout), echoing to the monitor.
bool waitFor(const char *token, uint32_t timeoutMs, String &out) {
  uint32_t start = millis();
  out = "";
  while (millis() - start < timeoutMs) {
    while (Modem.available()) {
      char c = (char)Modem.read();
      out += c;
      Serial.write(c);
      if (out.indexOf(token) >= 0) return true;
    }
    delay(5);
  }
  return false;
}

// One-shot HTTP POST of a JSON body over LTE (A76xx HTTP AT service).
void httpPost(const String &body) {
  httpBusy = true;
  Serial.println(F("\r\n---------------- LTE HTTP POST ----------------"));
  Serial.print(F("Payload: ")); Serial.println(body);

  sendAT("AT+HTTPTERM", nullptr, 800);      // close any stale session (ignore error)
  if (!sendAT("AT+HTTPINIT", "OK", 8000)) {
    Serial.println(F("!! HTTPINIT failed - data/PDP not ready? Check CGATT/APN."));
    httpBusy = false; return;
  }
  String urlCmd = String("AT+HTTPPARA=\"URL\",\"") + WEBHOOK_URL + "\"";
  sendAT(urlCmd.c_str(), "OK", 3000);
  sendAT("AT+HTTPPARA=\"CONTENT\",\"application/json\"", "OK", 3000);

  // Hand the body to the modem
  String dataCmd = String("AT+HTTPDATA=") + body.length() + ",10000";
  String r;
  sendATCapture(dataCmd.c_str(), "DOWNLOAD", 3000, r);   // module replies "DOWNLOAD"
  Modem.print(body);                                     // raw JSON (no CR/LF)
  waitFor("OK", 12000, r);                               // module confirms receipt

  // Fire it; result comes async as  +HTTPACTION: 1,<code>,<len>
  sendAT("AT+HTTPACTION=1", "OK", 5000);
  String act;
  if (waitFor("+HTTPACTION:", 45000, act)) {
    // The code+len follow on the same line; read until end-of-line before parsing.
    uint32_t t = millis();
    while (millis() - t < 1500) {
      while (Modem.available()) { char c = (char)Modem.read(); act += c; Serial.write(c); }
      if (act.indexOf('\n', act.indexOf("+HTTPACTION:")) >= 0) break;
      delay(5);
    }
    String line = act.substring(act.indexOf("+HTTPACTION:"));
    line.trim();
    Serial.print(F("\r\n>> Sunucu yaniti: ")); Serial.println(line);
    if (line.indexOf(",200,") >= 0) Serial.println(F("*** UPLOAD OK (HTTP 200) ***"));
    else                            Serial.println(F("!! Upload non-200 (bkz. kod)"));
  } else {
    Serial.println(F("!! +HTTPACTION yaniti gelmedi (timeout)"));
  }

  sendAT("AT+HTTPTERM", "OK", 3000);
  Serial.println(F("---------------- HTTP POST DONE ----------------\r\n"));
  httpBusy = false;
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0 < 3000)) delay(10);
  delay(300);

  Serial.println(F("\r\n\r\n#### ESP32-S3 <-> A7608E-H  AT + GNSS test ####"));
  Serial.printf("Reset reason: %s\r\n", resetReasonStr(esp_reset_reason()));
  Serial.printf("Modem UART: %d baud, RX=GPIO%d, TX=GPIO%d\r\n",
                MODEM_BAUD, MODEM_RX_PIN, MODEM_TX_PIN);

  Modem.begin(MODEM_BAUD, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);
  delay(200);

  runLteBootTest();

  // Set the data APN so the HTTP upload has a bearer once registered.
  sendAT("AT+CGDCONT=1,\"IP\",\"" APN "\"", "OK", 3000);

  powerOnGnss();
  Serial.println(F("Manual bridge active. Type AT commands + Enter.\r\n"));
  Serial.println(F("(Position will be POSTed once, on the first GNSS fix.)\r\n"));
}

void loop() {
  if (httpBusy) return;   // httpPost() owns the modem during an upload

  // PC -> modem (manual typing)
  while (Serial.available()) Modem.write(Serial.read());

  // Periodic GNSS poll (owns the modem read during the poll)
  if (gnssPowered && millis() - lastGnssPoll >= GNSS_POLL_MS) {
    lastGnssPoll = millis();
    pollGnss();
    return;
  }

  // modem -> PC (URCs, manual command replies)
  while (Modem.available()) Serial.write(Modem.read());
}
