// Copy this file to `config.h` and fill in your own values.
// `config.h` is intentionally untracked so credentials never reach the repository.
#pragma once

// Cellular APN of your SIM provider.
#define APN            "internet"

// MQTT broker. A public broker is fine because the payload is encrypted before
// it leaves the device, but use an authenticated broker for production.
#define MQTT_BROKER    "tcp://broker.example.com:1883"

// Must be unique per device on the broker (e.g. include the modem IMEI).
#define MQTT_CLIENTID  "trk01-CHANGE_ME"

// Short id reported inside the payload, and the topics this device uses.
#define DEVICE_ID      "trk01"
#define TOPIC          "yourprefix/trk01/pos"   // uplink  (device -> app)
#define CMD_TOPIC      "yourprefix/trk01/cmd"   // downlink (app -> device)
#define LOG_TOPIC      "yourprefix/trk01/log"   // optional remote console

// Wi-Fi is a bench convenience: it carries OTA firmware uploads so the board
// does not need a USB cable. Leave it disabled in the field.
#define WIFI_SSID      "your-wifi-name"
#define WIFI_PASS      "your-wifi-password"
#define OTA_HOSTNAME   "trk01"
#define OTA_PASSWORD   "choose-an-ota-password"

// 32-byte AES-256-GCM key, unique per device. Generate one with:
//   openssl rand -hex 32
// The same key must be entered in the web app's key vault to decrypt.
// Never commit the real key; keep it offline (paper/password manager).
static const uint8_t AES_KEY[32] = {
  0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07, 0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f,
  0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17, 0x18,0x19,0x1a,0x1b,0x1c,0x1d,0x1e,0x1f
};
