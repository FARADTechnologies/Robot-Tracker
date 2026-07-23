#include <Arduino.h>

#define RX2 16
#define TX2 17

// Değişkenler
float latitude = 0.0, longitude = 0.0;
int satelliteCount = 0;
String gsmBuffer = "";
unsigned long lastGPSRequest = 0;
const unsigned long GPS_INTERVAL = 1000; // Her 1 saniyede bir veri iste

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RX2, TX2);
  Serial2.println("AT+CGNSSPWR=1");
  Serial.println("Gerçek Zamanlı Sistem Başlatıldı.");
}

void loop() {
  // 1. ZAMAN YÖNETİMİ (Delay Kullanmadan!)
  // İşlemci burada asla durmaz, sürekli döner
  if (millis() - lastGPSRequest >= GPS_INTERVAL) {
    Serial2.println("AT+CGNSSINFO");
    lastGPSRequest = millis();
  }

  // 2. VERİ OKUMA (Beklemeden!)
  // Veri geldikçe parça parça tampona (buffer) alınır
  while (Serial2.available()) {
    char c = Serial2.read();
    if (c == '\n') { // Satır bittiğinde (Veri tam geldiğinde) işle
      processResponse(gsmBuffer);
      gsmBuffer = ""; // Tamponu boşalt
    } else {
      gsmBuffer += c;
    }
  }

  // 3. BURADA DİĞER İŞLERİ YAPABİLİRSİN
  // İşlemci boşta kalmaz, örneğin sensör okuyabilir veya şifreleme yapabilir.
}

void processResponse(String data) {
  if (data.indexOf("+CGNSSINFO:") != -1) {
    parseGPS(data);
    Serial.printf("FIX [%d Uydu] -> Lat: %.7f | Lon: %.7f\n", satelliteCount, latitude, longitude);
  } else if (data.indexOf("OK") != -1) {
    // Komut onaylarını burada yakalayabiliriz
  }
}

void parseGPS(String data) {
  // Virgülleri sayan profesyonel parser
  int commaCount = 0;
  int startPos = data.indexOf(':') + 1;
  
  for (int i = 0; i < data.length(); i++) {
    if (data[i] == ',') {
      commaCount++;
      int nextComma = data.indexOf(',', i + 1);
      String val = data.substring(i + 1, nextComma);
      val.trim();

      if (commaCount == 1) satelliteCount = val.toInt();
      if (commaCount == 5 && val.length() > 0) latitude = val.toFloat();
      if (commaCount == 7 && val.length() > 0) longitude = val.toFloat();
    }
  }
}