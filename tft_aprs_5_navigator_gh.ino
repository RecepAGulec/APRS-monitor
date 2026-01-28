/*
   APRS Decoder V6.0 - TA2EI
   Özellikler: Azimut (Derece), Mesafe (km), 2500km Filtresi, Türkçe Yönler
   
*/

#include <LibAPRSgr.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <SPI.h>
#include <math.h>

#define TFT_CS    8
#define TFT_RST   9
#define TFT_DC    10

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

// --- APRS Monitörün kullanılacağı yerin koordintları BURAYI DÜZENLEYİN! --- 
const float HOME_LAT = ee.eeee; // Ondalık olarak enlem bilgisi 
const float HOME_LON = bb.bbbb; // Ondalık olarak boylam bilgisi

volatile boolean gotPacket = false;
char staticCallsign[10];
char staticInfo[100];
uint8_t staticLen = 0;
int staticSSID = 0;
uint16_t packetCount = 0;

void setup() {
  tft.begin();
  tft.setRotation(3);
  tft.fillScreen(ILI9341_BLACK);
  refreshHeader();
  APRS_init(REF_5V, false);
}

void refreshHeader() {
  tft.fillRect(0, 0, 320, 18, ILI9341_BLUE);
  tft.setCursor(5, 5);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(1);
  tft.print(F("TA2EI APRS NAVIGATOR V6.0 | PAKET: "));
  tft.print(packetCount);
}

// Mesafe Hesaplama (Haversine)
float calculateDistance(float lat2, float lon2) {
  float dLat = (lat2 - HOME_LAT) * M_PI / 180.0;
  float dLon = (lon2 - HOME_LON) * M_PI / 180.0;
  float a = sin(dLat / 2) * sin(dLat / 2) +
            cos(HOME_LAT * M_PI / 180.0) * cos(lat2 * M_PI / 180.0) *
            sin(dLon / 2) * sin(dLon / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return 6371.0 * c;
}

// Azimut (kuzeye göre derece) Hesaplama
float calculateBearing(float lat2, float lon2) {
  float dLon = (lon2 - HOME_LON) * M_PI / 180.0;
  float y = sin(dLon) * cos(lat2 * M_PI / 180.0);
  float x = cos(HOME_LAT * M_PI / 180.0) * sin(lat2 * M_PI / 180.0) -
            sin(HOME_LAT * M_PI / 180.0) * cos(lat2 * M_PI / 180.0) * cos(dLon);
  float brng = atan2(y, x) * 180.0 / M_PI;
  if (brng < 0) brng += 360;
  return brng;
}

// Türkçe Yön Kısaltmaları
const char* getDirTR(float degree) {
  if (degree >= 337.5 || degree < 22.5) return "K";
  if (degree >= 22.5 && degree < 67.5) return "KD";
  if (degree >= 67.5 && degree < 112.5) return "D";
  if (degree >= 112.5 && degree < 157.5) return "GD";
  if (degree >= 157.5 && degree < 202.5) return "G";
  if (degree >= 202.5 && degree < 247.5) return "GB";
  if (degree >= 247.5 && degree < 292.5) return "B";
  if (degree >= 292.5 && degree < 337.5) return "KB";
  return "-";
}

float parseToFloat(char* buf, int start, int len) {
  char temp[10];
  int j = 0;
  for (int i = start; i < start + len; i++) {
    if ((buf[i] >= '0' && buf[i] <= '9') || buf[i] == '.') temp[j++] = buf[i];
  }
  temp[j] = '\0';
  if (len == 7) {
    char deg[3] = {temp[0], temp[1], '\0'};
    return atof(deg) + (atof(temp + 2) / 60.0);
  } else {
    char deg[4] = {temp[0], temp[1], temp[2], '\0'};
    return atof(deg) + (atof(temp + 3) / 60.0);
  }
}

void loop() {
  if (gotPacket) {
    packetCount++;
    refreshHeader();
    updateTFT();
    gotPacket = false;
  }
}

void updateTFT() {
  tft.fillRect(0, 19, 320, 221, ILI9341_BLACK);

  // 1. İstasyon Bilgisi
  tft.setCursor(10, 25);
  tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK);
  tft.setTextSize(3);
  tft.print(staticCallsign);
  if (staticSSID > 0) {
    tft.print(F("-"));
    tft.print(staticSSID);
  }

  tft.drawFastHLine(0, 52, 320, ILI9341_DARKGREY);

  int latPos = -1;
  char latDir = ' ';
  for (int i = 0; i < staticLen; i++) {
    if (staticInfo[i] == 'N' || staticInfo[i] == 'S') {
      latPos = i;
      latDir = staticInfo[i];
      break;
    }
  }

  if (latPos >= 7) {
    float latDec = parseToFloat(staticInfo, latPos - 7, 7);
    if (latDir == 'S') latDec *= -1;

    int lonPos = -1;
    char lonDir = ' ';
    for (int i = latPos + 1; i < staticLen; i++) {
      if (staticInfo[i] == 'E' || staticInfo[i] == 'W') {
        lonPos = i;
        lonDir = staticInfo[i];
        break;
      }
    }

    if (lonPos >= 8) {
      float lonDec = parseToFloat(staticInfo, lonPos - 8, 8);
      if (lonDir == 'W') lonDec *= -1;

      // ÜST SATIR: Koordinatlar (Font 2)
      tft.setCursor(10, 60);
      tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);
      tft.setTextSize(2);
      tft.print(latDec, 4); tft.print(latDir);
      tft.print(F(" / "));
      tft.print(lonDec, 4); tft.print(lonDir);

      // ALT SATIR: Mesafe ve Azimut (Font 2)
      float dist = calculateDistance(latDec, lonDec);
      tft.setCursor(10, 82);

      if (dist > 2500.0) {
        tft.setTextColor(ILI9341_RED, ILI9341_BLACK);
        tft.print(F("MESAFE HATASI (DX?)"));
      } else {
        float azimuth = calculateBearing(latDec, lonDec);
        tft.setTextColor(ILI9341_CYAN, ILI9341_BLACK);
        tft.print(dist, 1); tft.print(F(" km | "));
        tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
        tft.print((int)azimuth); tft.print((char)247);
        tft.print(F(" ")); tft.print(getDirTR(azimuth));
      }
    }
  } else {
    tft.setCursor(10, 65);
    tft.setTextSize(2);
    if (staticInfo[0] == '\'' || staticInfo[0] == '`' || staticInfo[0] == 0x1d) {
      tft.setTextColor(ILI9341_MAGENTA, ILI9341_BLACK);
      tft.print(F("> MIC-E (MOBIL) PAKETI"));
    } else {
      tft.setTextColor(ILI9341_ORANGE, ILI9341_BLACK);
      tft.print(F("> DURUM / BEACON"));
    }
  }

  tft.drawFastHLine(0, 105, 320, ILI9341_DARKGREY);

  // 3. Mesaj Alanı
  tft.setCursor(10, 115);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setTextSize(2);
  tft.setTextWrap(true);
  for (int i = 0; i < staticLen; i++) {
    if (staticInfo[i] >= 32 && staticInfo[i] <= 126) tft.print(staticInfo[i]);
    else tft.print(F("."));
  }
}

void aprs_msg_callback(struct AX25Msg *msg) {
  if (!gotPacket) {
    memset(staticCallsign, 0, sizeof(staticCallsign));
    strncpy(staticCallsign, msg->src.call, 9);
    staticSSID = msg->src.ssid;
    staticLen = (msg->len > 99) ? 99 : msg->len;
    for (int i = 0; i < staticLen; i++) staticInfo[i] = (char)msg->info[i];
    staticInfo[staticLen] = '\0';
    gotPacket = true;
  }
}
