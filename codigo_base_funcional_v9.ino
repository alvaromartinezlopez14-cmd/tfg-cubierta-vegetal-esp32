// Descomentar para activar la depuracion de Thinger.io.
//#define THINGER_SERIAL_DEBUG
#define THINGER_SERVER "ciot.ucam.edu"
// codigo_base_funcional_v9.ino
// Proyecto adaptado a M5Stack TimerCamera-F (ESP32 + OV3660), VGA, ROI configurable, Remote Console, bateria, uptime, mascara visual, selector ExG/HSV, ajuste visual de algoritmo y ROI, controles Thinger.io y modo automatico con Deep Sleep.
// En Arduino IDE selecciona la placa M5TimerCAM / M5Stack-Timer-CAM.

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiProv.h>
#if __has_include("esp_arduino_version.h")
#include "esp_arduino_version.h"
#endif
#ifndef ESP_ARDUINO_VERSION_MAJOR
#define ESP_ARDUINO_VERSION_MAJOR 2
#endif
#if ESP_ARDUINO_VERSION_MAJOR >= 3
#define WIFI_PROV_AUTH_ERROR_REASON NETWORK_PROV_WIFI_STA_AUTH_ERROR
#else
#define WIFI_PROV_AUTH_ERROR_REASON WIFI_PROV_STA_AUTH_ERROR
#endif
#if __has_include("esp_bt.h")
#include "esp_bt.h"
#define WIFI_PROV_CAN_RELEASE_BT_MEMORY 1
#else
#define WIFI_PROV_CAN_RELEASE_BT_MEMORY 0
#endif
#include "esp_camera.h"
#include <WebServer.h>
#include "img_converters.h"
#include <ThingerESP32.h>
#include <ThingerConsole.h>
#include "esp_sleep.h"
#include "esp_timer.h"
#include <Preferences.h>

// ====== Pines oficiales (M5Stack TimerCamera-F + OV3660) ======
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    15
#define XCLK_GPIO_NUM     27
#define SIOD_GPIO_NUM     25
#define SIOC_GPIO_NUM     23

#define Y9_GPIO_NUM       19
#define Y8_GPIO_NUM       36
#define Y7_GPIO_NUM       18
#define Y6_GPIO_NUM       39
#define Y5_GPIO_NUM        5
#define Y4_GPIO_NUM       34
#define Y3_GPIO_NUM       35
#define Y2_GPIO_NUM       32

#define VSYNC_GPIO_NUM    22
#define HREF_GPIO_NUM     26
#define PCLK_GPIO_NUM     21

// ====== Alimentacion y LED de la TimerCamera-F ======
#define POWER_HOLD_PIN    33
#define STATUS_LED_PIN     2
#define BATTERY_ADC_PIN    38

// ====== WiFi y aprovisionamiento BLE oficial de Espressif ======
// Las credenciales validas se cargan desde NVS en estos buffers globales.
// Su direccion permanece estable durante toda la ejecucion para que
// thing.add_wifi() pueda reutilizarlas de forma segura al reconectar.
const char* WIFI_PREFS_NAMESPACE = "wifi-config";
const char* WIFI_PROV_SERVICE_NAME = "PROV_TimerCamF";
const char* WIFI_PROV_POP = "CAMBIAR1";
const char* WIFI_PROV_SERVICE_KEY = nullptr;

char wifiSsid[33] = {0};
char wifiPassword[65] = {0};
char pendingWifiSsid[33] = {0};
char pendingWifiPassword[65] = {0};

uint8_t wifiProvisioningServiceUuid[16] = {
  0xb4, 0xdf, 0x5a, 0x1c, 0x3f, 0x6b, 0xf4, 0xbf,
  0xea, 0x4a, 0x82, 0x03, 0x04, 0x90, 0x1a, 0x02
};

bool wifiProvisioningMode = false;
portMUX_TYPE wifiProvisioningMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool wifiProvisioningCredentialsReceived = false;
volatile bool wifiProvisioningCredentialSuccess = false;
volatile bool wifiProvisioningCredentialFailed = false;
volatile bool wifiProvisioningGotIp = false;
volatile bool wifiProvisioningSaveAttempted = false;
bool wifiProvisioningRestartPending = false;
unsigned long wifiProvisioningRestartMs = 0;
const unsigned long WIFI_PROVISIONING_RESTART_DELAY_MS = 1500UL;


// ====== Thinger.io ======
#define USERNAME "TU_USUARIO_THINGER"
#define DEVICE_ID "TU_DISPOSITIVO_THINGER"
#define DEVICE_CREDENTIAL "TU_CREDENCIAL_THINGER"
#define BUCKET_ID "bucket_vegetacion"

ThingerESP32 thing(USERNAME, DEVICE_ID, DEVICE_CREDENTIAL);
ThingerConsole console(thing);

// ====== Web ======
WebServer server(80);

// ====== Configuracion analisis ======
const int ANALYSIS_W = 640;
const int ANALYSIS_H = 480;
const size_t FRAME_PIXELS = (size_t)ANALYSIS_W * (size_t)ANALYSIS_H;

// ====== Selector de algoritmo de deteccion ======
enum DetectionMode {
  DETECTION_EXG_NORMALIZED = 0,
  DETECTION_HSV_GREEN = 1
};

int detectionMode = DETECTION_EXG_NORMALIZED;
float exgThreshold = 0.12f;
int hsvHMin = 80;
int hsvHMax = 150;
int hsvSMin = 50;
int hsvVMin = 40;
int lastAnalysisDetectionMode = DETECTION_EXG_NORMALIZED;

// ====== ROI configurable ======
bool roiEnabled = true;
int roiLeftPercent = 20;
int roiRightPercent = 20;
int roiTopPercent = 20;
int roiBottomPercent = 20;

// ====== Imagen estable para configuracion visual de la ROI ======
// Es independiente de la ultima foto analizada y de la imagen usada para
// ajustar el algoritmo. Solo se sustituye cuando una nueva captura ROI se
// completa correctamente.
uint8_t* roiPreviewJpeg = nullptr;
size_t roiPreviewJpegLen = 0;

enum RoiPreviewState : uint8_t {
  ROI_PREVIEW_NO_IMAGE = 0,
  ROI_PREVIEW_CAPTURING,
  ROI_PREVIEW_READY,
  ROI_PREVIEW_ERROR
};

RoiPreviewState roiPreviewState = ROI_PREVIEW_NO_IMAGE;
String roiPreviewLastError = "";
bool roiPreviewProcessing = false;
bool pendingRoiPreviewCapture = false;
uint32_t roiPreviewVersion = 0;

// ====== Buffer RGB ======
uint8_t* rgbBuf = nullptr;

// ====== Ultima foto analizada ======
uint8_t* lastJpeg = nullptr;
size_t lastJpegLen = 0;

// ====== Mascara visual de la ultima deteccion ======
// Se guarda un byte por pixel: 1 = vegetacion detectada, 0 = no vegetacion.
// La imagen visual se sirve como BMP desde la web para evitar codificar JPEG en el ESP32.
uint8_t* lastDetectionMask = nullptr;
bool lastDetectionMaskValid = false;

// ====== Imagen estable para ajuste visual del algoritmo ======
// Esta imagen es independiente de la ultima medicion normal. Se captura
// expresamente desde la pagina de configuracion y se reutiliza al mover
// sliders para comparar los cambios sin alterar la escena fotografiada.
uint8_t* algorithmPreviewJpeg = nullptr;
size_t algorithmPreviewJpegLen = 0;

// Mascara activa que corresponde exactamente al ultimo resultado publicado.
uint8_t* algorithmPreviewMask = nullptr;
// Segundo buffer permanente para calcular una nueva mascara sin destruir
// el ultimo resultado valido mientras se procesa.
uint8_t* algorithmPreviewWorkMask = nullptr;

// JPEG comprimido de la superposicion que se envia a la pagina web.
// El analisis sigue realizandose en VGA; solo la visualizacion se reduce.
uint8_t* algorithmPreviewOverlayJpeg = nullptr;
size_t algorithmPreviewOverlayJpegLen = 0;
uint8_t* algorithmPreviewDisplayRgb = nullptr;

bool algorithmPreviewMaskValid = false;
bool algorithmPreviewDirty = false;
unsigned long algorithmPreviewDirtySinceMs = 0;

// Version de imagen publicada (cambia solo al terminar correctamente).
uint32_t algorithmPreviewVersion = 0;
// Version solicitada y version efectivamente procesada.
uint32_t algorithmPreviewRequestedVersion = 0;
uint32_t algorithmPreviewProcessedVersion = 0;

float algorithmPreviewVegPercent = -1.0f;
uint32_t algorithmPreviewVegetationPx = 0;
uint32_t algorithmPreviewRoiPx = 0;

// Instantanea de los parametros usados para generar la imagen/porcentaje visible.
int algorithmPreviewDetectionMode = DETECTION_EXG_NORMALIZED;
float algorithmPreviewExgThreshold = 0.0f;
int algorithmPreviewHMin = 0;
int algorithmPreviewHMax = 0;
int algorithmPreviewSMin = 0;
int algorithmPreviewVMin = 0;

enum AlgorithmPreviewState : uint8_t {
  PREVIEW_NO_IMAGE = 0,
  PREVIEW_CAPTURING,
  PREVIEW_PROCESSING,
  PREVIEW_READY,
  PREVIEW_ERROR
};

AlgorithmPreviewState algorithmPreviewState = PREVIEW_NO_IMAGE;
String algorithmPreviewLastError = "";
bool algorithmPreviewProcessing = false;
bool pendingAlgorithmPreviewCapture = false;

const unsigned long ALGORITHM_PREVIEW_DEBOUNCE_MS = 2000UL;
const int ALGORITHM_PREVIEW_DISPLAY_W = 320;
const int ALGORITHM_PREVIEW_DISPLAY_H = 240;
const uint8_t ALGORITHM_PREVIEW_JPEG_QUALITY = 78;

// ====== Preferencias persistentes del algoritmo ======
// Solo se escriben cuando el usuario pulsa "Guardar algoritmo por defecto".
// Los cambios ordinarios siguen aplicandose inmediatamente y se mantienen
// durante Deep Sleep mediante RTC_DATA_ATTR, sin castigar la memoria flash.
const char* ALGORITHM_PREFS_NAMESPACE = "veg-alg";
bool algorithmDefaultsAvailable = false;

// ====== Resultados ======
float    lastVegPercent   = -1.0f;
uint32_t lastVegetationPx = 0;
uint32_t lastRoiPx        = 0;
bool     lastAnalysisDone = false;

// ====== Estado camara ======
bool cameraReady = false;

// ====== Bateria TimerCamera-F ======
// M5Stack indica que TimerCamera-F permite leer la bateria mediante GPIO38.
// Nota: con USB conectado, esta lectura puede representar la tension de carga,
// no necesariamente el nivel real de la bateria. Para una lectura mas real,
// comprobar con USB desconectado y POWER_HOLD_PIN (GPIO33) en HIGH.
float lastBatteryVoltage = 0.0f;
int lastBatteryPercent = 0;
bool batteryReadValid = false;
unsigned long lastBatteryUpdateMs = 0;
const unsigned long BATTERY_UPDATE_INTERVAL_MS = 10000UL;
const float BATTERY_VOLTAGE_MULTIPLIER = 1.51f;
const float BATTERY_MIN_VOLTAGE = 3.350f;
const float BATTERY_MAX_VOLTAGE = 4.150f;

// ====== Modo automatico ======
bool autoModeEnabled = false;
unsigned long autoIntervalMs = 60000;
unsigned long lastAutoCaptureMs = 0;
unsigned long autoCaptureCount = 0;

// ====== Modo automatico con Deep Sleep ======
bool autoDeepSleepEnabled = false;
unsigned long awakeWindowSeconds = 60;
bool wokeFromDeepSleep = false;
bool autoDeepSleepCaptureDone = false;
unsigned long awakeWindowStartMs = 0;
unsigned long wakeCount = 0;

// Esperas especificas del ciclo automatico con Deep Sleep.
// Su objetivo es evitar que la placa capture/envie datos antes de que
// Thinger.io haya tenido tiempo de completar la autenticacion tras despertar.
const unsigned long THINGER_READY_WAIT_AFTER_WAKE_MS = 15000UL;
const unsigned long THINGER_STREAM_FLUSH_MS = 5000UL;

const uint32_t RTC_CONFIG_MAGIC = 0xA17B2026;
RTC_DATA_ATTR uint32_t rtcConfigMagic = 0;
RTC_DATA_ATTR bool rtcAutoModeEnabled = false;
RTC_DATA_ATTR bool rtcAutoDeepSleepEnabled = false;
RTC_DATA_ATTR unsigned long rtcAutoIntervalSeconds = 60;
RTC_DATA_ATTR unsigned long rtcAwakeWindowSeconds = 60;
RTC_DATA_ATTR unsigned long rtcAutoCaptureCount = 0;
RTC_DATA_ATTR unsigned long rtcWakeCount = 0;
RTC_DATA_ATTR unsigned long rtcTotalAwakeSeconds = 0;
RTC_DATA_ATTR int rtcDetectionMode = DETECTION_EXG_NORMALIZED;
RTC_DATA_ATTR float rtcExgThreshold = 0.05f;
RTC_DATA_ATTR int rtcHsvHMin = 80;
RTC_DATA_ATTR int rtcHsvHMax = 150;
RTC_DATA_ATTR int rtcHsvSMin = 50;
RTC_DATA_ATTR int rtcHsvVMin = 40;
RTC_DATA_ATTR bool rtcRoiEnabled = true;
RTC_DATA_ATTR int rtcRoiLeftPercent = 20;
RTC_DATA_ATTR int rtcRoiRightPercent = 20;
RTC_DATA_ATTR int rtcRoiTopPercent = 20;
RTC_DATA_ATTR int rtcRoiBottomPercent = 20;
RTC_DATA_ATTR float rtcLastVegPercent = -1.0f;
RTC_DATA_ATTR unsigned long rtcLastVegetationPixels = 0;

// ====== Solicitudes de control desde Thinger.io ======
bool pendingThingerCaptureNow = false;
unsigned long controlCaptureRequestCount = 0;
bool lastControlCaptureOk = false;

// ====== Remote Console ======
bool consoleWasConnected = false;
String consoleInputBuffer = "";

enum {
  SYSTEM_ACTION_NONE,
  SYSTEM_ACTION_RESTART,
  SYSTEM_ACTION_SHUTDOWN
};

int pendingSystemAction = SYSTEM_ACTION_NONE;
unsigned long pendingSystemActionMs = 0;
const unsigned long SYSTEM_ACTION_DELAY_MS = 1000UL;

// Prototipos relacionados con selector de algoritmo.
String detectionModeName();
String detectionModeShortName();
bool setDetectionModeValue(int newMode, bool invalidateResult);
bool setExgThresholdValue(float newThreshold, bool invalidateResult);
bool setHsvConfigValue(int newHMin, int newHMax, int newSMin, int newVMin, bool invalidateResult);
bool isValidAutoIntervalSeconds(int seconds);
bool setAutoIntervalSecondsValue(int seconds, bool resetTimer);
bool setAutomaticModeSeconds(int seconds);
bool setAutomaticModeSecondsWithDeepSleep(int seconds);
void stopAutomaticMode();
bool setAwakeWindowSecondsValue(unsigned long seconds);
bool setRoiConfigValue(bool enabled, int left, int right, int top, int bottom, bool invalidateResult);
void setRoiFullValue(bool invalidateResult);
void setRoiCenterValue(bool invalidateResult);
const char* roiPreviewStateName();
bool captureRoiPreviewImage();
String roiVisualStatusJson(bool ok, const String& message);
void handleRoiPreviewCapture();
void handleRoiPreviewJpeg();
void handleRoiVisualStatus();
void handleRoiVisualApply();
void saveRuntimeConfigToRtc();
void restoreRuntimeConfigFromRtc();
bool rtcHasValidConfig();
bool isWakeupFromDeepSleep();
void enterAutoDeepSleepCycle();
bool performAutomaticCaptureAndUpdateCount();
void waitForThingerReadyAfterWake(unsigned long waitMs, const char* reason);
String getTotalAwakeFormatted();
String algorithmSummaryHtml();
void scheduleAlgorithmPreviewRefresh();
bool ensureAlgorithmPreviewMaskBuffers();
bool captureAlgorithmPreviewImage();
bool regenerateAlgorithmPreview(uint32_t targetVersion);
bool generateAlgorithmPreviewOverlayJpeg(const uint8_t* sourceRgb,
                                         const uint8_t* sourceMask,
                                         uint8_t*& outJpeg,
                                         size_t& outJpegLen);
const char* algorithmPreviewStateName();
bool isVegetationByActiveAlgorithm(uint8_t r, uint8_t g, uint8_t b, size_t idx);
bool saveAlgorithmDefaultsToNvs();
bool loadAlgorithmDefaultsFromNvs();
String algorithmStatusJson(bool ok, const String& message);
void handleAlgorithmLiveUpdate();
void handleAlgorithmPreviewCapture();
void handleAlgorithmStatus();
void handleSaveAlgorithmDefaults();
void handleAlgorithmPreviewBmp();
void handleAlgorithmPreviewJpeg();
bool loadWifiCredentialsFromNvs();
bool saveWifiCredentialsToNvs(const char* newSsid, const char* newPassword);
bool isWifiProvisioningRequested();
bool setWifiProvisioningRequested(bool requested);
void releaseBluetoothMemoryForNormalMode();
void handleWifiProvisioningEvent(arduino_event_t* event);
void startWifiProvisioningMode();
void handleWifiProvisioningMode();
String escapeHtmlText(const String& text);
void handleWifiConfig();
void handleWifiStart();

// -------------------------------------------------
// Utilidades
// -------------------------------------------------
void initBatteryMonitor() {
  pinMode(BATTERY_ADC_PIN, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(BATTERY_ADC_PIN, ADC_11db);
}

float readBatteryVoltageNow() {
  const int samples = 8;
  uint32_t totalMv = 0;

  for (int i = 0; i < samples; i++) {
    totalMv += analogReadMilliVolts(BATTERY_ADC_PIN);
    delay(2);
  }

  float adcVoltage = ((float)totalMv / (float)samples) / 1000.0f;
  float batteryVoltage = adcVoltage * BATTERY_VOLTAGE_MULTIPLIER;

  return batteryVoltage;
}

int calculateBatteryPercent(float voltage) {
  if (voltage <= BATTERY_MIN_VOLTAGE) return 0;
  if (voltage >= BATTERY_MAX_VOLTAGE) return 100;

  float percent = ((voltage - BATTERY_MIN_VOLTAGE) /
                   (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE)) * 100.0f;

  if (percent < 0.0f) percent = 0.0f;
  if (percent > 100.0f) percent = 100.0f;

  return (int)(percent + 0.5f);
}

void updateBatteryStatus(bool force) {
  unsigned long now = millis();

  if (!force && batteryReadValid &&
      (now - lastBatteryUpdateMs < BATTERY_UPDATE_INTERVAL_MS)) {
    return;
  }

  lastBatteryVoltage = readBatteryVoltageNow();
  lastBatteryPercent = calculateBatteryPercent(lastBatteryVoltage);
  lastBatteryUpdateMs = now;
  batteryReadValid = true;
}

float getBatteryVoltage() {
  updateBatteryStatus(false);
  return lastBatteryVoltage;
}

int getBatteryPercent() {
  updateBatteryStatus(false);
  return lastBatteryPercent;
}

uint64_t getUptimeSeconds64() {
  return (uint64_t)(esp_timer_get_time() / 1000000LL);
}

unsigned long getUptimeSeconds() {
  uint64_t seconds = getUptimeSeconds64();
  if (seconds > 4294967295ULL) return 4294967295UL;
  return (unsigned long)seconds;
}

unsigned long getUptimeMinutes() {
  uint64_t minutes = getUptimeSeconds64() / 60ULL;
  if (minutes > 4294967295ULL) return 4294967295UL;
  return (unsigned long)minutes;
}

String getUptimeFormatted() {
  uint64_t totalSeconds = getUptimeSeconds64();

  uint32_t days = totalSeconds / 86400ULL;
  uint8_t hours = (totalSeconds % 86400ULL) / 3600ULL;
  uint8_t minutes = (totalSeconds % 3600ULL) / 60ULL;
  uint8_t seconds = totalSeconds % 60ULL;

  char buffer[32];

  if (days > 0) {
    snprintf(buffer, sizeof(buffer), "%lud %02u:%02u:%02u",
             (unsigned long)days, hours, minutes, seconds);
  } else {
    snprintf(buffer, sizeof(buffer), "%02u:%02u:%02u",
             hours, minutes, seconds);
  }

  return String(buffer);
}

String formatSecondsDuration(uint64_t totalSeconds) {
  uint32_t days = totalSeconds / 86400ULL;
  uint8_t hours = (totalSeconds % 86400ULL) / 3600ULL;
  uint8_t minutes = (totalSeconds % 3600ULL) / 60ULL;
  uint8_t seconds = totalSeconds % 60ULL;

  char buffer[32];
  if (days > 0) {
    snprintf(buffer, sizeof(buffer), "%lud %02u:%02u:%02u",
             (unsigned long)days, hours, minutes, seconds);
  } else {
    snprintf(buffer, sizeof(buffer), "%02u:%02u:%02u",
             hours, minutes, seconds);
  }
  return String(buffer);
}

unsigned long getTotalAwakeSeconds() {
  uint64_t seconds = (uint64_t)rtcTotalAwakeSeconds + getUptimeSeconds64();
  if (seconds > 4294967295ULL) return 4294967295UL;
  return (unsigned long)seconds;
}

unsigned long getTotalAwakeMinutes() {
  uint64_t minutes = ((uint64_t)rtcTotalAwakeSeconds + getUptimeSeconds64()) / 60ULL;
  if (minutes > 4294967295ULL) return 4294967295UL;
  return (unsigned long)minutes;
}

String getTotalAwakeFormatted() {
  return formatSecondsDuration((uint64_t)rtcTotalAwakeSeconds + getUptimeSeconds64());
}





String intervalText(unsigned long ms) {
  if (ms == 60000UL) return "60 segundos";
  if (ms == 120000UL) return "2 minutos";
  if (ms == 300000UL) return "5 minutos";
  if (ms == 86400000UL) return "24 horas";
  return String(ms / 1000UL) + " s";
}

String autoStatusText() {
  return autoModeEnabled ? "ACTIVO" : "PARADO";
}

String autoStatusClass() {
  return autoModeEnabled ? "ok" : "no";
}

bool isValidAutoIntervalSeconds(int seconds) {
  return (seconds == 60 || seconds == 120 || seconds == 300 || seconds == 86400);
}

bool setAutoIntervalSecondsValue(int seconds, bool resetTimer) {
  if (!isValidAutoIntervalSeconds(seconds)) {
    return false;
  }

  autoIntervalMs = (unsigned long)seconds * 1000UL;
  if (resetTimer) {
    lastAutoCaptureMs = 0;
    autoDeepSleepCaptureDone = false;
    awakeWindowStartMs = 0;
  }
  saveRuntimeConfigToRtc();
  return true;
}

bool setAwakeWindowSecondsValue(unsigned long seconds) {
  if (seconds < 10UL || seconds > 300UL) {
    return false;
  }

  awakeWindowSeconds = seconds;
  saveRuntimeConfigToRtc();
  return true;
}

bool setAutomaticModeSeconds(int seconds) {
  if (!setAutoIntervalSecondsValue(seconds, true)) {
    return false;
  }

  autoModeEnabled = true;
  autoDeepSleepEnabled = false;
  autoDeepSleepCaptureDone = false;
  awakeWindowStartMs = 0;
  saveRuntimeConfigToRtc();
  return true;
}

bool setAutomaticModeSecondsWithDeepSleep(int seconds) {
  if (!setAutoIntervalSecondsValue(seconds, true)) {
    return false;
  }

  autoModeEnabled = true;
  autoDeepSleepEnabled = true;
  autoDeepSleepCaptureDone = false;
  awakeWindowStartMs = 0;
  saveRuntimeConfigToRtc();
  return true;
}

void stopAutomaticMode() {
  autoModeEnabled = false;
  autoDeepSleepEnabled = false;
  autoDeepSleepCaptureDone = false;
  awakeWindowStartMs = 0;
  saveRuntimeConfigToRtc();
}

bool rtcHasValidConfig() {
  return rtcConfigMagic == RTC_CONFIG_MAGIC;
}

bool isWakeupFromDeepSleep() {
  return esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER;
}

void saveRuntimeConfigToRtc() {
  rtcConfigMagic = RTC_CONFIG_MAGIC;
  rtcAutoModeEnabled = autoModeEnabled;
  rtcAutoDeepSleepEnabled = autoDeepSleepEnabled;
  rtcAutoIntervalSeconds = autoIntervalMs / 1000UL;
  rtcAwakeWindowSeconds = awakeWindowSeconds;
  rtcAutoCaptureCount = autoCaptureCount;
  rtcWakeCount = wakeCount;
  rtcDetectionMode = detectionMode;
  rtcExgThreshold = exgThreshold;
  rtcHsvHMin = hsvHMin;
  rtcHsvHMax = hsvHMax;
  rtcHsvSMin = hsvSMin;
  rtcHsvVMin = hsvVMin;
  rtcRoiEnabled = roiEnabled;
  rtcRoiLeftPercent = roiLeftPercent;
  rtcRoiRightPercent = roiRightPercent;
  rtcRoiTopPercent = roiTopPercent;
  rtcRoiBottomPercent = roiBottomPercent;
  rtcLastVegPercent = lastVegPercent;
  rtcLastVegetationPixels = lastVegetationPx;
}

void restoreRuntimeConfigFromRtc() {
  if (!rtcHasValidConfig()) return;

  autoModeEnabled = rtcAutoModeEnabled;
  autoDeepSleepEnabled = rtcAutoDeepSleepEnabled;
  autoIntervalMs = rtcAutoIntervalSeconds * 1000UL;
  awakeWindowSeconds = rtcAwakeWindowSeconds;
  if (awakeWindowSeconds < 10UL || awakeWindowSeconds > 300UL) awakeWindowSeconds = 60UL;
  autoCaptureCount = rtcAutoCaptureCount;
  wakeCount = rtcWakeCount;

  if (rtcDetectionMode >= DETECTION_EXG_NORMALIZED && rtcDetectionMode <= DETECTION_HSV_GREEN) {
    detectionMode = rtcDetectionMode;
  }
  if (rtcExgThreshold >= -1.0f && rtcExgThreshold <= 1.0f) {
    exgThreshold = rtcExgThreshold;
  }
  if (rtcHsvHMin >= 0 && rtcHsvHMin <= 360 && rtcHsvHMax >= 0 && rtcHsvHMax <= 360 &&
      rtcHsvSMin >= 0 && rtcHsvSMin <= 255 && rtcHsvVMin >= 0 && rtcHsvVMin <= 255) {
    hsvHMin = rtcHsvHMin;
    hsvHMax = rtcHsvHMax;
    hsvSMin = rtcHsvSMin;
    hsvVMin = rtcHsvVMin;
  }
  setRoiConfigValue(rtcRoiEnabled, rtcRoiLeftPercent, rtcRoiRightPercent,
                    rtcRoiTopPercent, rtcRoiBottomPercent, false);

  if (rtcLastVegPercent >= 0.0f) {
    lastVegPercent = rtcLastVegPercent;
    lastVegetationPx = rtcLastVegetationPixels;
    lastAnalysisDone = true;
  }
}


// -------------------------------------------------
// Configuracion persistente por defecto del algoritmo (NVS/Preferences)
// -------------------------------------------------
bool saveAlgorithmDefaultsToNvs() {
  Preferences preferences;
  if (!preferences.begin(ALGORITHM_PREFS_NAMESPACE, false)) {
    Serial.println("NVS) No se pudo abrir el espacio de preferencias del algoritmo");
    return false;
  }

  bool ok = true;
  ok = ok && (preferences.putBool("valid", true) > 0);
  ok = ok && (preferences.putInt("mode", detectionMode) > 0);
  ok = ok && (preferences.putFloat("exg", exgThreshold) > 0);
  ok = ok && (preferences.putInt("hmin", hsvHMin) > 0);
  ok = ok && (preferences.putInt("hmax", hsvHMax) > 0);
  ok = ok && (preferences.putInt("smin", hsvSMin) > 0);
  ok = ok && (preferences.putInt("vmin", hsvVMin) > 0);
  preferences.end();

  algorithmDefaultsAvailable = ok;
  if (ok) {
    Serial.println("NVS) Algoritmo y parametros guardados como valores por defecto");
    if (console) {
      console.println("[NVS] Algoritmo y parametros guardados como valores por defecto");
      console.flush();
    }
  } else {
    Serial.println("NVS) Error guardando la configuracion por defecto");
  }
  return ok;
}

bool loadAlgorithmDefaultsFromNvs() {
  Preferences preferences;
  if (!preferences.begin(ALGORITHM_PREFS_NAMESPACE, true)) {
    algorithmDefaultsAvailable = false;
    return false;
  }

  bool valid = preferences.getBool("valid", false);
  if (!valid) {
    preferences.end();
    algorithmDefaultsAvailable = false;
    return false;
  }

  int storedMode = preferences.getInt("mode", detectionMode);
  float storedExg = preferences.getFloat("exg", exgThreshold);
  int storedHMin = preferences.getInt("hmin", hsvHMin);
  int storedHMax = preferences.getInt("hmax", hsvHMax);
  int storedSMin = preferences.getInt("smin", hsvSMin);
  int storedVMin = preferences.getInt("vmin", hsvVMin);
  preferences.end();

  bool valuesValid =
      storedMode >= DETECTION_EXG_NORMALIZED && storedMode <= DETECTION_HSV_GREEN &&
      storedExg >= -1.0f && storedExg <= 1.0f &&
      storedHMin >= 0 && storedHMin <= 360 &&
      storedHMax >= 0 && storedHMax <= 360 &&
      storedSMin >= 0 && storedSMin <= 255 &&
      storedVMin >= 0 && storedVMin <= 255;

  if (!valuesValid) {
    Serial.println("NVS) Configuracion por defecto invalida; se conservan los valores del codigo");
    algorithmDefaultsAvailable = false;
    return false;
  }

  detectionMode = storedMode;
  exgThreshold = storedExg;
  hsvHMin = storedHMin;
  hsvHMax = storedHMax;
  hsvSMin = storedSMin;
  hsvVMin = storedVMin;
  algorithmDefaultsAvailable = true;

  Serial.printf("NVS) Configuracion por defecto cargada: modo=%d, ExG=%.3f, HSV=%d-%d/%d/%d\n",
                detectionMode, exgThreshold, hsvHMin, hsvHMax, hsvSMin, hsvVMin);
  return true;
}

void consolePrintPrompt() {
  if (!console) return;
  console.print("esp32_vegetacion> ");
  console.flush();
}

void consolePrintHelp() {
  if (!console) return;

  console.println();
  console.println("=== COMANDOS DISPONIBLES ===");
  console.println("help       - Muestra esta ayuda");
  console.println("estado     - Muestra el estado completo del sistema");
  console.println("auto 60    - Activa capturas cada 60 segundos");
  console.println("auto 120   - Activa capturas cada 2 minutos");
  console.println("auto 300   - Activa capturas cada 5 minutos");
  console.println("auto 86400 - Activa capturas cada 24 horas");
  console.println("auto 24h   - Alias de auto 86400");
  console.println("auto off   - Detiene las capturas automaticas");
  console.println("auto_60    - Alias de auto 60");
  console.println("auto_120   - Alias de auto 120");
  console.println("auto_300   - Alias de auto 300");
  console.println("auto_86400 - Alias de auto 86400");
  console.println("auto_24h   - Alias de auto 24h");
  console.println("auto_off   - Alias de auto off");
  console.println("auto_sleep 60    - Automatico cada 60 s con Deep Sleep");
  console.println("auto_sleep 120   - Automatico cada 120 s con Deep Sleep");
  console.println("auto_sleep 300   - Automatico cada 300 s con Deep Sleep");
  console.println("auto_sleep 86400 - Automatico cada 24 h con Deep Sleep");
  console.println("auto_sleep 24h   - Alias de auto_sleep 86400");
  console.println("auto_sleep off   - Desactiva Deep Sleep automatico");
  console.println("algoritmo  - Muestra el algoritmo activo y las opciones");
  console.println("algoritmo 0 - Selecciona ExG normalizado");
  console.println("algoritmo 1 - Selecciona HSV por tono verde");
  console.println("reiniciar  - Reinicia el ESP32");
  console.println("apagar     - Apaga por bateria o entra en deep sleep con USB");
  console.println("=============================");
  console.flush();
}


void consolePrintStatus() {
  if (!console) return;

  console.println();
  console.println("=== ESTADO GENERAL ===");
  console.printf("WiFi: %s\r\n", WiFi.status() == WL_CONNECTED ? "CONECTADO" : "DESCONECTADO");
  console.printf("Direccion IP: %s\r\n", WiFi.localIP().toString().c_str());
  console.printf("Thinger.io: %s\r\n", thing.is_connected() ? "CONECTADO" : "DESCONECTADO");
  console.printf("Modo automatico: %s\r\n", autoModeEnabled ? "ACTIVO" : "PARADO");
  console.printf("Intervalo automatico: %lu segundos\r\n", autoIntervalMs / 1000UL);
  console.printf("Capturas automaticas: %lu\r\n", autoCaptureCount);

  if (lastAnalysisDone) {
    console.printf("Ultimo porcentaje vegetal: %.2f %%\r\n", lastVegPercent);
    console.printf("Ultimos pixeles vegetales: %lu\r\n", (unsigned long)lastVegetationPx);
    console.printf("Ultimos pixeles ROI analizados: %lu\r\n", (unsigned long)lastRoiPx);
  } else {
    console.println("Ultimo analisis: NO DISPONIBLE");
  }

  console.printf("Mascara visual: %s\r\n", lastDetectionMaskValid ? "DISPONIBLE" : "NO DISPONIBLE");
  console.printf("Algoritmo activo: %s (%d)\r\n", detectionModeName().c_str(), detectionMode);
  console.printf("Umbral ExG normalizado: %.3f\r\n", exgThreshold);
  console.printf("HSV: H=%d..%d, Smin=%d, Vmin=%d\r\n", hsvHMin, hsvHMax, hsvSMin, hsvVMin);
  console.printf("ROI: %s\r\n", roiEnabled ? "ACTIVA" : "DESACTIVADA - IMAGEN COMPLETA");
  console.printf("ROI recortes: L=%d%% R=%d%% T=%d%% B=%d%%\r\n",
                 roiLeftPercent, roiRightPercent,
                 roiTopPercent, roiBottomPercent);
  console.printf("ROI coordenadas: x=%d..%d, y=%d..%d\r\n",
                 getRoiX0(), getRoiX1(), getRoiY0(), getRoiY1());
  console.printf("ROI dimensiones: %d x %d px\r\n",
                 getRoiWidthPx(), getRoiHeightPx());
  console.printf("ROI pixeles configurados: %lu\r\n",
                 (unsigned long)getRoiTotalPx());

  updateBatteryStatus(true);
  String uptimeText = getUptimeFormatted();
  console.printf("Bateria: %.2f V (%d %%)\r\n", lastBatteryVoltage, lastBatteryPercent);
  console.printf("Tiempo activo: %s\r\n", uptimeText.c_str());
  console.printf("Uptime segundos: %lu\r\n", getUptimeSeconds());
  console.printf("Uptime minutos: %lu\r\n", getUptimeMinutes());

  console.printf("Heap libre: %u bytes\r\n", ESP.getFreeHeap());
  console.printf("PSRAM: %s\r\n", psramFound() ? "DISPONIBLE" : "NO DISPONIBLE");
  console.printf("PSRAM libre: %u bytes\r\n", psramFound() ? ESP.getFreePsram() : 0U);
  console.println("======================");
  console.flush();
}


void requestSystemAction(int action) {
  if (pendingSystemAction != SYSTEM_ACTION_NONE) {
    if (console) {
      console.println("[ERROR] Ya hay una accion de sistema pendiente.");
      console.flush();
    }
    return;
  }

  pendingSystemAction = action;
  pendingSystemActionMs = millis();
}

void processRemoteConsoleCommand(String command) {
  command.trim();
  command.toLowerCase();

  while (command.indexOf("  ") >= 0) {
    command.replace("  ", " ");
  }

  if (command.length() == 0) {
    consolePrintPrompt();
    return;
  }

  if (command == "help") {
    consolePrintHelp();
  }
  else if (command == "estado") {
    consolePrintStatus();
  }
  else if (command == "algoritmo") {
    console.println("[ALGORITMO] Opciones disponibles:");
    console.println("  algoritmo 0 - Exceso de Verde normalizado");
    console.println("  algoritmo 1 - HSV por tonos de verde");
    console.printf("[ALGORITMO] Activo: %s (%d)\r\n", detectionModeName().c_str(), detectionMode);
  }
  else if (command.startsWith("algoritmo ")) {
    String modeText = command.substring(10);
    modeText.trim();
    int requestedMode = modeText.toInt();
    if ((modeText == "0" || modeText == "1") &&
        setDetectionModeValue(requestedMode, true)) {
      console.printf("[ALGORITMO] Modo seleccionado: %s\r\n", detectionModeName().c_str());
    } else {
      console.println("[ERROR] Algoritmo no valido. Usa algoritmo 0 o algoritmo 1.");
    }
  }
  else if (command == "auto 60" || command == "auto_60") {
    if (setAutomaticModeSeconds(60)) {
      console.println("[CONSOLA] Modo automatico activado: 60 segundos");
    }
  }
  else if (command == "auto 120" || command == "auto_120") {
    if (setAutomaticModeSeconds(120)) {
      console.println("[CONSOLA] Modo automatico activado: 120 segundos");
    }
  }
  else if (command == "auto 300" || command == "auto_300") {
    if (setAutomaticModeSeconds(300)) {
      console.println("[CONSOLA] Modo automatico activado: 300 segundos");
    }
  }
  else if (command == "auto 86400" || command == "auto_86400" ||
           command == "auto 24h" || command == "auto_24h") {
    if (setAutomaticModeSeconds(86400)) {
      console.println("[CONSOLA] Modo automatico activado: 24 horas");
    }
  }
  else if (command == "auto_sleep 60") {
    if (setAutomaticModeSecondsWithDeepSleep(60)) console.println("[CONSOLA] Automatico con Deep Sleep: 60 segundos");
  }
  else if (command == "auto_sleep 120") {
    if (setAutomaticModeSecondsWithDeepSleep(120)) console.println("[CONSOLA] Automatico con Deep Sleep: 120 segundos");
  }
  else if (command == "auto_sleep 300") {
    if (setAutomaticModeSecondsWithDeepSleep(300)) console.println("[CONSOLA] Automatico con Deep Sleep: 300 segundos");
  }
  else if (command == "auto_sleep 86400" || command == "auto_sleep 24h") {
    if (setAutomaticModeSecondsWithDeepSleep(86400)) console.println("[CONSOLA] Automatico con Deep Sleep: 24 horas");
  }
  else if (command == "auto_sleep off") {
    autoDeepSleepEnabled = false;
    autoDeepSleepCaptureDone = false;
    saveRuntimeConfigToRtc();
    console.println("[CONSOLA] Deep Sleep automatico desactivado. El modo automatico queda en funcionamiento normal si estaba activo.");
  }
  else if (command == "auto off" || command == "auto_off") {
    stopAutomaticMode();
    console.println("[CONSOLA] Modo automatico detenido");
  }
  else if (command.startsWith("auto ")) {
    console.println("[ERROR] Intervalo no valido. Usa 60, 120, 300, 86400, 24h u off.");
  }
  else if (command == "reiniciar") {
    console.println("[SISTEMA] Reinicio solicitado. Reiniciando en 1 segundo...");
    console.flush();
    requestSystemAction(SYSTEM_ACTION_RESTART);
    return;
  }
  else if (command == "apagar") {
    console.println("[SISTEMA] Apagado solicitado. Con USB entrara en deep sleep.");
    console.flush();
    requestSystemAction(SYSTEM_ACTION_SHUTDOWN);
    return;
  }
  else {
    console.println("[ERROR] Comando no reconocido. Escribe help.");
  }

  consolePrintPrompt();
}


void handleRemoteConsole() {
  bool connected = (bool)console;

  if (connected && !consoleWasConnected) {
    consoleWasConnected = true;
    consoleInputBuffer = "";
    console.println();
    console.println("[CONSOLA] Remote Console conectada al sistema de vegetacion.");
    consolePrintHelp();
    consolePrintPrompt();
  }
  else if (!connected && consoleWasConnected) {
    consoleWasConnected = false;
    consoleInputBuffer = "";
  }

  if (!connected) return;

  while (console.available()) {
    int value = console.read();
    if (value < 0) break;

    char c = (char)value;
    if (c != '\r' && c != '\n') {
      if (consoleInputBuffer.length() < 127) {
        consoleInputBuffer += c;
      }
    }
  }

  if (consoleInputBuffer.length() > 0 && !console.available()) {
    String command = consoleInputBuffer;
    consoleInputBuffer = "";
    processRemoteConsoleCommand(command);
  }
}

void handlePendingSystemAction() {
  if (pendingSystemAction == SYSTEM_ACTION_NONE) return;
  if (millis() - pendingSystemActionMs < SYSTEM_ACTION_DELAY_MS) return;

  int action = pendingSystemAction;
  pendingSystemAction = SYSTEM_ACTION_NONE;

  if (action == SYSTEM_ACTION_RESTART) {
    Serial.println("CONSOLA) Reiniciando ESP32...");
    if (console) {
      console.println("[CONSOLA] Reiniciando ahora...");
      console.flush();
    }
    ESP.restart();
    return;
  }

  if (action == SYSTEM_ACTION_SHUTDOWN) {
    Serial.println("CONSOLA) Apagando TimerCamera-F...");

    stopAutomaticMode();

    if (cameraReady) {
      esp_camera_deinit();
      cameraReady = false;
    }

    if (console) {
      console.println("[CONSOLA] Apagando ahora...");
      console.flush();
    }

    thing.stop();
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    digitalWrite(STATUS_LED_PIN, LOW);

    // En bateria, GPIO33 LOW corta la alimentacion de la TimerCamera-F.
    digitalWrite(POWER_HOLD_PIN, LOW);

    // Si sigue alimentada por USB, el corte de bateria no puede apagarla.
    // En ese caso entra en deep sleep sin ninguna fuente de despertar configurada.
    delay(50);
    esp_deep_sleep_start();
  }
}

// Mantiene activas las tareas de red durante una espera controlada.
// Se usa solo en el modo automatico con Deep Sleep para dar tiempo a
// Thinger.io a completar la conexion/autenticacion antes de enviar datos
// y para dejar unos segundos de margen despues de thing.stream("medicion").
void waitForThingerReadyAfterWake(unsigned long waitMs, const char* reason) {
  unsigned long start = millis();

  Serial.printf("THINGER_WAIT) %s durante %lu ms\n", reason, waitMs);
  if (console) {
    console.printf("[THINGER_WAIT] %s durante %lu ms\r\n", reason, waitMs);
    console.flush();
  }

  while (millis() - start < waitMs) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("THINGER_WAIT) WiFi desconectado, intentando reconectar...");
      WiFi.reconnect();
      delay(250);
    }

    thing.handle();
    server.handleClient();
    handleRemoteConsole();
    delay(50);
  }
}

int getRoiX0() {
  if (!roiEnabled) return 0;
  return (ANALYSIS_W * roiLeftPercent) / 100;
}

int getRoiX1() {
  if (!roiEnabled) return ANALYSIS_W;
  return ANALYSIS_W - ((ANALYSIS_W * roiRightPercent) / 100);
}

int getRoiY0() {
  if (!roiEnabled) return 0;
  return (ANALYSIS_H * roiTopPercent) / 100;
}

int getRoiY1() {
  if (!roiEnabled) return ANALYSIS_H;
  return ANALYSIS_H - ((ANALYSIS_H * roiBottomPercent) / 100);
}

int getRoiWidthPx() {
  return getRoiX1() - getRoiX0();
}

int getRoiHeightPx() {
  return getRoiY1() - getRoiY0();
}

uint32_t getRoiTotalPx() {
  return (uint32_t)getRoiWidthPx() * (uint32_t)getRoiHeightPx();
}

bool parseIntegerValue(const String& text, int& value) {
  if (text.length() == 0) return false;

  char* endPtr = nullptr;
  long parsed = strtol(text.c_str(), &endPtr, 10);

  if (endPtr == text.c_str() || *endPtr != '\0') return false;
  if (parsed < -32768L || parsed > 32767L) return false;

  value = (int)parsed;
  return true;
}

bool parseFloatValue(const String& text, float& value) {
  if (text.length() == 0) return false;

  char* endPtr = nullptr;
  float parsed = strtof(text.c_str(), &endPtr);

  if (endPtr == text.c_str() || *endPtr != '\0') return false;
  value = parsed;
  return true;
}

String detectionModeName() {
  switch (detectionMode) {
    case DETECTION_EXG_NORMALIZED:
      return "Exceso de Verde normalizado";
    case DETECTION_HSV_GREEN:
      return "HSV por tonos de verde";
    default:
      return "Desconocido";
  }
}


String detectionModeShortName() {
  switch (detectionMode) {
    case DETECTION_EXG_NORMALIZED:
      return "exg_normalizado";
    case DETECTION_HSV_GREEN:
      return "hsv_verde";
    default:
      return "desconocido";
  }
}



const char* algorithmPreviewStateName() {
  switch (algorithmPreviewState) {
    case PREVIEW_NO_IMAGE:   return "SIN_IMAGEN";
    case PREVIEW_CAPTURING:  return "CAPTURANDO";
    case PREVIEW_PROCESSING: return "PROCESANDO";
    case PREVIEW_READY:      return "LISTO";
    case PREVIEW_ERROR:      return "ERROR";
    default:                 return "DESCONOCIDO";
  }
}

void scheduleAlgorithmPreviewRefresh() {
  if (algorithmPreviewJpeg == nullptr || algorithmPreviewJpegLen == 0) return;

  algorithmPreviewRequestedVersion++;
  algorithmPreviewDirty = true;
  algorithmPreviewDirtySinceMs = millis();

  // No se invalida el resultado anterior: permanece visible hasta que el
  // nuevo procesamiento termine correctamente.
  if (algorithmPreviewState != PREVIEW_CAPTURING &&
      algorithmPreviewState != PREVIEW_PROCESSING) {
    algorithmPreviewState = algorithmPreviewMaskValid ? PREVIEW_READY : PREVIEW_NO_IMAGE;
  }
}

void invalidateDetectionResult() {
  lastAnalysisDone = false;
  lastVegPercent = -1.0f;
  lastVegetationPx = 0;
  lastRoiPx = 0;
  lastDetectionMaskValid = false;
}

bool setDetectionModeValue(int newMode, bool invalidateResult) {
  if (newMode < DETECTION_EXG_NORMALIZED || newMode > DETECTION_HSV_GREEN) {
    return false;
  }

  if (detectionMode != newMode) {
    detectionMode = newMode;
    if (invalidateResult) invalidateDetectionResult();

    Serial.printf("ALGORITMO) Modo seleccionado: %s (%d)\n",
                  detectionModeName().c_str(), detectionMode);
    scheduleAlgorithmPreviewRefresh();
  }

  saveRuntimeConfigToRtc();
  return true;
}


bool setExgThresholdValue(float newThreshold, bool invalidateResult) {
  if (newThreshold < -1.0f || newThreshold > 1.0f) {
    return false;
  }

  if (exgThreshold != newThreshold) {
    exgThreshold = newThreshold;
    if (invalidateResult) invalidateDetectionResult();
    scheduleAlgorithmPreviewRefresh();
  }

  saveRuntimeConfigToRtc();
  return true;
}

bool setHsvConfigValue(int newHMin, int newHMax, int newSMin, int newVMin, bool invalidateResult) {
  if (newHMin < 0 || newHMin > 360 || newHMax < 0 || newHMax > 360) {
    return false;
  }

  if (newSMin < 0 || newSMin > 255 || newVMin < 0 || newVMin > 255) {
    return false;
  }

  bool changed = (hsvHMin != newHMin || hsvHMax != newHMax ||
                  hsvSMin != newSMin || hsvVMin != newVMin);

  hsvHMin = newHMin;
  hsvHMax = newHMax;
  hsvSMin = newSMin;
  hsvVMin = newVMin;

  if (changed && invalidateResult) invalidateDetectionResult();
  if (changed) scheduleAlgorithmPreviewRefresh();

  saveRuntimeConfigToRtc();
  return true;
}

bool setRoiConfigValue(bool enabled, int left, int right, int top, int bottom, bool invalidateResult) {
  if (left < 0 || left > 99 || right < 0 || right > 99 ||
      top < 0 || top > 99 || bottom < 0 || bottom > 99) {
    return false;
  }

  if ((left + right) >= 100 || (top + bottom) >= 100) {
    return false;
  }

  bool changed = (roiEnabled != enabled ||
                  roiLeftPercent != left || roiRightPercent != right ||
                  roiTopPercent != top || roiBottomPercent != bottom);

  roiEnabled = enabled;
  roiLeftPercent = left;
  roiRightPercent = right;
  roiTopPercent = top;
  roiBottomPercent = bottom;

  if (changed && invalidateResult) invalidateDetectionResult();
  if (changed) scheduleAlgorithmPreviewRefresh();

  Serial.printf("ROI) Control actualizado: enabled=%s, L=%d, R=%d, T=%d, B=%d\n",
                roiEnabled ? "true" : "false",
                roiLeftPercent, roiRightPercent,
                roiTopPercent, roiBottomPercent);

  saveRuntimeConfigToRtc();
  return true;
}

void setRoiFullValue(bool invalidateResult) {
  setRoiConfigValue(false, 0, 0, 0, 0, invalidateResult);
}

void setRoiCenterValue(bool invalidateResult) {
  setRoiConfigValue(true, 20, 20, 20, 20, invalidateResult);
}

String algorithmSummaryHtml() {
  String html = "<div class='box'>";
  html += "<h2>Algoritmo de deteccion</h2>";
  html += "<p><b>Algoritmo activo:</b> <span id='summaryMode'>" + detectionModeName() + "</span></p>";
  html += "<p><b>Umbral ExG normalizado:</b> <span id='summaryExg'>" + String(exgThreshold, 3) + "</span></p>";
  html += "<p><b>HSV:</b> <span id='summaryHsv'>H " + String(hsvHMin) + "-" + String(hsvHMax) + ", Smin " + String(hsvSMin) + ", Vmin " + String(hsvVMin) + "</span></p>";
  html += "<p class='mini'>ExG y HSV analizan directamente la imagen capturada.</p>";
  html += "<a class='btn gray' href='/algoritmo'>Configurar algoritmo</a>";
  html += "</div>";
  return html;
}


String roiSummaryHtml() {
  String html = "<div class='box'>";
  html += "<h2>Zona de analisis (ROI)</h2>";
  html += "<p><b>Estado:</b> ";
  html += roiEnabled ? "<span class='ok'>ACTIVA</span>" : "<span class='no'>DESACTIVADA - imagen completa</span>";
  html += "</p>";
  html += "<p><b>Recortes configurados:</b> izquierda " + String(roiLeftPercent) + "% | derecha " + String(roiRightPercent) + "% | arriba " + String(roiTopPercent) + "% | abajo " + String(roiBottomPercent) + "%</p>";
  html += "<p><b>Coordenadas reales:</b> x = " + String(getRoiX0()) + " hasta " + String(getRoiX1()) + " | y = " + String(getRoiY0()) + " hasta " + String(getRoiY1()) + "</p>";
  html += "<p><b>Tamano analizado:</b> " + String(getRoiWidthPx()) + " x " + String(getRoiHeightPx()) + " px</p>";
  html += "<p><b>Pixeles analizados:</b> " + String(getRoiTotalPx()) + "</p>";
  html += "<a class='btn gray' href='/roi'>Configurar ROI</a>";
  html += "</div>";
  return html;
}

String systemStatusHtml() {
  updateBatteryStatus(false);

  String uptimeText = getUptimeFormatted();

  String html = "<div class='box'>";
  html += "<h2>Estado del sistema</h2>";
  html += "<p><b>Bateria:</b> " + String(lastBatteryVoltage, 2) + " V (" + String(lastBatteryPercent) + " %)</p>";
  html += "<p><b>Tiempo activo:</b> " + uptimeText + "</p>";
  html += "<p><b>Uptime segundos:</b> " + String(getUptimeSeconds()) + " s</p>";
  html += "<p><b>Uptime minutos:</b> " + String(getUptimeMinutes()) + " min</p>";
  html += "<p class='mini'>Nota: con USB conectado, la bateria puede mostrar la tension de carga y no el nivel real.</p>";
  html += "</div>";

  return html;
}

void freeBuffer(uint8_t*& ptr, size_t& len) {
  if (ptr != nullptr) {
    free(ptr);
    ptr = nullptr;
  }
  len = 0;
}

void freeBufferNoLen(uint8_t*& ptr) {
  if (ptr != nullptr) {
    free(ptr);
    ptr = nullptr;
  }
}

void clearDetectionVisual() {
  if (lastDetectionMask != nullptr) {
    free(lastDetectionMask);
    lastDetectionMask = nullptr;
  }
  lastDetectionMaskValid = false;
}



// -------------------------------------------------
// Reserva memoria grande usando PSRAM cuando esta disponible
// -------------------------------------------------
void* allocateImageMemory(size_t bytes) {
  void* ptr = nullptr;

  if (psramFound()) {
    ptr = ps_malloc(bytes);
  }

  if (ptr == nullptr) {
    ptr = malloc(bytes);
  }

  if (ptr == nullptr) {
    Serial.printf("ERROR MEMORIA) No se pudieron reservar %u bytes\n", (unsigned int)bytes);
    if (console) {
      console.printf("[ERROR] Memoria insuficiente: no se pudieron reservar %u bytes\r\n", (unsigned int)bytes);
      console.flush();
    }
  }

  return ptr;
}

bool ensureDetectionMaskBuffer() {
  if (lastDetectionMask != nullptr) {
    return true;
  }

  lastDetectionMask = (uint8_t*)allocateImageMemory(FRAME_PIXELS);
  if (lastDetectionMask == nullptr) {
    Serial.println("ERROR MASCARA) No se pudo reservar memoria para la mascara visual");
    if (console) {
      console.println("[ERROR] No se pudo reservar memoria para la mascara visual");
      console.flush();
    }
    lastDetectionMaskValid = false;
    return false;
  }

  memset(lastDetectionMask, 0, FRAME_PIXELS);
  return true;
}


bool ensureAlgorithmPreviewMaskBuffers() {
  if (algorithmPreviewMask == nullptr) {
    algorithmPreviewMask = (uint8_t*)allocateImageMemory(FRAME_PIXELS);
  }

  if (algorithmPreviewWorkMask == nullptr) {
    algorithmPreviewWorkMask = (uint8_t*)allocateImageMemory(FRAME_PIXELS);
  }

  if (algorithmPreviewMask == nullptr || algorithmPreviewWorkMask == nullptr) {
    Serial.println("ERROR PREVIEW) No se pudieron reservar los buffers de mascara");
    algorithmPreviewLastError = "Memoria insuficiente para las mascaras de ajuste";
    return false;
  }

  return true;
}

// -------------------------------------------------
// CSS comun
// -------------------------------------------------
String commonCSS() {
  return
    "<style>"
    "body{font-family:Arial,sans-serif;max-width:1100px;margin:30px auto;padding:0 20px;background:#f4f4f4;}"
    "h1{color:#2c7a2c;} h2{color:#444;} h3{margin-top:0;}"
    "a.btn,button.btn{display:inline-block;margin:8px 4px;padding:12px 22px;background:#2c7a2c;color:#fff;text-decoration:none;border:none;border-radius:6px;font-size:15px;cursor:pointer;}"
    "a.btn:hover,button.btn:hover{background:#1e5c1e;}"
    "a.btn.red,button.btn.red{background:#c0392b;} a.btn.red:hover,button.btn.red:hover{background:#922b21;}"
    "a.btn.blue,button.btn.blue{background:#1a6fa8;} a.btn.blue:hover,button.btn.blue:hover{background:#145880;}"
    "a.btn.gray,button.btn.gray{background:#888;} a.btn.gray:hover,button.btn.gray:hover{background:#555;}"
    ".box{background:#fff;border-radius:8px;padding:20px;margin:16px 0;box-shadow:0 2px 6px rgba(0,0,0,0.1);}"
    ".ok{color:#2c7a2c;font-weight:bold;}"
    ".no{color:#c0392b;font-weight:bold;}"
    ".big{font-size:48px;font-weight:bold;color:#2c7a2c;text-align:center;padding:20px 0;}"
    ".grid3{display:grid;grid-template-columns:repeat(3,1fr);gap:12px;}"
    ".grid2{display:grid;grid-template-columns:1fr 1fr;gap:16px;}"
    ".slot{background:#fafafa;border:1px solid #ddd;border-radius:8px;padding:10px;text-align:center;min-height:190px;}"
    ".slot img{width:100%;max-width:240px;height:auto;border-radius:6px;border:1px solid #ccc;}"
    ".mini{font-size:13px;color:#666;}"
    ".preview img{max-width:100%;border-radius:8px;border:1px solid #bbb;}"
    ".roi-grid{display:grid;grid-template-columns:repeat(2,1fr);gap:12px;}"
    ".roi-field{background:#fafafa;border:1px solid #ddd;border-radius:6px;padding:12px;}"
    ".roi-field label{display:block;font-weight:bold;margin-bottom:6px;}"
    ".roi-field input,.roi-field select{width:100%;box-sizing:border-box;padding:10px;border:1px solid #bbb;border-radius:5px;font-size:15px;}"
    "img{max-width:100%;border-radius:6px;margin-top:12px;border:1px solid #ccc;}"
    "</style>";
}

// -------------------------------------------------
// Decodifica JPEG a RGB888
// -------------------------------------------------
bool decodeJpegToRGB(uint8_t* jpegData, size_t jpegLen) {
  if (rgbBuf == nullptr) {
    rgbBuf = (uint8_t*)allocateImageMemory(FRAME_PIXELS * 3);
    if (rgbBuf == nullptr) {
      Serial.println("ERROR: sin memoria rgbBuf");
      if (console) {
        console.println("[ERROR] Sin memoria para el buffer RGB888");
        console.flush();
      }
      return false;
    }
  }

  bool ok = fmt2rgb888(jpegData, jpegLen, PIXFORMAT_JPEG, rgbBuf);
  if (!ok) {
    Serial.println("ERROR: fmt2rgb888 fallo");
    if (console) {
      console.println("[ERROR] Fallo al convertir JPEG a RGB888");
      console.flush();
    }
  }
  return ok;
}

// -------------------------------------------------
// Inicializar camara
// -------------------------------------------------
bool initCamera() {
  if (cameraReady) return true;

  camera_config_t config;
  memset(&config, 0, sizeof(config));

  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;

  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;

  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;

  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size   = FRAMESIZE_VGA;
  config.jpeg_quality = 10;

  // La TimerCamera-F incorpora 8 MB de PSRAM.
  if (psramFound()) {
    config.fb_count     = 2;
    config.fb_location  = CAMERA_FB_IN_PSRAM;
    config.grab_mode    = CAMERA_GRAB_LATEST;
  } else {
    config.fb_count     = 1;
    config.fb_location  = CAMERA_FB_IN_DRAM;
    config.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("ERROR camara: 0x%x\n", err);
    if (console) {
      console.printf("[ERROR] No se pudo inicializar la camara: 0x%x\r\n", err);
      console.flush();
    }
    return false;
  }

  sensor_t* s = esp_camera_sensor_get();
  if (s) {
    // Configuracion del balance de blancos y de la exposicion del sensor.
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
    s->set_wb_mode(s, 1);
    s->set_exposure_ctrl(s, 1);

    // Orientacion recomendada para la TimerCamera-F.
    s->set_vflip(s, 1);
    s->set_hmirror(s, 0);

    Serial.printf("CAMARA) Sensor PID: 0x%02X\n", s->id.PID);
  }

  cameraReady = true;
  delay(300);
  return true;
}

// -------------------------------------------------
// Captura JPEG estable
// -------------------------------------------------
camera_fb_t* captureStable() {
  camera_fb_t* dummy = esp_camera_fb_get();
  if (dummy) esp_camera_fb_return(dummy);
  delay(150);
  return esp_camera_fb_get();
}

// -------------------------------------------------
// Copiar JPEG
// -------------------------------------------------
bool copyJpegToBuffer(uint8_t* src, size_t len, uint8_t*& dst, size_t& dstLen) {
  freeBuffer(dst, dstLen);

  dst = (uint8_t*)allocateImageMemory(len);
  if (dst == nullptr) return false;

  memcpy(dst, src, len);
  dstLen = len;
  return true;
}

void saveLastJpeg(uint8_t* buf, size_t len) {
  copyJpegToBuffer(buf, len, lastJpeg, lastJpegLen);
}


// -------------------------------------------------
// Captura y reprocesado estable para ajuste visual
// -------------------------------------------------
bool generateAlgorithmPreviewOverlayJpeg(const uint8_t* sourceRgb,
                                         const uint8_t* sourceMask,
                                         uint8_t*& outJpeg,
                                         size_t& outJpegLen) {
  outJpeg = nullptr;
  outJpegLen = 0;

  if (sourceRgb == nullptr || sourceMask == nullptr) {
    algorithmPreviewLastError = "No hay datos para generar la superposicion";
    return false;
  }

  const size_t displayPixels =
      (size_t)ALGORITHM_PREVIEW_DISPLAY_W * (size_t)ALGORITHM_PREVIEW_DISPLAY_H;
  const size_t displayBytes = displayPixels * 3U;

  if (algorithmPreviewDisplayRgb == nullptr) {
    algorithmPreviewDisplayRgb = (uint8_t*)allocateImageMemory(displayBytes);
    if (algorithmPreviewDisplayRgb == nullptr) {
      algorithmPreviewLastError = "Memoria insuficiente para la previsualizacion JPEG";
      return false;
    }
  }

  // Se reduce solo la imagen mostrada. El analisis y el porcentaje se calculan
  // previamente sobre la imagen VGA y la ROI reales. Para que el area roja
  // visible sea representativa, cada pixel de 320x240 resume su bloque 2x2
  // original mediante promedio de color y voto mayoritario de la mascara.
  for (int py = 0; py < ALGORITHM_PREVIEW_DISPLAY_H; py++) {
    int sourceY0 = (py * ANALYSIS_H) / ALGORITHM_PREVIEW_DISPLAY_H;
    int sourceY1 = ((py + 1) * ANALYSIS_H) / ALGORITHM_PREVIEW_DISPLAY_H;
    if (sourceY1 <= sourceY0) sourceY1 = sourceY0 + 1;

    for (int px = 0; px < ALGORITHM_PREVIEW_DISPLAY_W; px++) {
      int sourceX0 = (px * ANALYSIS_W) / ALGORITHM_PREVIEW_DISPLAY_W;
      int sourceX1 = ((px + 1) * ANALYSIS_W) / ALGORITHM_PREVIEW_DISPLAY_W;
      if (sourceX1 <= sourceX0) sourceX1 = sourceX0 + 1;

      uint32_t rSum = 0;
      uint32_t gSum = 0;
      uint32_t bSum = 0;
      uint16_t sampleCount = 0;
      uint16_t vegetationCount = 0;

      for (int sy = sourceY0; sy < sourceY1; sy++) {
        for (int sx = sourceX0; sx < sourceX1; sx++) {
          size_t sourceIndex =
              (size_t)sy * (size_t)ANALYSIS_W + (size_t)sx;
          size_t sourceRgbIndex = sourceIndex * 3U;

          rSum += sourceRgb[sourceRgbIndex + 0U];
          gSum += sourceRgb[sourceRgbIndex + 1U];
          bSum += sourceRgb[sourceRgbIndex + 2U];
          if (sourceMask[sourceIndex] != 0) vegetationCount++;
          sampleCount++;
        }
      }

      uint8_t r = sampleCount > 0 ? (uint8_t)(rSum / sampleCount) : 0;
      uint8_t g = sampleCount > 0 ? (uint8_t)(gSum / sampleCount) : 0;
      uint8_t b = sampleCount > 0 ? (uint8_t)(bSum / sampleCount) : 0;

      if (sampleCount > 0 && vegetationCount * 2U >= sampleCount) {
        r = 255;
        g = g / 3U;
        b = b / 3U;
      }

      size_t displayIndex =
          ((size_t)py * (size_t)ALGORITHM_PREVIEW_DISPLAY_W + (size_t)px) * 3U;
      // El conversor JPEG interpreta este buffer en orden BGR.
      // Se intercambian rojo y azul para que la vegetacion resaltada
      // aparezca roja en la previsualizacion final.
      algorithmPreviewDisplayRgb[displayIndex + 0U] = b;
      algorithmPreviewDisplayRgb[displayIndex + 1U] = g;
      algorithmPreviewDisplayRgb[displayIndex + 2U] = r;
    }
  }

  uint8_t* encodedJpeg = nullptr;
  size_t encodedLength = 0;
  bool encoded = fmt2jpg(algorithmPreviewDisplayRgb,
                         displayBytes,
                         ALGORITHM_PREVIEW_DISPLAY_W,
                         ALGORITHM_PREVIEW_DISPLAY_H,
                         PIXFORMAT_RGB888,
                         ALGORITHM_PREVIEW_JPEG_QUALITY,
                         &encodedJpeg,
                         &encodedLength);

  if (!encoded || encodedJpeg == nullptr || encodedLength == 0) {
    if (encodedJpeg != nullptr) free(encodedJpeg);
    algorithmPreviewLastError = "No se pudo comprimir la superposicion en JPEG";
    return false;
  }

  outJpeg = encodedJpeg;
  outJpegLen = encodedLength;
  return true;
}

bool regenerateAlgorithmPreview(uint32_t targetVersion) {
  if (algorithmPreviewProcessing) {
    return false;
  }

  if (algorithmPreviewJpeg == nullptr || algorithmPreviewJpegLen == 0) {
    algorithmPreviewState = PREVIEW_NO_IMAGE;
    algorithmPreviewLastError = "No hay una imagen de ajuste capturada";
    algorithmPreviewDirty = false;
    return false;
  }

  algorithmPreviewProcessing = true;
  algorithmPreviewState = PREVIEW_PROCESSING;
  algorithmPreviewLastError = "";
  algorithmPreviewDirty = false;

  const unsigned long totalStartMs = millis();
  const uint32_t freePsramBefore = ESP.getFreePsram();

  const unsigned long decodeStartMs = millis();
  if (!decodeJpegToRGB(algorithmPreviewJpeg, algorithmPreviewJpegLen)) {
    algorithmPreviewLastError = "Error al convertir la imagen JPEG a RGB888";
    algorithmPreviewState = PREVIEW_ERROR;
    algorithmPreviewProcessing = false;
    return false;
  }
  const unsigned long decodeElapsedMs = millis() - decodeStartMs;

  if (!ensureAlgorithmPreviewMaskBuffers()) {
    algorithmPreviewState = PREVIEW_ERROR;
    algorithmPreviewProcessing = false;
    return false;
  }

  memset(algorithmPreviewWorkMask, 0, FRAME_PIXELS);

  uint32_t vegetationPixels = 0;
  uint32_t roiPixels = 0;
  const int roiX0 = getRoiX0();
  const int roiX1 = getRoiX1();
  const int roiY0 = getRoiY0();
  const int roiY1 = getRoiY1();

  const unsigned long analysisStartMs = millis();
  for (int y = roiY0; y < roiY1; y++) {
    for (int x = roiX0; x < roiX1; x++) {
      size_t i = (size_t)y * (size_t)ANALYSIS_W + (size_t)x;
      uint8_t r = rgbBuf[i * 3U + 0U];
      uint8_t g = rgbBuf[i * 3U + 1U];
      uint8_t b = rgbBuf[i * 3U + 2U];

      bool isVeg = isVegetationByActiveAlgorithm(r, g, b, i);
      algorithmPreviewWorkMask[i] = isVeg ? 1U : 0U;
      roiPixels++;
      if (isVeg) vegetationPixels++;
    }
  }
  const unsigned long analysisElapsedMs = millis() - analysisStartMs;

  uint8_t* newOverlayJpeg = nullptr;
  size_t newOverlayJpegLen = 0;
  const unsigned long jpegStartMs = millis();
  if (!generateAlgorithmPreviewOverlayJpeg(rgbBuf,
                                           algorithmPreviewWorkMask,
                                           newOverlayJpeg,
                                           newOverlayJpegLen)) {
    algorithmPreviewState = PREVIEW_ERROR;
    algorithmPreviewProcessing = false;
    // El resultado anterior permanece intacto y visible.
    return false;
  }
  const unsigned long jpegElapsedMs = millis() - jpegStartMs;

  // Publicacion atomica: imagen, mascara y estadisticas cambian juntas.
  uint8_t* previousMask = algorithmPreviewMask;
  algorithmPreviewMask = algorithmPreviewWorkMask;
  algorithmPreviewWorkMask = previousMask;

  freeBuffer(algorithmPreviewOverlayJpeg, algorithmPreviewOverlayJpegLen);
  algorithmPreviewOverlayJpeg = newOverlayJpeg;
  algorithmPreviewOverlayJpegLen = newOverlayJpegLen;

  algorithmPreviewVegetationPx = vegetationPixels;
  algorithmPreviewRoiPx = roiPixels;
  algorithmPreviewVegPercent =
      (roiPixels > 0)
          ? 100.0f * ((float)vegetationPixels / (float)roiPixels)
          : 0.0f;

  algorithmPreviewDetectionMode = detectionMode;
  algorithmPreviewExgThreshold = exgThreshold;
  algorithmPreviewHMin = hsvHMin;
  algorithmPreviewHMax = hsvHMax;
  algorithmPreviewSMin = hsvSMin;
  algorithmPreviewVMin = hsvVMin;

  algorithmPreviewMaskValid = true;
  algorithmPreviewProcessedVersion = targetVersion;
  algorithmPreviewVersion++;
  algorithmPreviewState = PREVIEW_READY;
  algorithmPreviewProcessing = false;
  algorithmPreviewLastError = "";

  const unsigned long totalElapsedMs = millis() - totalStartMs;
  const uint32_t freePsramAfter = ESP.getFreePsram();

  Serial.printf(
      "PREVIEW) vSolicitada=%lu vProcesada=%lu imagen=%lu | %s | "
      "%.4f %% (%lu/%lu px) | decode=%lu ms analisis=%lu ms jpeg=%lu ms "
      "total=%lu ms | JPEG=%u bytes | PSRAM antes=%u despues=%u\n",
      (unsigned long)algorithmPreviewRequestedVersion,
      (unsigned long)algorithmPreviewProcessedVersion,
      (unsigned long)algorithmPreviewVersion,
      detectionModeName().c_str(),
      algorithmPreviewVegPercent,
      (unsigned long)algorithmPreviewVegetationPx,
      (unsigned long)algorithmPreviewRoiPx,
      decodeElapsedMs,
      analysisElapsedMs,
      jpegElapsedMs,
      totalElapsedMs,
      (unsigned int)algorithmPreviewOverlayJpegLen,
      (unsigned int)freePsramBefore,
      (unsigned int)freePsramAfter);

  // Si durante una operacion anterior quedo otra version solicitada, se
  // conserva como pendiente y se volvera a procesar tras el debounce.
  if (algorithmPreviewRequestedVersion > algorithmPreviewProcessedVersion) {
    algorithmPreviewDirty = true;
    algorithmPreviewDirtySinceMs = millis();
  }

  return true;
}

bool captureAlgorithmPreviewImage() {
  if (algorithmPreviewProcessing) {
    return false;
  }

  algorithmPreviewProcessing = true;
  algorithmPreviewState = PREVIEW_CAPTURING;
  algorithmPreviewLastError = "";

  const unsigned long captureStartMs = millis();

  if (!initCamera()) {
    algorithmPreviewLastError = "La camara no esta disponible";
    algorithmPreviewState = PREVIEW_ERROR;
    algorithmPreviewProcessing = false;
    return false;
  }

  camera_fb_t* fb = captureStable();
  if (fb == nullptr || fb->buf == nullptr || fb->len == 0) {
    if (fb != nullptr) esp_camera_fb_return(fb);
    algorithmPreviewLastError = "No se pudo capturar una imagen JPEG valida";
    algorithmPreviewState = PREVIEW_ERROR;
    algorithmPreviewProcessing = false;
    Serial.println("PREVIEW) Error capturando la imagen para ajuste");
    return false;
  }

  uint8_t* newPreviewJpeg = (uint8_t*)allocateImageMemory(fb->len);
  if (newPreviewJpeg == nullptr) {
    esp_camera_fb_return(fb);
    algorithmPreviewLastError = "Memoria insuficiente para guardar la captura";
    algorithmPreviewState = PREVIEW_ERROR;
    algorithmPreviewProcessing = false;
    return false;
  }

  memcpy(newPreviewJpeg, fb->buf, fb->len);
  size_t newPreviewJpegLen = fb->len;
  esp_camera_fb_return(fb);

  const unsigned long captureElapsedMs = millis() - captureStartMs;
  Serial.printf("PREVIEW) Captura JPEG preparada: %u bytes en %lu ms\n",
                (unsigned int)newPreviewJpegLen,
                captureElapsedMs);

  // La imagen anterior se conserva hasta comprobar que la nueva produce un
  // resultado completo y valido.
  uint8_t* previousJpeg = algorithmPreviewJpeg;
  size_t previousJpegLen = algorithmPreviewJpegLen;

  algorithmPreviewJpeg = newPreviewJpeg;
  algorithmPreviewJpegLen = newPreviewJpegLen;

  if (algorithmPreviewRequestedVersion <= algorithmPreviewProcessedVersion) {
    algorithmPreviewRequestedVersion = algorithmPreviewProcessedVersion + 1U;
  }
  const uint32_t targetVersion = algorithmPreviewRequestedVersion;

  algorithmPreviewProcessing = false;
  bool ok = regenerateAlgorithmPreview(targetVersion);

  if (ok) {
    if (previousJpeg != nullptr) free(previousJpeg);
    return true;
  }

  // Si la captura o el procesamiento falla, se restaura la imagen anterior y
  // se mantiene visible el ultimo resultado valido.
  freeBuffer(algorithmPreviewJpeg, algorithmPreviewJpegLen);
  algorithmPreviewJpeg = previousJpeg;
  algorithmPreviewJpegLen = previousJpegLen;
  return false;
}

// -------------------------------------------------
// -------------------------------------------------

// -------------------------------------------------
// -------------------------------------------------

// -------------------------------------------------
// Algoritmos de deteccion
// -------------------------------------------------

bool isVegetationExgNormalized(uint8_t r, uint8_t g, uint8_t b) {
  int sum = (int)r + (int)g + (int)b;
  if (sum <= 0) return false;

  float rn = (float)r / (float)sum;
  float gn = (float)g / (float)sum;
  float bn = (float)b / (float)sum;

  float exg = (2.0f * gn) - rn - bn;

  return exg >= exgThreshold;
}

void rgbToHsv(uint8_t r, uint8_t g, uint8_t b, float& hueDeg, uint8_t& sat255, uint8_t& val255) {
  uint8_t maxVal = max(r, max(g, b));
  uint8_t minVal = min(r, min(g, b));
  uint8_t delta = maxVal - minVal;

  val255 = maxVal;

  if (maxVal == 0) {
    sat255 = 0;
    hueDeg = 0.0f;
    return;
  }

  sat255 = (uint8_t)(((int)delta * 255) / maxVal);

  if (delta == 0) {
    hueDeg = 0.0f;
    return;
  }

  if (maxVal == r) {
    hueDeg = 60.0f * ((float)((int)g - (int)b) / (float)delta);
    if (hueDeg < 0.0f) hueDeg += 360.0f;
  } else if (maxVal == g) {
    hueDeg = 60.0f * (2.0f + ((float)((int)b - (int)r) / (float)delta));
  } else {
    hueDeg = 60.0f * (4.0f + ((float)((int)r - (int)g) / (float)delta));
  }

  if (hueDeg >= 360.0f) hueDeg -= 360.0f;
}

bool isHueInConfiguredGreenRange(float hueDeg) {
  if (hsvHMin <= hsvHMax) {
    return hueDeg >= (float)hsvHMin && hueDeg <= (float)hsvHMax;
  }

  return hueDeg >= (float)hsvHMin || hueDeg <= (float)hsvHMax;
}

bool isVegetationHsvGreen(uint8_t r, uint8_t g, uint8_t b) {
  float h = 0.0f;
  uint8_t s = 0;
  uint8_t v = 0;

  rgbToHsv(r, g, b, h, s, v);

  return isHueInConfiguredGreenRange(h) &&
         (s >= hsvSMin) &&
         (v >= hsvVMin);
}

bool isVegetationByActiveAlgorithm(uint8_t r, uint8_t g, uint8_t b, size_t idx) {
  (void)idx;
  switch (detectionMode) {
    case DETECTION_EXG_NORMALIZED:
      return isVegetationExgNormalized(r, g, b);
    case DETECTION_HSV_GREEN:
      return isVegetationHsvGreen(r, g, b);
    default:
      return isVegetationExgNormalized(r, g, b);
  }
}


// -------------------------------------------------
// Enviar a Thinger
// -------------------------------------------------
void sendMeasurementToThinger() {
  if (!lastAnalysisDone) return;

  updateBatteryStatus(true);
  String uptimeText = getUptimeFormatted();

  Serial.printf("[BATERIA] Voltaje: %.2f V, porcentaje: %d %%\n",
                lastBatteryVoltage, lastBatteryPercent);
  Serial.printf("[SISTEMA] Tiempo activo: %s\n", uptimeText.c_str());

  thing.stream("medicion");
  Serial.println("THINGER) Medicion enviada");
  if (console) {
    console.printf("[BATERIA] Voltaje: %.2f V, porcentaje: %d %%\r\n",
                   lastBatteryVoltage, lastBatteryPercent);
    console.printf("[SISTEMA] Tiempo activo: %s\r\n", uptimeText.c_str());
    console.println("[THINGER] Medicion enviada mediante thing.stream(\"medicion\")");
    console.flush();
  }
}

// -------------------------------------------------
// Analisis completo
// -------------------------------------------------
bool performAnalysis(bool sendToThingerFlag) {
  unsigned long analysisStartMs = millis();


  if (!initCamera()) {
    Serial.println("ERROR camara");
    if (console) {
      console.println("[ERROR] No se pudo preparar la camara para el analisis");
      console.flush();
    }
    return false;
  }

  camera_fb_t* fb = captureStable();
  if (!fb) {
    Serial.println("ERROR captura");
    if (console) {
      console.println("[ERROR] No se pudo capturar la imagen");
      console.flush();
    }
    return false;
  }

  saveLastJpeg(fb->buf, fb->len);

  bool ok = decodeJpegToRGB(fb->buf, fb->len);
  esp_camera_fb_return(fb);

  if (!ok) {
    Serial.println("ERROR decodificando JPEG");
    if (console) {
      console.println("[ERROR] Error decodificando la imagen JPEG");
      console.flush();
    }
    return false;
  }

  bool visualMaskAvailable = ensureDetectionMaskBuffer();
  if (visualMaskAvailable) {
    memset(lastDetectionMask, 0, FRAME_PIXELS);
  }

  uint32_t vegetationPixels = 0;
  uint32_t roiPixels = 0;

  const int roiX0 = getRoiX0();
  const int roiX1 = getRoiX1();
  const int roiY0 = getRoiY0();
  const int roiY1 = getRoiY1();

  for (int y = roiY0; y < roiY1; y++) {
    for (int x = roiX0; x < roiX1; x++) {
      size_t i = y * ANALYSIS_W + x;

      uint8_t r = rgbBuf[i * 3 + 0];
      uint8_t g = rgbBuf[i * 3 + 1];
      uint8_t b = rgbBuf[i * 3 + 2];

      bool isVeg = isVegetationByActiveAlgorithm(r, g, b, i);

      roiPixels++;
      if (isVeg) {
        vegetationPixels++;
      }

      if (visualMaskAvailable) {
        lastDetectionMask[i] = isVeg ? 1 : 0;
      }
    }
  }

  lastVegPercent   = (roiPixels > 0) ? 100.0f * ((float)vegetationPixels / (float)roiPixels) : 0.0f;
  if (lastVegPercent < 0.5f) lastVegPercent = 0.0f;
  lastVegetationPx = vegetationPixels;
  lastRoiPx        = roiPixels;
  lastAnalysisDone = true;
  lastAnalysisDetectionMode = detectionMode;
  lastDetectionMaskValid = visualMaskAvailable;

  Serial.printf("ALGORITMO) %s (%d)\n", detectionModeName().c_str(), detectionMode);
  Serial.printf("VEG) %.2f%% (%u/%u px)\n", lastVegPercent, vegetationPixels, roiPixels);
  Serial.printf("ROI) %s, x=%d..%d, y=%d..%d, tamano=%dx%d px\n",
                roiEnabled ? "ACTIVA" : "IMAGEN COMPLETA",
                roiX0, roiX1, roiY0, roiY1,
                roiX1 - roiX0, roiY1 - roiY0);
  Serial.printf("ANALISIS) Tiempo total: %lu ms\n", millis() - analysisStartMs);
  Serial.printf("MASCARA) %s para la ultima imagen\n", lastDetectionMaskValid ? "Generada" : "No disponible");

  if (console) {
    console.printf("[ANALISIS] Algoritmo: %s (%d)\r\n", detectionModeName().c_str(), detectionMode);
    console.printf("[ANALISIS] Finalizado en %lu ms\r\n", millis() - analysisStartMs);
    console.printf("[ANALISIS] Vegetacion: %.2f %% (%lu/%lu px)\r\n",
                   lastVegPercent,
                   (unsigned long)lastVegetationPx,
                   (unsigned long)lastRoiPx);
    console.printf("[MASCARA] %s para la ultima imagen\r\n", lastDetectionMaskValid ? "Generada" : "No disponible");
    console.flush();
  }

  if (sendToThingerFlag) {
    sendMeasurementToThinger();
  }

  return true;
}

bool performAutomaticCaptureAndUpdateCount() {
  bool ok = performAnalysis(false);

  if (ok) {
    autoCaptureCount++;
    rtcAutoCaptureCount = autoCaptureCount;
    rtcLastVegPercent = lastVegPercent;
    rtcLastVegetationPixels = lastVegetationPx;
    saveRuntimeConfigToRtc();
    sendMeasurementToThinger();

    if (console) {
      console.printf("[AUTO] Captura automatica completada. Total: %lu\r\n", autoCaptureCount);
      console.flush();
    }
  } else {
    Serial.println("AUTO) Error en analisis automatico");
    if (console) {
      console.println("[ERROR] Fallo en el analisis automatico");
      console.flush();
    }
  }

  return ok;
}

void enterAutoDeepSleepCycle() {
  rtcTotalAwakeSeconds += getUptimeSeconds();
  saveRuntimeConfigToRtc();

  Serial.printf("AUTO_SLEEP) Entrando en Deep Sleep durante %lu segundos\n", autoIntervalMs / 1000UL);
  if (console) {
    console.printf("[AUTO_SLEEP] Deep Sleep durante %lu segundos\r\n", autoIntervalMs / 1000UL);
    console.flush();
  }

  unsigned long flushStart = millis();
  while (millis() - flushStart < 2000UL) {
    thing.handle();
    server.handleClient();
    delay(10);
  }

  if (cameraReady) {
    esp_camera_deinit();
    cameraReady = false;
  }

  thing.stop();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  digitalWrite(STATUS_LED_PIN, LOW);

  esp_sleep_enable_timer_wakeup((uint64_t)(autoIntervalMs / 1000UL) * 1000000ULL);
  delay(50);
  esp_deep_sleep_start();
}

// -------------------------------------------------
// Servir imagen de slot
// -------------------------------------------------

// -------------------------------------------------
// -------------------------------------------------

// -------------------------------------------------
// Captura estable y estado para configuracion visual de la ROI
// -------------------------------------------------
const char* roiPreviewStateName() {
  switch (roiPreviewState) {
    case ROI_PREVIEW_NO_IMAGE:  return "SIN_IMAGEN";
    case ROI_PREVIEW_CAPTURING: return "CAPTURANDO";
    case ROI_PREVIEW_READY:     return "LISTO";
    case ROI_PREVIEW_ERROR:     return "ERROR";
    default:                    return "DESCONOCIDO";
  }
}

String escapeJsonText(const String& input) {
  String escaped;
  escaped.reserve(input.length() + 8);

  for (size_t i = 0; i < input.length(); i++) {
    char c = input.charAt(i);
    switch (c) {
      case '\\': escaped += "\\\\"; break;
      case '"':  escaped += "\\\""; break;
      case '\n': escaped += "\\n"; break;
      case '\r': break;
      case '\t': escaped += "\\t"; break;
      default:   escaped += c; break;
    }
  }

  return escaped;
}

bool captureRoiPreviewImage() {
  if (roiPreviewProcessing || algorithmPreviewProcessing) {
    roiPreviewLastError = "La camara esta ocupada con otra operacion";
    roiPreviewState = ROI_PREVIEW_ERROR;
    return false;
  }

  roiPreviewProcessing = true;
  roiPreviewState = ROI_PREVIEW_CAPTURING;
  roiPreviewLastError = "";

  if (!initCamera()) {
    roiPreviewLastError = "La camara no esta disponible";
    roiPreviewState = ROI_PREVIEW_ERROR;
    roiPreviewProcessing = false;
    return false;
  }

  const unsigned long captureStartMs = millis();
  camera_fb_t* fb = captureStable();

  if (fb == nullptr || fb->buf == nullptr || fb->len == 0) {
    if (fb != nullptr) esp_camera_fb_return(fb);
    roiPreviewLastError = "No se pudo capturar una imagen JPEG valida";
    roiPreviewState = ROI_PREVIEW_ERROR;
    roiPreviewProcessing = false;
    return false;
  }

  uint8_t* newJpeg = (uint8_t*)allocateImageMemory(fb->len);
  if (newJpeg == nullptr) {
    esp_camera_fb_return(fb);
    roiPreviewLastError = "Memoria insuficiente para guardar la imagen ROI";
    roiPreviewState = ROI_PREVIEW_ERROR;
    roiPreviewProcessing = false;
    return false;
  }

  memcpy(newJpeg, fb->buf, fb->len);
  const size_t newJpegLen = fb->len;
  esp_camera_fb_return(fb);

  // La captura anterior solo se libera despues de disponer de una nueva
  // imagen valida, por lo que un fallo nunca deja la pagina en blanco.
  uint8_t* previousJpeg = roiPreviewJpeg;
  roiPreviewJpeg = newJpeg;
  roiPreviewJpegLen = newJpegLen;
  if (previousJpeg != nullptr) free(previousJpeg);

  roiPreviewVersion++;
  roiPreviewState = ROI_PREVIEW_READY;
  roiPreviewLastError = "";
  roiPreviewProcessing = false;

  Serial.printf("ROI_PREVIEW) Captura preparada: %u bytes, version %lu, %lu ms\n",
                (unsigned int)roiPreviewJpegLen,
                (unsigned long)roiPreviewVersion,
                millis() - captureStartMs);
  return true;
}

String roiVisualStatusJson(bool ok, const String& message) {
  const bool previewAvailable = roiPreviewJpeg != nullptr && roiPreviewJpegLen > 0;

  String json = "{";
  json += "\"ok\":" + String(ok ? "true" : "false");
  json += ",\"message\":\"" + escapeJsonText(message) + "\"";
  json += ",\"enabled\":" + String(roiEnabled ? "true" : "false");
  json += ",\"left\":" + String(roiLeftPercent);
  json += ",\"right\":" + String(roiRightPercent);
  json += ",\"top\":" + String(roiTopPercent);
  json += ",\"bottom\":" + String(roiBottomPercent);
  json += ",\"x0\":" + String(getRoiX0());
  json += ",\"x1\":" + String(getRoiX1());
  json += ",\"y0\":" + String(getRoiY0());
  json += ",\"y1\":" + String(getRoiY1());
  json += ",\"widthPx\":" + String(getRoiWidthPx());
  json += ",\"heightPx\":" + String(getRoiHeightPx());
  json += ",\"totalPx\":" + String((unsigned long)getRoiTotalPx());
  json += ",\"previewAvailable\":" + String(previewAvailable ? "true" : "false");
  json += ",\"previewState\":\"" + String(roiPreviewStateName()) + "\"";
  json += ",\"previewError\":\"" + escapeJsonText(roiPreviewLastError) + "\"";
  json += ",\"previewVersion\":" + String((unsigned long)roiPreviewVersion);
  json += ",\"previewBytes\":" + String((unsigned int)roiPreviewJpegLen);
  json += "}";
  return json;
}

void handleRoiPreviewCapture() {
  server.sendHeader("Cache-Control", "no-store");

  if (pendingRoiPreviewCapture || roiPreviewProcessing || algorithmPreviewProcessing) {
    server.send(409, "application/json",
                roiVisualStatusJson(false, "La camara esta ocupada. Espera y vuelve a intentarlo."));
    return;
  }

  pendingRoiPreviewCapture = true;
  roiPreviewState = ROI_PREVIEW_CAPTURING;
  roiPreviewLastError = "";
  server.send(202, "application/json",
              roiVisualStatusJson(true, "Captura ROI solicitada"));
}

void handleRoiPreviewJpeg() {
  if (roiPreviewJpeg == nullptr || roiPreviewJpegLen == 0) {
    server.send(404, "text/plain", "No hay imagen de referencia para configurar la ROI");
    return;
  }

  WiFiClient client = server.client();
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
  server.sendHeader("Pragma", "no-cache");
  server.setContentLength(roiPreviewJpegLen);
  server.send(200, "image/jpeg", "");
  client.write(roiPreviewJpeg, roiPreviewJpegLen);
}

void handleRoiVisualStatus() {
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", roiVisualStatusJson(true, ""));
}

void handleRoiVisualApply() {
  server.sendHeader("Cache-Control", "no-store");

  if (!server.hasArg("enabled") || !server.hasArg("left") ||
      !server.hasArg("right") || !server.hasArg("top") ||
      !server.hasArg("bottom")) {
    server.send(400, "application/json",
                roiVisualStatusJson(false, "Faltan valores de configuracion ROI"));
    return;
  }

  int newEnabled = 0;
  int newLeft = 0;
  int newRight = 0;
  int newTop = 0;
  int newBottom = 0;

  const bool validNumbers =
      parseIntegerValue(server.arg("enabled"), newEnabled) &&
      parseIntegerValue(server.arg("left"), newLeft) &&
      parseIntegerValue(server.arg("right"), newRight) &&
      parseIntegerValue(server.arg("top"), newTop) &&
      parseIntegerValue(server.arg("bottom"), newBottom);

  if (!validNumbers || (newEnabled != 0 && newEnabled != 1)) {
    server.send(400, "application/json",
                roiVisualStatusJson(false, "Los valores de ROI no son validos"));
    return;
  }

  // Imagen completa siempre se guarda con los cuatro recortes a cero.
  if (newEnabled == 0) {
    newLeft = 0;
    newRight = 0;
    newTop = 0;
    newBottom = 0;
  }

  if (!setRoiConfigValue(newEnabled == 1,
                         newLeft, newRight, newTop, newBottom, true)) {
    server.send(400, "application/json",
                roiVisualStatusJson(false,
                  "ROI no valida. Cada recorte debe estar entre 0 y 99 por ciento."));
    return;
  }

  server.send(200, "application/json",
              roiVisualStatusJson(true,
                newEnabled == 1 ? "Configuracion ROI guardada" : "Imagen completa activada"));
}

// -------------------------------------------------
// PAGINA CONFIGURACION ROI
// -------------------------------------------------
void sendRoiConfigPage(const String& message, bool isError) {
  String html;
  html.reserve(30000);
  html = "<!doctype html><html><head><meta charset='utf-8'>";
  html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<title>Configurar ROI</title>";
  html += commonCSS();
  html += R"ROIHTML(
<style>
  .roi-visual-wrap{position:relative;width:100%;max-width:820px;aspect-ratio:4/3;margin:14px auto;background:#222;border:1px solid #aaa;border-radius:8px;overflow:hidden;}
  .roi-visual-wrap img{position:absolute;inset:0;width:100%;height:100%;object-fit:contain;margin:0;border:0;border-radius:0;display:none;background:#111;}
  .roi-visual-wrap canvas{position:absolute;inset:0;width:100%;height:100%;touch-action:none;cursor:crosshair;display:none;}
  .roi-placeholder{position:absolute;inset:0;display:flex;align-items:center;justify-content:center;padding:20px;text-align:center;color:#eee;background:#333;}
  .roi-toolbar{display:flex;flex-wrap:wrap;gap:8px;align-items:center;}
  .roi-status{padding:10px 12px;border-radius:6px;background:#eef6ee;color:#245b24;margin-top:10px;}
  .roi-status.error{background:#fdecec;color:#922b21;}
  .roi-info-grid{display:grid;grid-template-columns:repeat(4,1fr);gap:10px;margin-top:12px;}
  .roi-info-item{background:#fafafa;border:1px solid #ddd;border-radius:6px;padding:10px;}
  .roi-info-item b{display:block;margin-bottom:4px;color:#444;}
  .roi-advanced-title{display:flex;justify-content:space-between;align-items:center;gap:10px;}
  .roi-help{font-size:13px;color:#666;line-height:1.45;}
  button:disabled{opacity:.55;cursor:not-allowed;}
  @media(max-width:760px){
    .roi-grid,.roi-info-grid{grid-template-columns:1fr 1fr;}
  }
  @media(max-width:480px){
    .roi-grid,.roi-info-grid{grid-template-columns:1fr;}
  }
</style>
</head><body>
<h1>Configuracion de la ROI</h1>
)ROIHTML";

  if (message.length() > 0) {
    html += "<div class='box'><p class='";
    html += isError ? "no" : "ok";
    html += "'>" + message + "</p></div>";
  }

  html += algorithmSummaryHtml();
  html += roiSummaryHtml();

  html += R"ROIHTML(
<div class='box'>
  <h2>Configuracion visual de la ROI</h2>
  <p class='roi-help'>Captura una imagen y ajusta el rectangulo verde sobre la zona que quieras analizar. Puedes moverlo desde el interior y redimensionarlo desde sus bordes o esquinas. La imagen permanece fija mientras realizas el ajuste.</p>

  <div class='roi-toolbar'>
    <button id='captureRoiBtn' class='btn blue' type='button'>Capturar imagen para configurar ROI</button>
    <button id='applyRoiBtn' class='btn' type='button'>Guardar configuracion ROI</button>
  </div>

  <div id='roiVisualWrap' class='roi-visual-wrap'>
    <div id='roiPlaceholder' class='roi-placeholder'>Sin imagen de referencia. Pulsa “Capturar imagen para configurar ROI”.</div>
    <img id='roiImage' alt='Imagen de referencia para configurar ROI'>
    <canvas id='roiCanvas'></canvas>
  </div>

  <div id='roiVisualStatus' class='roi-status'>Consultando el estado de la ROI…</div>

  <div class='roi-info-grid'>
    <div class='roi-info-item'><b>Estado</b><span id='infoEnabled'>-</span></div>
    <div class='roi-info-item'><b>Origen</b><span id='infoOrigin'>x: - | y: -</span></div>
    <div class='roi-info-item'><b>Tamano</b><span id='infoSize'>- x - px</span></div>
    <div class='roi-info-item'><b>Pixeles</b><span id='infoPixels'>-</span></div>
    <div class='roi-info-item'><b>Izquierda</b><span id='infoLeft'>- %</span></div>
    <div class='roi-info-item'><b>Derecha</b><span id='infoRight'>- %</span></div>
    <div class='roi-info-item'><b>Superior</b><span id='infoTop'>- %</span></div>
    <div class='roi-info-item'><b>Inferior</b><span id='infoBottom'>- %</span></div>
  </div>
</div>

<div class='box'>
  <div class='roi-advanced-title'>
    <h2>Ajustes numericos</h2>
    <span class='mini'>Alternativa avanzada sincronizada con el rectangulo</span>
  </div>
  <form id='roiForm' action='/set_roi' method='get'>
    <div class='roi-grid'>
      <div class='roi-field'>
        <label for='enabled'>Estado de la ROI</label>
        <select id='enabled' name='enabled'>
          <option value='1'>Activada</option>
          <option value='0'>Desactivada - imagen completa</option>
        </select>
      </div>
      <div class='roi-field'><label for='left'>Recorte izquierdo (%)</label><input id='left' name='left' type='number' min='0' max='99' step='1' required></div>
      <div class='roi-field'><label for='right'>Recorte derecho (%)</label><input id='right' name='right' type='number' min='0' max='99' step='1' required></div>
      <div class='roi-field'><label for='top'>Recorte superior (%)</label><input id='top' name='top' type='number' min='0' max='99' step='1' required></div>
      <div class='roi-field'><label for='bottom'>Recorte inferior (%)</label><input id='bottom' name='bottom' type='number' min='0' max='99' step='1' required></div>
    </div>
    <p><button class='btn' type='submit'>Guardar configuracion ROI</button></p>
  </form>
</div>

<div class='box'>
  <h2>Opciones rapidas</h2>
  <a id='fullPresetBtn' class='btn blue' href='/roi_preset?mode=full'>Imagen completa</a>
  <a id='centralPresetBtn' class='btn gray' href='/roi_preset?mode=central'>ROI central actual 20 %-80 %</a>
</div>

<div class='box'>
  <a class='btn gray' href='/'>Volver al inicio</a>
  <a class='btn gray' href='/porcentaje'>Ir a porcentaje</a>
</div>

<script>
(() => {
  const W=640, H=480, MAX_CROP=99, MIN_SPAN=0.02;
  const $=id=>document.getElementById(id);
  const image=$('roiImage'), canvas=$('roiCanvas'), wrap=$('roiVisualWrap');
  const ctx=canvas.getContext('2d');
  const placeholder=$('roiPlaceholder'), statusBox=$('roiVisualStatus');
  const captureBtn=$('captureRoiBtn'), applyBtn=$('applyRoiBtn');
  const form=$('roiForm');
  const inputs={enabled:$('enabled'),left:$('left'),right:$('right'),top:$('top'),bottom:$('bottom')};

  let draft={enabled:true,left:20,right:20,top:20,bottom:20};
  let draftDirty=false;
  let dragging=false, dragMode='', startPoint=null, startRect=null;
  let lastPreviewVersion=-1;
  let imageReady=false;
  let applying=false;

  const clamp=(v,min,max)=>Math.max(min,Math.min(max,v));
  const intValue=(v,fallback=0)=>Number.isFinite(Number(v))?Math.round(Number(v)):fallback;

  function setStatus(text,error=false){
    statusBox.textContent=text;
    statusBox.classList.toggle('error',error);
  }

  function normalizedRect(){
    if(!draft.enabled) return {x0:0,y0:0,x1:1,y1:1};
    return {
      x0:draft.left/100,
      y0:draft.top/100,
      x1:1-draft.right/100,
      y1:1-draft.bottom/100
    };
  }

  function sanitizeDraft(){
    draft.enabled=!!draft.enabled;
    draft.left=clamp(intValue(draft.left),0,MAX_CROP);
    draft.right=clamp(intValue(draft.right),0,MAX_CROP);
    draft.top=clamp(intValue(draft.top),0,MAX_CROP);
    draft.bottom=clamp(intValue(draft.bottom),0,MAX_CROP);
    if(!draft.enabled){draft.left=0;draft.right=0;draft.top=0;draft.bottom=0;}
  }

  function syncInputs(){
    sanitizeDraft();
    inputs.enabled.value=draft.enabled?'1':'0';
    inputs.left.value=draft.left;
    inputs.right.value=draft.right;
    inputs.top.value=draft.top;
    inputs.bottom.value=draft.bottom;
    [inputs.left,inputs.right,inputs.top,inputs.bottom].forEach(el=>el.disabled=!draft.enabled);
    updateInfo();
    drawOverlay();
  }

  function updateInfo(){
    sanitizeDraft();
    const x0=draft.enabled?Math.floor(W*draft.left/100):0;
    const x1=draft.enabled?W-Math.floor(W*draft.right/100):W;
    const y0=draft.enabled?Math.floor(H*draft.top/100):0;
    const y1=draft.enabled?H-Math.floor(H*draft.bottom/100):H;
    const width=Math.max(0,x1-x0), height=Math.max(0,y1-y0);
    $('infoEnabled').textContent=draft.enabled?'ACTIVA':'IMAGEN COMPLETA';
    $('infoOrigin').textContent=`x: ${x0} | y: ${y0}`;
    $('infoSize').textContent=`${width} x ${height} px`;
    $('infoPixels').textContent=(width*height).toLocaleString('es-ES');
    $('infoLeft').textContent=`${draft.left} %`;
    $('infoRight').textContent=`${draft.right} %`;
    $('infoTop').textContent=`${draft.top} %`;
    $('infoBottom').textContent=`${draft.bottom} %`;
  }

  function resizeCanvas(){
    const rect=wrap.getBoundingClientRect();
    const width=Math.max(1,Math.round(rect.width));
    const height=Math.max(1,Math.round(rect.height));
    if(canvas.width!==width||canvas.height!==height){canvas.width=width;canvas.height=height;}
    drawOverlay();
  }

  function canvasRect(){
    const r=normalizedRect();
    return {x0:r.x0*canvas.width,y0:r.y0*canvas.height,x1:r.x1*canvas.width,y1:r.y1*canvas.height};
  }

  function drawHandle(x,y){
    const size=Math.max(8,Math.min(canvas.width,canvas.height)*0.018);
    ctx.fillStyle='#fff';ctx.strokeStyle='#188b3a';ctx.lineWidth=2;
    ctx.beginPath();ctx.rect(x-size/2,y-size/2,size,size);ctx.fill();ctx.stroke();
  }

  function drawOverlay(){
    if(!imageReady||canvas.width<2||canvas.height<2){ctx.clearRect(0,0,canvas.width,canvas.height);return;}
    ctx.clearRect(0,0,canvas.width,canvas.height);
    const r=canvasRect();

    if(draft.enabled){
      ctx.fillStyle='rgba(0,0,0,.46)';
      ctx.fillRect(0,0,canvas.width,r.y0);
      ctx.fillRect(0,r.y1,canvas.width,canvas.height-r.y1);
      ctx.fillRect(0,r.y0,r.x0,r.y1-r.y0);
      ctx.fillRect(r.x1,r.y0,canvas.width-r.x1,r.y1-r.y0);
    }

    ctx.strokeStyle=draft.enabled?'#22c55e':'#38bdf8';
    ctx.lineWidth=Math.max(2,canvas.width/240);
    ctx.setLineDash(draft.enabled?[]:[10,7]);
    ctx.strokeRect(r.x0,r.y0,r.x1-r.x0,r.y1-r.y0);
    ctx.setLineDash([]);

    if(draft.enabled){
      const mx=(r.x0+r.x1)/2, my=(r.y0+r.y1)/2;
      [[r.x0,r.y0],[mx,r.y0],[r.x1,r.y0],[r.x1,my],[r.x1,r.y1],[mx,r.y1],[r.x0,r.y1],[r.x0,my]].forEach(p=>drawHandle(p[0],p[1]));
    }

    ctx.font=`bold ${Math.max(13,canvas.width/42)}px Arial`;
    ctx.fillStyle=draft.enabled?'#fff':'#e0f2fe';
    ctx.strokeStyle='rgba(0,0,0,.75)';ctx.lineWidth=3;
    const label=draft.enabled?'ROI seleccionada':'Imagen completa';
    const tx=r.x0+10, ty=Math.max(22,r.y0+24);
    ctx.strokeText(label,tx,ty);ctx.fillText(label,tx,ty);
  }

  function setDraftFromRect(r){
    r.x0=clamp(r.x0,0,1-MIN_SPAN); r.x1=clamp(r.x1,r.x0+MIN_SPAN,1);
    r.y0=clamp(r.y0,0,1-MIN_SPAN); r.y1=clamp(r.y1,r.y0+MIN_SPAN,1);
    draft.enabled=true;
    draft.left=clamp(Math.round(r.x0*100),0,MAX_CROP);
    draft.right=clamp(Math.round((1-r.x1)*100),0,MAX_CROP);
    draft.top=clamp(Math.round(r.y0*100),0,MAX_CROP);
    draft.bottom=clamp(Math.round((1-r.y1)*100),0,MAX_CROP);
    draftDirty=true;
    syncInputs();
    setStatus('ROI modificada visualmente. Pulsa “Guardar configuracion ROI” para aplicarla.');
  }

  function pointerPosition(ev){
    const b=canvas.getBoundingClientRect();
    return {x:(ev.clientX-b.left)*canvas.width/b.width,y:(ev.clientY-b.top)*canvas.height/b.height};
  }

  function hitTest(p){
    const r=canvasRect(), radius=Math.max(14,Math.min(canvas.width,canvas.height)*.035);
    const mx=(r.x0+r.x1)/2,my=(r.y0+r.y1)/2;
    const handles=[['nw',r.x0,r.y0],['n',mx,r.y0],['ne',r.x1,r.y0],['e',r.x1,my],['se',r.x1,r.y1],['s',mx,r.y1],['sw',r.x0,r.y1],['w',r.x0,my]];
    for(const h of handles) if(Math.hypot(p.x-h[1],p.y-h[2])<=radius) return h[0];
    if(p.x>=r.x0&&p.x<=r.x1&&p.y>=r.y0&&p.y<=r.y1) return 'move';
    return '';
  }

  function cursorFor(mode){return ({nw:'nwse-resize',se:'nwse-resize',ne:'nesw-resize',sw:'nesw-resize',n:'ns-resize',s:'ns-resize',e:'ew-resize',w:'ew-resize',move:'move'})[mode]||'crosshair';}

  canvas.addEventListener('pointerdown',ev=>{
    if(!imageReady)return;
    ev.preventDefault();
    if(!draft.enabled){draft.enabled=true;draftDirty=true;syncInputs();}
    const p=pointerPosition(ev),mode=hitTest(p);
    if(!mode)return;
    dragging=true;dragMode=mode;startPoint=p;startRect=normalizedRect();
    canvas.setPointerCapture(ev.pointerId);canvas.style.cursor=cursorFor(mode);
  });

  canvas.addEventListener('pointermove',ev=>{
    if(!imageReady)return;
    const p=pointerPosition(ev);
    if(!dragging){canvas.style.cursor=cursorFor(hitTest(p));return;}
    ev.preventDefault();
    const dx=(p.x-startPoint.x)/canvas.width,dy=(p.y-startPoint.y)/canvas.height;
    const r={...startRect};
    if(dragMode==='move'){
      const width=r.x1-r.x0,height=r.y1-r.y0;
      let x0=r.x0+dx,y0=r.y0+dy;
      x0=clamp(x0,0,1-width);
      y0=clamp(y0,0,1-height);
      r.x0=x0;r.x1=x0+width;r.y0=y0;r.y1=y0+height;
    }else{
      if(dragMode.includes('w'))r.x0=clamp(r.x0+dx,0,r.x1-MIN_SPAN);
      if(dragMode.includes('e'))r.x1=clamp(r.x1+dx,r.x0+MIN_SPAN,1);
      if(dragMode.includes('n'))r.y0=clamp(r.y0+dy,0,r.y1-MIN_SPAN);
      if(dragMode.includes('s'))r.y1=clamp(r.y1+dy,r.y0+MIN_SPAN,1);
    }
    setDraftFromRect(r);
  });

  function finishDrag(ev){if(!dragging)return;dragging=false;dragMode='';try{canvas.releasePointerCapture(ev.pointerId);}catch(_){ }canvas.style.cursor='crosshair';}
  canvas.addEventListener('pointerup',finishDrag);
  canvas.addEventListener('pointercancel',finishDrag);

  function readInputs(){
    draft.enabled=inputs.enabled.value==='1';
    draft.left=intValue(inputs.left.value,draft.left);
    draft.right=intValue(inputs.right.value,draft.right);
    draft.top=intValue(inputs.top.value,draft.top);
    draft.bottom=intValue(inputs.bottom.value,draft.bottom);
    sanitizeDraft();draftDirty=true;syncInputs();
    setStatus('Valores modificados. Pulsa “Guardar configuracion ROI” para aplicarlos.');
  }

  Object.values(inputs).forEach(el=>el.addEventListener('input',readInputs));
  Object.values(inputs).forEach(el=>el.addEventListener('change',readInputs));

  async function applyDraft(message='Aplicando configuracion ROI…'){
    if(applying)return;
    applying=true;applyBtn.disabled=true;setStatus(message);
    const q=new URLSearchParams({enabled:draft.enabled?'1':'0',left:String(draft.left),right:String(draft.right),top:String(draft.top),bottom:String(draft.bottom)});
    try{
      const response=await fetch('/roi_visual_apply?'+q.toString(),{cache:'no-store'});
      const data=await response.json();
      if(!response.ok||!data.ok)throw new Error(data.message||'No se pudo aplicar la ROI');
      draft={enabled:!!data.enabled,left:data.left,right:data.right,top:data.top,bottom:data.bottom};
      draftDirty=false;syncInputs();setStatus(data.message||'ROI actualizada correctamente');
    }catch(err){setStatus(err.message||'Error aplicando la ROI',true);}
    finally{applying=false;applyBtn.disabled=false;}
  }

  form.addEventListener('submit',ev=>{ev.preventDefault();readInputs();applyDraft();});
  applyBtn.addEventListener('click',()=>applyDraft());

  $('fullPresetBtn').addEventListener('click',ev=>{ev.preventDefault();draft={enabled:false,left:0,right:0,top:0,bottom:0};draftDirty=true;syncInputs();applyDraft('Activando imagen completa…');});
  $('centralPresetBtn').addEventListener('click',ev=>{ev.preventDefault();draft={enabled:true,left:20,right:20,top:20,bottom:20};draftDirty=true;syncInputs();applyDraft('Aplicando ROI central…');});

  captureBtn.addEventListener('click',async()=>{
    captureBtn.disabled=true;setStatus('Solicitando una nueva captura para configurar la ROI…');
    try{
      const response=await fetch('/roi_capture',{cache:'no-store'});
      const data=await response.json();
      if(!response.ok||!data.ok)throw new Error(data.message||'No se pudo solicitar la captura');
      setStatus('Capturando imagen…');
    }catch(err){captureBtn.disabled=false;setStatus(err.message||'Error capturando la imagen',true);}
  });

  function loadPreview(version){
    imageReady=false;
    image.onload=()=>{imageReady=true;placeholder.style.display='none';image.style.display='block';canvas.style.display='block';resizeCanvas();setStatus(draftDirty?'Imagen preparada. Hay cambios de ROI sin guardar.':'Imagen preparada para configurar la ROI.');};
    image.onerror=()=>{imageReady=false;setStatus('No se pudo cargar la imagen de referencia.',true);};
    image.src='/roi_preview.jpg?v='+version+'&t='+Date.now();
  }

  function applyServerState(data,force=false){
    if((!draftDirty&&!dragging)||force){
      draft={enabled:!!data.enabled,left:data.left,right:data.right,top:data.top,bottom:data.bottom};
      syncInputs();
    }
    if(data.previewAvailable&&data.previewVersion!==lastPreviewVersion){lastPreviewVersion=data.previewVersion;loadPreview(lastPreviewVersion);}
    if(!data.previewAvailable){placeholder.style.display='flex';image.style.display='none';canvas.style.display='none';imageReady=false;}
    const busy=data.previewState==='CAPTURANDO';captureBtn.disabled=busy;
    if(data.previewState==='CAPTURANDO')setStatus('Capturando imagen para configurar la ROI…');
    else if(data.previewState==='ERROR')setStatus(data.previewError||'Error preparando la imagen ROI',true);
    else if(data.previewState==='SIN_IMAGEN')setStatus('Sin imagen de referencia. Pulsa el boton de captura.');
    else if(!draftDirty&&!applying)setStatus('ROI lista para configurar.');
  }

  async function pollStatus(){
    try{
      const response=await fetch('/roi_visual_status?ts='+Date.now(),{cache:'no-store'});
      if(!response.ok)return;
      applyServerState(await response.json());
    }catch(_){ }
  }

  image.addEventListener('load',resizeCanvas);
  window.addEventListener('resize',resizeCanvas);
  syncInputs();pollStatus();setInterval(pollStatus,1000);
})();
</script>
</body></html>
)ROIHTML";

  server.send(isError ? 400 : 200, "text/html", html);
}

void handleRoiConfig() {
  sendRoiConfigPage("", false);
}

void handleSetRoi() {
  if (!server.hasArg("enabled") || !server.hasArg("left") ||
      !server.hasArg("right") || !server.hasArg("top") ||
      !server.hasArg("bottom")) {
    sendRoiConfigPage("Error: faltan valores de configuracion. Se conserva la configuracion anterior.", true);
    return;
  }

  int newEnabled = 0;
  int newLeft = 0;
  int newRight = 0;
  int newTop = 0;
  int newBottom = 0;

  bool validNumbers = parseIntegerValue(server.arg("enabled"), newEnabled) &&
                      parseIntegerValue(server.arg("left"), newLeft) &&
                      parseIntegerValue(server.arg("right"), newRight) &&
                      parseIntegerValue(server.arg("top"), newTop) &&
                      parseIntegerValue(server.arg("bottom"), newBottom);

  if (!validNumbers || (newEnabled != 0 && newEnabled != 1)) {
    sendRoiConfigPage("Error: los valores introducidos no son validos. Se conserva la configuracion anterior.", true);
    return;
  }

  if (newLeft < 0 || newLeft > 99 || newRight < 0 || newRight > 99 ||
      newTop < 0 || newTop > 99 || newBottom < 0 || newBottom > 99) {
    sendRoiConfigPage("Error: cada porcentaje debe estar comprendido entre 0 y 99 %. Se conserva la configuracion anterior.", true);
    return;
  }

  if ((newLeft + newRight) >= 100 || (newTop + newBottom) >= 100) {
    sendRoiConfigPage("Error: la suma de izquierda y derecha, o de arriba y abajo, debe ser inferior al 100 %. Se conserva la configuracion anterior.", true);
    return;
  }

  setRoiConfigValue(newEnabled == 1, newLeft, newRight, newTop, newBottom, true);

  sendRoiConfigPage("Configuracion ROI guardada correctamente.", false);
}

void handleRoiPreset() {
  if (!server.hasArg("mode")) {
    sendRoiConfigPage("Error: falta el modo de configuracion rapida. Se conserva la configuracion anterior.", true);
    return;
  }

  String mode = server.arg("mode");

  if (mode == "full") {
    setRoiFullValue(true);
    sendRoiConfigPage("Configurada la imagen completa: se analizaran 640 x 480 pixeles.", false);
    return;
  }

  if (mode == "central") {
    setRoiCenterValue(true);
    sendRoiConfigPage("Configurada la ROI central del 20 %-80 %.", false);
    return;
  }

  sendRoiConfigPage("Error: opcion rapida desconocida. Se conserva la configuracion anterior.", true);
}

// -------------------------------------------------
// PAGINA CONFIGURACION ALGORITMO
// -------------------------------------------------
String algorithmStatusJson(bool ok, const String& message) {
  String json = "{";
  json += "\"ok\":" + String(ok ? "true" : "false");
  json += ",\"message\":\"" + message + "\"";
  json += ",\"mode\":" + String(detectionMode);
  json += ",\"modeName\":\"" + detectionModeName() + "\"";
  json += ",\"exg\":" + String(exgThreshold, 3);
  json += ",\"hmin\":" + String(hsvHMin);
  json += ",\"hmax\":" + String(hsvHMax);
  json += ",\"smin\":" + String(hsvSMin);
  json += ",\"vmin\":" + String(hsvVMin);

  bool previewAvailable =
      algorithmPreviewMaskValid &&
      algorithmPreviewOverlayJpeg != nullptr &&
      algorithmPreviewOverlayJpegLen > 0;

  bool previewUpdating =
      algorithmPreviewDirty ||
      pendingAlgorithmPreviewCapture ||
      algorithmPreviewProcessing ||
      algorithmPreviewState == PREVIEW_CAPTURING ||
      algorithmPreviewState == PREVIEW_PROCESSING;

  json += ",\"previewAvailable\":" + String(previewAvailable ? "true" : "false");
  json += ",\"previewUpdating\":" + String(previewUpdating ? "true" : "false");
  json += ",\"previewState\":\"" + String(algorithmPreviewStateName()) + "\"";
  json += ",\"previewError\":\"" + algorithmPreviewLastError + "\"";
  json += ",\"previewVersion\":" + String((unsigned long)algorithmPreviewVersion);
  json += ",\"requestedVersion\":" + String((unsigned long)algorithmPreviewRequestedVersion);
  json += ",\"processedVersion\":" + String((unsigned long)algorithmPreviewProcessedVersion);

  // Estos datos corresponden siempre a la misma mascara y al mismo JPEG
  // publicado. No se sustituyen por ceros durante el procesamiento.
  json += ",\"previewPercent\":" + String(algorithmPreviewVegPercent, 4);
  json += ",\"previewVegetationPx\":" + String((unsigned long)algorithmPreviewVegetationPx);
  json += ",\"previewRoiPx\":" + String((unsigned long)algorithmPreviewRoiPx);
  json += ",\"previewMode\":" + String(algorithmPreviewDetectionMode);
  json += ",\"previewExg\":" + String(algorithmPreviewExgThreshold, 3);
  json += ",\"previewHMin\":" + String(algorithmPreviewHMin);
  json += ",\"previewHMax\":" + String(algorithmPreviewHMax);
  json += ",\"previewSMin\":" + String(algorithmPreviewSMin);
  json += ",\"previewVMin\":" + String(algorithmPreviewVMin);
  json += ",\"previewJpegBytes\":" + String((unsigned int)algorithmPreviewOverlayJpegLen);
  json += ",\"defaultsSaved\":" + String(algorithmDefaultsAvailable ? "true" : "false");
  json += "}";
  return json;
}

void sendAlgorithmConfigPage(const String& message, bool isError) {
  String html = "<!doctype html><html><head><meta charset='utf-8'>";
  html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<title>Configurar algoritmo</title>";
  html += commonCSS();
  html += R"rawliteral(
<style>
.algorithm-control{display:grid;grid-template-columns:minmax(170px,1fr) minmax(260px,2fr) 110px;gap:12px;align-items:center;margin:14px 0;}
.algorithm-control label{font-weight:bold;}
.algorithm-control input[type=range]{width:100%;accent-color:#2c7a2c;}
.algorithm-control input[type=number]{width:100%;box-sizing:border-box;padding:9px;border:1px solid #bbb;border-radius:5px;font-size:15px;}
.preview-panel{text-align:center;}
.preview-panel img{width:100%;max-width:900px;height:auto;background:#eee;}
.preview-placeholder{padding:60px 15px;border:2px dashed #bbb;border-radius:8px;color:#666;background:#fafafa;}
.live-status{padding:10px 12px;border-radius:6px;background:#eef7ee;color:#245d24;margin-top:12px;}
.live-status.error{background:#fdeeee;color:#9b2922;}
.action-row{display:flex;flex-wrap:wrap;align-items:center;gap:6px;}
@media(max-width:760px){.algorithm-control{grid-template-columns:1fr;}.algorithm-control input[type=number]{max-width:none;}}
</style>
)rawliteral";
  html += "</head><body>";
  html += "<h1>Configuracion visual del algoritmo</h1>";

  if (message.length() > 0) {
    html += "<div class='box'><p class='";
    html += isError ? "no" : "ok";
    html += "'>" + message + "</p></div>";
  }

  html += algorithmSummaryHtml();

  html += R"rawliteral(
<div class='box'>
  <h2>Seleccionar algoritmo</h2>
  <div class='roi-field'>
    <label for='mode'>Algoritmo activo</label>
    <select id='mode'>
)rawliteral";
  html += "<option value='0'" + String(detectionMode == DETECTION_EXG_NORMALIZED ? " selected" : "") + ">Exceso de Verde normalizado</option>";
  html += "<option value='1'" + String(detectionMode == DETECTION_HSV_GREEN ? " selected" : "") + ">HSV por tonos de verde</option>";
  html += R"rawliteral(
    </select>
  </div>
  <p class='mini'>El cambio se aplica inmediatamente. No es necesario guardar para probarlo.</p>
</div>

<div class='box'>
  <h2>Parametros ExG normalizado</h2>
  <div class='algorithm-control'>
    <label for='exgRange'>Umbral ExG (-1.00 a 1.00)</label>
    <input id='exgRange' type='range' min='-1' max='1' step='0.005'>
    <input id='exgNumber' type='number' min='-1' max='1' step='0.005'>
  </div>
</div>

<div class='box'>
  <h2>Parametros HSV</h2>
  <p class='mini'>H se expresa en grados 0-360. S y V se expresan en escala 0-255.</p>
  <div class='algorithm-control'>
    <label for='hminRange'>Hue minimo</label>
    <input id='hminRange' type='range' min='0' max='360' step='1'>
    <input id='hminNumber' type='number' min='0' max='360' step='1'>
  </div>
  <div class='algorithm-control'>
    <label for='hmaxRange'>Hue maximo</label>
    <input id='hmaxRange' type='range' min='0' max='360' step='1'>
    <input id='hmaxNumber' type='number' min='0' max='360' step='1'>
  </div>
  <div class='algorithm-control'>
    <label for='sminRange'>Saturacion minima</label>
    <input id='sminRange' type='range' min='0' max='255' step='1'>
    <input id='sminNumber' type='number' min='0' max='255' step='1'>
  </div>
  <div class='algorithm-control'>
    <label for='vminRange'>Brillo minimo</label>
    <input id='vminRange' type='range' min='0' max='255' step='1'>
    <input id='vminNumber' type='number' min='0' max='255' step='1'>
  </div>
</div>

<div class='box preview-panel'>
  <h2>Previsualizacion de la deteccion</h2>
  <p class='mini'>La imagen original permanece fija. Al cambiar un parametro solo se reprocesa la misma escena y los pixeles detectados se resaltan en rojo.</p>
  <div id='previewPlaceholder' class='preview-placeholder'>Pulsa <b>Capturar imagen para ajuste</b> para crear una imagen de referencia estable.</div>
  <img id='previewImage' alt='Superposicion de deteccion' style='display:none'>
  <p id='previewMetrics' class='mini'></p>
  <div id='liveStatus' class='live-status'>Preparado.</div>
</div>

<div class='box'>
  <div class='action-row'>
    <button id='capturePreviewBtn' class='btn blue' type='button'>Capturar imagen para ajuste</button>
    <button id='saveDefaultsBtn' class='btn' type='button'>Guardar algoritmo por defecto</button>
    <a class='btn gray' href='/'>Volver al inicio</a>
    <a class='btn gray' href='/porcentaje'>Ir a porcentaje</a>
  </div>
  <p class='mini'>Los sliders aplican los cambios inmediatamente. El boton verde solo guarda el algoritmo y sus parametros en memoria permanente para futuros arranques.</p>
</div>

<script>
const ids={mode:'mode',exgR:'exgRange',exgN:'exgNumber',hminR:'hminRange',hminN:'hminNumber',hmaxR:'hmaxRange',hmaxN:'hmaxNumber',sminR:'sminRange',sminN:'sminNumber',vminR:'vminRange',vminN:'vminNumber'};
const APPLY_DEBOUNCE_MS=2000;
let applyTimer=null;
let lastPreviewVersion=-1;
let applyRequestRunning=false;
let applyAgain=false;

// Protege los controles mientras existe una edicion local pendiente de
// confirmacion por el ESP32. El polling puede seguir actualizando la
// imagen y las metricas, pero no devuelve los sliders al valor anterior.
let localEditPending=false;

// Identifica la ultima edicion realizada en la pagina. Una respuesta antigua
// no puede confirmar ni sobrescribir un cambio mas reciente.
let localEditVersion=0;

function el(id){return document.getElementById(id);}
function setLiveStatus(text,error=false){
  const n=el('liveStatus');
  n.textContent=text;
  n.className='live-status'+(error?' error':'');
}
function syncPair(source,target){el(target).value=el(source).value;}
function setIfIdle(id,value){
  const node=el(id);
  if(document.activeElement!==node)node.value=value;
}
function previewModeName(data){
  return Number(data.previewMode)===1?'HSV por tonos de verde':'Exceso de Verde normalizado';
}
function previewParameterText(data){
  if(Number(data.previewMode)===1){
    return 'H '+data.previewHMin+'-'+data.previewHMax+
           ', Smin '+data.previewSMin+', Vmin '+data.previewVMin;
  }
  return 'Umbral ExG '+Number(data.previewExg).toFixed(3);
}

function applyServerState(data){
  // Mientras hay una edicion local pendiente, el polling sigue actualizando
  // imagen, porcentaje y estado, pero no puede devolver los controles al
  // valor anterior que todavia conserva el ESP32.
  if(!localEditPending){
    setIfIdle(ids.mode,data.mode);
    setIfIdle(ids.exgR,data.exg);
    setIfIdle(ids.exgN,data.exg);
    setIfIdle(ids.hminR,data.hmin);
    setIfIdle(ids.hminN,data.hmin);
    setIfIdle(ids.hmaxR,data.hmax);
    setIfIdle(ids.hmaxN,data.hmax);
    setIfIdle(ids.sminR,data.smin);
    setIfIdle(ids.sminN,data.smin);
    setIfIdle(ids.vminR,data.vmin);
    setIfIdle(ids.vminN,data.vmin);
  }

  if(el('summaryMode'))el('summaryMode').textContent=data.modeName;
  if(el('summaryExg'))el('summaryExg').textContent=Number(data.exg).toFixed(3);
  if(el('summaryHsv'))el('summaryHsv').textContent=
      'H '+data.hmin+'-'+data.hmax+', Smin '+data.smin+', Vmin '+data.vmin;

  const img=el('previewImage');
  const placeholder=el('previewPlaceholder');
  const captureBtn=el('capturePreviewBtn');
  const busy=Boolean(data.previewUpdating);

  captureBtn.disabled=busy;
  captureBtn.style.opacity=busy?'0.6':'1';

  if(data.previewAvailable){
    placeholder.style.display='none';
    img.style.display='block';

    if(Number(data.previewVersion)!==lastPreviewVersion){
      lastPreviewVersion=Number(data.previewVersion);
      img.src='/algorithm_preview.jpg?v='+data.previewVersion;
    }

    el('previewMetrics').textContent=
      'Algoritmo: '+previewModeName(data)+
      ' | '+previewParameterText(data)+
      ' | Vegetacion: '+Number(data.previewPercent).toFixed(2)+' %'+
      ' | Pixeles: '+data.previewVegetationPx+' / '+data.previewRoiPx;
  }else{
    img.style.display='none';
    placeholder.style.display='block';
    el('previewMetrics').textContent='';
  }

  if(data.previewState==='CAPTURANDO'){
    setLiveStatus('Capturando una nueva imagen JPEG para ajuste...');
  }else if(data.previewState==='PROCESANDO'){
    setLiveStatus('Procesando la imagen y generando la nueva superposicion...');
  }else if(data.previewState==='ERROR'){
    setLiveStatus(data.previewError||'Se produjo un error. Se conserva el ultimo resultado valido.',true);
  }else if(data.previewUpdating){
    setLiveStatus('Cambio recibido. Esperando 2 segundos para procesar el ultimo valor...');
  }else if(data.previewState==='LISTO'){
    setLiveStatus('Previsualizacion actualizada correctamente.');
  }else{
    setLiveStatus('Pulsa Capturar imagen para ajuste para comenzar.');
  }
}

function queryParams(){
  const p=new URLSearchParams();
  p.set('mode',el(ids.mode).value);
  p.set('exg',el(ids.exgN).value);
  p.set('hmin',el(ids.hminN).value);
  p.set('hmax',el(ids.hmaxN).value);
  p.set('smin',el(ids.sminN).value);
  p.set('vmin',el(ids.vminN).value);
  return p;
}

async function sendCurrentValues(){
  // Se conserva la version exacta de la configuracion que se envia.
  const versionSent=localEditVersion;
  const params=queryParams();

  try{
    const response=await fetch(
      '/algorithm_live_update?'+params.toString(),
      {cache:'no-store'}
    );
    const data=await response.json();

    if(!response.ok||!data.ok){
      throw new Error(data.message||'No se pudo aplicar la configuracion');
    }

    // Solo se confirma la edicion si el usuario no ha realizado otro cambio
    // mientras esta peticion estaba en curso.
    if(versionSent===localEditVersion){
      localEditPending=false;
    }else{
      applyAgain=true;
    }

    applyServerState(data);
  }catch(err){
    // Evita que la pagina quede bloqueada permanentemente si falla la ultima
    // peticion enviada.
    if(versionSent===localEditVersion){
      localEditPending=false;
    }
    throw err;
  }
}

async function applyNow(){
  clearTimeout(applyTimer);

  if(applyRequestRunning){
    applyAgain=true;
    return;
  }

  applyRequestRunning=true;
  try{
    do{
      applyAgain=false;
      await sendCurrentValues();
    }while(applyAgain);
  }catch(err){
    setLiveStatus(err.message,true);
  }finally{
    applyRequestRunning=false;
  }
}

function scheduleApply(){
  clearTimeout(applyTimer);

  // Desde el primer movimiento, el polling deja de sobrescribir los controles.
  localEditPending=true;
  localEditVersion++;

  setLiveStatus('Valor preparado. Se procesara cuando lleve 2 segundos sin cambios.');
  applyTimer=setTimeout(applyNow,APPLY_DEBOUNCE_MS);
}

async function capturePreview(){
  clearTimeout(applyTimer);
  setLiveStatus('Solicitando una nueva captura...');
  el('capturePreviewBtn').disabled=true;

  try{
    const response=await fetch('/algorithm_capture',{cache:'no-store'});
    const data=await response.json();
    if(!response.ok||!data.ok)throw new Error(data.message||'Error solicitando la captura');
    applyServerState(data);
  }catch(err){
    setLiveStatus(err.message,true);
    el('capturePreviewBtn').disabled=false;
  }
}

async function saveDefaults(){
  setLiveStatus('Guardando configuracion por defecto...');
  try{
    const response=await fetch('/save_algorithm_defaults',{cache:'no-store'});
    const data=await response.json();
    if(!response.ok||!data.ok)throw new Error(data.message||'No se pudo guardar');
    applyServerState(data);
    setLiveStatus('Algoritmo y parametros guardados como valores por defecto.');
  }catch(err){
    setLiveStatus(err.message,true);
  }
}

async function pollStatus(){
  try{
    const r=await fetch('/algorithm_status?ts='+Date.now(),{cache:'no-store'});
    if(!r.ok)return;
    const d=await r.json();
    applyServerState(d);
  }catch(e){}
}

el(ids.mode).addEventListener('change',scheduleApply);

[
  [ids.exgR,ids.exgN],
  [ids.hminR,ids.hminN],
  [ids.hmaxR,ids.hmaxN],
  [ids.sminR,ids.sminN],
  [ids.vminR,ids.vminN]
].forEach(pair=>{
  const updateFromRange=()=>{
    syncPair(pair[0],pair[1]);
    scheduleApply();
  };

  const updateFromNumber=()=>{
    syncPair(pair[1],pair[0]);
    scheduleApply();
  };

  // input sincroniza el valor mientras se arrastra el slider.
  el(pair[0]).addEventListener('input',updateFromRange);

  // change garantiza que se registre el valor final al soltarlo.
  el(pair[0]).addEventListener('change',updateFromRange);

  // Se mantiene la introduccion manual por numero.
  el(pair[1]).addEventListener('input',updateFromNumber);
  el(pair[1]).addEventListener('change',updateFromNumber);
});
el('capturePreviewBtn').addEventListener('click',capturePreview);
el('saveDefaultsBtn').addEventListener('click',saveDefaults);

el('previewImage').addEventListener('error',()=>{
  setLiveStatus('No se pudo descargar la nueva previsualizacion. Se intentara de nuevo.',true);
});

pollStatus();
setInterval(pollStatus,1000);
</script>
)rawliteral";

  html += "</body></html>";
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
  server.send(isError ? 400 : 200, "text/html", html);
}

void handleAlgorithmConfig() {
  sendAlgorithmConfigPage("", false);
}

void handleAlgorithmLiveUpdate() {
  if (!server.hasArg("mode") || !server.hasArg("exg") ||
      !server.hasArg("hmin") || !server.hasArg("hmax") ||
      !server.hasArg("smin") || !server.hasArg("vmin")) {
    server.send(400, "application/json", algorithmStatusJson(false, "Faltan parametros"));
    return;
  }

  int newMode = 0;
  float newExg = 0.0f;
  int newHMin = 0, newHMax = 0, newSMin = 0, newVMin = 0;
  bool parsed = parseIntegerValue(server.arg("mode"), newMode) &&
                parseFloatValue(server.arg("exg"), newExg) &&
                parseIntegerValue(server.arg("hmin"), newHMin) &&
                parseIntegerValue(server.arg("hmax"), newHMax) &&
                parseIntegerValue(server.arg("smin"), newSMin) &&
                parseIntegerValue(server.arg("vmin"), newVMin);

  bool valid = parsed &&
               newMode >= DETECTION_EXG_NORMALIZED && newMode <= DETECTION_HSV_GREEN &&
               newExg >= -1.0f && newExg <= 1.0f &&
               newHMin >= 0 && newHMin <= 360 &&
               newHMax >= 0 && newHMax <= 360 &&
               newSMin >= 0 && newSMin <= 255 &&
               newVMin >= 0 && newVMin <= 255;

  if (!valid) {
    server.send(400, "application/json", algorithmStatusJson(false, "Valores fuera de rango"));
    return;
  }

  setDetectionModeValue(newMode, true);
  setExgThresholdValue(newExg, true);
  setHsvConfigValue(newHMin, newHMax, newSMin, newVMin, true);

  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", algorithmStatusJson(true, "Configuracion aplicada"));
}

void handleAlgorithmPreviewCapture() {
  if (!pendingAlgorithmPreviewCapture) {
    pendingAlgorithmPreviewCapture = true;
    algorithmPreviewRequestedVersion++;
    algorithmPreviewDirty = false;
    algorithmPreviewState = PREVIEW_CAPTURING;
    algorithmPreviewLastError = "";
  }

  // La peticion web responde inmediatamente. La captura y el procesamiento se
  // realizan desde loop(), evitando dejar la pagina esperando una operacion larga.
  server.sendHeader("Cache-Control", "no-store");
  server.send(202, "application/json",
              algorithmStatusJson(true, "Captura de ajuste solicitada"));
}

void handleAlgorithmStatus() {
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
  server.send(200, "application/json", algorithmStatusJson(true, ""));
}

void handleSaveAlgorithmDefaults() {
  bool ok = saveAlgorithmDefaultsToNvs();
  server.sendHeader("Cache-Control", "no-store");
  server.send(ok ? 200 : 500, "application/json",
              algorithmStatusJson(ok, ok ? "Configuracion guardada por defecto" : "Error guardando en memoria permanente"));
}

// Se conserva la ruta antigua por compatibilidad. Ahora aplica los valores y
// los guarda como predeterminados, pero la pagina nueva ya no depende de ella.
void handleSetAlgorithm() {
  if (!server.hasArg("mode") || !server.hasArg("exg") ||
      !server.hasArg("hmin") || !server.hasArg("hmax") ||
      !server.hasArg("smin") || !server.hasArg("vmin")) {
    sendAlgorithmConfigPage("Error: faltan parametros. Se conserva la configuracion anterior.", true);
    return;
  }

  int newMode = 0, newHMin = 0, newHMax = 0, newSMin = 0, newVMin = 0;
  float newExg = 0.0f;
  bool ok = parseIntegerValue(server.arg("mode"), newMode) &&
            parseFloatValue(server.arg("exg"), newExg) &&
            parseIntegerValue(server.arg("hmin"), newHMin) &&
            parseIntegerValue(server.arg("hmax"), newHMax) &&
            parseIntegerValue(server.arg("smin"), newSMin) &&
            parseIntegerValue(server.arg("vmin"), newVMin);

  if (!ok || !setDetectionModeValue(newMode, true) ||
      !setExgThresholdValue(newExg, true) ||
      !setHsvConfigValue(newHMin, newHMax, newSMin, newVMin, true)) {
    sendAlgorithmConfigPage("Error: valores de algoritmo no validos.", true);
    return;
  }

  bool saved = saveAlgorithmDefaultsToNvs();
  sendAlgorithmConfigPage(saved ? "Algoritmo guardado como configuracion por defecto." : "Parametros aplicados, pero no se pudieron guardar en memoria permanente.", !saved);
}

// -------------------------------------------------
// PAGINA INICIO
// -------------------------------------------------
void handleRoot() {
  String html = "<!doctype html><html><head><meta charset='utf-8'><title>Vegetacion</title>";
  html += commonCSS();
  html += "</head><body>";
  html += "<h1>Analisis de Vegetacion</h1>";
  html += "<div class='box'>";
  html += "<p>Selecciona una opcion:</p>";
  html += "<a class='btn blue' href='/porcentaje'>Porcentaje manual</a>";
  html += "<a class='btn gray' href='/auto'>Porcentaje automatico</a>";
  html += "<a class='btn gray' href='/roi'>Configurar ROI</a>";
  html += "<a class='btn gray' href='/algoritmo'>Configurar algoritmo</a>";
  html += "<a class='btn gray' href='/wifi'>Configurar Wi-Fi</a>";
  html += "</div>";
  html += algorithmSummaryHtml();
  html += systemStatusHtml();
  html += "</body></html>";
  server.send(200, "text/html", html);
}


// -------------------------------------------------
// -------------------------------------------------

// -------------------------------------------------
// -------------------------------------------------

// -------------------------------------------------
// -------------------------------------------------

// -------------------------------------------------
// -------------------------------------------------

// -------------------------------------------------
// -------------------------------------------------

// -------------------------------------------------
// PAGINA PORCENTAJE
// -------------------------------------------------
void handlePorcentaje() {
  String html = "<!doctype html><html><head><meta charset='utf-8'><title>Porcentaje</title>";
  html += commonCSS();
  html += "</head><body>";
  html += "<h1>Porcentaje de Vegetacion</h1>";

  if (!lastAnalysisDone) {
    html += "<div class='box'>";
    html += "<p>Todavia no se ha realizado ninguna medicion.</p>";
    html += "<p>Pulsa el boton para hacer la primera foto y calcular:</p>";
    html += "<a class='btn' href='/analizar'>Hacer foto y calcular</a>";
    html += "</div>";
  } else {
    html += "<div class='box'>";
    html += "<div class='big'>" + String(lastVegPercent, 1) + "%</div>";
    html += "<p style='text-align:center;color:#666;'>vegetacion detectada</p>";
    html += "<p><b>Pixeles vegetacion:</b> " + String(lastVegetationPx) + "</p>";
    html += "<p><b>Pixeles ROI total:</b> " + String(lastRoiPx) + "</p>";
    html += "<p><b>Algoritmo usado:</b> " + detectionModeName() + "</p>";
    html += "</div>";

    if (lastJpeg != nullptr) {
      html += "<div class='box'>";
      html += "<p><b>Foto analizada:</b></p>";
      html += "<img src='/ultima_foto'>";
      html += "</div>";
    }

    if (lastDetectionMaskValid) {
      html += "<div class='box'>";
      html += "<h2>Visualizacion de deteccion</h2>";
      html += "<p class='mini'>Los pixeles resaltados son exactamente los clasificados como vegetacion en el ultimo analisis.</p>";
      html += "<div class='grid2'>";
      html += "<div><p><b>Superposicion sobre la imagen original:</b></p><img src='/detection_overlay.bmp?ts=" + String(millis()) + "'></div>";
      html += "<div><p><b>Mascara binaria:</b></p><img src='/detection_mask.bmp?ts=" + String(millis()) + "'></div>";
      html += "</div>";
      html += "</div>";
    }

    html += "<div class='box'>";
    html += "<a class='btn' href='/analizar'>Nueva foto y recalcular</a>";
    html += "</div>";
  }

  html += algorithmSummaryHtml();
  html += roiSummaryHtml();
  html += systemStatusHtml();

  html += "<br><a class='btn gray' href='/'>Volver al inicio</a>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}


// -------------------------------------------------
// PAGINA AUTOMATICA
// -------------------------------------------------
void handleAuto() {
  String html = "<!doctype html><html><head><meta charset='utf-8'><title>Automatico</title>";
  if (autoModeEnabled) {
    html += "<meta http-equiv='refresh' content='5'>";
  }
  html += commonCSS();
  html += "</head><body>";

  html += "<h1>Porcentaje automatico</h1>";

  html += "<div class='box'>";
  html += "<p><b>Estado:</b> <span class='" + autoStatusClass() + "'>" + autoStatusText() + "</span></p>";
  html += "<p><b>Intervalo activo:</b> " + intervalText(autoIntervalMs) + "</p>";
  html += "<p><b>Deep Sleep automatico:</b> ";
  html += autoDeepSleepEnabled ? "<span class='ok'>ACTIVO</span>" : "<span class='no'>PARADO</span>";
  html += "</p>";
  html += "<p><b>Ventana despierta:</b> " + String(awakeWindowSeconds) + " s</p>";
  html += "<p><b>Capturas automaticas realizadas:</b> " + String(autoCaptureCount) + "</p>";
  html += "<p><b>Despertares Deep Sleep:</b> " + String(wakeCount) + "</p>";
  html += "<p><b>Tiempo activo acumulado:</b> " + getTotalAwakeFormatted() + "</p>";
  html += "</div>";

  html += systemStatusHtml();
  html += algorithmSummaryHtml();

  html += "<div class='box'>";
  html += "<h2>Elegir intervalo sin Deep Sleep</h2>";
  html += "<a class='btn blue' href='/auto_start?sec=60&sleep=0'>Cada 60 segundos</a>";
  html += "<a class='btn blue' href='/auto_start?sec=120&sleep=0'>Cada 2 minutos</a>";
  html += "<a class='btn blue' href='/auto_start?sec=300&sleep=0'>Cada 5 minutos</a>";
  html += "<a class='btn blue' href='/auto_start?sec=86400&sleep=0'>Cada 24 horas</a>";
  html += "<h2>Elegir intervalo con Deep Sleep</h2>";
  html += "<a class='btn' href='/auto_start?sec=60&sleep=1'>60 s + Deep Sleep</a>";
  html += "<a class='btn' href='/auto_start?sec=120&sleep=1'>120 s + Deep Sleep</a>";
  html += "<a class='btn' href='/auto_start?sec=300&sleep=1'>300 s + Deep Sleep</a>";
  html += "<a class='btn' href='/auto_start?sec=86400&sleep=1'>24 h + Deep Sleep</a>";
  html += "<a class='btn red' href='/auto_stop'>Parar automatico</a>";
  html += "</div>";

  html += roiSummaryHtml();

  if (!lastAnalysisDone) {
    html += "<div class='box'><p>Todavia no hay ninguna medicion.</p></div>";
  } else {
    html += "<div class='grid2'>";

    html += "<div class='box'>";
    html += "<h2>Ultimos valores</h2>";
    html += "<div class='big'>" + String(lastVegPercent, 1) + "%</div>";
    html += "<p><b>Pixeles vegetacion:</b> " + String(lastVegetationPx) + "</p>";
    html += "<p><b>Pixeles ROI total:</b> " + String(lastRoiPx) + "</p>";
    html += "<p><b>Algoritmo usado:</b> " + detectionModeName() + "</p>";

    if (autoModeEnabled) {
      unsigned long elapsed = millis() - lastAutoCaptureMs;
      unsigned long remaining = (elapsed >= autoIntervalMs) ? 0 : (autoIntervalMs - elapsed) / 1000UL;
      html += "<p><b>Proxima captura en:</b> " + String(remaining) + " s</p>";
    }

    html += "</div>";

    html += "<div class='box'>";
    html += "<h2>Ultima foto automatica</h2>";
    if (lastJpeg != nullptr && lastJpegLen > 0) {
      html += "<img src='/ultima_foto?ts=" + String(millis()) + "'>";
    } else {
      html += "<p class='mini'>No hay foto disponible.</p>";
    }
    html += "</div>";

    html += "</div>";

    if (lastDetectionMaskValid) {
      html += "<div class='box'>";
      html += "<h2>Visualizacion de deteccion</h2>";
      html += "<p class='mini'>Imagen actualizada con el ultimo analisis manual o automatico.</p>";
      html += "<div class='grid2'>";
      html += "<div><p><b>Superposicion:</b></p><img src='/detection_overlay.bmp?ts=" + String(millis()) + "'></div>";
      html += "<div><p><b>Mascara binaria:</b></p><img src='/detection_mask.bmp?ts=" + String(millis()) + "'></div>";
      html += "</div>";
      html += "</div>";
    }
  }

  html += "<div class='box'>";
  html += "<a class='btn gray' href='/'>Volver al inicio</a>";
  html += "<a class='btn gray' href='/porcentaje'>Ir a porcentaje manual</a>";
  html += "</div>";

  html += "</body></html>";
  server.send(200, "text/html", html);
}


// -------------------------------------------------
// Iniciar automatico
// -------------------------------------------------
void handleAutoStart() {
  if (!server.hasArg("sec")) {
    server.send(400, "text/plain", "Falta parametro sec");
    return;
  }

  int sec = server.arg("sec").toInt();
  if (!isValidAutoIntervalSeconds(sec)) {
    server.send(400, "text/plain", "Intervalo invalido. Usa 60, 120, 300 o 86400");
    return;
  }

  bool useSleep = server.hasArg("sleep") && server.arg("sleep").toInt() == 1;
  if (useSleep) {
    setAutomaticModeSecondsWithDeepSleep(sec);
  } else {
    setAutomaticModeSeconds(sec);
  }

  server.sendHeader("Location", "/auto");
  server.send(302, "text/plain", "");
}

// -------------------------------------------------
// Parar automatico
// -------------------------------------------------
void handleAutoStop() {
  stopAutomaticMode();
  server.sendHeader("Location", "/auto");
  server.send(302, "text/plain", "");
}

// -------------------------------------------------
// Analisis manual
// -------------------------------------------------
void handleAnalizar() {
  if (!performAnalysis(true)) {
    server.send(500, "text/plain", "ERROR realizando analisis");
    return;
  }

  server.sendHeader("Location", "/porcentaje");
  server.send(302, "text/plain", "");
}


// -------------------------------------------------
// Ultima foto
// -------------------------------------------------
void handleUltimaFoto() {
  if (lastJpeg == nullptr || lastJpegLen == 0) {
    server.send(404, "text/plain", "No hay foto guardada");
    return;
  }

  WiFiClient client = server.client();
  server.setContentLength(lastJpegLen);
  server.send(200, "image/jpeg", "");
  client.write(lastJpeg, lastJpegLen);
}

void writeBmpHeader(WiFiClient& client, uint32_t imageSize, uint32_t fileSize) {
  uint8_t header[54];
  memset(header, 0, sizeof(header));

  header[0] = 'B';
  header[1] = 'M';

  header[2] = (uint8_t)(fileSize);
  header[3] = (uint8_t)(fileSize >> 8);
  header[4] = (uint8_t)(fileSize >> 16);
  header[5] = (uint8_t)(fileSize >> 24);

  header[10] = 54;
  header[14] = 40;

  uint32_t width = ANALYSIS_W;
  uint32_t height = ANALYSIS_H;

  header[18] = (uint8_t)(width);
  header[19] = (uint8_t)(width >> 8);
  header[20] = (uint8_t)(width >> 16);
  header[21] = (uint8_t)(width >> 24);

  header[22] = (uint8_t)(height);
  header[23] = (uint8_t)(height >> 8);
  header[24] = (uint8_t)(height >> 16);
  header[25] = (uint8_t)(height >> 24);

  header[26] = 1;
  header[28] = 24;

  header[34] = (uint8_t)(imageSize);
  header[35] = (uint8_t)(imageSize >> 8);
  header[36] = (uint8_t)(imageSize >> 16);
  header[37] = (uint8_t)(imageSize >> 24);

  client.write(header, sizeof(header));
}

bool prepareRgbForDetectionView() {
  if (!lastDetectionMaskValid || lastDetectionMask == nullptr) {
    server.send(404, "text/plain", "No hay mascara visual generada");
    return false;
  }

  if (lastJpeg == nullptr || lastJpegLen == 0) {
    server.send(404, "text/plain", "No hay ultima foto");
    return false;
  }

  if (!decodeJpegToRGB(lastJpeg, lastJpegLen)) {
    server.send(500, "text/plain", "Error decodificando ultima foto");
    return false;
  }

  return true;
}

void streamDetectionBmp(bool overlayMode) {
  if (!prepareRgbForDetectionView()) return;

  const uint32_t rowSize = ((uint32_t)ANALYSIS_W * 3U + 3U) & ~3U;
  const uint32_t imageSize = rowSize * (uint32_t)ANALYSIS_H;
  const uint32_t fileSize = 54U + imageSize;

  uint8_t* rowBuffer = (uint8_t*)malloc(rowSize);
  if (rowBuffer == nullptr) {
    server.send(500, "text/plain", "Error memoria fila BMP");
    return;
  }

  WiFiClient client = server.client();
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
  server.setContentLength(fileSize);
  server.send(200, "image/bmp", "");

  writeBmpHeader(client, imageSize, fileSize);

  for (int y = ANALYSIS_H - 1; y >= 0; y--) {
    memset(rowBuffer, 0, rowSize);

    for (int x = 0; x < ANALYSIS_W; x++) {
      size_t i = (size_t)y * (size_t)ANALYSIS_W + (size_t)x;
      size_t rgbIdx = i * 3U;
      size_t bmpIdx = (size_t)x * 3U;

      bool isVeg = (lastDetectionMask[i] != 0);

      if (overlayMode) {
        uint8_t r = rgbBuf[rgbIdx + 0];
        uint8_t g = rgbBuf[rgbIdx + 1];
        uint8_t b = rgbBuf[rgbIdx + 2];

        if (isVeg) {
          // Superposicion roja intensa: se mantiene parte de la imagen original
          // y se resaltan exactamente los pixeles contados como vegetacion.
          r = 255;
          g = g / 3;
          b = b / 3;
        }

        rowBuffer[bmpIdx + 0] = b;
        rowBuffer[bmpIdx + 1] = g;
        rowBuffer[bmpIdx + 2] = r;
      } else {
        if (isVeg) {
          // Mascara binaria: vegetacion en blanco, resto en negro.
          rowBuffer[bmpIdx + 0] = 255;
          rowBuffer[bmpIdx + 1] = 255;
          rowBuffer[bmpIdx + 2] = 255;
        } else {
          rowBuffer[bmpIdx + 0] = 0;
          rowBuffer[bmpIdx + 1] = 0;
          rowBuffer[bmpIdx + 2] = 0;
        }
      }
    }

    client.write(rowBuffer, rowSize);
  }

  free(rowBuffer);
}

void handleDetectionOverlay() {
  streamDetectionBmp(true);
}

void handleDetectionMask() {
  streamDetectionBmp(false);
}

bool prepareRgbForAlgorithmPreview() {
  if (!algorithmPreviewMaskValid || algorithmPreviewMask == nullptr) {
    server.send(404, "text/plain", "No hay superposicion de ajuste generada");
    return false;
  }
  if (algorithmPreviewJpeg == nullptr || algorithmPreviewJpegLen == 0) {
    server.send(404, "text/plain", "No hay imagen base de ajuste");
    return false;
  }
  if (!decodeJpegToRGB(algorithmPreviewJpeg, algorithmPreviewJpegLen)) {
    server.send(500, "text/plain", "Error decodificando la imagen de ajuste");
    return false;
  }
  return true;
}

void handleAlgorithmPreviewJpeg() {
  if (algorithmPreviewOverlayJpeg == nullptr ||
      algorithmPreviewOverlayJpegLen == 0 ||
      !algorithmPreviewMaskValid) {
    server.send(404, "text/plain", "No hay previsualizacion JPEG disponible");
    return;
  }

  WiFiClient client = server.client();
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
  server.sendHeader("Pragma", "no-cache");
  server.setContentLength(algorithmPreviewOverlayJpegLen);
  server.send(200, "image/jpeg", "");
  client.write(algorithmPreviewOverlayJpeg, algorithmPreviewOverlayJpegLen);
}

void handleAlgorithmPreviewBmp() {
  if (!prepareRgbForAlgorithmPreview()) return;

  const uint32_t rowSize = ((uint32_t)ANALYSIS_W * 3U + 3U) & ~3U;
  const uint32_t imageSize = rowSize * (uint32_t)ANALYSIS_H;
  const uint32_t fileSize = 54U + imageSize;
  uint8_t* rowBuffer = (uint8_t*)malloc(rowSize);
  if (rowBuffer == nullptr) {
    server.send(500, "text/plain", "Error memoria fila BMP");
    return;
  }

  WiFiClient client = server.client();
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
  server.setContentLength(fileSize);
  server.send(200, "image/bmp", "");
  writeBmpHeader(client, imageSize, fileSize);

  for (int y = ANALYSIS_H - 1; y >= 0; y--) {
    memset(rowBuffer, 0, rowSize);
    for (int x = 0; x < ANALYSIS_W; x++) {
      size_t i = (size_t)y * (size_t)ANALYSIS_W + (size_t)x;
      size_t rgbIdx = i * 3U;
      size_t bmpIdx = (size_t)x * 3U;
      uint8_t r = rgbBuf[rgbIdx + 0U];
      uint8_t g = rgbBuf[rgbIdx + 1U];
      uint8_t b = rgbBuf[rgbIdx + 2U];

      if (algorithmPreviewMask[i] != 0) {
        r = 255;
        g = g / 3;
        b = b / 3;
      }

      rowBuffer[bmpIdx + 0U] = b;
      rowBuffer[bmpIdx + 1U] = g;
      rowBuffer[bmpIdx + 2U] = r;
    }
    client.write(rowBuffer, rowSize);
  }

  free(rowBuffer);
}

// -------------------------------------------------
// Ajuste umbral
// -------------------------------------------------

// -------------------------------------------------
// Configuracion Wi-Fi persistente y aprovisionamiento BLE
// -------------------------------------------------
bool loadWifiCredentialsFromNvs() {
  memset(wifiSsid, 0, sizeof(wifiSsid));
  memset(wifiPassword, 0, sizeof(wifiPassword));

  Preferences preferences;
  if (!preferences.begin(WIFI_PREFS_NAMESPACE, true)) {
    Serial.println("WIFI_NVS) No existen credenciales Wi-Fi guardadas");
    return false;
  }

  bool valid = preferences.getBool("valid", false);
  size_t ssidLength = preferences.getString("ssid", wifiSsid, sizeof(wifiSsid));
  preferences.getString("pass", wifiPassword, sizeof(wifiPassword));
  preferences.end();

  bool valuesValid = valid &&
                     ssidLength > 0 &&
                     wifiSsid[0] != '\0' &&
                     strlen(wifiSsid) <= 32U &&
                     strlen(wifiPassword) <= 64U;

  if (!valuesValid) {
    memset(wifiSsid, 0, sizeof(wifiSsid));
    memset(wifiPassword, 0, sizeof(wifiPassword));
    Serial.println("WIFI_NVS) No hay una configuracion Wi-Fi valida");
    return false;
  }

  Serial.printf("WIFI_NVS) Credenciales cargadas para la red: %s\n", wifiSsid);
  return true;
}

bool saveWifiCredentialsToNvs(const char* newSsid, const char* newPassword) {
  if (newSsid == nullptr || newPassword == nullptr) return false;

  size_t ssidLength = strlen(newSsid);
  size_t passwordLength = strlen(newPassword);
  if (ssidLength == 0U || ssidLength > 32U || passwordLength > 64U) {
    Serial.println("WIFI_NVS) Credenciales recibidas con longitud no valida");
    return false;
  }

  Preferences preferences;
  if (!preferences.begin(WIFI_PREFS_NAMESPACE, false)) {
    Serial.println("WIFI_NVS) No se pudo abrir el espacio de preferencias Wi-Fi");
    return false;
  }

  bool ok = true;
  ok = ok && (preferences.putString("ssid", newSsid) > 0U);
  ok = ok && (preferences.putString("pass", newPassword) > 0U);
  ok = ok && (preferences.putBool("valid", true) > 0U);
  ok = ok && (preferences.putBool("provision", false) > 0U);
  preferences.end();

  if (ok) {
    Serial.printf("WIFI_NVS) Nueva red validada y guardada: %s\n", newSsid);
  } else {
    Serial.println("WIFI_NVS) Error guardando las nuevas credenciales");
  }
  return ok;
}

bool isWifiProvisioningRequested() {
  Preferences preferences;
  if (!preferences.begin(WIFI_PREFS_NAMESPACE, true)) {
    return false;
  }
  bool requested = preferences.getBool("provision", false);
  preferences.end();
  return requested;
}

bool setWifiProvisioningRequested(bool requested) {
  Preferences preferences;
  if (!preferences.begin(WIFI_PREFS_NAMESPACE, false)) {
    Serial.println("WIFI_NVS) No se pudo guardar la solicitud de aprovisionamiento");
    return false;
  }
  bool ok = preferences.putBool("provision", requested) > 0U;
  preferences.end();
  return ok;
}

void releaseBluetoothMemoryForNormalMode() {
#if WIFI_PROV_CAN_RELEASE_BT_MEMORY
  esp_err_t result = esp_bt_mem_release(ESP_BT_MODE_BTDM);
  if (result == ESP_OK) {
    Serial.println("WIFI_PROV) Memoria Bluetooth liberada para el funcionamiento normal");
  } else if (result == ESP_ERR_NOT_FOUND) {
    Serial.println("WIFI_PROV) No habia memoria Bluetooth reservada que liberar");
  } else {
    Serial.printf("WIFI_PROV) Aviso: esp_bt_mem_release devolvio %d\n", (int)result);
  }
#else
  Serial.println("WIFI_PROV) La compilacion no expone esp_bt.h; no se libera memoria Bluetooth");
#endif
}

void handleWifiProvisioningEvent(arduino_event_t* event) {
  if (event == nullptr) return;

  switch (event->event_id) {
    case ARDUINO_EVENT_PROV_START:
      Serial.println("WIFI_PROV) Aprovisionamiento BLE iniciado");
      Serial.printf("WIFI_PROV) Dispositivo: %s\n", WIFI_PROV_SERVICE_NAME);
      Serial.printf("WIFI_PROV) PIN/PoP: %s\n", WIFI_PROV_POP);
      break;

    case ARDUINO_EVENT_PROV_CRED_RECV: {
      const char* receivedSsid =
          (const char*)event->event_info.prov_cred_recv.ssid;
      const char* receivedPassword =
          (const char*)event->event_info.prov_cred_recv.password;

      portENTER_CRITICAL(&wifiProvisioningMux);
      memset(pendingWifiSsid, 0, sizeof(pendingWifiSsid));
      memset(pendingWifiPassword, 0, sizeof(pendingWifiPassword));

      if (receivedSsid != nullptr) {
        strncpy(pendingWifiSsid, receivedSsid, sizeof(pendingWifiSsid) - 1U);
      }
      if (receivedPassword != nullptr) {
        strncpy(pendingWifiPassword, receivedPassword,
                sizeof(pendingWifiPassword) - 1U);
      }

      wifiProvisioningCredentialsReceived = pendingWifiSsid[0] != '\0';
      wifiProvisioningCredentialSuccess = false;
      wifiProvisioningCredentialFailed = false;
      wifiProvisioningGotIp = false;
      wifiProvisioningSaveAttempted = false;
      portEXIT_CRITICAL(&wifiProvisioningMux);

      Serial.printf("WIFI_PROV) Credenciales recibidas para la red: %s\n",
                    pendingWifiSsid);
      Serial.println("WIFI_PROV) La contrasena no se muestra por seguridad");
      break;
    }

    case ARDUINO_EVENT_PROV_CRED_FAIL:
      portENTER_CRITICAL(&wifiProvisioningMux);
      wifiProvisioningCredentialFailed = true;
      wifiProvisioningCredentialSuccess = false;
      wifiProvisioningGotIp = false;
      wifiProvisioningCredentialsReceived = false;
      wifiProvisioningSaveAttempted = false;
      memset(pendingWifiSsid, 0, sizeof(pendingWifiSsid));
      memset(pendingWifiPassword, 0, sizeof(pendingWifiPassword));
      portEXIT_CRITICAL(&wifiProvisioningMux);

      if (event->event_info.prov_fail_reason == WIFI_PROV_AUTH_ERROR_REASON) {
        Serial.println("WIFI_PROV) Error: contrasena Wi-Fi incorrecta");
      } else {
        Serial.println("WIFI_PROV) Error: no se pudo encontrar o conectar con la red");
      }
      Serial.println("WIFI_PROV) Puede volver a intentarlo desde ESP BLE Provisioning");
      break;

    case ARDUINO_EVENT_PROV_CRED_SUCCESS:
      portENTER_CRITICAL(&wifiProvisioningMux);
      wifiProvisioningCredentialSuccess = true;
      wifiProvisioningCredentialFailed = false;
      portEXIT_CRITICAL(&wifiProvisioningMux);
      Serial.println("WIFI_PROV) Credenciales aceptadas por el punto de acceso");
      break;

    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      if (wifiProvisioningMode) {
        portENTER_CRITICAL(&wifiProvisioningMux);
        wifiProvisioningGotIp = true;
        portEXIT_CRITICAL(&wifiProvisioningMux);
        Serial.print("WIFI_PROV) Direccion IP obtenida: ");
        Serial.println(WiFi.localIP());
      }
      break;

    case ARDUINO_EVENT_PROV_END:
      Serial.println("WIFI_PROV) Proceso BLE finalizado");
      break;

    default:
      break;
  }
}

void startWifiProvisioningMode() {
  wifiProvisioningMode = true;
  wifiProvisioningCredentialsReceived = false;
  wifiProvisioningCredentialSuccess = false;
  wifiProvisioningCredentialFailed = false;
  wifiProvisioningGotIp = false;
  wifiProvisioningSaveAttempted = false;
  wifiProvisioningRestartPending = false;
  memset(pendingWifiSsid, 0, sizeof(pendingWifiSsid));
  memset(pendingWifiPassword, 0, sizeof(pendingWifiPassword));

  Serial.println("\n=== MODO EXCLUSIVO DE CONFIGURACION WI-FI ===");
  Serial.println("WIFI_PROV) La camara, la web normal y Thinger.io no se iniciaran en este modo");
  Serial.println("WIFI_PROV) Abra la aplicacion ESP BLE Provisioning");
  Serial.printf("WIFI_PROV) Seleccione: %s\n", WIFI_PROV_SERVICE_NAME);
  Serial.printf("WIFI_PROV) Introduzca el PIN/PoP: %s\n", WIFI_PROV_POP);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.onEvent(handleWifiProvisioningEvent);

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  WiFiProv.beginProvision(
      NETWORK_PROV_SCHEME_BLE,
      NETWORK_PROV_SCHEME_HANDLER_FREE_BTDM,
      NETWORK_PROV_SECURITY_1,
      WIFI_PROV_POP,
      WIFI_PROV_SERVICE_NAME,
      WIFI_PROV_SERVICE_KEY,
      wifiProvisioningServiceUuid,
      true);
#else
  WiFiProv.beginProvision(
      WIFI_PROV_SCHEME_BLE,
      WIFI_PROV_SCHEME_HANDLER_FREE_BTDM,
      WIFI_PROV_SECURITY_1,
      WIFI_PROV_POP,
      WIFI_PROV_SERVICE_NAME,
      WIFI_PROV_SERVICE_KEY,
      wifiProvisioningServiceUuid,
      true);
#endif

  WiFiProv.printQR(WIFI_PROV_SERVICE_NAME, WIFI_PROV_POP, "ble");
}

void handleWifiProvisioningMode() {
  char validatedSsid[33] = {0};
  char validatedPassword[65] = {0};
  bool credentialsReady = false;

  portENTER_CRITICAL(&wifiProvisioningMux);
  if (!wifiProvisioningSaveAttempted &&
      wifiProvisioningCredentialsReceived &&
      wifiProvisioningCredentialSuccess &&
      wifiProvisioningGotIp) {
    wifiProvisioningSaveAttempted = true;
    strncpy(validatedSsid, pendingWifiSsid, sizeof(validatedSsid) - 1U);
    strncpy(validatedPassword, pendingWifiPassword,
            sizeof(validatedPassword) - 1U);
    credentialsReady = true;
  }
  portEXIT_CRITICAL(&wifiProvisioningMux);

  if (credentialsReady) {
    if (saveWifiCredentialsToNvs(validatedSsid, validatedPassword)) {
      wifiProvisioningRestartPending = true;
      wifiProvisioningRestartMs = millis();
      Serial.println("WIFI_PROV) Configuracion guardada correctamente");
      Serial.println("WIFI_PROV) Reiniciando para recuperar web, camara y Thinger.io...");
    } else {
      Serial.println("WIFI_PROV) No se reinicia porque no se pudieron guardar las credenciales");
      Serial.println("WIFI_PROV) Envie de nuevo la configuracion desde la aplicacion");
    }
  }

  if (wifiProvisioningRestartPending &&
      millis() - wifiProvisioningRestartMs >= WIFI_PROVISIONING_RESTART_DELAY_MS) {
    ESP.restart();
  }
}

String escapeHtmlText(const String& text) {
  String escaped;
  escaped.reserve(text.length() + 16U);
  for (size_t i = 0; i < text.length(); i++) {
    char c = text.charAt(i);
    switch (c) {
      case '&': escaped += "&amp;"; break;
      case '<': escaped += "&lt;"; break;
      case '>': escaped += "&gt;"; break;
      case '\"': escaped += "&quot;"; break;
      case '\'': escaped += "&#39;"; break;
      default: escaped += c; break;
    }
  }
  return escaped;
}

void handleWifiConfig() {
  bool connected = WiFi.status() == WL_CONNECTED;
  String currentNetwork = connected ? WiFi.SSID() : String(wifiSsid);
  if (currentNetwork.length() == 0) currentNetwork = "No disponible";
  String currentIp = connected ? WiFi.localIP().toString() : "No disponible";

  String html = "<!doctype html><html><head><meta charset='utf-8'><title>Configurar Wi-Fi</title>";
  html += commonCSS();
  html += "</head><body>";
  html += "<h1>Configurar Wi-Fi</h1>";

  html += "<div class='box'>";
  html += "<h2>Estado actual</h2>";
  html += "<p><b>Estado Wi-Fi:</b> <span class='";
  html += connected ? "ok'>CONECTADO" : "no'>DESCONECTADO";
  html += "</span></p>";
  html += "<p><b>Red actual:</b> " + escapeHtmlText(currentNetwork) + "</p>";
  html += "<p><b>Direccion IP:</b> " + escapeHtmlText(currentIp) + "</p>";
  html += "<p><b>Nombre Bluetooth:</b> " + String(WIFI_PROV_SERVICE_NAME) + "</p>";
  html += "<p><b>PIN de aprovisionamiento:</b> " + String(WIFI_PROV_POP) + "</p>";
  html += "<p class='mini'>La contrasena Wi-Fi nunca se muestra en esta pagina.</p>";
  html += "</div>";

  html += "<div class='box'>";
  html += "<h2>Como configurar una nueva red Wi-Fi</h2>";
  html += "<ol>";
  html += "<li>Instala en el telefono la aplicacion <b>ESP BLE Provisioning</b>, desarrollada por Espressif.</li>";
  html += "<li>Activa el Bluetooth del telefono.</li>";
  html += "<li>Concede los permisos de Bluetooth, dispositivos cercanos o ubicacion que solicite el sistema operativo.</li>";
  html += "<li>Pulsa al final de esta pagina <b>Iniciar nueva configuracion Wi-Fi</b>.</li>";
  html += "<li>La TimerCamera-F se reiniciara y esta pagina dejara temporalmente de responder.</li>";
  html += "<li>Abre la aplicacion <b>ESP BLE Provisioning</b>.</li>";
  html += "<li>Pulsa la opcion para buscar, escanear o aprovisionar un nuevo dispositivo.</li>";
  html += "<li>Selecciona el dispositivo <b>" + String(WIFI_PROV_SERVICE_NAME) + "</b>.</li>";
  html += "<li>Cuando solicite el Proof of Possession o PIN, introduce <b>" + String(WIFI_PROV_POP) + "</b>.</li>";
  html += "<li>Selecciona la nueva red Wi-Fi. El ESP32 debe conectarse a una red de 2,4 GHz.</li>";
  html += "<li>Introduce la contrasena de la red y confirma el aprovisionamiento.</li>";
  html += "<li>Espera hasta que la aplicacion indique que la conexion ha finalizado correctamente.</li>";
  html += "<li>La TimerCamera-F guardara las credenciales y se reiniciara automaticamente.</li>";
  html += "<li>Tras el reinicio se recuperaran la web local, la camara, el analisis, la ROI, la configuracion del algoritmo, el modo automatico y Thinger.io.</li>";
  html += "</ol>";
  html += "</div>";

  html += "<div class='box'>";
  html += "<p class='no'>Al pulsar el boton, el dispositivo se reiniciara y esta pagina dejara de responder. Continua el proceso desde la aplicacion ESP BLE Provisioning.</p>";
  html += "<a class='btn red' href='/wifi_start'>Iniciar nueva configuracion Wi-Fi</a>";
  html += "<a class='btn gray' href='/'>Volver</a>";
  html += "</div>";
  html += "</body></html>";

  server.send(200, "text/html", html);
}

void handleWifiStart() {
  if (!setWifiProvisioningRequested(true)) {
    server.send(500, "text/plain",
                "No se pudo guardar la solicitud de configuracion Wi-Fi");
    return;
  }

  String html = "<!doctype html><html><head><meta charset='utf-8'><title>Reinicio Wi-Fi</title>";
  html += commonCSS();
  html += "</head><body><h1>Configuracion Wi-Fi iniciada</h1>";
  html += "<div class='box'>";
  html += "<p>La TimerCamera-F va a reiniciarse en modo exclusivo Bluetooth.</p>";
  html += "<p>Abre ahora <b>ESP BLE Provisioning</b>, selecciona <b>";
  html += WIFI_PROV_SERVICE_NAME;
  html += "</b> e introduce el PIN <b>";
  html += WIFI_PROV_POP;
  html += "</b>.</p>";
  html += "<p>Esta pagina dejara de responder durante el proceso.</p>";
  html += "</div></body></html>";
  server.send(200, "text/html", html);

  Serial.println("WIFI_PROV) Cambio de red solicitado desde la web local");
  requestSystemAction(SYSTEM_ACTION_RESTART);
}

// -------------------------------------------------
// SETUP
// -------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(2000);

  consoleInputBuffer.reserve(128);

  // Mantiene alimentada la TimerCamera-F cuando funciona con bateria.
  pinMode(POWER_HOLD_PIN, OUTPUT);
  digitalWrite(POWER_HOLD_PIN, HIGH);

  // LED de estado apagado para no contaminar las imagenes.
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  bool wifiCredentialsAvailable = loadWifiCredentialsFromNvs();
  if (isWifiProvisioningRequested() || !wifiCredentialsAvailable) {
    startWifiProvisioningMode();
    return;
  }

  // En este ciclo no se usara Bluetooth. Se libera su memoria antes de
  // iniciar TLS de Thinger.io, la camara y los buffers de imagen.
  // Un reinicio posterior recupera Bluetooth para volver a aprovisionar.
  releaseBluetoothMemoryForNormalMode();

  wokeFromDeepSleep = isWakeupFromDeepSleep();
  if (rtcHasValidConfig()) {
    restoreRuntimeConfigFromRtc();
  }

  // En un arranque normal se cargan los valores guardados por defecto en NVS.
  // En un despertar desde Deep Sleep prevalece la configuracion temporal RTC.
  if (!wokeFromDeepSleep) {
    loadAlgorithmDefaultsFromNvs();
    saveRuntimeConfigToRtc();
  }

  if (wokeFromDeepSleep && autoDeepSleepEnabled) {
    wakeCount++;
    rtcWakeCount = wakeCount;
    autoDeepSleepCaptureDone = false;
    awakeWindowStartMs = 0;
    saveRuntimeConfigToRtc();
    Serial.println("RTC) Despertar desde Deep Sleep automatico. Configuracion restaurada.");
  }

  initBatteryMonitor();
  updateBatteryStatus(true);

  Serial.println("\n=== INICIO M5STACK TIMERCAMERA-F ===");
  Serial.printf("PSRAM detectada: %s\n", psramFound() ? "SI" : "NO");
  if (psramFound()) {
    Serial.printf("PSRAM total: %u bytes\n", ESP.getPsramSize());
    Serial.printf("PSRAM libre: %u bytes\n", ESP.getFreePsram());
  }

  Serial.printf("Resolucion activa: %d x %d (VGA)\n", ANALYSIS_W, ANALYSIS_H);
  Serial.printf("Pixeles por imagen: %u\n", (unsigned int)FRAME_PIXELS);
  Serial.printf("Buffer RGB888 necesario: %u bytes\n", (unsigned int)(FRAME_PIXELS * 3U));
  Serial.printf("ROI inicial: %s, %d x %d px, %u pixeles\n",
                roiEnabled ? "ACTIVA" : "IMAGEN COMPLETA",
                getRoiWidthPx(), getRoiHeightPx(),
                (unsigned int)getRoiTotalPx());
  Serial.printf("Algoritmo inicial: %s (%d)\n", detectionModeName().c_str(), detectionMode);
  Serial.printf("Umbral ExG inicial: %.3f\n", exgThreshold);
  Serial.printf("HSV inicial: H=%d..%d, Smin=%d, Vmin=%d\n", hsvHMin, hsvHMax, hsvSMin, hsvVMin);
  Serial.printf("Bateria inicial: %.2f V (%d %%)\n", lastBatteryVoltage, lastBatteryPercent);
  Serial.printf("Tiempo activo inicial: %s\n", getUptimeFormatted().c_str());
  Serial.printf("Wakeup desde Deep Sleep: %s\n", wokeFromDeepSleep ? "SI" : "NO");
  Serial.printf("Deep Sleep automatico inicial: %s\n", autoDeepSleepEnabled ? "ACTIVO" : "PARADO");
  Serial.printf("Ventana despierta inicial: %lu s\n", awakeWindowSeconds);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(wifiSsid, wifiPassword);

  Serial.print("WiFi conectando");
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("ERROR WiFi");
    return;
  }

  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  thing.add_wifi(wifiSsid, wifiPassword);

  thing["veg_percent"] >> outputValue(lastVegPercent);
  thing["vegetation_px"] >> outputValue(lastVegetationPx);
  thing["vegetation_pixels"] >> outputValue(lastVegetationPx);
  thing["roi_px"] >> outputValue(lastRoiPx);
  thing["auto_mode"] >> outputValue(autoModeEnabled);
  thing["auto_interval_sec"] >> [](pson& out) { out = (unsigned long)(autoIntervalMs / 1000UL); };
  thing["auto_interval"] >> [](pson& out) { out = (unsigned long)(autoIntervalMs / 1000UL); };
  thing["auto_count"] >> [](pson& out) { out = (unsigned long)autoCaptureCount; };
  thing["auto_capture_count"] >> [](pson& out) { out = (unsigned long)autoCaptureCount; };
  thing["auto_deep_sleep_enabled"] >> [](pson& out) { out = autoDeepSleepEnabled; };
  thing["awake_window_seconds"] >> [](pson& out) { out = (unsigned long)awakeWindowSeconds; };
  thing["wake_count"] >> [](pson& out) { out = (unsigned long)wakeCount; };
  thing["total_awake_seconds"] >> [](pson& out) { out = getTotalAwakeSeconds(); };
  thing["total_awake_minutes"] >> [](pson& out) { out = getTotalAwakeMinutes(); };
  thing["total_awake_formatted"] >> [](pson& out) { String t = getTotalAwakeFormatted(); out = t.c_str(); };

  thing["roi_enabled"] >> [](pson& out) { out = roiEnabled; };
  thing["roi_left_percent"] >> [](pson& out) { out = roiLeftPercent; };
  thing["roi_right_percent"] >> [](pson& out) { out = roiRightPercent; };
  thing["roi_top_percent"] >> [](pson& out) { out = roiTopPercent; };
  thing["roi_bottom_percent"] >> [](pson& out) { out = roiBottomPercent; };
  thing["roi_width_px"] >> [](pson& out) { out = getRoiWidthPx(); };
  thing["roi_height_px"] >> [](pson& out) { out = getRoiHeightPx(); };
  thing["roi_total_px"] >> [](pson& out) { out = (unsigned long)getRoiTotalPx(); };

  thing["detection_mode"] = [](pson& in, pson& out) {
    if (in.is_empty()) {
      out = detectionMode;
    } else {
      int requestedMode = (int)in;
      setDetectionModeValue(requestedMode, true);
      out = detectionMode;
    }
  };
  thing["detection_mode_name"] >> [](pson& out) {
    String modeText = detectionModeName();
    out = modeText.c_str();
  };
  thing["exg_threshold"] = [](pson& in, pson& out) {
    if (in.is_empty()) {
      out = exgThreshold;
    } else {
      float newValue = (float)in;
      setExgThresholdValue(newValue, true);
      out = exgThreshold;
    }
  };
  thing["hsv_h_min"] = [](pson& in, pson& out) {
    if (in.is_empty()) { out = hsvHMin; }
    else { setHsvConfigValue((int)in, hsvHMax, hsvSMin, hsvVMin, true); out = hsvHMin; }
  };
  thing["hsv_h_max"] = [](pson& in, pson& out) {
    if (in.is_empty()) { out = hsvHMax; }
    else { setHsvConfigValue(hsvHMin, (int)in, hsvSMin, hsvVMin, true); out = hsvHMax; }
  };
  thing["hsv_s_min"] = [](pson& in, pson& out) {
    if (in.is_empty()) { out = hsvSMin; }
    else { setHsvConfigValue(hsvHMin, hsvHMax, (int)in, hsvVMin, true); out = hsvSMin; }
  };
  thing["hsv_v_min"] = [](pson& in, pson& out) {
    if (in.is_empty()) { out = hsvVMin; }
    else { setHsvConfigValue(hsvHMin, hsvHMax, hsvSMin, (int)in, true); out = hsvVMin; }
  };

  // Estado de la previsualizacion visual del algoritmo. La URL es local y
  // puede usarse en un widget Image/MJPEG cuando el navegador esta en la
  // misma red que la TimerCamera-F. Para acceso externo sigue siendo necesario
  // publicar/subir la imagen a un almacenamiento accesible desde Internet.
  thing["algorithm_preview_available"] >> [](pson& out) {
    out = algorithmPreviewMaskValid &&
          algorithmPreviewOverlayJpeg != nullptr &&
          algorithmPreviewOverlayJpegLen > 0;
  };
  thing["algorithm_preview_version"] >> [](pson& out) { out = (unsigned long)algorithmPreviewVersion; };
  thing["algorithm_preview_percent"] >> [](pson& out) { out = algorithmPreviewVegPercent; };
  thing["algorithm_preview_state"] >> [](pson& out) { out = algorithmPreviewStateName(); };
  thing["algorithm_overlay_url"] >> [](pson& out) {
    String url = String("http://") + WiFi.localIP().toString() + "/algorithm_preview.jpg";
    out = url.c_str();
  };

  // ====== Recursos de control para widgets de Thinger.io ======
  thing["control_detection_mode"] = [](pson& in, pson& out) {
    if (!in.is_empty()) {
      int requestedMode = (int)in;
      setDetectionModeValue(requestedMode, true);
    }
    out = detectionMode;
  };

  thing["control_exg_threshold"] = [](pson& in, pson& out) {
    if (!in.is_empty()) {
      float rawValue = (float)in;
      float newValue = rawValue;
      if (newValue > 1.0f) {
        newValue = newValue / 100.0f;
      }
      Serial.printf("THINGER) control_exg_threshold recibido: %.4f, aplicado: %.4f\n", rawValue, newValue);
      if (newValue >= -0.05f && newValue <= 0.20f) {
        setExgThresholdValue(newValue, true);
      } else {
        Serial.println("THINGER) control_exg_threshold fuera de rango, se ignora");
      }
    }
    out = exgThreshold;
  };

  thing["control_hsv_h_min"] = [](pson& in, pson& out) {
    if (!in.is_empty()) {
      setHsvConfigValue((int)in, hsvHMax, hsvSMin, hsvVMin, true);
    }
    out = hsvHMin;
  };

  thing["control_hsv_h_max"] = [](pson& in, pson& out) {
    if (!in.is_empty()) {
      setHsvConfigValue(hsvHMin, (int)in, hsvSMin, hsvVMin, true);
    }
    out = hsvHMax;
  };

  thing["control_hsv_s_min"] = [](pson& in, pson& out) {
    if (!in.is_empty()) {
      setHsvConfigValue(hsvHMin, hsvHMax, (int)in, hsvVMin, true);
    }
    out = hsvSMin;
  };

  thing["control_hsv_v_min"] = [](pson& in, pson& out) {
    if (!in.is_empty()) {
      setHsvConfigValue(hsvHMin, hsvHMax, hsvSMin, (int)in, true);
    }
    out = hsvVMin;
  };

  thing["control_capture_algorithm_preview"] = [](pson& in, pson& out) {
    if (!in.is_empty() && (int)in == 1 && !pendingAlgorithmPreviewCapture) {
      pendingAlgorithmPreviewCapture = true;
      algorithmPreviewRequestedVersion++;
      algorithmPreviewDirty = false;
      algorithmPreviewState = PREVIEW_CAPTURING;
      algorithmPreviewLastError = "";
    }
    out = pendingAlgorithmPreviewCapture;
  };

  thing["control_auto_mode"] = [](pson& in, pson& out) {
    if (!in.is_empty()) {
      int requested = (int)in;
      if (requested == 0) {
        stopAutomaticMode();
      } else if (requested == 1) {
        autoModeEnabled = true;
        autoDeepSleepEnabled = false;
        lastAutoCaptureMs = 0;
        autoDeepSleepCaptureDone = false;
        saveRuntimeConfigToRtc();
      }
    }
    out = autoModeEnabled;
  };

  thing["control_auto_interval"] = [](pson& in, pson& out) {
    if (!in.is_empty()) {
      int requestedInterval = (int)in;
      setAutoIntervalSecondsValue(requestedInterval, autoModeEnabled);
    }
    out = (unsigned long)(autoIntervalMs / 1000UL);
  };

  thing["control_auto_60"] = [](pson& in, pson& out) {
    if (!in.is_empty() && (int)in == 1) setAutomaticModeSeconds(60);
    out = (autoModeEnabled && !autoDeepSleepEnabled && autoIntervalMs == 60000UL);
  };

  thing["control_auto_120"] = [](pson& in, pson& out) {
    if (!in.is_empty() && (int)in == 1) setAutomaticModeSeconds(120);
    out = (autoModeEnabled && !autoDeepSleepEnabled && autoIntervalMs == 120000UL);
  };

  thing["control_auto_300"] = [](pson& in, pson& out) {
    if (!in.is_empty() && (int)in == 1) setAutomaticModeSeconds(300);
    out = (autoModeEnabled && !autoDeepSleepEnabled && autoIntervalMs == 300000UL);
  };

  thing["control_auto_24h"] = [](pson& in, pson& out) {
    if (!in.is_empty() && (int)in == 1) setAutomaticModeSeconds(86400);
    out = (autoModeEnabled && !autoDeepSleepEnabled && autoIntervalMs == 86400000UL);
  };

  thing["control_auto_60_sleep"] = [](pson& in, pson& out) {
    if (!in.is_empty() && (int)in == 1) setAutomaticModeSecondsWithDeepSleep(60);
    out = (autoModeEnabled && autoDeepSleepEnabled && autoIntervalMs == 60000UL);
  };

  thing["control_auto_120_sleep"] = [](pson& in, pson& out) {
    if (!in.is_empty() && (int)in == 1) setAutomaticModeSecondsWithDeepSleep(120);
    out = (autoModeEnabled && autoDeepSleepEnabled && autoIntervalMs == 120000UL);
  };

  thing["control_auto_300_sleep"] = [](pson& in, pson& out) {
    if (!in.is_empty() && (int)in == 1) setAutomaticModeSecondsWithDeepSleep(300);
    out = (autoModeEnabled && autoDeepSleepEnabled && autoIntervalMs == 300000UL);
  };

  thing["control_auto_24h_sleep"] = [](pson& in, pson& out) {
    if (!in.is_empty() && (int)in == 1) setAutomaticModeSecondsWithDeepSleep(86400);
    out = (autoModeEnabled && autoDeepSleepEnabled && autoIntervalMs == 86400000UL);
  };

  thing["control_auto_stop"] = [](pson& in, pson& out) {
    if (!in.is_empty() && (int)in == 1) {
      stopAutomaticMode();
    }
    out = !autoModeEnabled;
  };

  thing["control_auto_deep_sleep"] = [](pson& in, pson& out) {
    if (!in.is_empty()) {
      int requested = (int)in;
      if (requested == 0) {
        autoDeepSleepEnabled = false;
        autoDeepSleepCaptureDone = false;
        saveRuntimeConfigToRtc();
      } else if (requested == 1) {
        autoDeepSleepEnabled = true;
        autoDeepSleepCaptureDone = false;
        awakeWindowStartMs = 0;
        saveRuntimeConfigToRtc();
      }
    }
    out = autoDeepSleepEnabled;
  };

  thing["control_awake_window_seconds"] = [](pson& in, pson& out) {
    if (!in.is_empty()) {
      setAwakeWindowSecondsValue((unsigned long)((int)in));
    }
    out = (unsigned long)awakeWindowSeconds;
  };

  thing["control_capture_now"] = [](pson& in, pson& out) {
    if (!in.is_empty() && (int)in == 1) {
      pendingThingerCaptureNow = true;
    }
    out = pendingThingerCaptureNow;
  };

  thing["control_roi_enabled"] = [](pson& in, pson& out) {
    if (!in.is_empty()) {
      int requested = (int)in;
      if (requested == 0 || requested == 1) {
        setRoiConfigValue(requested == 1, roiLeftPercent, roiRightPercent, roiTopPercent, roiBottomPercent, true);
      }
    }
    out = roiEnabled;
  };

  thing["control_roi_left"] = [](pson& in, pson& out) {
    if (!in.is_empty()) {
      setRoiConfigValue(roiEnabled, (int)in, roiRightPercent, roiTopPercent, roiBottomPercent, true);
    }
    out = roiLeftPercent;
  };

  thing["control_roi_right"] = [](pson& in, pson& out) {
    if (!in.is_empty()) {
      setRoiConfigValue(roiEnabled, roiLeftPercent, (int)in, roiTopPercent, roiBottomPercent, true);
    }
    out = roiRightPercent;
  };

  thing["control_roi_top"] = [](pson& in, pson& out) {
    if (!in.is_empty()) {
      setRoiConfigValue(roiEnabled, roiLeftPercent, roiRightPercent, (int)in, roiBottomPercent, true);
    }
    out = roiTopPercent;
  };

  thing["control_roi_bottom"] = [](pson& in, pson& out) {
    if (!in.is_empty()) {
      setRoiConfigValue(roiEnabled, roiLeftPercent, roiRightPercent, roiTopPercent, (int)in, true);
    }
    out = roiBottomPercent;
  };

  thing["control_roi_full"] = [](pson& in, pson& out) {
    if (!in.is_empty() && (int)in == 1) {
      setRoiFullValue(true);
    }
    out = !roiEnabled;
  };

  thing["control_roi_center"] = [](pson& in, pson& out) {
    if (!in.is_empty() && (int)in == 1) {
      setRoiCenterValue(true);
    }
    out = (roiEnabled && roiLeftPercent == 20 && roiRightPercent == 20 && roiTopPercent == 20 && roiBottomPercent == 20);
  };

  thing["control_reboot"] = [](pson& in, pson& out) {
    if (!in.is_empty() && (int)in == 1) {
      Serial.println("THINGER) Reinicio solicitado desde dashboard");
      requestSystemAction(SYSTEM_ACTION_RESTART);
    }
    out = pendingSystemAction == SYSTEM_ACTION_RESTART;
  };

  thing["control_sleep"] = [](pson& in, pson& out) {
    if (!in.is_empty() && (int)in == 1) {
      Serial.println("THINGER) Deep Sleep solicitado desde dashboard");
      requestSystemAction(SYSTEM_ACTION_SHUTDOWN);
    }
    out = pendingSystemAction == SYSTEM_ACTION_SHUTDOWN;
  };

  thing["battery_voltage"] >> [](pson& out) { updateBatteryStatus(false); out = lastBatteryVoltage; };
  thing["battery_percent"] >> [](pson& out) { updateBatteryStatus(false); out = lastBatteryPercent; };
  thing["uptime_seconds"] >> [](pson& out) { out = getUptimeSeconds(); };
  thing["uptime_minutes"] >> [](pson& out) { out = getUptimeMinutes(); };
  thing["uptime_formatted"] >> [](pson& out) {
    String uptimeText = getUptimeFormatted();
    out = uptimeText.c_str();
  };

  thing["medicion"] >> [](pson& out) {
    out["veg_percent"] = lastVegPercent;
    out["vegetation_px"] = (unsigned long) lastVegetationPx;
    out["vegetation_pixels"] = (unsigned long) lastVegetationPx;
    out["roi_px"] = (unsigned long) lastRoiPx;
    out["auto_mode"] = autoModeEnabled;
    out["auto_interval_sec"] = (unsigned long)(autoIntervalMs / 1000UL);
    out["auto_interval"] = (unsigned long)(autoIntervalMs / 1000UL);
    out["auto_count"] = (unsigned long)autoCaptureCount;
    out["auto_capture_count"] = (unsigned long)autoCaptureCount;
    out["auto_deep_sleep_enabled"] = autoDeepSleepEnabled;
    out["awake_window_seconds"] = (unsigned long)awakeWindowSeconds;
    out["wake_count"] = (unsigned long)wakeCount;
    out["total_awake_seconds"] = getTotalAwakeSeconds();
    out["total_awake_minutes"] = getTotalAwakeMinutes();
    { String totalAwakeText = getTotalAwakeFormatted(); out["total_awake_formatted"] = totalAwakeText.c_str(); }
    out["roi_enabled"] = roiEnabled;
    out["roi_left_percent"] = roiLeftPercent;
    out["roi_right_percent"] = roiRightPercent;
    out["roi_top_percent"] = roiTopPercent;
    out["roi_bottom_percent"] = roiBottomPercent;
    out["roi_width_px"] = getRoiWidthPx();
    out["roi_height_px"] = getRoiHeightPx();
    out["roi_total_px"] = (unsigned long)getRoiTotalPx();
    out["detection_mode"] = detectionMode;
    String modeText = detectionModeName();
    out["detection_mode_name"] = modeText.c_str();
    out["exg_threshold"] = exgThreshold;
    out["hsv_h_min"] = hsvHMin;
    out["hsv_h_max"] = hsvHMax;
    out["hsv_s_min"] = hsvSMin;
    out["hsv_v_min"] = hsvVMin;

    updateBatteryStatus(false);
    String uptimeText = getUptimeFormatted();
    out["battery_voltage"] = lastBatteryVoltage;
    out["battery_percent"] = lastBatteryPercent;
    out["uptime_seconds"] = getUptimeSeconds();
    out["uptime_minutes"] = getUptimeMinutes();
    out["uptime_formatted"] = uptimeText.c_str();
  };

  if (!initCamera()) {
    Serial.println("ERROR camara");
    return;
  }

  server.on("/",                HTTP_GET, handleRoot);
  server.on("/porcentaje",      HTTP_GET, handlePorcentaje);
  server.on("/auto",            HTTP_GET, handleAuto);
  server.on("/auto_start",      HTTP_GET, handleAutoStart);
  server.on("/auto_stop",       HTTP_GET, handleAutoStop);
  server.on("/analizar",        HTTP_GET, handleAnalizar);
  server.on("/ultima_foto",     HTTP_GET, handleUltimaFoto);
  server.on("/detection_overlay.bmp", HTTP_GET, handleDetectionOverlay);
  server.on("/detection_mask.bmp",    HTTP_GET, handleDetectionMask);
  server.on("/roi",             HTTP_GET, handleRoiConfig);
  server.on("/set_roi",         HTTP_GET, handleSetRoi);
  server.on("/roi_capture",     HTTP_GET, handleRoiPreviewCapture);
  server.on("/roi_preview.jpg", HTTP_GET, handleRoiPreviewJpeg);
  server.on("/roi_visual_status", HTTP_GET, handleRoiVisualStatus);
  server.on("/roi_visual_apply",  HTTP_GET, handleRoiVisualApply);
  server.on("/algoritmo",       HTTP_GET, handleAlgorithmConfig);
  server.on("/set_algorithm",   HTTP_GET, handleSetAlgorithm);
  server.on("/algorithm_live_update", HTTP_GET, handleAlgorithmLiveUpdate);
  server.on("/algorithm_capture", HTTP_GET, handleAlgorithmPreviewCapture);
  server.on("/algorithm_status", HTTP_GET, handleAlgorithmStatus);
  server.on("/algorithm_preview.jpg", HTTP_GET, handleAlgorithmPreviewJpeg);
  server.on("/algorithm_preview.bmp", HTTP_GET, handleAlgorithmPreviewBmp);
  server.on("/save_algorithm_defaults", HTTP_GET, handleSaveAlgorithmDefaults);
  server.on("/roi_preset",      HTTP_GET, handleRoiPreset);
  server.on("/wifi",            HTTP_GET, handleWifiConfig);
  server.on("/wifi_start",      HTTP_GET, handleWifiStart);

  server.begin();

  Serial.print("Abre: http://");
  Serial.println(WiFi.localIP());
  Serial.println("=== FIN SETUP TIMERCAMERA-F ===");
}

// -------------------------------------------------
// LOOP
// -------------------------------------------------
void loop() {
  if (wifiProvisioningMode) {
    handleWifiProvisioningMode();
    delay(10);
    return;
  }

  thing.handle();
  server.handleClient();
  handleRemoteConsole();

  if (pendingRoiPreviewCapture &&
      !roiPreviewProcessing &&
      !algorithmPreviewProcessing) {
    pendingRoiPreviewCapture = false;
    captureRoiPreviewImage();
  }

  if (pendingAlgorithmPreviewCapture && !algorithmPreviewProcessing && !roiPreviewProcessing) {
    pendingAlgorithmPreviewCapture = false;
    captureAlgorithmPreviewImage();
  }

  if (algorithmPreviewDirty &&
      !algorithmPreviewProcessing &&
      millis() - algorithmPreviewDirtySinceMs >= ALGORITHM_PREVIEW_DEBOUNCE_MS) {
    const uint32_t targetVersion = algorithmPreviewRequestedVersion;
    regenerateAlgorithmPreview(targetVersion);
  }

  if (pendingThingerCaptureNow) {
    pendingThingerCaptureNow = false;
    Serial.println("THINGER) Captura manual solicitada desde dashboard");
    lastControlCaptureOk = performAnalysis(true);
    if (lastControlCaptureOk) {
      controlCaptureRequestCount++;
    }
  }

  if (autoModeEnabled) {
    unsigned long now = millis();

    if (autoDeepSleepEnabled) {
      if (!autoDeepSleepCaptureDone) {
        Serial.println("AUTO_SLEEP) Captura automatica con Deep Sleep");
        if (console) {
          console.printf("[AUTO_SLEEP] Captura. Intervalo: %lu segundos. Ventana despierta: %lu segundos\r\n",
                         autoIntervalMs / 1000UL, awakeWindowSeconds);
          console.flush();
        }
        waitForThingerReadyAfterWake(THINGER_READY_WAIT_AFTER_WAKE_MS,
                                      "espera inicial tras despertar antes de capturar/enviar");

        performAutomaticCaptureAndUpdateCount();

        waitForThingerReadyAfterWake(THINGER_STREAM_FLUSH_MS,
                                      "margen posterior a thing.stream para asegurar registro en bucket");

        autoDeepSleepCaptureDone = true;
        awakeWindowStartMs = millis();
        saveRuntimeConfigToRtc();
      } else if (awakeWindowStartMs > 0 && (now - awakeWindowStartMs >= awakeWindowSeconds * 1000UL)) {
        enterAutoDeepSleepCycle();
      }
    } else {
      if (lastAutoCaptureMs == 0 || (now - lastAutoCaptureMs >= autoIntervalMs)) {
        Serial.println("AUTO) Nueva captura automatica");
        if (console) {
          console.printf("[AUTO] Comienza captura automatica. Intervalo: %lu segundos\r\n",
                         autoIntervalMs / 1000UL);
          console.flush();
        }

        performAutomaticCaptureAndUpdateCount();
        lastAutoCaptureMs = now;
      }
    }
  }

  handlePendingSystemAction();
}