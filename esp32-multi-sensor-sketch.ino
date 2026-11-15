/* ESP32_Multi_Sensors BME280 + BH1750 + MQ-2 + Soil + Rain + Relay
 * GPIO:
 *   21 = SDA (BME280 + BH1750)
 *   22 = SCL (BME280 + BH1750)
 *   32 = MQ-2 AOUT (ADC1_CH4)
 *   34 = Soil AOUT (ADC1_CH6, input-only)
 *   35 = Rain AOUT (ADC1_CH7, input-only)
 *   25 = Relay IN
 *    0 = Boot button (5s hold = factory reset)
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <BH1750.h>
#include <esp_timer.h>
#include <esp_system.h>
#include <esp_wifi.h>

/* ------------------------------------------------------------------ */
/* Constants                                                         */
/* ------------------------------------------------------------------ */
#define FW_VERSION              "2.0.1"
#define FW_BUILD                __DATE__ " " __TIME__

#define PIN_SDA                 21
#define PIN_SCL                 22
#define PIN_MQ2                 32
#define PIN_SOIL                34
#define PIN_RAIN                35
#define PIN_RELAY               25
#define PIN_BOOT                0

#define I2C_CLOCK_HZ            100000UL
#define BME280_I2C_ADDR         0x76
#define BH1750_I2C_ADDR         0x23

#define DEF_AP_SSID             "ESP32_Multi_Sensors"
#define DEF_AP_PASS             "12345678"
#define AP_CHANNEL              6
#define AP_HIDDEN               false
#define AP_MAX_CLIENTS          4

#define BOOT_HOLD_MS            5000ULL
#define BOOT_DEBOUNCE_MS        40ULL

#define SAMPLE_MIN_MS           500ULL
#define SAMPLE_MAX_MS           3600000ULL
#define SAMPLE_DEFAULT_MS       2000ULL

#define NVS_NAMESPACE           "msens"
#define NVS_KEY_BLOB            "blob"
#define NVS_KEY_META            "meta"

#define NVS_SAVE_MIN_GAP_MS     300000ULL
#define NVS_SAVE_MAX_DEFER_MS   900000ULL
#define NVS_SAVE_USER_COOLDOWN_MS 5000ULL

#define CFG_MAGIC               0x4D53454EUL
#define CFG_SCHEMA              3UL

#define ADC_SAMPLES             16ULL
#define ADC_MAX                 4095UL

#define SOIL_DRY_DEF            2600UL
#define SOIL_WET_DEF            1100UL
#define MQ2_CLEAN_DEF           400UL
#define MQ2_DIRTY_DEF           3200UL
#define RAIN_DRY_DEF            3800UL
#define RAIN_WET_DEF            900UL

#define REBOOT_FLUSH_MS         400UL
#define I2C_RECOVER_MIN_GAP_MS  30000ULL

/* ------------------------------------------------------------------ */
/* Config struct                                                    */
/* ------------------------------------------------------------------ */
struct __attribute__((packed)) Config {
  uint32_t magic;
  uint32_t schema;

  char     ap_ssid[33];
  char     ap_pass[65];

  uint32_t sample_interval_ms;

  uint32_t relay_mode;          
  uint32_t relay_rule;         
  uint32_t relay_threshold;
  uint32_t relay_min_on_ms;
  uint32_t relay_invert;
  uint32_t relay_rain_pct;

  uint32_t soil_dry;
  uint32_t soil_wet;
  uint32_t mq2_clean;
  uint32_t mq2_dirty;
  uint32_t rain_dry;
  uint32_t rain_wet;

  int32_t  temp_off_x10;
  int32_t  hum_off_x10;
  int32_t  pres_off_pa;
  int32_t  lux_off_x10;

  uint32_t reserved[5];
};

struct __attribute__((packed)) Meta {
  uint32_t magic2;             
  uint32_t schema;
  uint32_t boot_count;
  uint32_t save_count;
  uint32_t payload_crc;
  uint32_t meta_crc;
};

/* ------------------------------------------------------------------ */
/* Globals                                                              */
/* ------------------------------------------------------------------ */
static Config        g_cfg;
static portMUX_TYPE  g_cfg_mux  = portMUX_INITIALIZER_UNLOCKED;
static volatile bool g_cfg_dirty = false;
static uint32_t      g_cfg_gen   = 0;
static uint64_t      g_last_save_ms   = 0;
static uint64_t      g_first_dirty_ms = 0;

static uint32_t      g_boot_count = 0;
static uint32_t      g_save_count = 0;

struct __attribute__((packed)) Snapshot {
  int32_t  temp_x10;
  int32_t  hum_x10;
  int32_t  pres_pa;
  int32_t  lux_x10;
  uint16_t mq2_adc;
  uint16_t soil_adc;
  uint16_t rain_adc;
  uint8_t  mq2_pct;
  uint8_t  soil_pct;
  uint8_t  rain_pct;
  uint8_t  bme_ok;
  uint8_t  bh_ok;
  uint8_t  relay_on;
  uint8_t  valid;
  uint8_t  adc_fault;
  uint8_t  pad[3];
  uint64_t ts_us;
};
static Snapshot       g_snap;
static portMUX_TYPE   g_snap_mux = portMUX_INITIALIZER_UNLOCKED;

static Adafruit_BME280 g_bme;
static BH1750          g_bh;
static bool            g_bme_ok     = false;
static bool            g_bh_ok      = false;
static uint32_t        g_bme_fail   = 0;
static uint32_t        g_bh_fail    = 0;
static uint64_t        g_bme_recover_ms = 0;
static uint64_t        g_bh_recover_ms  = 0;

static uint64_t        g_last_sample_us = 0;
static uint64_t        g_relay_on_us    = 0;
static bool            g_relay_state    = false;
static portMUX_TYPE    g_relay_mux      = portMUX_INITIALIZER_UNLOCKED;

static WebServer       g_server(80);
static DNSServer       g_dns;
static bool            g_dns_up = false;

static volatile bool   g_reboot_req     = false;
static volatile bool   g_factory_req    = false;

static portMUX_TYPE    g_btn_mux        = portMUX_INITIALIZER_UNLOCKED;
static volatile bool   g_btn_down       = false;
static volatile uint64_t g_btn_down_ms  = 0;
static volatile bool   g_btn_armed      = false;
static volatile bool   g_btn_handled    = false;

/* ------------------------------------------------------------------ */
/* 64-bit millisecond helper                                         */
/* ------------------------------------------------------------------ */
static inline uint64_t millis64() {
  return (uint64_t)esp_timer_get_time() / 1000ULL;
}

/* ------------------------------------------------------------------ */
/* CRC32                                                               */
/* ------------------------------------------------------------------ */
static uint32_t crc32_calc(const void* data, size_t len) {
  static uint32_t table[256];
  static bool init = false;
  if (!init) {
    for (uint32_t i = 0; i < 256; i++) {
      uint32_t c = i;
      for (int k = 0; k < 8; k++)
        c = (c & 1UL) ? (0xEDB88320UL ^ (c >> 1)) : (c >> 1);
      table[i] = c;
    }
    init = true;
  }
  const uint8_t* p = (const uint8_t*)data;
  uint32_t c = 0xFFFFFFFFUL;
  for (size_t i = 0; i < len; i++)
    c = table[(c ^ p[i]) & 0xFFUL] ^ (c >> 8);
  return c ^ 0xFFFFFFFFUL;
}

/* ------------------------------------------------------------------ */
/* Config validation and defaults                       */
/* ------------------------------------------------------------------ */
static void cfg_defaults(Config& c) {
  memset(&c, 0, sizeof(c));
  c.magic = CFG_MAGIC;
  c.schema = CFG_SCHEMA;
  strncpy(c.ap_ssid, DEF_AP_SSID, sizeof(c.ap_ssid) - 1);
  strncpy(c.ap_pass, DEF_AP_PASS, sizeof(c.ap_pass) - 1);
  c.sample_interval_ms = SAMPLE_DEFAULT_MS;
  c.relay_mode = 0;
  c.relay_rule = 0;
  c.relay_threshold = SOIL_DRY_DEF;
  c.relay_min_on_ms = 0;
  c.relay_invert = 0;
  c.relay_rain_pct = 50;
  c.soil_dry = SOIL_DRY_DEF;
  c.soil_wet = SOIL_WET_DEF;
  c.mq2_clean = MQ2_CLEAN_DEF;
  c.mq2_dirty = MQ2_DIRTY_DEF;
  c.rain_dry = RAIN_DRY_DEF;
  c.rain_wet = RAIN_WET_DEF;
}

static int32_t clamp_i32(int32_t v, int32_t lo, int32_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static void cfg_sanitize(Config& c) {
  if (c.sample_interval_ms < SAMPLE_MIN_MS) c.sample_interval_ms = SAMPLE_MIN_MS;
  if (c.sample_interval_ms > SAMPLE_MAX_MS) c.sample_interval_ms = SAMPLE_MAX_MS;
  if (c.relay_mode > 3) c.relay_mode = 0;
  if (c.relay_rule > 3) c.relay_rule = 0;
  if (c.relay_threshold > ADC_MAX) c.relay_threshold = ADC_MAX;
  if (c.relay_min_on_ms > 86400000UL) c.relay_min_on_ms = 86400000UL;
  if (c.relay_invert > 1) c.relay_invert = 0;
  if (c.relay_rain_pct > 100) c.relay_rain_pct = 100;
  if (c.soil_dry > ADC_MAX) c.soil_dry = ADC_MAX;
  if (c.soil_wet > ADC_MAX) c.soil_wet = ADC_MAX;
  if (c.mq2_clean > ADC_MAX) c.mq2_clean = ADC_MAX;
  if (c.mq2_dirty > ADC_MAX) c.mq2_dirty = ADC_MAX;
  if (c.rain_dry > ADC_MAX) c.rain_dry = ADC_MAX;
  if (c.rain_wet > ADC_MAX) c.rain_wet = ADC_MAX;

  c.temp_off_x10 = clamp_i32(c.temp_off_x10, -500, 500);
  c.hum_off_x10  = clamp_i32(c.hum_off_x10,  -500, 500);
  c.pres_off_pa  = clamp_i32(c.pres_off_pa,  -5000, 5000);
  c.lux_off_x10  = clamp_i32(c.lux_off_x10,  -5000, 5000);

  if (c.soil_dry < c.soil_wet) {
    uint32_t t = c.soil_dry; c.soil_dry = c.soil_wet; c.soil_wet = t;
  }
  if (c.mq2_clean > c.mq2_dirty) {
    uint32_t t = c.mq2_clean; c.mq2_clean = c.mq2_dirty; c.mq2_dirty = t;
  }
  if (c.rain_dry < c.rain_wet) {
    uint32_t t = c.rain_dry; c.rain_dry = c.rain_wet; c.rain_wet = t;
  }

  c.ap_ssid[32] = 0;
  c.ap_pass[64] = 0;
  if (c.ap_ssid[0] == 0) {
    memset(c.ap_ssid, 0, sizeof(c.ap_ssid));
    strncpy(c.ap_ssid, DEF_AP_SSID, sizeof(c.ap_ssid) - 1);
  }
  size_t pl = strlen(c.ap_pass);
  if (pl < 8) {
    memset(c.ap_pass, 0, sizeof(c.ap_pass));
    strncpy(c.ap_pass, DEF_AP_PASS, sizeof(c.ap_pass) - 1);
  }
  if (pl > 63) c.ap_pass[63] = 0;
}

/* ------------------------------------------------------------------ */
/* Persistent storage                                                 */
/* ------------------------------------------------------------------ */
static bool nvs_load() {
  Preferences p;
  if (!p.begin(NVS_NAMESPACE, true)) {
    cfg_defaults(g_cfg);
    cfg_sanitize(g_cfg);
    nvs_mark_dirty();
    g_boot_count++;
    return false;
  }
  Meta m;
  size_t ml = p.getBytesLength(NVS_KEY_META);
  bool meta_ok = false;
  if (ml == sizeof(Meta)) {
    if (p.getBytes(NVS_KEY_META, &m, sizeof(m)) == sizeof(m)) {
      Meta chk = m;
      uint32_t save = chk.meta_crc;
      chk.meta_crc = 0;
      if (crc32_calc(&chk, sizeof(chk)) == save && m.magic2 == 0x4D455441UL) {
        meta_ok = true;
      }
    }
  }
  if (meta_ok) {
    g_boot_count = m.boot_count;
    g_save_count = m.save_count;
  }

  Config tmp;
  memset(&tmp, 0, sizeof(tmp));
  size_t cl = p.getBytesLength(NVS_KEY_BLOB);
  bool cfg_ok = false;
  if (cl == sizeof(Config)) {
    if (p.getBytes(NVS_KEY_BLOB, &tmp, sizeof(tmp)) == sizeof(tmp)) {
      if (tmp.magic == CFG_MAGIC && tmp.schema == CFG_SCHEMA) {
        if (!meta_ok || crc32_calc(&tmp, sizeof(tmp)) == m.payload_crc) {
          cfg_ok = true;
        }
      }
    }
  }
  p.end();

  if (cfg_ok) {
    g_cfg = tmp;
    cfg_sanitize(g_cfg);
  } else {
    cfg_defaults(g_cfg);
    cfg_sanitize(g_cfg);
    nvs_mark_dirty();
  }
  g_boot_count++;
  return cfg_ok;
}

static bool nvs_save() {
  Config cfg_copy;
  uint32_t gen_at_copy;
  portENTER_CRITICAL(&g_cfg_mux);
  cfg_copy = g_cfg;
  gen_at_copy = g_cfg_gen;
  portEXIT_CRITICAL(&g_cfg_mux);
  cfg_sanitize(cfg_copy);

  Meta m;
  memset(&m, 0, sizeof(m));
  m.magic2 = 0x4D455441UL;
  m.schema = CFG_SCHEMA;
  m.boot_count = g_boot_count;
  m.save_count = g_save_count;
  m.payload_crc = crc32_calc(&cfg_copy, sizeof(cfg_copy));
  m.meta_crc = 0;
  m.meta_crc = crc32_calc(&m, sizeof(m));

  Preferences p;
  if (!p.begin(NVS_NAMESPACE, false)) return false;
  bool ok1 = (p.putBytes(NVS_KEY_BLOB, &cfg_copy, sizeof(cfg_copy)) == sizeof(cfg_copy));
  bool ok2 = (p.putBytes(NVS_KEY_META, &m, sizeof(m)) == sizeof(m));
  p.end();

  if (!(ok1 && ok2)) return false;

  g_save_count++;
  g_last_save_ms = millis64();

  portENTER_CRITICAL(&g_cfg_mux);
  if (g_cfg_gen == gen_at_copy) {
    g_cfg_dirty = false;
  }
  portEXIT_CRITICAL(&g_cfg_mux);

  return true;
}

static void nvs_mark_dirty() {
  portENTER_CRITICAL(&g_cfg_mux);
  if (!g_cfg_dirty) {
    g_cfg_dirty = true;
    g_first_dirty_ms = millis64();
  }
  g_cfg_gen++;
  portEXIT_CRITICAL(&g_cfg_mux);
}

static void nvs_tick() {
  bool dirty;
  uint64_t first_dirty;
  portENTER_CRITICAL(&g_cfg_mux);
  dirty = g_cfg_dirty;
  first_dirty = g_first_dirty_ms;
  portEXIT_CRITICAL(&g_cfg_mux);
  if (!dirty) return;
  uint64_t now = millis64();
  uint64_t since_last  = now - g_last_save_ms;
  uint64_t since_first = now - first_dirty;
  if (since_last >= NVS_SAVE_MIN_GAP_MS || since_first >= NVS_SAVE_MAX_DEFER_MS) {
    nvs_save();
  }
}

static void nvs_wipe() {
  Preferences p;
  if (p.begin(NVS_NAMESPACE, false)) {
    p.clear();
    p.end();
  }
}

/* ------------------------------------------------------------------ */
/* I2C / sensors                                                      */
/* ------------------------------------------------------------------ */
static void i2c_bus_recover() {
  pinMode(PIN_SDA, INPUT_PULLUP);
  pinMode(PIN_SCL, INPUT_PULLUP);
  delayMicroseconds(10);
  for (int i = 0; i < 9; i++) {
    pinMode(PIN_SCL, OUTPUT); digitalWrite(PIN_SCL, LOW);
    delayMicroseconds(6);
    pinMode(PIN_SCL, INPUT_PULLUP);
    delayMicroseconds(6);
  }
  pinMode(PIN_SDA, OUTPUT); digitalWrite(PIN_SDA, LOW);
  delayMicroseconds(6);
  pinMode(PIN_SDA, INPUT_PULLUP);
  delayMicroseconds(6);
}

static void i2c_init() {
  Wire.end();
  Wire.begin(PIN_SDA, PIN_SCL, I2C_CLOCK_HZ);
  Wire.setTimeOut(25);
}

static bool bme_bringup() {
  if (g_bme.begin(BME280_I2C_ADDR, &Wire)) {
    g_bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                      Adafruit_BME280::SAMPLING_X2,
                      Adafruit_BME280::SAMPLING_X16,
                      Adafruit_BME280::SAMPLING_X1,
                      Adafruit_BME280::FILTER_X16,
                      Adafruit_BME280::STANDBY_MS_1000);
    return true;
  }
  return false;
}

static bool bh_bringup() {
  return g_bh.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, BH1750_I2C_ADDR, &Wire);
}

/* ------------------------------------------------------------------ */
/* ADC helpers                                                        */
/* ------------------------------------------------------------------ */
static uint16_t adc_sample(uint8_t pin) {
  uint64_t sum = 0;
  for (uint64_t i = 0; i < ADC_SAMPLES; i++) {
    sum += (uint64_t)analogRead(pin);
    delayMicroseconds(60);
  }
  return (uint16_t)(sum / ADC_SAMPLES);
}

static uint8_t pct_from(uint32_t v, uint32_t low_meaning_high, uint32_t hi_meaning_high) {
  if (hi_meaning_high == low_meaning_high) return 0;
  int64_t num = (int64_t)v - (int64_t)low_meaning_high;
  int64_t den = (int64_t)hi_meaning_high - (int64_t)low_meaning_high;
  if (den == 0) return 0;
  int64_t p = (num * 100LL) / den;
  if (p < 0) p = 0;
  if (p > 100) p = 100;
  return (uint8_t)p;
}

static int32_t sat_i32(int64_t v, int32_t lo, int32_t hi) {
  if (v < (int64_t)lo) return lo;
  if (v > (int64_t)hi) return hi;
  return (int32_t)v;
}

/* ------------------------------------------------------------------ */
/* Relay driver                                                       */
/* ------------------------------------------------------------------ */
static void relay_apply_locked(const Config& cfg, bool on) {
  bool level = on;
  if (cfg.relay_invert) level = !level;
  digitalWrite(PIN_RELAY, level ? HIGH : LOW);
  g_relay_state = on;
  if (on) g_relay_on_us = (uint64_t)esp_timer_get_time();
}

static void relay_update_locked(const Config& cfg, uint16_t soil_adc, uint16_t mq2_adc,
                                uint8_t rain_pct, bool rule_ok) {
  uint32_t mode = cfg.relay_mode;
  if (mode == 0) { if (g_relay_state) relay_apply_locked(cfg, false); return; }
  if (mode == 2) { if (!g_relay_state) relay_apply_locked(cfg, true); return; }
  if (mode == 3) { if (g_relay_state) relay_apply_locked(cfg, false); return; }

  bool want = false;
  if (rule_ok) {
    uint32_t rule = cfg.relay_rule;
    if (rule == 0)      want = (uint64_t)soil_adc > (uint64_t)cfg.relay_threshold;
    else if (rule == 1) want = (uint64_t)soil_adc < (uint64_t)cfg.relay_threshold;
    else if (rule == 2) want = (uint64_t)mq2_adc  > (uint64_t)cfg.relay_threshold;
    else if (rule == 3) want = (uint64_t)rain_pct > (uint64_t)cfg.relay_rain_pct;
  }

  if (want) {
    if (!g_relay_state) relay_apply_locked(cfg, true);
  } else {
    if (g_relay_state) {
      uint64_t held = (uint64_t)esp_timer_get_time() - g_relay_on_us;
      if (cfg.relay_min_on_ms == 0 || held >= (uint64_t)cfg.relay_min_on_ms * 1000ULL) {
        relay_apply_locked(cfg, false);
      }
    }
  }
}

static void relay_update(const Config& cfg, uint16_t soil_adc, uint16_t mq2_adc,
                         uint8_t rain_pct, bool rule_ok) {
  portENTER_CRITICAL(&g_relay_mux);
  relay_update_locked(cfg, soil_adc, mq2_adc, rain_pct, rule_ok);
  portEXIT_CRITICAL(&g_relay_mux);
}

/* ------------------------------------------------------------------ */
/* Sampling                                                           */
/* ------------------------------------------------------------------ */
static void sample_tick() {
  uint64_t now = (uint64_t)esp_timer_get_time();
  Config cfg;
  portENTER_CRITICAL(&g_cfg_mux);
  cfg = g_cfg;
  portEXIT_CRITICAL(&g_cfg_mux);
  if ((now - g_last_sample_us) < (uint64_t)cfg.sample_interval_ms * 1000ULL) return;
  g_last_sample_us = now;

  if (!g_bme_ok) {
    g_bme_fail++;
    if (g_bme_fail > 3) {
      uint64_t now_ms = millis64();
      if (g_bme_recover_ms == 0 || (now_ms - g_bme_recover_ms) >= I2C_RECOVER_MIN_GAP_MS) {
        i2c_bus_recover();
        i2c_init();
        g_bme_recover_ms = now_ms;
      }
      g_bme_fail = 0;
    }
    g_bme_ok = bme_bringup();
  }
  if (!g_bh_ok) {
    g_bh_fail++;
    if (g_bh_fail > 3) {
      uint64_t now_ms = millis64();
      if (g_bh_recover_ms == 0 || (now_ms - g_bh_recover_ms) >= I2C_RECOVER_MIN_GAP_MS) {
        i2c_bus_recover();
        i2c_init();
        g_bh_recover_ms = now_ms;
      }
      g_bh_fail = 0;
    }
    g_bh_ok = bh_bringup();
  }

  Snapshot s;
  memset(&s, 0, sizeof(s));
  s.ts_us = now;

  if (g_bme_ok) {
    float t = g_bme.readTemperature();
    float h = g_bme.readHumidity();
    float p = g_bme.readPressure();
    if (!isnan(t) && !isnan(h) && !isnan(p) && h >= 0.0f && h <= 100.0f && t > -40.0f && t < 85.0f) {
      int64_t tx = (int64_t)lroundf(t * 10.0f) + cfg.temp_off_x10;
      int64_t hx = (int64_t)lroundf(h * 10.0f) + cfg.hum_off_x10;
      int64_t px = (int64_t)lroundf(p) + cfg.pres_off_pa;
      s.temp_x10 = sat_i32(tx, -400, 850);
      s.hum_x10 = sat_i32(hx, 0, 1000);
      s.pres_pa = sat_i32(px, 30000, 110000);
      s.bme_ok = 1;
    }
  }

  if (g_bh_ok) {
    float l = g_bh.readLightLevel();
    if (!isnan(l) && l >= 0.0f) {
      int64_t lx = (int64_t)lroundf(l * 10.0f) + cfg.lux_off_x10;
      s.lux_x10 = sat_i32(lx, 0, 2000000);
      s.bh_ok = 1;
    }
  }

  uint16_t mq2 = adc_sample(PIN_MQ2);
  uint16_t soil = adc_sample(PIN_SOIL);
  uint16_t rain = adc_sample(PIN_RAIN);
  s.mq2_adc = mq2;
  s.soil_adc = soil;
  s.rain_adc = rain;

  bool soil_ok = (soil > 20 && soil < (uint16_t)(ADC_MAX - 20));
  bool mq2_ok  = (mq2  > 20 && mq2  < (uint16_t)(ADC_MAX - 20));
  bool rain_ok = (rain > 20 && rain < (uint16_t)(ADC_MAX - 20));

  s.mq2_pct  = pct_from(mq2,  cfg.mq2_clean,  cfg.mq2_dirty);
  s.soil_pct = pct_from(soil, cfg.soil_dry,   cfg.soil_wet);
  s.rain_pct = pct_from(rain, cfg.rain_dry,   cfg.rain_wet);

  bool rule_input_ok = true;
  if (cfg.relay_mode == 1) {
    if (cfg.relay_rule == 0 || cfg.relay_rule == 1) rule_input_ok = soil_ok;
    else if (cfg.relay_rule == 2)                  rule_input_ok = mq2_ok;
    else if (cfg.relay_rule == 3)                  rule_input_ok = rain_ok;
  }

  relay_update(cfg, soil, mq2, s.rain_pct, rule_input_ok);

  portENTER_CRITICAL(&g_relay_mux);
  s.relay_on = g_relay_state ? 1 : 0;
  portEXIT_CRITICAL(&g_relay_mux);
  s.adc_fault = (soil_ok ? 0 : 1) | (mq2_ok ? 0 : 2) | (rain_ok ? 0 : 4);
  s.valid = 1;

  portENTER_CRITICAL(&g_snap_mux);
  g_snap = s;
  portEXIT_CRITICAL(&g_snap_mux);
}

/* ------------------------------------------------------------------ */
/* Boot button                                                        */
/* ------------------------------------------------------------------ */
static void btn_tick() {
  bool raw = (digitalRead(PIN_BOOT) == LOW);
  uint64_t now_ms = millis64();

  if (raw) {
    portENTER_CRITICAL(&g_btn_mux);
    if (!g_btn_down) {
      g_btn_down = true;
      g_btn_down_ms = now_ms;
      g_btn_armed = false;
      g_btn_handled = false;
    }
    uint64_t held_ms = now_ms - g_btn_down_ms;
    if (!g_btn_armed && !g_btn_handled && held_ms >= BOOT_DEBOUNCE_MS) {
      g_btn_armed = true;
    }
    if (g_btn_armed && !g_btn_handled && held_ms >= BOOT_HOLD_MS) {
      g_btn_handled = true;
      g_factory_req = true;
    }
    portEXIT_CRITICAL(&g_btn_mux);
  } else {
    portENTER_CRITICAL(&g_btn_mux);
    if (g_btn_down) {
      g_btn_down = false;
      g_btn_armed = false;
      g_btn_handled = false;
    }
    portEXIT_CRITICAL(&g_btn_mux);
  }
}

static uint32_t btn_hold_pct() {
  uint32_t result = 0;
  portENTER_CRITICAL(&g_btn_mux);
  if (g_btn_armed && g_btn_down) {
    uint64_t now_ms = millis64();
    uint64_t held_ms = now_ms - g_btn_down_ms;
    uint64_t p = (held_ms * 100ULL) / BOOT_HOLD_MS;
    if (p > 100ULL) p = 100ULL;
    result = (uint32_t)p;
  }
  portEXIT_CRITICAL(&g_btn_mux);
  return result;
}

/* ------------------------------------------------------------------ */
/* HTML UI                                                            */
/* ------------------------------------------------------------------ */
static const char CSS[] PROGMEM =
":root{--bg:#0b1220;--bg2:#0f172a;--card:#1a2436;--card2:#232f45;--fg:#e6edf6;"
"--muted:#8ca0bd;--line:#2a3a55;--accent:#4fc3f7;--ok:#34d399;--warn:#fbbf24;--err:#f87171}"
"*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}"
"html,body{margin:0;padding:0;background:linear-gradient(180deg,var(--bg) 0%,var(--bg2) 100%);"
"color:var(--fg);font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,sans-serif;"
"min-height:100vh;-webkit-font-smoothing:antialiased}"
".wrap{max-width:1120px;margin:0 auto;padding:14px}"
"header{display:flex;align-items:center;justify-content:space-between;gap:10px;"
"flex-wrap:wrap;margin-bottom:14px}"
"h1{margin:0;font-size:19px;font-weight:700;letter-spacing:.2px}"
".pill{background:var(--card2);border:1px solid var(--line);padding:6px 12px;"
"border-radius:99px;font-size:12px;color:var(--muted);font-variant-numeric:tabular-nums}"
"nav{display:flex;gap:6px;flex-wrap:wrap;margin-bottom:14px}"
"nav a{color:var(--fg);text-decoration:none;background:var(--card);padding:10px 14px;"
"border-radius:10px;font-size:13px;border:1px solid var(--line);transition:.15s}"
"nav a:hover,nav a.active{background:var(--accent);color:#0a1220;border-color:var(--accent)}"
".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(210px,1fr));gap:12px}"
".card{background:var(--card);border:1px solid var(--line);border-radius:14px;padding:16px}"
".card h3{margin:0 0 10px 0;font-size:11px;color:var(--muted);letter-spacing:1.2px;"
"text-transform:uppercase;font-weight:700}"
".val{font-size:30px;font-weight:700;font-variant-numeric:tabular-nums;line-height:1.1}"
".unit{font-size:13px;color:var(--muted);margin-left:4px;font-weight:500}"
".sub{font-size:11px;color:var(--muted);margin-top:6px;font-variant-numeric:tabular-nums}"
".bar{height:6px;background:var(--card2);border-radius:99px;overflow:hidden;margin-top:12px}"
".bar>i{display:block;height:100%;width:0;background:var(--accent);"
"transition:width .35s ease,background .3s}"
".bar.ok>i{background:var(--ok)}.bar.warn>i{background:var(--warn)}.bar.err>i{background:var(--err)}"
".badge{display:inline-block;font-size:11px;padding:3px 9px;border-radius:99px;"
"background:var(--card2);color:var(--muted)}"
".badge.on{background:rgba(52,211,153,.16);color:var(--ok)}"
".badge.off{background:rgba(140,160,189,.14)}"
".btn{display:inline-flex;align-items:center;justify-content:center;gap:6px;"
"background:var(--accent);color:#0a1220;border:none;padding:10px 16px;border-radius:10px;"
"font-size:13px;font-weight:700;cursor:pointer;transition:.15s;min-height:40px}"
".btn:hover{filter:brightness(1.08)}.btn:active{transform:scale(.97)}"
".btn.ghost{background:transparent;border:1px solid var(--line);color:var(--fg)}"
".btn.danger{background:var(--err);color:#fff}"
".btn.warn{background:var(--warn);color:#0a1220}"
".row{display:flex;gap:8px;flex-wrap:wrap;align-items:center}"
".field{margin-bottom:14px}"
".field label{display:block;font-size:11px;color:var(--muted);text-transform:uppercase;"
"letter-spacing:.7px;font-weight:700;margin-bottom:6px}"
".field input,.field select{width:100%;background:var(--card2);border:1px solid var(--line);"
"color:var(--fg);padding:11px 12px;border-radius:10px;font-size:15px;outline:none;"
"transition:.15s;min-height:44px}"
".field input:focus,.field select:focus{border-color:var(--accent)}"
".field .hint{font-size:11px;color:var(--muted);margin-top:5px}"
".toast{position:fixed;bottom:18px;left:50%;transform:translate(-50%,150%);"
"background:var(--card2);border:1px solid var(--line);border-radius:12px;padding:12px 20px;"
"font-size:13px;box-shadow:0 12px 32px rgba(0,0,0,.5);transition:transform .25s;"
"z-index:99;max-width:90vw}"
".toast.show{transform:translate(-50%,0)}"
".toast.ok{border-color:var(--ok)}.toast.err{border-color:var(--err)}"
"footer{margin-top:26px;padding-top:14px;border-top:1px solid var(--line);"
"text-align:center;font-size:11px;color:var(--muted)}"
".sect{background:var(--card);border:1px solid var(--line);border-radius:14px;"
"padding:16px;margin-bottom:12px}"
".sect h2{margin:0 0 12px;font-size:14px;letter-spacing:.4px}"
".kv{display:flex;justify-content:space-between;padding:6px 0;"
"border-bottom:1px solid var(--line);font-size:13px;gap:8px;word-break:break-word}"
".kv:last-child{border-bottom:none}"
".kv .k{color:var(--muted)}.kv .v{font-variant-numeric:tabular-nums;text-align:right}"
".cli{display:flex;justify-content:space-between;padding:8px 0;"
"border-bottom:1px solid var(--line);font-size:12px;gap:8px;flex-wrap:wrap}"
"@media(max-width:480px){.val{font-size:25px}h1{font-size:16px}.wrap{padding:11px}}";

static const char JS_COMMON[] PROGMEM =
"function T(m,c){var t=document.getElementById('toast');if(!t)return;t.textContent=m;"
"t.className='toast show '+(c||'');clearTimeout(window.__tt);"
"window.__tt=setTimeout(function(){t.className='toast';},2600);}"
"function F(u,d){return fetch(u,{method:'POST',cache:'no-store',"
"headers:{'Content-Type':'application/x-www-form-urlencoded'},body:d})"
".then(r=>r.json()).catch(e=>({ok:false,err:'net'}));}"
"function G(u){return fetch(u,{cache:'no-store'}).then(r=>r.json()).catch(()=>null);}"
"function fmtup(s){s=Number(s)||0;var d=Math.floor(s/86400),h=Math.floor(s%86400/3600),"
"m=Math.floor(s%3600/60);var o='';if(d)o+=d+'d ';if(d||h)o+=h+'h ';o+=m+'m';return o;}"
"function fnum(v,d){var n=Number(v);if(!isFinite(n))return '--';return n.toFixed(d);}";

static String html_head(const char* title, const char* active) {
  String s;
  s.reserve(6000);
  s += F("<!DOCTYPE html><html lang='en'><head><meta charset='utf-8'>"
         "<meta name='viewport' content='width=device-width,initial-scale=1,"
         "maximum-scale=1,user-scalable=no'>"
         "<meta name='theme-color' content='#0b1220'>"
         "<meta name='format-detection' content='telephone=no'>"
         "<meta name='apple-mobile-web-app-capable' content='yes'>"
         "<meta name='apple-mobile-web-app-status-bar-style' content='black-translucent'>"
         "<meta name='apple-mobile-web-app-title' content='ESP32 Sensors'>"
         "<meta name='mobile-web-app-capable' content='yes'>");
  s += "<title>"; s += title; s += "</title><style>";
  s += CSS;
  s += "</style></head><body><div class='wrap'><header><h1>&#128225; Multi Sensors</h1>"
       "<span class='pill' id='up'>--</span></header><nav>";
  s += "<a href='/'";     if (!strcmp(active,"dash")) s += " class='active'"; s += ">Dashboard</a>";
  s += "<a href='/settings'"; if (!strcmp(active,"set")) s += " class='active'"; s += ">Settings</a>";
  s += "<a href='/wifi'"; if (!strcmp(active,"wifi")) s += " class='active'"; s += ">WiFi AP</a>";
  s += "<a href='/system'"; if (!strcmp(active,"sys")) s += " class='active'"; s += ">System</a>";
  s += "</nav>";
  return s;
}

static String html_tail() {
  String s;
  s.reserve(1500);
  s += "<footer>ESP32 Multi Sensors &bull; " FW_VERSION "</footer>"
       "<div id='toast' class='toast'></div><script>";
  s += JS_COMMON;
  s += "</script></div></body></html>";
  return s;
}

static String page_dashboard() {
  String s;
  s.reserve(12000);
  s += html_head("Dashboard", "dash");

  s += "<div class='grid'>";

  s += "<div class='card'><h3>Temperature</h3>"
       "<div class='val'><span id='T'>--</span><span class='unit'>&deg;C</span></div>"
       "<div class='sub' id='Ts'>BME280 waiting</div></div>";

  s += "<div class='card'><h3>Humidity</h3>"
       "<div class='val'><span id='H'>--</span><span class='unit'>%</span></div>"
       "<div class='sub' id='Hs'>BME280 waiting</div></div>";

  s += "<div class='card'><h3>Pressure</h3>"
       "<div class='val'><span id='P'>--</span><span class='unit'>hPa</span></div>"
       "<div class='sub' id='Ps'>BME280 waiting</div></div>";

  s += "<div class='card'><h3>Light</h3>"
       "<div class='val'><span id='L'>--</span><span class='unit'>lx</span></div>"
       "<div class='sub' id='Ls'>BH1750 waiting</div></div>";

  s += "<div class='card'><h3>Air Quality (MQ-2)</h3>"
       "<div class='val'><span id='M'>--</span><span class='unit'>%</span></div>"
       "<div class='bar' id='Mb'><i></i></div>"
       "<div class='sub'>ADC <span id='Ma'>--</span></div></div>";

  s += "<div class='card'><h3>Soil Moisture</h3>"
       "<div class='val'><span id='S'>--</span><span class='unit'>%</span></div>"
       "<div class='bar' id='Sb'><i></i></div>"
       "<div class='sub'>ADC <span id='Sa'>--</span></div></div>";

  s += "<div class='card'><h3>Rain</h3>"
       "<div class='val'><span id='R'>--</span><span class='unit'>%</span></div>"
       "<div class='bar' id='Rb'><i></i></div>"
       "<div class='sub'>ADC <span id='Ra'>--</span></div></div>";

  s += "<div class='card'><h3>Relay</h3>"
       "<div class='val'><span id='Rel'>--</span></div>"
       "<div class='sub'><span class='badge' id='Relb'>--</span></div>"
       "<div class='row' style='margin-top:10px'>"
       "<button class='btn ghost' onclick='SR(\"on\")'>On</button>"
       "<button class='btn ghost' onclick='SR(\"off\")'>Force Off</button>"
       "<button class='btn ghost' onclick='SR(\"disable\")'>Disable</button>"
       "<button class='btn ghost' onclick='SR(\"auto\")'>Auto</button>"
       "</div></div>";

  s += "</div>";
  s += "<script>";
  s += "function SR(a){F('/api/relay','a='+encodeURIComponent(a))"
       ".then(j=>{if(j&&j.ok)T('Relay '+a,'ok');else T((j&&j.err)||'failed','err');});}";
  s += "var last=null;"
       "function paint(d){if(!d)return;"
       "if(d.valid){"
       "document.getElementById('T').textContent=fnum(d.temp/10,1);"
       "document.getElementById('H').textContent=fnum(d.hum/10,1);"
       "document.getElementById('P').textContent=fnum(d.pres/100,2);"
       "document.getElementById('L').textContent=fnum(d.lux/10,0);"
       "document.getElementById('M').textContent=d.mq2p;"
       "document.getElementById('Ma').textContent=d.mq2;"
       "document.getElementById('S').textContent=d.soilp;"
       "document.getElementById('Sa').textContent=d.soil;"
       "document.getElementById('R').textContent=d.rainp;"
       "document.getElementById('Ra').textContent=d.rain;"
       "document.getElementById('Rel').textContent=d.relay?'ON':'OFF';"
       "var rb=document.getElementById('Relb');"
       "rb.textContent=d.relay?'ACTIVE':'IDLE';"
       "rb.className='badge '+(d.relay?'on':'off');"
       "var mb=document.getElementById('Mb');"
       "mb.className='bar '+(d.mq2p>66?'err':d.mq2p>33?'warn':'ok');"
       "mb.firstElementChild.style.width=d.mq2p+'%';"
       "var sb=document.getElementById('Sb');"
       "sb.className='bar '+(d.soilp<33?'err':d.soilp<66?'warn':'ok');"
       "sb.firstElementChild.style.width=d.soilp+'%';"
       "var rb2=document.getElementById('Rb');"
       "rb2.className='bar '+(d.rainp>66?'err':d.rainp>33?'warn':'ok');"
       "rb2.firstElementChild.style.width=d.rainp+'%';"
       "document.getElementById('Ts').textContent='BME280 '+(d.bme?'OK':'FAIL');"
       "document.getElementById('Hs').textContent='BME280 '+(d.bme?'OK':'FAIL');"
       "document.getElementById('Ps').textContent='BME280 '+(d.bme?'OK':'FAIL');"
       "document.getElementById('Ls').textContent='BH1750 '+(d.bh?'OK':'FAIL');"
       "var ft=d.fault||0;"
       "document.getElementById('Sa').textContent=d.soil+(ft&1?' (FAULT)':'');"
       "document.getElementById('Ma').textContent=d.mq2 +(ft&2?' (FAULT)':'');"
       "document.getElementById('Ra').textContent=d.rain+(ft&4?' (FAULT)':'');"
       "} else {"
       "['T','H','P','L','M','S','R'].forEach(function(id){"
       "document.getElementById(id).textContent='--';});"
       "['Ts','Hs','Ps','Ls'].forEach(function(id){"
       "document.getElementById(id).textContent='waiting';});"
       "}"
       "document.getElementById('up').textContent='Up '+fmtup(d.uptime);"
       "}"
       "function poll(){G('/api/data').then(paint).catch(()=>{});}"
       "poll();setInterval(poll,2500);";
  s += "</script>";
  s += html_tail();
  return s;
}

static String page_settings() {
  String s;
  s.reserve(14000);
  s += html_head("Settings", "set");
  s += "<form id='sf' onsubmit='return SV(event)'>";

  s += "<div class='sect'><h2>Soil Moisture</h2><div class='grid'>"
       "<div class='field'><label>Dry ADC</label>"
       "<input id='soil_dry' name='soil_dry' type='number' min='0' max='4095'>"
       "<div class='hint'>Reading in dry air</div></div>"
       "<div class='field'><label>Wet ADC</label>"
       "<input id='soil_wet' name='soil_wet' type='number' min='0' max='4095'>"
       "<div class='hint'>Reading in water</div></div>"
       "</div></div>";

  s += "<div class='sect'><h2>MQ-2 Air Quality</h2><div class='grid'>"
       "<div class='field'><label>Clean ADC</label>"
       "<input id='mq2_clean' name='mq2_clean' type='number' min='0' max='4095'></div>"
       "<div class='field'><label>Dirty ADC</label>"
       "<input id='mq2_dirty' name='mq2_dirty' type='number' min='0' max='4095'></div>"
       "</div></div>";

  s += "<div class='sect'><h2>Rain Sensor</h2><div class='grid'>"
       "<div class='field'><label>Dry ADC</label>"
       "<input id='rain_dry' name='rain_dry' type='number' min='0' max='4095'></div>"
       "<div class='field'><label>Wet ADC</label>"
       "<input id='rain_wet' name='rain_wet' type='number' min='0' max='4095'></div>"
       "</div></div>";

  s += "<div class='sect'><h2>BME280 / BH1750 Offsets</h2><div class='grid'>"
       "<div class='field'><label>Temperature (&deg;C x10)</label>"
       "<input id='temp_off' name='temp_off' type='number' min='-500' max='500'>"
       "<div class='hint'>15 = +1.5&deg;C</div></div>"
       "<div class='field'><label>Humidity (% x10)</label>"
       "<input id='hum_off' name='hum_off' type='number' min='-500' max='500'></div>"
       "<div class='field'><label>Pressure (Pa)</label>"
       "<input id='pres_off' name='pres_off' type='number' min='-5000' max='5000'></div>"
       "<div class='field'><label>Lux (x10)</label>"
       "<input id='lux_off' name='lux_off' type='number' min='-5000' max='5000'></div>"
       "</div></div>";

  s += "<div class='sect'><h2>Relay</h2><div class='grid'>"
       "<div class='field'><label>Mode</label><select id='relay_mode' name='relay_mode'>"
       "<option value='0'>Off</option><option value='1'>Auto</option>"
       "<option value='2'>Manual ON</option><option value='3'>Manual OFF</option></select></div>"
       "<div class='field'><label>Auto Rule</label><select id='relay_rule' name='relay_rule'>"
       "<option value='0'>Soil above threshold</option>"
       "<option value='1'>Soil below threshold</option>"
       "<option value='2'>MQ-2 above threshold</option>"
       "<option value='3'>Rain above %</option></select></div>"
       "<div class='field'><label>Threshold (ADC)</label>"
       "<input id='relay_thr' name='relay_thr' type='number' min='0' max='4095'></div>"
       "<div class='field'><label>Rain Threshold %</label>"
       "<input id='relay_rain' name='relay_rain' type='number' min='0' max='100'></div>"
       "<div class='field'><label>Min ON time (ms)</label>"
       "<input id='relay_ms' name='relay_ms' type='number' min='0' max='86400000'></div>"
       "<div class='field'><label>Invert</label><select id='relay_inv' name='relay_inv'>"
       "<option value='0'>No</option><option value='1'>Yes</option></select></div>"
       "</div></div>";

  s += "<div class='sect'><h2>Sampling</h2><div class='grid'>"
       "<div class='field'><label>Interval (ms)</label>"
       "<input id='sample_ms' name='sample_ms' type='number' min='500' max='3600000' step='100'>"
       "<div class='hint'>500 - 3600000</div></div>"
       "</div></div>";

  s += "<div class='row'><button class='btn' type='submit'>Save</button>"
       "<button class='btn ghost' type='button' onclick='LD()'>Reload</button></div>";
  s += "</form>";

  s += "<script>";
  s += "function fill(d){if(!d)return;for(var k in d){var e=document.getElementById(k);if(e)e.value=d[k];}}"
       "function LD(){G('/api/settings').then(fill);}"
       "function SV(ev){ev.preventDefault();var f=document.getElementById('sf');"
       "var fd=new FormData(f);var b=new URLSearchParams();"
       "fd.forEach((v,k)=>b.append(k,v));"
       "F('/api/settings',b.toString()).then(j=>{"
       "if(j&&j.ok)T('Saved','ok');"
       "else T((j&&j.fields)?('Bad: '+j.fields):((j&&j.err)||'failed'),'err');"
       "});return false;}"
       "LD();";
  s += "</script>";
  s += html_tail();
  return s;
}

static String page_wifi() {
  String s;
  s.reserve(9000);
  s += html_head("WiFi AP", "wifi");

  s += "<form id='wf' onsubmit='return SV(event)'>"
       "<div class='sect'><h2>Access Point</h2>"
       "<div class='field'><label>SSID</label>"
       "<input id='ssid' type='text' maxlength='32' required></div>"
       "<div class='field'><label>Password</label>"
       "<input id='pass' type='text' minlength='8' maxlength='63' required>"
       "<div class='hint'>8 - 63 characters</div></div>"
       "<div class='row'><button class='btn' type='submit'>Apply &amp; Reboot</button>"
       "<button class='btn ghost' type='button' onclick='LD()'>Reload</button></div>"
       "</div></form>";

  s += "<div class='sect'><h2>Connected Clients</h2><div id='cls'>loading...</div></div>";

  s += "<div class='sect'><h2>Factory Reset</h2>"
       "<div class='hint' style='margin-bottom:12px'>Erases all settings, then reboots. "
       "Alternatively hold BOOT (GPIO0) for 5 seconds.</div>"
       "<button class='btn danger' type='button' onclick='FR()'>Factory Reset</button></div>";

  s += "<script>";
  s += "function LD(){G('/api/wifi').then(d=>{if(d){document.getElementById('ssid').value=d.ssid;"
       "document.getElementById('pass').value=d.pass;}});loadCli();}"
       "function loadCli(){G('/api/clients').then(d=>{var e=document.getElementById('cls');"
       "if(!d||!d.clients||!d.clients.length){e.innerHTML='<div class=\"sub\">No clients</div>';return;}"
       "var h='';d.clients.forEach(c=>{"
       "h+='<div class=\"cli\"><span>'+c.mac+'</span><span>'+c.rssi+' dBm</span></div>';});"
       "e.innerHTML=h;});}"
       "function SV(ev){ev.preventDefault();var b=new URLSearchParams();"
       "b.append('ssid',document.getElementById('ssid').value);"
       "b.append('pass',document.getElementById('pass').value);"
       "F('/api/wifi',b.toString()).then(j=>{if(j&&j.ok){T('Saved, rebooting...','ok');"
       "setTimeout(()=>location.href='/',4500);}else T((j&&j.err)||'failed','err');});return false;}"
       "function FR(){if(!confirm('Erase all settings?'))return;"
       "F('/api/factory','').then(()=>{T('Resetting...','ok');setTimeout(()=>location.href='/',5000);});}"
       "LD();setInterval(loadCli,6000);";
  s += "</script>";
  s += html_tail();
  return s;
}

static String page_system() {
  String s;
  s.reserve(8000);
  s += html_head("System", "sys");

  s += "<div class='sect'><h2>Info</h2><div id='info'>loading...</div></div>";
  s += "<div class='sect'><h2>Boot Button (GPIO0)</h2>"
       "<div class='sub' style='margin-bottom:10px'>Hold 5s to factory reset</div>"
       "<div class='bar' id='bb'><i></i></div>"
       "<div class='sub' id='bs' style='margin-top:8px'>idle</div></div>";
  s += "<div class='sect'><h2>Reboot</h2>"
       "<button class='btn warn' type='button' onclick='RB()'>Reboot</button></div>";

  s += "<script>";
  s += "function RB(){if(!confirm('Reboot?'))return;F('/api/reboot','').then(()=>{"
       "T('Rebooting...','ok');setTimeout(()=>location.href='/',5000);});}"
       "function U(){G('/api/sys').then(d=>{if(!d)return;"
       "var e=document.getElementById('info');"
       "e.innerHTML='';"
       "var rows=[['Firmware',d.fw],['Build',d.build],['Uptime',fmtup(d.uptime)],"
       "['Boot count',d.boots],['Free heap',Math.floor(d.heap/1024)+' KB'],"
       "['Min heap',Math.floor(d.heapmin/1024)+' KB'],['Total heap',Math.floor(d.heaptot/1024)+' KB'],"
       "['Chip',d.chip+' rev '+d.rev],['CPU',d.cpu+' MHz'],"
       "['Flash',Math.floor(d.flash/1024)+' KB'],['Reset reason',d.rst]];"
       "rows.forEach(r=>{var div=document.createElement('div');div.className='kv';"
       "div.innerHTML='<span class=\"k\">'+r[0]+'</span><span class=\"v\">'+r[1]+'</span>';"
       "e.appendChild(div);});"
       "var b=document.getElementById('bb');"
       "b.className='bar '+(d.hold>=100?'err':d.hold>0?'warn':'');"
       "b.firstElementChild.style.width=(d.hold||0)+'%';"
       "document.getElementById('bs').textContent=d.btn?('holding '+d.hold+'%'):'idle';"
       "});}"
       "U();setInterval(U,2000);";
  s += "</script>";
  s += html_tail();
  return s;
}

/* ------------------------------------------------------------------ */
/* HTTP handlers                                                      */
/* ------------------------------------------------------------------ */
static void h_root()     { g_server.send(200, "text/html; charset=utf-8", page_dashboard()); }
static void h_settings() { g_server.send(200, "text/html; charset=utf-8", page_settings()); }
static void h_wifi()     { g_server.send(200, "text/html; charset=utf-8", page_wifi()); }
static void h_system()   { g_server.send(200, "text/html; charset=utf-8", page_system()); }

static void h_redirect_root() {
  g_server.sendHeader("Location", String("http://") + WiFi.softAPIP().toString() + "/", true);
  g_server.send(302, "text/plain", "");
}

static void h_not_found() {
  String host = g_server.hostHeader();
  if (host.length() != 0 && host == WiFi.softAPIP().toString()) {
    g_server.send(404, "text/plain", "not found");
    return;
  }
  h_redirect_root();
}

static void h_api_data() {
  Snapshot s;
  portENTER_CRITICAL(&g_snap_mux);
  s = g_snap;
  portEXIT_CRITICAL(&g_snap_mux);
  uint64_t uptime_s = (uint64_t)esp_timer_get_time() / 1000000ULL;

  String j;
  j.reserve(320);
  j += F("{\"valid\":");    j += s.valid ? F("true") : F("false");
  j += F(",\"temp\":");     j += String((long)s.temp_x10);
  j += F(",\"hum\":");      j += String((long)s.hum_x10);
  j += F(",\"pres\":");     j += String((long)s.pres_pa);
  j += F(",\"lux\":");      j += String((long)s.lux_x10);
  j += F(",\"mq2\":");      j += String((unsigned)s.mq2_adc);
  j += F(",\"soil\":");     j += String((unsigned)s.soil_adc);
  j += F(",\"rain\":");     j += String((unsigned)s.rain_adc);
  j += F(",\"mq2p\":");     j += String((unsigned)s.mq2_pct);
  j += F(",\"soilp\":");    j += String((unsigned)s.soil_pct);
  j += F(",\"rainp\":");    j += String((unsigned)s.rain_pct);
  j += F(",\"bme\":");      j += s.bme_ok ? F("true") : F("false");
  j += F(",\"bh\":");       j += s.bh_ok ? F("true") : F("false");
  j += F(",\"relay\":");    j += s.relay_on ? F("true") : F("false");
  j += F(",\"fault\":");    j += String((unsigned)s.adc_fault);
  j += F(",\"uptime\":");   j += String((unsigned long)uptime_s);
  j += F("}");
  g_server.sendHeader("Cache-Control", "no-store");
  g_server.send(200, "application/json", j);
}

static bool parse_u32(const String& s, uint32_t& out) {
  if (s.length() == 0 || s.length() > 12) return false;
  uint64_t v = 0;
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c < '0' || c > '9') return false;
    v = v * 10ULL + (uint64_t)(c - '0');
    if (v > 0x7FFFFFFFULL) return false;
  }
  out = (uint32_t)v;
  return true;
}

static bool parse_i32(const String& s, int32_t& out) {
  if (s.length() == 0 || s.length() > 12) return false;
  bool neg = false;
  size_t i = 0;
  if (s[0] == '-' || s[0] == '+') {
    neg = (s[0] == '-');
    i = 1;
    if (s.length() == 1) return false;
  }
  uint64_t v = 0;
  for (; i < s.length(); i++) {
    char c = s[i];
    if (c < '0' || c > '9') return false;
    v = v * 10ULL + (uint64_t)(c - '0');
    if (v > 0x7FFFFFFFULL) return false;
  }
  out = (int32_t)(neg ? -(int64_t)v : (int64_t)v);
  return true;
}

static void json_escape_into(String& out, const char* s) {
  for (; *s; ++s) {
    char c = *s;
    if (c == '"' || c == '\\') { out += '\\'; out += c; }
    else if ((uint8_t)c < 0x20) {
      char buf[7];
      snprintf(buf, sizeof(buf), "\\u%04x", (unsigned)c);
      out += buf;
    } else out += c;
  }
}

static void h_api_settings() {
  if (g_server.method() == HTTP_GET) {
    Config c;
    portENTER_CRITICAL(&g_cfg_mux);
    c = g_cfg;
    portEXIT_CRITICAL(&g_cfg_mux);
    String j;
    j.reserve(600);
    j += F("{\"soil_dry\":"); j += String((unsigned long)c.soil_dry);
    j += F(",\"soil_wet\":"); j += String((unsigned long)c.soil_wet);
    j += F(",\"mq2_clean\":"); j += String((unsigned long)c.mq2_clean);
    j += F(",\"mq2_dirty\":"); j += String((unsigned long)c.mq2_dirty);
    j += F(",\"rain_dry\":"); j += String((unsigned long)c.rain_dry);
    j += F(",\"rain_wet\":"); j += String((unsigned long)c.rain_wet);
    j += F(",\"temp_off\":"); j += String((long)c.temp_off_x10);
    j += F(",\"hum_off\":"); j += String((long)c.hum_off_x10);
    j += F(",\"pres_off\":"); j += String((long)c.pres_off_pa);
    j += F(",\"lux_off\":"); j += String((long)c.lux_off_x10);
    j += F(",\"relay_mode\":"); j += String((unsigned long)c.relay_mode);
    j += F(",\"relay_rule\":"); j += String((unsigned long)c.relay_rule);
    j += F(",\"relay_thr\":"); j += String((unsigned long)c.relay_threshold);
    j += F(",\"relay_rain\":"); j += String((unsigned long)c.relay_rain_pct);
    j += F(",\"relay_ms\":"); j += String((unsigned long)c.relay_min_on_ms);
    j += F(",\"relay_inv\":"); j += String((unsigned long)c.relay_invert);
    j += F(",\"sample_ms\":"); j += String((unsigned long)c.sample_interval_ms);
    j += F("}");
    g_server.sendHeader("Cache-Control", "no-store");
    g_server.send(200, "application/json", j);
    return;
  }
  if (g_server.method() != HTTP_POST) { g_server.send(405, "text/plain", ""); return; }

  Config n;
  portENTER_CRITICAL(&g_cfg_mux);
  n = g_cfg;
  portEXIT_CRITICAL(&g_cfg_mux);

  uint32_t u; int32_t i;
  String bad;
  auto PU = [&](const char* k, uint32_t& dst) {
    if (!g_server.hasArg(k)) return;
    if (parse_u32(g_server.arg(k), u)) dst = u;
    else { if (bad.length()) bad += ','; bad += k; }
  };
  auto PI = [&](const char* k, int32_t& dst) {
    if (!g_server.hasArg(k)) return;
    if (parse_i32(g_server.arg(k), i)) dst = i;
    else { if (bad.length()) bad += ','; bad += k; }
  };

  PU("soil_dry",   n.soil_dry);
  PU("soil_wet",   n.soil_wet);
  PU("mq2_clean",  n.mq2_clean);
  PU("mq2_dirty",  n.mq2_dirty);
  PU("rain_dry",   n.rain_dry);
  PU("rain_wet",   n.rain_wet);
  PI("temp_off",   n.temp_off_x10);
  PI("hum_off",    n.hum_off_x10);
  PI("pres_off",   n.pres_off_pa);
  PI("lux_off",    n.lux_off_x10);
  PU("relay_mode", n.relay_mode);
  PU("relay_rule", n.relay_rule);
  PU("relay_thr",  n.relay_threshold);
  PU("relay_rain", n.relay_rain_pct);
  PU("relay_ms",   n.relay_min_on_ms);
  PU("relay_inv",  n.relay_invert);
  PU("sample_ms",  n.sample_interval_ms);

  cfg_sanitize(n);

  if (bad.length()) {
    String j = F("{\"ok\":false,\"err\":\"bad_field\",\"fields\":\"");
    j += bad;
    j += F("\"}");
    g_server.sendHeader("Cache-Control", "no-store");
    g_server.send(200, "application/json", j);
    return;
  }

  portENTER_CRITICAL(&g_cfg_mux);
  g_cfg = n;
  portEXIT_CRITICAL(&g_cfg_mux);
  nvs_mark_dirty();

  uint64_t now_ms = millis64();
  if (g_last_save_ms == 0 || (now_ms - g_last_save_ms) >= NVS_SAVE_USER_COOLDOWN_MS) {
    bool saved = nvs_save();
    g_server.sendHeader("Cache-Control", "no-store");
    if (saved) g_server.send(200, "application/json", "{\"ok\":true}");
    else       g_server.send(200, "application/json", "{\"ok\":false,\"err\":\"nvs\"}");
  } else {
    g_server.sendHeader("Cache-Control", "no-store");
    g_server.send(200, "application/json", "{\"ok\":true,\"deferred\":true}");
  }
}

static void h_api_wifi() {
  if (g_server.method() == HTTP_GET) {
    Config c;
    portENTER_CRITICAL(&g_cfg_mux);
    c = g_cfg;
    portEXIT_CRITICAL(&g_cfg_mux);
    String j;
    j.reserve(120);
    j += F("{\"ssid\":\"");
    json_escape_into(j, c.ap_ssid);
    j += F("\",\"pass\":\"");
    json_escape_into(j, c.ap_pass);
    j += F("\"}");
    g_server.sendHeader("Cache-Control", "no-store");
    g_server.send(200, "application/json", j);
    return;
  }
  if (g_server.method() != HTTP_POST) { g_server.send(405, "text/plain", ""); return; }

  String ssid = g_server.arg("ssid");
  String pass = g_server.arg("pass");
  if (ssid.length() < 1 || ssid.length() > 32) {
    g_server.send(200, "application/json", "{\"ok\":false,\"err\":\"ssid_len\"}");
    return;
  }
  if (pass.length() < 8 || pass.length() > 63) {
    g_server.send(200, "application/json", "{\"ok\":false,\"err\":\"pass_len\"}");
    return;
  }

  portENTER_CRITICAL(&g_cfg_mux);
  memset(g_cfg.ap_ssid, 0, sizeof(g_cfg.ap_ssid));
  memset(g_cfg.ap_pass, 0, sizeof(g_cfg.ap_pass));
  strncpy(g_cfg.ap_ssid, ssid.c_str(), sizeof(g_cfg.ap_ssid) - 1);
  strncpy(g_cfg.ap_pass, pass.c_str(), sizeof(g_cfg.ap_pass) - 1);
  portEXIT_CRITICAL(&g_cfg_mux);

  nvs_mark_dirty();
  bool saved = nvs_save();
  g_server.sendHeader("Cache-Control", "no-store");
  if (saved) {
    g_server.send(200, "application/json", "{\"ok\":true}");
    delay(REBOOT_FLUSH_MS);
    g_reboot_req = true;
  } else {
    g_server.send(200, "application/json", "{\"ok\":false,\"err\":\"nvs\"}");
  }
}

static void h_api_clients() {
  wifi_sta_list_t list;
  memset(&list, 0, sizeof(list));
  esp_wifi_ap_get_sta_list(&list);

  String j;
  j.reserve(800);
  j += F("{\"clients\":[");
  uint32_t count = list.num;
  if (count > 8) count = 8;
  for (uint32_t i = 0; i < count; i++) {
    if (i) j += F(",");
    const uint8_t* mac = list.sta[i].mac;
    char macbuf[18];
    snprintf(macbuf, sizeof(macbuf), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    j += F("{\"mac\":\""); j += macbuf;
    j += F("\",\"rssi\":"); j += String((int)list.sta[i].rssi);
    j += F("}");
  }
  j += F("]}");
  g_server.sendHeader("Cache-Control", "no-store");
  g_server.send(200, "application/json", j);
}

static void h_api_relay() {
  if (g_server.method() != HTTP_POST) { g_server.send(405, "text/plain", ""); return; }
  String a = g_server.arg("a");
  uint32_t mode;
  if (a == "on")           mode = 2;
  else if (a == "off")     mode = 3;
  else if (a == "auto")    mode = 1;
  else if (a == "disable") mode = 0;
  else { g_server.send(200, "application/json", "{\"ok\":false,\"err\":\"bad\"}"); return; }

  Config cfg;
  portENTER_CRITICAL(&g_cfg_mux);
  g_cfg.relay_mode = mode;
  cfg = g_cfg;
  portEXIT_CRITICAL(&g_cfg_mux);
  nvs_mark_dirty();

  Snapshot snap;
  portENTER_CRITICAL(&g_snap_mux);
  snap = g_snap;
  portEXIT_CRITICAL(&g_snap_mux);

  bool snap_ok = true;
  if (cfg.relay_mode == 1) {
    if (cfg.relay_rule == 0 || cfg.relay_rule == 1) snap_ok = !(snap.adc_fault & 1);
    else if (cfg.relay_rule == 2)                  snap_ok = !(snap.adc_fault & 2);
    else if (cfg.relay_rule == 3)                  snap_ok = !(snap.adc_fault & 4);
  }
  relay_update(cfg, snap.soil_adc, snap.mq2_adc, snap.rain_pct, snap_ok);

  g_server.sendHeader("Cache-Control", "no-store");
  g_server.send(200, "application/json", "{\"ok\":true}");
}

static void h_api_factory() {
  g_server.sendHeader("Cache-Control", "no-store");
  g_server.send(200, "application/json", "{\"ok\":true}");
  delay(REBOOT_FLUSH_MS);
  g_factory_req = true;
}

static void h_api_reboot() {
  g_server.sendHeader("Cache-Control", "no-store");
  g_server.send(200, "application/json", "{\"ok\":true}");
  delay(REBOOT_FLUSH_MS);
  g_reboot_req = true;
}

static void h_api_sys() {
  uint32_t hold = btn_hold_pct();
  uint64_t up_s = (uint64_t)esp_timer_get_time() / 1000000ULL;
  String j;
  j.reserve(640);
  j += F("{\"fw\":\"" FW_VERSION "\"");
  j += F(",\"build\":\"" FW_BUILD "\"");
  j += F(",\"uptime\":"); j += String((unsigned long)up_s);
  j += F(",\"boots\":");  j += String((unsigned long)g_boot_count);
  j += F(",\"saves\":");  j += String((unsigned long)g_save_count);
  j += F(",\"heap\":");   j += String((unsigned long)ESP.getFreeHeap());
  j += F(",\"heapmin\":"); j += String((unsigned long)ESP.getMinFreeHeap());
  j += F(",\"heaptot\":"); j += String((unsigned long)ESP.getHeapSize());
  j += F(",\"flash\":");  j += String((unsigned long)ESP.getFlashChipSize());
  j += F(",\"chip\":\""); j += ESP.getChipModel(); j += F("\"");
  j += F(",\"rev\":");    j += String((unsigned)ESP.getChipRevision());
  j += F(",\"cpu\":");    j += String((unsigned long)ESP.getCpuFreqMHz());
  j += F(",\"rst\":");    j += String((unsigned)esp_reset_reason());
  j += F(",\"btn\":");    j += g_btn_armed ? F("true") : F("false");
  j += F(",\"hold\":");   j += String(hold);
  j += F("}");
  g_server.sendHeader("Cache-Control", "no-store");
  g_server.send(200, "application/json", j);
}

/* ------------------------------------------------------------------ */
/* AP / DNS / routes                                                  */
/* ------------------------------------------------------------------ */
static void ap_start() {
  Config c;
  portENTER_CRITICAL(&g_cfg_mux);
  c = g_cfg;
  portEXIT_CRITICAL(&g_cfg_mux);

  char ssid[33], pass[65];
  memcpy(ssid, c.ap_ssid, sizeof(ssid)); ssid[32] = 0;
  memcpy(pass, c.ap_pass, sizeof(pass)); pass[64] = 0;

  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  WiFi.softAPsetHostname("esp32-msens");
  WiFi.softAP(ssid, pass, AP_CHANNEL, AP_HIDDEN, AP_MAX_CLIENTS);
  delay(50);

  if (!g_dns_up) {
    g_dns.setErrorReplyCode(DNSReplyCode::NoError);
    g_dns.start(53, "*", WiFi.softAPIP());
    g_dns_up = true;
  }
}

static void register_routes() {
  g_server.on("/", HTTP_GET, h_root);
  g_server.on("/settings", HTTP_GET, h_settings);
  g_server.on("/wifi", HTTP_GET, h_wifi);
  g_server.on("/system", HTTP_GET, h_system);

  g_server.on("/api/data", HTTP_GET, h_api_data);
  g_server.on("/api/settings", HTTP_ANY, h_api_settings);
  g_server.on("/api/wifi", HTTP_ANY, h_api_wifi);
  g_server.on("/api/clients", HTTP_GET, h_api_clients);
  g_server.on("/api/relay", HTTP_POST, h_api_relay);
  g_server.on("/api/factory", HTTP_POST, h_api_factory);
  g_server.on("/api/reboot", HTTP_POST, h_api_reboot);
  g_server.on("/api/sys", HTTP_GET, h_api_sys);

  g_server.on("/generate_204", HTTP_GET, h_redirect_root);
  g_server.on("/gen_204", HTTP_GET, h_redirect_root);
  g_server.on("/hotspot-detect.html", HTTP_GET, h_redirect_root);
  g_server.on("/connecttest.txt", HTTP_GET, h_redirect_root);
  g_server.on("/ncsi.txt", HTTP_GET, h_redirect_root);
  g_server.on("/redirect", HTTP_GET, h_redirect_root);
  g_server.on("/canonical.html", HTTP_GET, h_redirect_root);
  g_server.on("/success.txt", HTTP_GET, h_redirect_root);
  g_server.on("/favicon.ico", HTTP_GET, []() { g_server.send(204, "text/plain", ""); });

  g_server.onNotFound(h_not_found);
}

/* ------------------------------------------------------------------ */
/* Reset                                                              */
/* ------------------------------------------------------------------ */
static void do_factory_reset() {
  if (g_dns_up) { g_dns.stop(); g_dns_up = false; }
  g_server.stop();
  nvs_wipe();
  delay(200);
  ESP.restart();
}

static void do_reboot() {
  bool dirty;
  portENTER_CRITICAL(&g_cfg_mux);
  dirty = g_cfg_dirty;
  portEXIT_CRITICAL(&g_cfg_mux);
  if (dirty) nvs_save();
  if (g_dns_up) { g_dns.stop(); g_dns_up = false; }
  g_server.stop();
  delay(200);
  ESP.restart();
}

/* ------------------------------------------------------------------ */
/* Setup / Loop                                                       */
/* ------------------------------------------------------------------ */
void setup() {
  pinMode(PIN_RELAY, OUTPUT);
  {
    Preferences p;
    uint32_t inv = 0;
    if (p.begin(NVS_NAMESPACE, true)) {
      Config tmp;
      if (p.getBytesLength(NVS_KEY_BLOB) == sizeof(Config) &&
          p.getBytes(NVS_KEY_BLOB, &tmp, sizeof(tmp)) == sizeof(tmp) &&
          tmp.magic == CFG_MAGIC && tmp.schema == CFG_SCHEMA) {
        inv = tmp.relay_invert & 1U;
      }
      p.end();
    }
    digitalWrite(PIN_RELAY, inv ? HIGH : LOW);
  }

  pinMode(PIN_BOOT, INPUT_PULLUP);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  pinMode(PIN_MQ2,  INPUT_PULLUP);
  pinMode(PIN_SOIL, INPUT_PULLUP);
  pinMode(PIN_RAIN, INPUT_PULLUP);

  bool loaded = nvs_load();

  digitalWrite(PIN_RELAY, g_cfg.relay_invert ? HIGH : LOW);

  i2c_init();
  g_bme_ok = bme_bringup();
  g_bh_ok  = bh_bringup();

  g_last_sample_us = 0;

  sample_tick();

  ap_start();
  register_routes();
  g_server.begin();
  g_server.enableCORS(true);

  if (!loaded) {
    nvs_mark_dirty();
  }
}

void loop() {
  btn_tick();

  if (g_factory_req) {
    g_factory_req = false;
    do_factory_reset();
  }

  if (g_reboot_req) {
    g_reboot_req = false;
    do_reboot();
  }

  sample_tick();
  nvs_tick();

  if (g_dns_up) g_dns.processNextRequest();
  g_server.handleClient();

  delay(1);
}
