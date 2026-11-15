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
#include <esp_log.h>
#include <nvs_flash.h>
#include <math.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>

static const uint8_t PIN_SDA = 21;
static const uint8_t PIN_SCL = 22;
static const uint8_t PIN_MQ2 = 32;
static const uint8_t PIN_SOIL = 34;
static const uint8_t PIN_RAIN = 35;
static const uint8_t PIN_RELAY = 25;
static const uint8_t PIN_BOOT = 0;

static const char* const DEFAULT_SSID = "ESP32_Multi_Sensors";
static const char* const DEFAULT_PASSWORD = "12345678";
static const char* const AP_URL = "http://192.168.4.1/";

static const char* const NVS_NS_A = "ms_cfg_a";
static const char* const NVS_NS_B = "ms_cfg_b";
static const char* const NVS_CFG_KEY = "config";
static const uint32_t CFG_MAGIC = 0x4D534332UL;
static const uint16_t CFG_VERSION = 1;

struct __attribute__((packed)) WiFiConfig {
  char ssid[32];
  char password[64];
  uint8_t channel;
  uint8_t hidden;
  uint8_t maxClients;
  uint8_t reserved;
};

struct __attribute__((packed)) BmeConfig {
  uint8_t enabled;
  uint8_t reserved[7];
  uint64_t intervalMs;
  int32_t tempOffsetX100;
  int32_t humOffsetX100;
  int32_t pressureOffsetPa;
};

struct __attribute__((packed)) BhConfig {
  uint8_t enabled;
  uint8_t reserved[7];
  uint64_t intervalMs;
  int32_t luxScaleX1000;
  int32_t luxOffset;
};

struct __attribute__((packed)) MqConfig {
  uint8_t enabled;
  uint8_t alarmEnabled;
  uint8_t reserved[6];
  uint64_t intervalMs;
  uint64_t warmupMs;
  uint16_t samples;
  uint16_t alarmRaw;
  uint16_t hysteresisRaw;
  uint16_t reserved2;
};

struct __attribute__((packed)) SoilConfig {
  uint8_t enabled;
  uint8_t autoRelay;
  uint8_t reserved[6];
  uint64_t intervalMs;
  uint16_t dryRaw;
  uint16_t wetRaw;
  int16_t offsetPct;
  uint8_t startPct;
  uint8_t stopPct;
  uint16_t reserved2;
};

struct __attribute__((packed)) RainConfig {
  uint8_t enabled;
  uint8_t inhibitRelay;
  uint8_t reserved[6];
  uint64_t intervalMs;
  uint16_t dryRaw;
  uint16_t wetRaw;
  int16_t offsetPct;
  uint8_t alertPct;
  uint8_t inhibitPct;
  uint16_t reserved2;
};

struct __attribute__((packed)) RelayConfig {
  uint8_t enabled;
  uint8_t activeLow;
  uint8_t mode;
  uint8_t reserved0;
  uint64_t minSwitchMs;
  uint64_t maxOnMs;
  uint32_t reserved1;
};

struct __attribute__((packed)) AppConfig {
  uint32_t magic;
  uint16_t version;
  uint16_t reserved0;
  uint64_t sequence;
  uint32_t crc;
  uint32_t reserved1;
  uint64_t flags;
  WiFiConfig wifi;
  BmeConfig bme;
  BhConfig bh;
  MqConfig mq;
  SoilConfig soil;
  RainConfig rain;
  RelayConfig relay;
};

struct BmeState {
  bool present;
  bool ok;
  bool hasRead;
  uint64_t nextAttempt;
  uint64_t lastRead;
  float t;
  float h;
  float p;
};

struct BhState {
  bool present;
  bool ok;
  bool hasRead;
  uint64_t nextAttempt;
  uint64_t lastRead;
  float lux;
};

struct MqState {
  bool ok;
  bool hasRead;
  uint64_t lastRead;
  bool ready;
  bool alarm;
  uint16_t raw;
};

struct SoilState {
  bool ok;
  bool hasRead;
  uint64_t lastRead;
  uint16_t raw;
  uint8_t pct;
};

struct RainState {
  bool ok;
  bool hasRead;
  uint64_t lastRead;
  uint16_t raw;
  uint8_t pct;
  bool alert;
  bool inhibit;
};

static AppConfig gConfig;
static AppConfig gLastSaved;
static bool gLastSavedValid = false;
static uint8_t gLastSlot = 1;

static uint64_t gBootMs = 0;
static bool gApPending = false;
static uint64_t gApPendingAt = 0;
static bool gFactoryResetStarted = false;

static bool gRelayOn = false;
static bool gRelaySafety = false;
static uint64_t gRelayOnSince = 0;
static uint64_t gLastRelayChange = 0;
static uint64_t gRelayLastRun = 0;

static BmeState gBme = {};
static BhState gBh = {};
static MqState gMq = {};
static SoilState gSoil = {};
static RainState gRain = {};

static Adafruit_BME280 gBmeSensor;
static BH1750 gBhSensor(0x23);
static WebServer gServer(80);
static DNSServer gDns;

struct LogSilencer {
  LogSilencer() {
    esp_log_level_set("*", ESP_LOG_NONE);
  }
};
static LogSilencer gLogSilencer;

const char HTML_HEAD[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<meta name="color-scheme" content="dark light">
<title>ESP32 Multi Sensors</title>
<style>
:root{color-scheme:dark light;--bg:#0b1220;--card:#111c2e;--card2:#0f172a;--fg:#e5eefb;--muted:#94a3b8;--border:#243449;--accent:#38bdf8;--accent2:#0ea5e9;--danger:#ef4444;--ok:#22c55e;--warn:#f59e0b;--input:#0b1626}
@media (prefers-color-scheme:light){:root{--bg:#f1f5f9;--card:#fff;--card2:#f8fafc;--fg:#0f172a;--muted:#475569;--border:#cbd5e1;--accent:#0284c7;--accent2:#0369a1;--input:#fff}}
*{box-sizing:border-box}
html,body{margin:0;padding:0}
body{background:var(--bg);color:var(--fg);font-family:system-ui,-apple-system,"Segoe UI",Roboto,Helvetica,Arial,sans-serif;line-height:1.45;min-height:100vh}
a{color:var(--accent);text-decoration:none}
a:hover{text-decoration:underline}
header{padding:16px 16px 8px;max-width:1200px;margin:0 auto}
header h1{margin:0;font-size:clamp(1.4rem,4vw,2.1rem);letter-spacing:-.02em}
header p{margin:4px 0 0;color:var(--muted)}
nav{display:flex;gap:8px;overflow-x:auto;padding:8px 16px 12px;max-width:1200px;margin:0 auto;scrollbar-width:thin}
nav a{flex:0 0 auto;padding:8px 12px;border:1px solid var(--border);border-radius:999px;background:var(--card);color:var(--fg);font-size:.9rem;white-space:nowrap}
nav a:hover{border-color:var(--accent);text-decoration:none}
main{max-width:1200px;margin:0 auto;padding:0 16px 32px}
h1,h2,h3{line-height:1.2}
h1{font-size:clamp(1.5rem,4vw,2rem);margin:8px 0 16px}
h2{font-size:1rem;margin:0 0 6px;color:var(--muted);font-weight:600;text-transform:uppercase;letter-spacing:.06em}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(210px,1fr));gap:16px}
.card{background:var(--card);border:1px solid var(--border);border-radius:16px;padding:16px;box-shadow:0 1px 0 rgba(255,255,255,.03) inset}
.value{font-size:clamp(1.7rem,5vw,2.5rem);font-weight:750;letter-spacing:-.03em;line-height:1.1}
.unit{font-size:.9rem;color:var(--muted);font-weight:500;margin-left:2px}
.state{font-size:.85rem;color:var(--muted);margin-top:6px}
.state.ok{color:var(--ok)}
.state.warn{color:var(--warn)}
.state.err{color:var(--danger)}
.status-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(140px,1fr));gap:12px;margin-bottom:16px}
.status-grid .card{padding:12px 14px}
.status-label{font-size:.75rem;text-transform:uppercase;letter-spacing:.08em;color:var(--muted)}
.status-value{font-size:1.05rem;font-weight:650;margin-top:2px;word-break:break-word}
.msg{padding:12px 14px;border-radius:12px;margin:0 0 16px;font-weight:600}
.msg.ok{background:rgba(34,197,94,.15);border:1px solid rgba(34,197,94,.55);color:var(--ok)}
.msg.err{background:rgba(239,68,68,.15);border:1px solid rgba(239,68,68,.55);color:var(--danger)}
.msg.warn{background:rgba(245,158,11,.15);border:1px solid rgba(245,158,11,.55);color:var(--warn)}
.btn{display:inline-flex;align-items:center;justify-content:center;gap:6px;min-height:42px;padding:10px 16px;border-radius:10px;border:1px solid var(--border);background:var(--card2);color:var(--fg);font:inherit;font-weight:650;cursor:pointer;text-decoration:none}
.btn:hover{border-color:var(--accent);text-decoration:none}
.btn.primary{background:var(--accent);border-color:var(--accent);color:#03111d}
.btn.danger{background:var(--danger);border-color:var(--danger);color:#fff}
.btn:disabled{opacity:.45;cursor:not-allowed}
.btn-row{display:flex;gap:8px;flex-wrap:wrap;margin-top:12px}
label{display:block;margin:14px 0 6px;font-weight:650}
label.check{display:flex;align-items:center;gap:10px;margin:16px 0;font-weight:600}
label.check input{width:auto;margin:0}
input,select{width:100%;padding:11px 12px;border-radius:10px;border:1px solid var(--border);background:var(--input);color:var(--fg);font:inherit}
input:focus,select:focus{outline:2px solid var(--accent);outline-offset:1px}
input[type="checkbox"]{accent-color:var(--accent)}
form{max-width:720px}
.form-actions{display:flex;gap:8px;flex-wrap:wrap;margin-top:20px}
.muted{color:var(--muted)}
code{background:var(--card2);border:1px solid var(--border);border-radius:6px;padding:2px 5px;font-size:.9em}
footer{max-width:1200px;margin:0 auto;padding:20px 16px 32px;color:var(--muted);font-size:.85rem;border-top:1px solid var(--border)}
@media (max-width:560px){header{padding-top:12px}main{padding-left:12px;padding-right:12px}nav{padding-left:12px;padding-right:12px}.card{padding:14px}.btn{flex:1 1 auto}}
</style>
</head>
<body>
<header>
<h1>ESP32 Multi Sensors</h1>
<p>Environmental and irrigation controller</p>
</header>
<nav>
<a href="/">Dashboard</a>
<a href="/settings/bme280">BME280</a>
<a href="/settings/bh1750">BH1750</a>
<a href="/settings/mq2">MQ-2</a>
<a href="/settings/soil">Soil</a>
<a href="/settings/rain">Rain</a>
<a href="/settings/relay">Relay</a>
<a href="/settings/wifi">WiFi AP</a>
</nav>
<main>
)HTML";

const char HTML_FOOT[] PROGMEM = R"HTML(
</main>
<footer>ESP32 Multi Sensors • AP 192.168.4.1</footer>
</body>
</html>
)HTML";

const char HTML_DASH[] PROGMEM = R"HTML(
<h1>Dashboard</h1>
<div id="alertBar" class="msg err" style="display:none"><span id="alertText"></span></div>
<div class="status-grid">
<div class="card"><div class="status-label">Network</div><div class="status-value" id="netStatus">Connecting</div></div>
<div class="card"><div class="status-label">Uptime</div><div class="status-value" id="uptime">--</div></div>
<div class="card"><div class="status-label">Free Heap</div><div class="status-value" id="heap">--</div></div>
<div class="card"><div class="status-label">AP Clients</div><div class="status-value" id="clients">--</div></div>
<div class="card"><div class="status-label">AP IP</div><div class="status-value" id="ip">--</div></div>
</div>
<div class="grid">
<section class="card"><h2>Temperature</h2><div class="value"><span id="temp">--</span><span class="unit">°C</span></div><div class="state" id="bmeState">Waiting</div></section>
<section class="card"><h2>Humidity</h2><div class="value"><span id="hum">--</span><span class="unit">%</span></div><div class="state" id="bmeState2">BME280</div></section>
<section class="card"><h2>Pressure</h2><div class="value"><span id="press">--</span><span class="unit">hPa</span></div><div class="state" id="bmeState3">BME280</div></section>
<section class="card"><h2>Light</h2><div class="value"><span id="lux">--</span><span class="unit">lx</span></div><div class="state" id="bhState">Waiting</div></section>
<section class="card"><h2>MQ-2 Gas</h2><div class="value"><span id="mqRaw">--</span><span class="unit">ADC</span></div><div class="state" id="mqState">Waiting</div></section>
<section class="card"><h2>Soil Moisture</h2><div class="value"><span id="soilPct">--</span><span class="unit">%</span></div><div class="state" id="soilState">Waiting</div></section>
<section class="card"><h2>Rain Wetness</h2><div class="value"><span id="rainPct">--</span><span class="unit">%</span></div><div class="state" id="rainState">Waiting</div></section>
</div>
<section class="card" style="margin-top:16px">
<h2>Relay Control</h2>
<div class="value" id="relayState">--</div>
<div class="state" id="relayMode">--</div>
<div class="state" id="relaySafety">--</div>
<div class="btn-row">
<button type="button" class="btn" id="relayAuto">Auto</button>
<button type="button" class="btn" id="relayOff">Force Off</button>
<button type="button" class="btn danger" id="relayOn">Force On</button>
</div>
</section>
<script>
(function(){
var $=function(id){return document.getElementById(id);};
var relayBusy=false;
function fmt(v,d){if(typeof v!=='number'||!isFinite(v))return '--';return v.toFixed(d);}
function fmtUptime(ms){if(typeof ms!=='number'||!isFinite(ms)||ms<0)return '--';var s=Math.floor(ms/1000);var d=Math.floor(s/86400);s%=86400;var h=Math.floor(s/3600);s%=3600;var m=Math.floor(s/60);s%=60;return (d>0?d+'d ':'')+String(h).padStart(2,'0')+':'+String(m).padStart(2,'0')+':'+String(s).padStart(2,'0');}
function setState(id,text,cls){var e=$(id);if(!e)return;e.textContent=text;e.className='state'+(cls?' '+cls:'');}
function refresh(){
return fetch('/api/data',{cache:'no-store'}).then(function(r){if(!r.ok)throw new Error('http');return r.json();}).then(function(d){
$('netStatus').textContent='Online';
$('uptime').textContent=fmtUptime(d.uptime_ms);
$('heap').textContent=(Math.round(d.heap/1024))+' KB';
$('clients').textContent=String(d.clients);
$('ip').textContent=d.ip;
var b=d.bme||{};var bh=d.bh||{};var mq=d.mq||{};var soil=d.soil||{};var rain=d.rain||{};var relay=d.relay||{};var alerts=d.alerts||{};
if(!b.en){setState('bmeState','Disabled','');$('temp').textContent='--';$('hum').textContent='--';$('press').textContent='--';}
else if(!b.ok){setState('bmeState','Sensor unavailable','err');$('temp').textContent='--';$('hum').textContent='--';$('press').textContent='--';}
else{setState('bmeState','OK','ok');$('temp').textContent=fmt(b.t,2);$('hum').textContent=fmt(b.h,2);$('press').textContent=fmt(b.p,1);}
$('bmeState2').textContent=b.en?(b.ok?'BME280 OK':'BME280 unavailable'):'BME280 disabled';
$('bmeState3').textContent=b.en?(b.ok?'BME280 OK':'BME280 unavailable'):'BME280 disabled';
if(!bh.en){setState('bhState','Disabled','');$('lux').textContent='--';}
else if(!bh.ok){setState('bhState','Sensor unavailable','err');$('lux').textContent='--';}
else{setState('bhState','OK','ok');$('lux').textContent=fmt(bh.lux,0);}
if(!mq.en){setState('mqState','Disabled','');$('mqRaw').textContent='--';}
else if(!mq.ok){setState('mqState','No reading','err');$('mqRaw').textContent='--';}
else{$('mqRaw').textContent=String(mq.raw);setState('mqState',mq.alarm?'Gas alarm':(mq.ready?'Normal':'Warming up'),mq.alarm?'err':(mq.ready?'ok':'warn'));}
if(!soil.en){setState('soilState','Disabled','');$('soilPct').textContent='--';}
else if(!soil.ok){setState('soilState','No reading','err');$('soilPct').textContent='--';}
else{$('soilPct').textContent=String(soil.pct);setState('soilState','OK','ok');}
if(!rain.en){setState('rainState','Disabled','');$('rainPct').textContent='--';}
else if(!rain.ok){setState('rainState','No reading','err');$('rainPct').textContent='--';}
else{$('rainPct').textContent=String(rain.pct);setState('rainState',rain.alert?'Rain alert':(rain.inhibit?'Inhibit active':'Normal'),rain.alert?'err':(rain.inhibit?'warn':'ok'));}
$('relayState').textContent=relay.on?'ON':'OFF';
$('relayMode').textContent=['Auto','Force Off','Force On'][relay.mode]||'Unknown';
$('relaySafety').textContent=relay.safety?'LOCKED OUT':'Normal';
$('relaySafety').className=relay.safety?'state err':'state ok';
$('relayAuto').disabled=!relay.enabled;
$('relayOff').disabled=!relay.enabled;
$('relayOn').disabled=!relay.enabled;
var list=[];
if(alerts.mq)list.push('MQ-2 gas alarm');
if(alerts.soil_dry)list.push('Soil is dry');
if(alerts.rain)list.push('Rain detected');
if(alerts.relay_safety)list.push('Relay safety lockout');
if(rain.inhibit&&!alerts.rain)list.push('Rain inhibit active');
var bar=$('alertBar');
if(list.length){bar.style.display='block';$('alertText').textContent=list.join(' • ');}
else{bar.style.display='none';$('alertText').textContent='';}
}).catch(function(){$('netStatus').textContent='Offline';});
}
function setRelay(mode){
if(relayBusy)return Promise.resolve();
relayBusy=true;
return fetch('/api/relay',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'mode='+encodeURIComponent(mode)}).then(function(r){if(!r.ok)throw new Error('http');return r.json();}).then(function(){return refresh();}).catch(function(){$('netStatus').textContent='Offline';}).finally(function(){relayBusy=false;});
}
$('relayAuto').addEventListener('click',function(){setRelay('auto');});
$('relayOff').addEventListener('click',function(){setRelay('off');});
$('relayOn').addEventListener('click',function(){setRelay('on');});
document.addEventListener('visibilitychange',function(){if(!document.hidden)refresh();});
setInterval(refresh,3000);
refresh();
})();
</script>
)HTML";

static uint64_t nowMs64() {
  int64_t us = esp_timer_get_time();
  if (us < 0) us = 0;
  return (uint64_t)us / 1000ULL;
}

static uint64_t clampU64(uint64_t v, uint64_t lo, uint64_t hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

static int32_t clampI32(int32_t v, int32_t lo, int32_t hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

static uint16_t clampU16(uint16_t v, uint16_t lo, uint16_t hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

static void copyText(char* dst, size_t size, const char* src) {
  if (size == 0) return;
  size_t n = strlen(src);
  if (n >= size) n = size - 1;
  memcpy(dst, src, n);
  dst[n] = 0;
}

static void htmlEscape(const char* src, char* dst, size_t dstSize) {
  size_t o = 0;
  for (size_t i = 0; src[i] != 0 && o + 1 < dstSize; i++) {
    const char* rep = nullptr;
    switch (src[i]) {
      case '&': rep = "&amp;"; break;
      case '<': rep = "&lt;"; break;
      case '>': rep = "&gt;"; break;
      case '"': rep = "&quot;"; break;
      case '\'': rep = "&#39;"; break;
      default: dst[o++] = src[i]; break;
    }
    if (rep != nullptr) {
      size_t n = strlen(rep);
      if (o + n < dstSize) {
        memcpy(dst + o, rep, n);
        o += n;
      }
    }
  }
  dst[o] = 0;
}

static uint32_t crc32Bytes(const uint8_t* data, size_t len) {
  uint32_t crc = 0xFFFFFFFFUL;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      crc = (crc >> 1) ^ (0xEDB88320UL & (0UL - (crc & 1UL)));
    }
  }
  return ~crc;
}

static void setDefaults() {
  memset(&gConfig, 0, sizeof(gConfig));
  gConfig.magic = CFG_MAGIC;
  gConfig.version = CFG_VERSION;

  copyText(gConfig.wifi.ssid, sizeof(gConfig.wifi.ssid), DEFAULT_SSID);
  copyText(gConfig.wifi.password, sizeof(gConfig.wifi.password), DEFAULT_PASSWORD);
  gConfig.wifi.channel = 6;
  gConfig.wifi.hidden = 0;
  gConfig.wifi.maxClients = 4;

  gConfig.bme.enabled = 1;
  gConfig.bme.intervalMs = 5000ULL;
  gConfig.bme.tempOffsetX100 = 0;
  gConfig.bme.humOffsetX100 = 0;
  gConfig.bme.pressureOffsetPa = 0;

  gConfig.bh.enabled = 1;
  gConfig.bh.intervalMs = 5000ULL;
  gConfig.bh.luxScaleX1000 = 1000;
  gConfig.bh.luxOffset = 0;

  gConfig.mq.enabled = 1;
  gConfig.mq.alarmEnabled = 1;
  gConfig.mq.intervalMs = 2000ULL;
  gConfig.mq.warmupMs = 60000ULL;
  gConfig.mq.samples = 8;
  gConfig.mq.alarmRaw = 1800;
  gConfig.mq.hysteresisRaw = 100;

  gConfig.soil.enabled = 1;
  gConfig.soil.autoRelay = 1;
  gConfig.soil.intervalMs = 3000ULL;
  gConfig.soil.dryRaw = 3200;
  gConfig.soil.wetRaw = 1200;
  gConfig.soil.offsetPct = 0;
  gConfig.soil.startPct = 30;
  gConfig.soil.stopPct = 45;

  gConfig.rain.enabled = 1;
  gConfig.rain.inhibitRelay = 1;
  gConfig.rain.intervalMs = 3000ULL;
  gConfig.rain.dryRaw = 3000;
  gConfig.rain.wetRaw = 1000;
  gConfig.rain.offsetPct = 0;
  gConfig.rain.alertPct = 20;
  gConfig.rain.inhibitPct = 25;

  gConfig.relay.enabled = 1;
  gConfig.relay.activeLow = 1;
  gConfig.relay.mode = 0;
  gConfig.relay.minSwitchMs = 30000ULL;
  gConfig.relay.maxOnMs = 1800000ULL;

  gConfig.flags = 0;
  gConfig.sequence = 0;
  gConfig.crc = 0;
}

static bool sanitizeConfig(AppConfig& c) {
  AppConfig before = c;

  c.magic = CFG_MAGIC;
  c.version = CFG_VERSION;
  c.reserved0 = 0;
  c.reserved1 = 0;

  c.wifi.reserved = 0;
  c.wifi.ssid[sizeof(c.wifi.ssid) - 1] = 0;
  size_t sl = strlen(c.wifi.ssid);
  if (sl == 0) {
    copyText(c.wifi.ssid, sizeof(c.wifi.ssid), DEFAULT_SSID);
  } else if (sl > 31) {
    c.wifi.ssid[31] = 0;
  }

  c.wifi.password[sizeof(c.wifi.password) - 1] = 0;
  size_t pl = strlen(c.wifi.password);
  if (pl > 0 && (pl < 8 || pl > 63)) {
    copyText(c.wifi.password, sizeof(c.wifi.password), DEFAULT_PASSWORD);
  }

  if (c.wifi.channel < 1) c.wifi.channel = 1;
  if (c.wifi.channel > 13) c.wifi.channel = 13;
  if (c.wifi.maxClients < 1) c.wifi.maxClients = 1;
  if (c.wifi.maxClients > 8) c.wifi.maxClients = 8;
  c.wifi.hidden = c.wifi.hidden ? 1 : 0;

  c.bme.enabled = c.bme.enabled ? 1 : 0;
  memset(c.bme.reserved, 0, sizeof(c.bme.reserved));
  c.bme.intervalMs = clampU64(c.bme.intervalMs, 1000ULL, 86400000ULL);
  c.bme.tempOffsetX100 = clampI32(c.bme.tempOffsetX100, -2000, 2000);
  c.bme.humOffsetX100 = clampI32(c.bme.humOffsetX100, -5000, 5000);
  c.bme.pressureOffsetPa = clampI32(c.bme.pressureOffsetPa, -10000, 10000);

  c.bh.enabled = c.bh.enabled ? 1 : 0;
  memset(c.bh.reserved, 0, sizeof(c.bh.reserved));
  c.bh.intervalMs = clampU64(c.bh.intervalMs, 1000ULL, 86400000ULL);
  c.bh.luxScaleX1000 = clampI32(c.bh.luxScaleX1000, 1, 100000);
  c.bh.luxOffset = clampI32(c.bh.luxOffset, -200000, 200000);

  c.mq.enabled = c.mq.enabled ? 1 : 0;
  c.mq.alarmEnabled = c.mq.alarmEnabled ? 1 : 0;
  memset(c.mq.reserved, 0, sizeof(c.mq.reserved));
  c.mq.reserved2 = 0;
  c.mq.intervalMs = clampU64(c.mq.intervalMs, 1000ULL, 86400000ULL);
  c.mq.warmupMs = clampU64(c.mq.warmupMs, 0ULL, 86400000ULL);
  c.mq.samples = clampU16(c.mq.samples, 1, 64);
  c.mq.alarmRaw = clampU16(c.mq.alarmRaw, 0, 4095);
  c.mq.hysteresisRaw = clampU16(c.mq.hysteresisRaw, 0, 4095);

  c.soil.enabled = c.soil.enabled ? 1 : 0;
  c.soil.autoRelay = c.soil.autoRelay ? 1 : 0;
  memset(c.soil.reserved, 0, sizeof(c.soil.reserved));
  c.soil.reserved2 = 0;
  c.soil.intervalMs = clampU64(c.soil.intervalMs, 1000ULL, 86400000ULL);
  c.soil.dryRaw = clampU16(c.soil.dryRaw, 0, 4095);
  c.soil.wetRaw = clampU16(c.soil.wetRaw, 0, 4095);
  if (c.soil.dryRaw <= c.soil.wetRaw) {
    if (c.soil.wetRaw < 4095) {
      c.soil.dryRaw = c.soil.wetRaw + 1;
    } else {
      c.soil.dryRaw = 4095;
      c.soil.wetRaw = 4094;
    }
  }
  c.soil.offsetPct = (int16_t)clampI32(c.soil.offsetPct, -100, 100);
  if (c.soil.startPct > 99) c.soil.startPct = 99;
  if (c.soil.stopPct <= c.soil.startPct) c.soil.stopPct = c.soil.startPct + 1;
  if (c.soil.stopPct > 100) c.soil.stopPct = 100;

  c.rain.enabled = c.rain.enabled ? 1 : 0;
  c.rain.inhibitRelay = c.rain.inhibitRelay ? 1 : 0;
  memset(c.rain.reserved, 0, sizeof(c.rain.reserved));
  c.rain.reserved2 = 0;
  c.rain.intervalMs = clampU64(c.rain.intervalMs, 1000ULL, 86400000ULL);
  c.rain.dryRaw = clampU16(c.rain.dryRaw, 0, 4095);
  c.rain.wetRaw = clampU16(c.rain.wetRaw, 0, 4095);
  if (c.rain.dryRaw <= c.rain.wetRaw) {
    if (c.rain.wetRaw < 4095) {
      c.rain.dryRaw = c.rain.wetRaw + 1;
    } else {
      c.rain.dryRaw = 4095;
      c.rain.wetRaw = 4094;
    }
  }
  c.rain.offsetPct = (int16_t)clampI32(c.rain.offsetPct, -100, 100);
  if (c.rain.alertPct > 100) c.rain.alertPct = 100;
  if (c.rain.inhibitPct > 100) c.rain.inhibitPct = 100;

  c.relay.enabled = c.relay.enabled ? 1 : 0;
  c.relay.activeLow = c.relay.activeLow ? 1 : 0;
  if (c.relay.mode > 2) c.relay.mode = 2;
  c.relay.reserved0 = 0;
  c.relay.reserved1 = 0;
  c.relay.minSwitchMs = clampU64(c.relay.minSwitchMs, 2000ULL, 86400000ULL);
  c.relay.maxOnMs = clampU64(c.relay.maxOnMs, 0ULL, 604800000ULL);
  if (c.relay.maxOnMs > 0ULL && c.relay.maxOnMs < 10000ULL) {
    c.relay.maxOnMs = 10000ULL;
  }

  return memcmp(&before, &c, sizeof(c)) != 0;
}

static bool readSlot(const char* ns, AppConfig& out) {
  Preferences p;
  if (!p.begin(ns, true)) return false;
  if (!p.isKey(NVS_CFG_KEY)) {
    p.end();
    return false;
  }
  size_t len = p.getBytesLength(NVS_CFG_KEY);
  if (len != sizeof(out)) {
    p.end();
    return false;
  }
  size_t got = p.getBytes(NVS_CFG_KEY, &out, sizeof(out));
  p.end();
  if (got != sizeof(out)) return false;
  if (out.magic != CFG_MAGIC || out.version != CFG_VERSION) return false;
  uint32_t saved = out.crc;
  out.crc = 0;
  uint32_t calc = crc32Bytes((const uint8_t*)&out, sizeof(out));
  out.crc = saved;
  return calc == saved;
}

static bool saveConfig() {
  sanitizeConfig(gConfig);

  if (gLastSavedValid) {
    AppConfig a = gConfig;
    AppConfig b = gLastSaved;
    a.crc = 0;
    a.sequence = 0;
    b.crc = 0;
    b.sequence = 0;
    if (memcmp(&a, &b, sizeof(AppConfig)) == 0) return false;
  }

  uint64_t seq = gLastSavedValid ? (gLastSaved.sequence + 1ULL) : 1ULL;
  uint8_t slot = gLastSavedValid ? (gLastSlot ^ 1U) : 0U;

  AppConfig out = gConfig;
  out.sequence = seq;
  out.crc = 0;
  out.crc = crc32Bytes((const uint8_t*)&out, sizeof(out));

  Preferences p;
  if (!p.begin(slot == 0 ? NVS_NS_A : NVS_NS_B, false)) return false;
  size_t written = p.putBytes(NVS_CFG_KEY, &out, sizeof(out));
  p.end();
  if (written != sizeof(out)) return false;

  gConfig.sequence = out.sequence;
  gConfig.crc = out.crc;
  gLastSaved = out;
  gLastSavedValid = true;
  gLastSlot = slot;
  return true;
}

static void loadConfig() {
  AppConfig a;
  AppConfig b;
  bool va = readSlot(NVS_NS_A, a);
  bool vb = readSlot(NVS_NS_B, b);

  if (va || vb) {
    bool useA = va && (!vb || a.sequence >= b.sequence);
    gConfig = useA ? a : b;
    gLastSlot = useA ? 0 : 1;
    gLastSaved = gConfig;
    gLastSavedValid = true;
    sanitizeConfig(gConfig);
    saveConfig();
  } else {
    setDefaults();
    gLastSavedValid = false;
    gLastSlot = 1;
    saveConfig();
  }
}

static bool initBme() {
  if (gBmeSensor.begin(0x76)) {
    gBme.present = true;
    gBme.hasRead = false;
    return true;
  }
  if (gBmeSensor.begin(0x77)) {
    gBme.present = true;
    gBme.hasRead = false;
    return true;
  }
  gBme.present = false;
  gBme.ok = false;
  return false;
}

static bool initBh() {
  if (gBhSensor.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
    gBh.present = true;
    gBh.hasRead = false;
    return true;
  }
  gBh.present = false;
  gBh.ok = false;
  return false;
}

static uint16_t readAverageRaw(uint8_t pin, uint16_t samples) {
  if (samples < 1) samples = 1;
  if (samples > 64) samples = 64;
  uint32_t sum = 0;
  for (uint16_t i = 0; i < samples; i++) {
    sum += analogRead(pin);
  }
  return (uint16_t)(sum / samples);
}

static void taskBme(uint64_t now) {
  if (!gConfig.bme.enabled) {
    gBme.ok = false;
    return;
  }

  if (!gBme.present) {
    if (now < gBme.nextAttempt) return;
    if (!initBme()) {
      gBme.nextAttempt = now + 30000ULL;
      return;
    }
  }

  if (!gBme.hasRead || (now - gBme.lastRead) >= gConfig.bme.intervalMs) {
    gBme.hasRead = true;
    gBme.lastRead = now;

    float t = gBmeSensor.readTemperature();
    float h = gBmeSensor.readHumidity();
    float p = gBmeSensor.readPressure() / 100.0f;

    if (!isfinite(t) || !isfinite(h) || !isfinite(p) || p <= 0.0f) {
      gBme.ok = false;
      gBme.present = false;
      gBme.hasRead = false;
      gBme.nextAttempt = now + 30000ULL;
      return;
    }

    t += ((float)gConfig.bme.tempOffsetX100) / 100.0f;
    h += ((float)gConfig.bme.humOffsetX100) / 100.0f;
    p += ((float)gConfig.bme.pressureOffsetPa) / 100.0f;

    if (!isfinite(t) || !isfinite(h) || !isfinite(p) || p < 100.0f || p > 1200.0f) {
      gBme.ok = false;
      gBme.present = false;
      gBme.hasRead = false;
      gBme.nextAttempt = now + 30000ULL;
      return;
    }

    if (h < 0.0f) h = 0.0f;
    if (h > 100.0f) h = 100.0f;

    gBme.t = t;
    gBme.h = h;
    gBme.p = p;
    gBme.ok = true;
  }
}

static void taskBh(uint64_t now) {
  if (!gConfig.bh.enabled) {
    gBh.ok = false;
    return;
  }

  if (!gBh.present) {
    if (now < gBh.nextAttempt) return;
    if (!initBh()) {
      gBh.nextAttempt = now + 30000ULL;
      return;
    }
  }

  if (!gBh.hasRead || (now - gBh.lastRead) >= gConfig.bh.intervalMs) {
    gBh.hasRead = true;
    gBh.lastRead = now;

    float raw = gBhSensor.readLightLevel();
    if (!isfinite(raw) || raw < 0.0f) {
      gBh.ok = false;
      gBh.present = false;
      gBh.hasRead = false;
      gBh.nextAttempt = now + 30000ULL;
      return;
    }

    float lux = raw * ((float)gConfig.bh.luxScaleX1000 / 1000.0f) + (float)gConfig.bh.luxOffset;
    if (!isfinite(lux)) {
      gBh.ok = false;
      gBh.present = false;
      gBh.hasRead = false;
      gBh.nextAttempt = now + 30000ULL;
      return;
    }
    if (lux < 0.0f) lux = 0.0f;
    if (lux > 200000.0f) lux = 200000.0f;
    gBh.lux = lux;
    gBh.ok = true;
  }
}

static void taskMq(uint64_t now) {
  if (!gConfig.mq.enabled) {
    gMq.ok = false;
    gMq.ready = false;
    gMq.alarm = false;
    return;
  }

  if (!gMq.hasRead || (now - gMq.lastRead) >= gConfig.mq.intervalMs) {
    gMq.hasRead = true;
    gMq.lastRead = now;
    gMq.raw = readAverageRaw(PIN_MQ2, gConfig.mq.samples);
    gMq.ok = true;
    gMq.ready = (now - gBootMs) >= gConfig.mq.warmupMs;

    if (!gConfig.mq.alarmEnabled || !gMq.ready) {
      gMq.alarm = false;
    } else {
      if (gMq.alarm) {
        if ((uint32_t)gMq.raw + gConfig.mq.hysteresisRaw < gConfig.mq.alarmRaw) {
          gMq.alarm = false;
        }
      } else {
        if (gMq.raw >= gConfig.mq.alarmRaw) {
          gMq.alarm = true;
        }
      }
    }
  }
}

static void taskSoil(uint64_t now) {
  if (!gConfig.soil.enabled) {
    gSoil.ok = false;
    return;
  }

  if (!gSoil.hasRead || (now - gSoil.lastRead) >= gConfig.soil.intervalMs) {
    gSoil.hasRead = true;
    gSoil.lastRead = now;
    uint16_t raw = readAverageRaw(PIN_SOIL, 8);
    int32_t denom = (int32_t)gConfig.soil.dryRaw - (int32_t)gConfig.soil.wetRaw;
    int32_t num = (int32_t)gConfig.soil.dryRaw - (int32_t)raw;
    int32_t pct = 0;
    if (denom != 0) pct = (num * 100) / denom;
    pct += gConfig.soil.offsetPct;
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    gSoil.raw = raw;
    gSoil.pct = (uint8_t)pct;
    gSoil.ok = true;
  }
}

static void taskRain(uint64_t now) {
  if (!gConfig.rain.enabled) {
    gRain.ok = false;
    gRain.alert = false;
    gRain.inhibit = false;
    return;
  }

  if (!gRain.hasRead || (now - gRain.lastRead) >= gConfig.rain.intervalMs) {
    gRain.hasRead = true;
    gRain.lastRead = now;
    uint16_t raw = readAverageRaw(PIN_RAIN, 8);
    int32_t denom = (int32_t)gConfig.rain.dryRaw - (int32_t)gConfig.rain.wetRaw;
    int32_t num = (int32_t)gConfig.rain.dryRaw - (int32_t)raw;
    int32_t pct = 0;
    if (denom != 0) pct = (num * 100) / denom;
    pct += gConfig.rain.offsetPct;
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    gRain.raw = raw;
    gRain.pct = (uint8_t)pct;
    gRain.alert = gRain.pct >= gConfig.rain.alertPct;
    gRain.inhibit = gRain.pct >= gConfig.rain.inhibitPct;
    gRain.ok = true;
  }
}

static bool rainInhibitActive() {
  if (!gConfig.rain.enabled || !gConfig.rain.inhibitRelay) return false;
  if (!gRain.ok) return true;
  return gRain.inhibit;
}

static void applyRelayPhysical() {
  bool level = gRelayOn;
  if (gConfig.relay.activeLow) level = !level;
  digitalWrite(PIN_RELAY, level ? HIGH : LOW);
}

static void setRelay(bool on, uint64_t now) {
  if (gRelayOn != on) {
    gRelayOn = on;
    gLastRelayChange = now;
    if (on) gRelayOnSince = now;
  }
  applyRelayPhysical();
}

static void taskRelay(uint64_t now) {
  if (gRelayLastRun != 0 && (now - gRelayLastRun) < 200ULL) return;
  gRelayLastRun = now;

  bool desired = gRelayOn;
  bool immediate = false;

  if (!gConfig.relay.enabled) {
    desired = false;
    immediate = true;
    gRelaySafety = false;
  } else if (gConfig.relay.mode == 1) {
    desired = false;
    immediate = true;
    gRelaySafety = false;
  } else if (gConfig.relay.mode == 2) {
    desired = true;
    immediate = true;
    gRelaySafety = false;
  } else {
    if (!gConfig.soil.enabled || !gConfig.soil.autoRelay) {
      desired = false;
      immediate = true;
      gRelaySafety = false;
    } else {
      if (gRelaySafety) {
        desired = false;
        immediate = true;
        if (gSoil.ok && gSoil.pct >= gConfig.soil.stopPct) {
          gRelaySafety = false;
        }
      } else {
        if (!gSoil.ok) {
          desired = false;
          immediate = true;
        } else if (rainInhibitActive()) {
          desired = false;
          immediate = true;
        } else {
          if (gSoil.pct <= gConfig.soil.startPct) {
            desired = true;
          } else if (gSoil.pct >= gConfig.soil.stopPct) {
            desired = false;
          }
        }
      }

      if (gRelayOn && gConfig.relay.maxOnMs > 0ULL && (now - gRelayOnSince) >= gConfig.relay.maxOnMs) {
        gRelaySafety = true;
        desired = false;
        immediate = true;
      }
    }
  }

  if (desired != gRelayOn) {
    if (immediate || (now - gLastRelayChange) >= gConfig.relay.minSwitchMs) {
      setRelay(desired, now);
    }
  }
}

static void sendNoCacheHeaders() {
  gServer.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  gServer.sendHeader("Pragma", "no-cache");
  gServer.sendHeader("X-Content-Type-Options", "nosniff");
  gServer.sendHeader("X-Frame-Options", "DENY");
  gServer.sendHeader("Referrer-Policy", "no-referrer");
}

static void sendPageStart() {
  sendNoCacheHeaders();
  gServer.setContentLength(CONTENT_LENGTH_UNKNOWN);
  gServer.send(200, "text/html", "");
  gServer.sendContent_P(HTML_HEAD, sizeof(HTML_HEAD) - 1);
}

static void sendPageEnd() {
  gServer.sendContent_P(HTML_FOOT, sizeof(HTML_FOOT) - 1);
  gServer.sendContent("");
}

static void sendRedirect(const char* path) {
  sendNoCacheHeaders();
  gServer.sendHeader("Location", path);
  gServer.send(303, "text/plain", "");
}

static void sendSavedNotice(bool saved, bool error) {
  if (saved) gServer.sendContent("<div class=\"msg ok\">Settings saved.</div>");
  if (error) gServer.sendContent("<div class=\"msg err\">Invalid input. Nothing was saved.</div>");
}

static bool parseU64Arg(const char* name, uint64_t minV, uint64_t maxV, uint64_t& out) {
  if (!gServer.hasArg(name)) return false;
  String s = gServer.arg(name);
  if (s.length() == 0 || s[0] == '-') return false;
  char* end = nullptr;
  errno = 0;
  unsigned long long v = strtoull(s.c_str(), &end, 10);
  if (errno != 0 || end == s.c_str() || *end != 0) return false;
  if (v < minV || v > maxV) return false;
  out = (uint64_t)v;
  return true;
}

static bool parseI64Arg(const char* name, int64_t minV, int64_t maxV, int64_t& out) {
  if (!gServer.hasArg(name)) return false;
  String s = gServer.arg(name);
  if (s.length() == 0) return false;
  char* end = nullptr;
  errno = 0;
  long long v = strtoll(s.c_str(), &end, 10);
  if (errno != 0 || end == s.c_str() || *end != 0) return false;
  if (v < minV || v > maxV) return false;
  out = (int64_t)v;
  return true;
}

static bool parseFloatScaledArg(const char* name, float minV, float maxV, int32_t scale, int32_t& out) {
  if (!gServer.hasArg(name)) return false;
  String s = gServer.arg(name);
  if (s.length() == 0) return false;
  char* end = nullptr;
  errno = 0;
  float v = strtof(s.c_str(), &end);
  if (errno != 0 || end == s.c_str() || *end != 0 || !isfinite(v)) return false;
  if (v < minV || v > maxV) return false;
  out = (int32_t)lroundf(v * (float)scale);
  return true;
}

static bool parseSsidArg(char* dst, size_t size) {
  if (!gServer.hasArg("ssid")) return false;
  String s = gServer.arg("ssid");
  size_t n = s.length();
  if (n < 1 || n > 31 || n >= size) return false;
  memcpy(dst, s.c_str(), n);
  dst[n] = 0;
  return true;
}

static bool applyWifiPasswordArg(char* dst, size_t size, bool open) {
  if (open) {
    dst[0] = 0;
    return true;
  }

  String np = gServer.hasArg("newpass") ? gServer.arg("newpass") : String();
  if (np.length() == 0) {
    return dst[0] != 0;
  }
  if (np.length() < 8 || np.length() > 63 || np.length() >= size) return false;
  memcpy(dst, np.c_str(), np.length());
  dst[np.length()] = 0;
  return true;
}

static void sendWifiSavedPage(bool restart) {
  sendPageStart();
  gServer.sendContent("<h1>WiFi AP Settings</h1>");
  gServer.sendContent("<div class=\"msg ok\">Settings saved.</div>");

  if (restart) {
    char ssid[256];
    char buf[512];
    htmlEscape(gConfig.wifi.ssid, ssid, sizeof(ssid));
    snprintf(buf, sizeof(buf),
             "<p>The access point will restart in about 5 seconds. Reconnect to <strong>%s</strong>.</p>",
             ssid);
    gServer.sendContent(buf);
    gServer.sendContent("<p class=\"muted\">If this page disconnects, reconnect to the AP and open <code>http://192.168.4.1/</code>.</p>");
  } else {
    gServer.sendContent("<p>No AP restart was required.</p>");
  }

  gServer.sendContent("<p><a class=\"btn primary\" href=\"/settings/wifi\">Back to WiFi settings</a></p>");
  sendPageEnd();
}

static void handleDashboard() {
  sendPageStart();
  gServer.sendContent_P(HTML_DASH, sizeof(HTML_DASH) - 1);
  sendPageEnd();
}

static void handleBmePage() {
  bool saved = gServer.hasArg("saved") && gServer.arg("saved") == "1";
  bool error = gServer.hasArg("error") && gServer.arg("error") == "1";
  char buf[192];

  sendPageStart();
  gServer.sendContent("<h1>BME280 Settings</h1>");
  sendSavedNotice(saved, error);
  gServer.sendContent("<form method=\"post\" action=\"/save/bme280\">");
  gServer.sendContent("<label class=\"check\"><input type=\"checkbox\" name=\"enabled\" value=\"1\"");
  if (gConfig.bme.enabled) gServer.sendContent(" checked");
  gServer.sendContent("> Enable BME280</label>");

  snprintf(buf, sizeof(buf),
           "<label>Read interval (ms)</label><input type=\"number\" name=\"interval\" min=\"1000\" max=\"86400000\" step=\"1\" required value=\"%" PRIu64 "\">",
           gConfig.bme.intervalMs);
  gServer.sendContent(buf);

  snprintf(buf, sizeof(buf),
           "<label>Temperature offset (°C)</label><input type=\"number\" name=\"toff\" min=\"-20\" max=\"20\" step=\"0.01\" required value=\"%.2f\">",
           (double)((float)gConfig.bme.tempOffsetX100 / 100.0f));
  gServer.sendContent(buf);

  snprintf(buf, sizeof(buf),
           "<label>Humidity offset (%)</label><input type=\"number\" name=\"hoff\" min=\"-50\" max=\"50\" step=\"0.01\" required value=\"%.2f\">",
           (double)((float)gConfig.bme.humOffsetX100 / 100.0f));
  gServer.sendContent(buf);

  snprintf(buf, sizeof(buf),
           "<label>Pressure offset (Pa)</label><input type=\"number\" name=\"poff\" min=\"-10000\" max=\"10000\" step=\"1\" required value=\"%" PRId32 "\">",
           gConfig.bme.pressureOffsetPa);
  gServer.sendContent(buf);

  gServer.sendContent("<div class=\"form-actions\"><button class=\"btn primary\" type=\"submit\">Save</button><a class=\"btn\" href=\"/\">Cancel</a></div></form>");
  sendPageEnd();
}

static void handleBmeSave() {
  uint64_t interval;
  int32_t tempOff;
  int32_t humOff;
  int64_t pressOff;

  if (!parseU64Arg("interval", 1000ULL, 86400000ULL, interval) ||
      !parseFloatScaledArg("toff", -20.0f, 20.0f, 100, tempOff) ||
      !parseFloatScaledArg("hoff", -50.0f, 50.0f, 100, humOff) ||
      !parseI64Arg("poff", -10000, 10000, pressOff)) {
    sendRedirect("/settings/bme280?error=1");
    return;
  }

  AppConfig temp = gConfig;
  temp.bme.enabled = gServer.hasArg("enabled") ? 1 : 0;
  temp.bme.intervalMs = interval;
  temp.bme.tempOffsetX100 = tempOff;
  temp.bme.humOffsetX100 = humOff;
  temp.bme.pressureOffsetPa = (int32_t)pressOff;
  sanitizeConfig(temp);
  gConfig = temp;
  gBme.hasRead = false;
  saveConfig();
  sendRedirect("/settings/bme280?saved=1");
}

static void handleBhPage() {
  bool saved = gServer.hasArg("saved") && gServer.arg("saved") == "1";
  bool error = gServer.hasArg("error") && gServer.arg("error") == "1";
  char buf[192];

  sendPageStart();
  gServer.sendContent("<h1>BH1750 Settings</h1>");
  sendSavedNotice(saved, error);
  gServer.sendContent("<form method=\"post\" action=\"/save/bh1750\">");
  gServer.sendContent("<label class=\"check\"><input type=\"checkbox\" name=\"enabled\" value=\"1\"");
  if (gConfig.bh.enabled) gServer.sendContent(" checked");
  gServer.sendContent("> Enable BH1750</label>");

  snprintf(buf, sizeof(buf),
           "<label>Read interval (ms)</label><input type=\"number\" name=\"interval\" min=\"1000\" max=\"86400000\" step=\"1\" required value=\"%" PRIu64 "\">",
           gConfig.bh.intervalMs);
  gServer.sendContent(buf);

  snprintf(buf, sizeof(buf),
           "<label>Lux scale multiplier</label><input type=\"number\" name=\"scale\" min=\"0.001\" max=\"100\" step=\"0.001\" required value=\"%.3f\">",
           (double)((float)gConfig.bh.luxScaleX1000 / 1000.0f));
  gServer.sendContent(buf);

  snprintf(buf, sizeof(buf),
           "<label>Lux offset</label><input type=\"number\" name=\"loff\" min=\"-200000\" max=\"200000\" step=\"1\" required value=\"%" PRId32 "\">",
           gConfig.bh.luxOffset);
  gServer.sendContent(buf);

  gServer.sendContent("<div class=\"form-actions\"><button class=\"btn primary\" type=\"submit\">Save</button><a class=\"btn\" href=\"/\">Cancel</a></div></form>");
  sendPageEnd();
}

static void handleBhSave() {
  uint64_t interval;
  int32_t scale;
  int64_t offset;

  if (!parseU64Arg("interval", 1000ULL, 86400000ULL, interval) ||
      !parseFloatScaledArg("scale", 0.001f, 100.0f, 1000, scale) ||
      !parseI64Arg("loff", -200000, 200000, offset)) {
    sendRedirect("/settings/bh1750?error=1");
    return;
  }

  AppConfig temp = gConfig;
  temp.bh.enabled = gServer.hasArg("enabled") ? 1 : 0;
  temp.bh.intervalMs = interval;
  temp.bh.luxScaleX1000 = scale;
  temp.bh.luxOffset = (int32_t)offset;
  sanitizeConfig(temp);
  gConfig = temp;
  gBh.hasRead = false;
  saveConfig();
  sendRedirect("/settings/bh1750?saved=1");
}

static void handleMqPage() {
  bool saved = gServer.hasArg("saved") && gServer.arg("saved") == "1";
  bool error = gServer.hasArg("error") && gServer.arg("error") == "1";
  char buf[224];

  sendPageStart();
  gServer.sendContent("<h1>MQ-2 Settings</h1>");
  sendSavedNotice(saved, error);
  gServer.sendContent("<form method=\"post\" action=\"/save/mq2\">");
  gServer.sendContent("<label class=\"check\"><input type=\"checkbox\" name=\"enabled\" value=\"1\"");
  if (gConfig.mq.enabled) gServer.sendContent(" checked");
  gServer.sendContent("> Enable MQ-2</label>");
  gServer.sendContent("<label class=\"check\"><input type=\"checkbox\" name=\"alarm\" value=\"1\"");
  if (gConfig.mq.alarmEnabled) gServer.sendContent(" checked");
  gServer.sendContent("> Enable gas alarm</label>");

  snprintf(buf, sizeof(buf),
           "<label>Read interval (ms)</label><input type=\"number\" name=\"interval\" min=\"1000\" max=\"86400000\" step=\"1\" required value=\"%" PRIu64 "\">",
           gConfig.mq.intervalMs);
  gServer.sendContent(buf);

  snprintf(buf, sizeof(buf),
           "<label>Warm-up (ms)</label><input type=\"number\" name=\"warmup\" min=\"0\" max=\"86400000\" step=\"1\" required value=\"%" PRIu64 "\">",
           gConfig.mq.warmupMs);
  gServer.sendContent(buf);

  snprintf(buf, sizeof(buf),
           "<label>Samples</label><input type=\"number\" name=\"samples\" min=\"1\" max=\"64\" step=\"1\" required value=\"%u\">",
           (unsigned)gConfig.mq.samples);
  gServer.sendContent(buf);

  snprintf(buf, sizeof(buf),
           "<label>Alarm raw threshold</label><input type=\"number\" name=\"alarmRaw\" min=\"0\" max=\"4095\" step=\"1\" required value=\"%u\">",
           (unsigned)gConfig.mq.alarmRaw);
  gServer.sendContent(buf);

  snprintf(buf, sizeof(buf),
           "<label>Hysteresis raw</label><input type=\"number\" name=\"hyst\" min=\"0\" max=\"4095\" step=\"1\" required value=\"%u\">",
           (unsigned)gConfig.mq.hysteresisRaw);
  gServer.sendContent(buf);

  gServer.sendContent("<div class=\"form-actions\"><button class=\"btn primary\" type=\"submit\">Save</button><a class=\"btn\" href=\"/\">Cancel</a></div></form>");
  sendPageEnd();
}

static void handleMqSave() {
  uint64_t interval;
  uint64_t warmup;
  uint64_t samples;
  uint64_t alarmRaw;
  uint64_t hysteresis;

  if (!parseU64Arg("interval", 1000ULL, 86400000ULL, interval) ||
      !parseU64Arg("warmup", 0ULL, 86400000ULL, warmup) ||
      !parseU64Arg("samples", 1ULL, 64ULL, samples) ||
      !parseU64Arg("alarmRaw", 0ULL, 4095ULL, alarmRaw) ||
      !parseU64Arg("hyst", 0ULL, 4095ULL, hysteresis)) {
    sendRedirect("/settings/mq2?error=1");
    return;
  }

  AppConfig temp = gConfig;
  temp.mq.enabled = gServer.hasArg("enabled") ? 1 : 0;
  temp.mq.alarmEnabled = gServer.hasArg("alarm") ? 1 : 0;
  temp.mq.intervalMs = interval;
  temp.mq.warmupMs = warmup;
  temp.mq.samples = (uint16_t)samples;
  temp.mq.alarmRaw = (uint16_t)alarmRaw;
  temp.mq.hysteresisRaw = (uint16_t)hysteresis;
  sanitizeConfig(temp);
  gConfig = temp;
  gMq.hasRead = false;
  saveConfig();
  sendRedirect("/settings/mq2?saved=1");
}

static void handleSoilPage() {
  bool saved = gServer.hasArg("saved") && gServer.arg("saved") == "1";
  bool error = gServer.hasArg("error") && gServer.arg("error") == "1";
  char buf[224];

  sendPageStart();
  gServer.sendContent("<h1>Soil Moisture Settings</h1>");
  sendSavedNotice(saved, error);
  gServer.sendContent("<form method=\"post\" action=\"/save/soil\">");
  gServer.sendContent("<label class=\"check\"><input type=\"checkbox\" name=\"enabled\" value=\"1\"");
  if (gConfig.soil.enabled) gServer.sendContent(" checked");
  gServer.sendContent("> Enable soil sensor</label>");
  gServer.sendContent("<label class=\"check\"><input type=\"checkbox\" name=\"autoRelay\" value=\"1\"");
  if (gConfig.soil.autoRelay) gServer.sendContent(" checked");
  gServer.sendContent("> Allow automatic relay control from soil</label>");

  snprintf(buf, sizeof(buf),
           "<label>Read interval (ms)</label><input type=\"number\" name=\"interval\" min=\"1000\" max=\"86400000\" step=\"1\" required value=\"%" PRIu64 "\">",
           gConfig.soil.intervalMs);
  gServer.sendContent(buf);

  snprintf(buf, sizeof(buf),
           "<label>Dry raw value</label><input type=\"number\" name=\"dry\" min=\"0\" max=\"4095\" step=\"1\" required value=\"%u\">",
           (unsigned)gConfig.soil.dryRaw);
  gServer.sendContent(buf);

  snprintf(buf, sizeof(buf),
           "<label>Wet raw value</label><input type=\"number\" name=\"wet\" min=\"0\" max=\"4095\" step=\"1\" required value=\"%u\">",
           (unsigned)gConfig.soil.wetRaw);
  gServer.sendContent(buf);

  snprintf(buf, sizeof(buf),
           "<label>Moisture offset (%)</label><input type=\"number\" name=\"offset\" min=\"-100\" max=\"100\" step=\"1\" required value=\"%d\">",
           (int)gConfig.soil.offsetPct);
  gServer.sendContent(buf);

  snprintf(buf, sizeof(buf),
           "<label>Relay start moisture (%)</label><input type=\"number\" name=\"start\" min=\"0\" max=\"99\" step=\"1\" required value=\"%u\">",
           (unsigned)gConfig.soil.startPct);
  gServer.sendContent(buf);

  snprintf(buf, sizeof(buf),
           "<label>Relay stop moisture (%)</label><input type=\"number\" name=\"stop\" min=\"1\" max=\"100\" step=\"1\" required value=\"%u\">",
           (unsigned)gConfig.soil.stopPct);
  gServer.sendContent(buf);

  gServer.sendContent("<p class=\"muted\">Dry raw must be greater than wet raw. Stop moisture must be greater than start moisture.</p>");
  gServer.sendContent("<div class=\"form-actions\"><button class=\"btn primary\" type=\"submit\">Save</button><a class=\"btn\" href=\"/\">Cancel</a></div></form>");
  sendPageEnd();
}

static void handleSoilSave() {
  uint64_t interval;
  uint64_t dry;
  uint64_t wet;
  int64_t offset;
  int64_t startPct;
  int64_t stopPct;

  if (!parseU64Arg("interval", 1000ULL, 86400000ULL, interval) ||
      !parseU64Arg("dry", 0ULL, 4095ULL, dry) ||
      !parseU64Arg("wet", 0ULL, 4095ULL, wet) ||
      !parseI64Arg("offset", -100, 100, offset) ||
      !parseI64Arg("start", 0, 99, startPct) ||
      !parseI64Arg("stop", 1, 100, stopPct) ||
      dry <= wet ||
      startPct >= stopPct) {
    sendRedirect("/settings/soil?error=1");
    return;
  }

  AppConfig temp = gConfig;
  temp.soil.enabled = gServer.hasArg("enabled") ? 1 : 0;
  temp.soil.autoRelay = gServer.hasArg("autoRelay") ? 1 : 0;
  temp.soil.intervalMs = interval;
  temp.soil.dryRaw = (uint16_t)dry;
  temp.soil.wetRaw = (uint16_t)wet;
  temp.soil.offsetPct = (int16_t)offset;
  temp.soil.startPct = (uint8_t)startPct;
  temp.soil.stopPct = (uint8_t)stopPct;
  sanitizeConfig(temp);
  gConfig = temp;
  gSoil.hasRead = false;
  saveConfig();
  sendRedirect("/settings/soil?saved=1");
}

static void handleRainPage() {
  bool saved = gServer.hasArg("saved") && gServer.arg("saved") == "1";
  bool error = gServer.hasArg("error") && gServer.arg("error") == "1";
  char buf[224];

  sendPageStart();
  gServer.sendContent("<h1>Rain Sensor Settings</h1>");
  sendSavedNotice(saved, error);
  gServer.sendContent("<form method=\"post\" action=\"/save/rain\">");
  gServer.sendContent("<label class=\"check\"><input type=\"checkbox\" name=\"enabled\" value=\"1\"");
  if (gConfig.rain.enabled) gServer.sendContent(" checked");
  gServer.sendContent("> Enable rain sensor</label>");
  gServer.sendContent("<label class=\"check\"><input type=\"checkbox\" name=\"inhibit\" value=\"1\"");
  if (gConfig.rain.inhibitRelay) gServer.sendContent(" checked");
  gServer.sendContent("> Inhibit irrigation when rain is detected</label>");

  snprintf(buf, sizeof(buf),
           "<label>Read interval (ms)</label><input type=\"number\" name=\"interval\" min=\"1000\" max=\"86400000\" step=\"1\" required value=\"%" PRIu64 "\">",
           gConfig.rain.intervalMs);
  gServer.sendContent(buf);

  snprintf(buf, sizeof(buf),
           "<label>Dry raw value</label><input type=\"number\" name=\"dry\" min=\"0\" max=\"4095\" step=\"1\" required value=\"%u\">",
           (unsigned)gConfig.rain.dryRaw);
  gServer.sendContent(buf);

  snprintf(buf, sizeof(buf),
           "<label>Wet raw value</label><input type=\"number\" name=\"wet\" min=\"0\" max=\"4095\" step=\"1\" required value=\"%u\">",
           (unsigned)gConfig.rain.wetRaw);
  gServer.sendContent(buf);

  snprintf(buf, sizeof(buf),
           "<label>Wetness offset (%)</label><input type=\"number\" name=\"offset\" min=\"-100\" max=\"100\" step=\"1\" required value=\"%d\">",
           (int)gConfig.rain.offsetPct);
  gServer.sendContent(buf);

  snprintf(buf, sizeof(buf),
           "<label>Rain alert threshold (%)</label><input type=\"number\" name=\"alert\" min=\"0\" max=\"100\" step=\"1\" required value=\"%u\">",
           (unsigned)gConfig.rain.alertPct);
  gServer.sendContent(buf);

  snprintf(buf, sizeof(buf),
           "<label>Rain inhibit threshold (%)</label><input type=\"number\" name=\"inhibitPct\" min=\"0\" max=\"100\" step=\"1\" required value=\"%u\">",
           (unsigned)gConfig.rain.inhibitPct);
  gServer.sendContent(buf);

  gServer.sendContent("<p class=\"muted\">Dry raw must be greater than wet raw. A missing rain reading is treated as an active inhibit when inhibit is enabled.</p>");
  gServer.sendContent("<div class=\"form-actions\"><button class=\"btn primary\" type=\"submit\">Save</button><a class=\"btn\" href=\"/\">Cancel</a></div></form>");
  sendPageEnd();
}

static void handleRainSave() {
  uint64_t interval;
  uint64_t dry;
  uint64_t wet;
  int64_t offset;
  uint64_t alertPct;
  uint64_t inhibitPct;

  if (!parseU64Arg("interval", 1000ULL, 86400000ULL, interval) ||
      !parseU64Arg("dry", 0ULL, 4095ULL, dry) ||
      !parseU64Arg("wet", 0ULL, 4095ULL, wet) ||
      !parseI64Arg("offset", -100, 100, offset) ||
      !parseU64Arg("alert", 0ULL, 100ULL, alertPct) ||
      !parseU64Arg("inhibitPct", 0ULL, 100ULL, inhibitPct) ||
      dry <= wet) {
    sendRedirect("/settings/rain?error=1");
    return;
  }

  AppConfig temp = gConfig;
  temp.rain.enabled = gServer.hasArg("enabled") ? 1 : 0;
  temp.rain.inhibitRelay = gServer.hasArg("inhibit") ? 1 : 0;
  temp.rain.intervalMs = interval;
  temp.rain.dryRaw = (uint16_t)dry;
  temp.rain.wetRaw = (uint16_t)wet;
  temp.rain.offsetPct = (int16_t)offset;
  temp.rain.alertPct = (uint8_t)alertPct;
  temp.rain.inhibitPct = (uint8_t)inhibitPct;
  sanitizeConfig(temp);
  gConfig = temp;
  gRain.hasRead = false;
  saveConfig();
  sendRedirect("/settings/rain?saved=1");
}

static void handleRelayPage() {
  bool saved = gServer.hasArg("saved") && gServer.arg("saved") == "1";
  bool error = gServer.hasArg("error") && gServer.arg("error") == "1";
  char buf[512];

  const char* sel0 = gConfig.relay.mode == 0 ? " selected" : "";
  const char* sel1 = gConfig.relay.mode == 1 ? " selected" : "";
  const char* sel2 = gConfig.relay.mode == 2 ? " selected" : "";

  sendPageStart();
  gServer.sendContent("<h1>Relay Settings</h1>");
  sendSavedNotice(saved, error);
  gServer.sendContent("<form method=\"post\" action=\"/save/relay\">");
  gServer.sendContent("<label class=\"check\"><input type=\"checkbox\" name=\"enabled\" value=\"1\"");
  if (gConfig.relay.enabled) gServer.sendContent(" checked");
  gServer.sendContent("> Enable relay control</label>");
  gServer.sendContent("<label class=\"check\"><input type=\"checkbox\" name=\"activeLow\" value=\"1\"");
  if (gConfig.relay.activeLow) gServer.sendContent(" checked");
  gServer.sendContent("> Relay module is active-low</label>");

  snprintf(buf, sizeof(buf),
           "<label>Mode</label><select name=\"mode\">"
           "<option value=\"0\"%s>Auto irrigation</option>"
           "<option value=\"1\"%s>Force off</option>"
           "<option value=\"2\"%s>Force on</option>"
           "</select>",
           sel0, sel1, sel2);
  gServer.sendContent(buf);

  snprintf(buf, sizeof(buf),
           "<label>Minimum switch time (ms)</label><input type=\"number\" name=\"minSwitch\" min=\"2000\" max=\"86400000\" step=\"1\" required value=\"%" PRIu64 "\">",
           gConfig.relay.minSwitchMs);
  gServer.sendContent(buf);

  snprintf(buf, sizeof(buf),
           "<label>Maximum continuous auto-on time (ms)</label><input type=\"number\" name=\"maxOn\" min=\"0\" max=\"604800000\" step=\"1\" required value=\"%" PRIu64 "\">",
           gConfig.relay.maxOnMs);
  gServer.sendContent(buf);

  gServer.sendContent("<p class=\"muted\">Force On changes back to Auto after a reboot. Maximum continuous auto-on applies only in Auto mode. Set it to 0 to disable that limit.</p>");
  gServer.sendContent("<div class=\"form-actions\"><button class=\"btn primary\" type=\"submit\">Save</button><a class=\"btn\" href=\"/\">Cancel</a></div></form>");
  sendPageEnd();
}

static void handleRelaySave() {
  uint64_t mode;
  uint64_t minSwitch;
  uint64_t maxOn;

  if (!parseU64Arg("mode", 0ULL, 2ULL, mode) ||
      !parseU64Arg("minSwitch", 2000ULL, 86400000ULL, minSwitch) ||
      !parseU64Arg("maxOn", 0ULL, 604800000ULL, maxOn) ||
      (maxOn > 0ULL && maxOn < 10000ULL)) {
    sendRedirect("/settings/relay?error=1");
    return;
  }

  uint8_t oldMode = gConfig.relay.mode;
  bool oldActiveLow = gConfig.relay.activeLow != 0;

  AppConfig temp = gConfig;
  temp.relay.enabled = gServer.hasArg("enabled") ? 1 : 0;
  temp.relay.activeLow = gServer.hasArg("activeLow") ? 1 : 0;
  temp.relay.mode = (uint8_t)mode;
  temp.relay.minSwitchMs = minSwitch;
  temp.relay.maxOnMs = maxOn;
  sanitizeConfig(temp);
  gConfig = temp;

  if (gConfig.relay.mode != oldMode) {
    gRelaySafety = false;
    gLastRelayChange = 0;
  }

  saveConfig();

  uint64_t now = nowMs64();
  if (!gConfig.relay.enabled || gConfig.relay.mode == 1) {
    setRelay(false, now);
  } else if (gConfig.relay.mode == 2) {
    setRelay(true, now);
  } else {
    if (oldActiveLow != (gConfig.relay.activeLow != 0)) {
      applyRelayPhysical();
    }
  }
  gRelayLastRun = 0;
  sendRedirect("/settings/relay?saved=1");
}

static void handleWifiPage() {
  bool error = gServer.hasArg("error") && gServer.arg("error") == "1";
  char ssid[256];
  char buf[1024];

  htmlEscape(gConfig.wifi.ssid, ssid, sizeof(ssid));

  sendPageStart();
  gServer.sendContent("<h1>WiFi AP Settings</h1>");
  if (error) gServer.sendContent("<div class=\"msg err\">Invalid input. Nothing was saved.</div>");

  snprintf(buf, sizeof(buf),
           "<form method=\"post\" action=\"/save/wifi\">"
           "<label>SSID</label><input type=\"text\" name=\"ssid\" maxlength=\"31\" required value=\"%s\">",
           ssid);
  gServer.sendContent(buf);

  gServer.sendContent("<label>New password</label><input type=\"password\" name=\"newpass\" minlength=\"8\" maxlength=\"63\" autocomplete=\"new-password\" placeholder=\"Leave blank to keep current password\">");
  gServer.sendContent("<label class=\"check\"><input type=\"checkbox\" name=\"open\" value=\"1\" id=\"open\"");
  if (gConfig.wifi.password[0] == 0) gServer.sendContent(" checked");
  gServer.sendContent("> Open network (no password)</label>");

  snprintf(buf, sizeof(buf),
           "<label>Channel</label><input type=\"number\" name=\"channel\" min=\"1\" max=\"13\" step=\"1\" required value=\"%u\">",
           (unsigned)gConfig.wifi.channel);
  gServer.sendContent(buf);

  snprintf(buf, sizeof(buf),
           "<label>Maximum clients</label><input type=\"number\" name=\"maxclients\" min=\"1\" max=\"8\" step=\"1\" required value=\"%u\">",
           (unsigned)gConfig.wifi.maxClients);
  gServer.sendContent(buf);

  gServer.sendContent("<label class=\"check\"><input type=\"checkbox\" name=\"hidden\" value=\"1\"");
  if (gConfig.wifi.hidden) gServer.sendContent(" checked");
  gServer.sendContent("> Hide SSID</label>");

  gServer.sendContent("<p class=\"muted\">Changing SSID, password, channel, maximum clients, or hidden mode restarts the AP after 5 seconds and disconnects clients.</p>");
  gServer.sendContent("<div class=\"form-actions\"><button class=\"btn primary\" type=\"submit\">Save</button><a class=\"btn\" href=\"/\">Cancel</a></div></form>");
  gServer.sendContent("<script>(function(){var o=document.getElementById('open');var p=document.querySelector('input[name=\"newpass\"]');function sync(){p.disabled=o.checked;if(o.checked)p.value='';}o.addEventListener('change',sync);sync();})();</script>");
  sendPageEnd();
}

static void handleWifiSave() {
  bool open = gServer.hasArg("open");
  uint64_t channel;
  uint64_t maxClients;

  AppConfig temp = gConfig;
  if (!parseSsidArg(temp.wifi.ssid, sizeof(temp.wifi.ssid)) ||
      !applyWifiPasswordArg(temp.wifi.password, sizeof(temp.wifi.password), open) ||
      !parseU64Arg("channel", 1ULL, 13ULL, channel) ||
      !parseU64Arg("maxclients", 1ULL, 8ULL, maxClients)) {
    sendRedirect("/settings/wifi?error=1");
    return;
  }

  temp.wifi.channel = (uint8_t)channel;
  temp.wifi.maxClients = (uint8_t)maxClients;
  temp.wifi.hidden = gServer.hasArg("hidden") ? 1 : 0;
  temp.wifi.reserved = 0;
  sanitizeConfig(temp);

  WiFiConfig oldWifi = gConfig.wifi;
  gConfig = temp;
  saveConfig();

  bool changed = memcmp(&oldWifi, &gConfig.wifi, sizeof(WiFiConfig)) != 0;
  if (changed) {
    gApPending = true;
    gApPendingAt = nowMs64() + 5000ULL;
  }
  sendWifiSavedPage(changed);
}

static void handleApiData() {
  IPAddress ip = WiFi.softAPIP();
  char json[1024];

  int n = snprintf(json, sizeof(json),
    "{\"uptime_ms\":%" PRIu64 ",\"heap\":%u,\"clients\":%u,\"ip\":\"%u.%u.%u.%u\","
    "\"relay\":{\"enabled\":%u,\"on\":%u,\"mode\":%u,\"activeLow\":%u,\"safety\":%u},"
    "\"bme\":{\"en\":%u,\"ok\":%u,\"t\":%.2f,\"h\":%.2f,\"p\":%.2f},"
    "\"bh\":{\"en\":%u,\"ok\":%u,\"lux\":%.2f},"
    "\"mq\":{\"en\":%u,\"ok\":%u,\"raw\":%u,\"ready\":%u,\"alarm\":%u},"
    "\"soil\":{\"en\":%u,\"ok\":%u,\"pct_ok\":%u,\"raw\":%u,\"pct\":%u},"
    "\"rain\":{\"en\":%u,\"ok\":%u,\"pct_ok\":%u,\"raw\":%u,\"pct\":%u,\"alert\":%u,\"inhibit\":%u},"
    "\"alerts\":{\"mq\":%u,\"soil_dry\":%u,\"rain\":%u,\"relay_safety\":%u}}",
    nowMs64(),
    (unsigned)ESP.getFreeHeap(),
    (unsigned)WiFi.softAPgetStationNum(),
    (unsigned)ip[0], (unsigned)ip[1], (unsigned)ip[2], (unsigned)ip[3],
    (unsigned)gConfig.relay.enabled,
    (unsigned)gRelayOn,
    (unsigned)gConfig.relay.mode,
    (unsigned)gConfig.relay.activeLow,
    (unsigned)gRelaySafety,
    (unsigned)gConfig.bme.enabled,
    (unsigned)gBme.ok,
    (double)gBme.t,
    (double)gBme.h,
    (double)gBme.p,
    (unsigned)gConfig.bh.enabled,
    (unsigned)gBh.ok,
    (double)gBh.lux,
    (unsigned)gConfig.mq.enabled,
    (unsigned)gMq.ok,
    (unsigned)gMq.raw,
    (unsigned)gMq.ready,
    (unsigned)gMq.alarm,
    (unsigned)gConfig.soil.enabled,
    (unsigned)gSoil.ok,
    (unsigned)gSoil.ok,
    (unsigned)gSoil.raw,
    (unsigned)gSoil.pct,
    (unsigned)gConfig.rain.enabled,
    (unsigned)gRain.ok,
    (unsigned)gRain.ok,
    (unsigned)gRain.raw,
    (unsigned)gRain.pct,
    (unsigned)gRain.alert,
    (unsigned)rainInhibitActive(),
    (unsigned)gMq.alarm,
    (unsigned)(gSoil.ok && gSoil.pct <= gConfig.soil.startPct),
    (unsigned)gRain.alert,
    (unsigned)gRelaySafety
  );

  if (n < 0 || n >= (int)sizeof(json)) {
    sendNoCacheHeaders();
    gServer.send(500, "application/json", "{}");
    return;
  }

  sendNoCacheHeaders();
  gServer.send(200, "application/json", json);
}

static void handleApiRelay() {
  if (!gServer.hasArg("mode")) {
    sendNoCacheHeaders();
    gServer.send(400, "application/json", "{\"ok\":0}");
    return;
  }

  String m = gServer.arg("mode");
  uint8_t mode = 255;
  if (m == "auto") mode = 0;
  else if (m == "off") mode = 1;
  else if (m == "on") mode = 2;

  if (mode > 2) {
    sendNoCacheHeaders();
    gServer.send(400, "application/json", "{\"ok\":0}");
    return;
  }

  uint8_t oldMode = gConfig.relay.mode;
  if (mode != oldMode) {
    gRelaySafety = false;
    gLastRelayChange = 0;
  }

  gConfig.relay.mode = mode;
  saveConfig();

  uint64_t now = nowMs64();
  if (!gConfig.relay.enabled || mode == 1) {
    setRelay(false, now);
  } else if (mode == 2) {
    setRelay(true, now);
  }
  gRelayLastRun = 0;

  sendNoCacheHeaders();
  gServer.send(200, "application/json", "{\"ok\":1}");
}

static void handleNotFound() {
  String uri = gServer.uri();
  if (uri == "/favicon.ico") {
    gServer.send(204, "text/plain", "");
    return;
  }
  if (uri.startsWith("/api/")) {
    sendNoCacheHeaders();
    gServer.send(404, "application/json", "{}");
    return;
  }
  sendNoCacheHeaders();
  gServer.sendHeader("Location", AP_URL);
  gServer.send(302, "text/plain", "");
}

static void performFactoryReset() {
  gRelayOn = false;
  applyRelayPhysical();
  gDns.stop();
  gServer.stop();
  WiFi.softAPdisconnect(true);
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);

  Preferences p;
  p.begin(NVS_NS_A, false);
  p.clear();
  p.end();
  p.begin(NVS_NS_B, false);
  p.clear();
  p.end();

  nvs_flash_erase();
  esp_restart();
  while (true) {
    yield();
  }
}

static void taskBootButton(uint64_t now) {
  static bool low = false;
  static uint64_t lowSince = 0;

  if (gFactoryResetStarted) return;

  if (digitalRead(PIN_BOOT) == LOW) {
    if (!low) {
      low = true;
      lowSince = now;
    } else if ((now - lowSince) >= 5000ULL) {
      gFactoryResetStarted = true;
      performFactoryReset();
    }
  } else {
    low = false;
    lowSince = 0;
  }
}

static void applyApConfig() {
  gDns.stop();
  WiFi.persistent(false);
  esp_wifi_set_storage(WIFI_STORAGE_RAM);
  WiFi.mode(WIFI_AP);

  IPAddress ip(192, 168, 4, 1);
  IPAddress gw(192, 168, 4, 1);
  IPAddress mask(255, 255, 255, 0);
  WiFi.softAPConfig(ip, gw, mask);

  const char* pass = gConfig.wifi.password[0] ? gConfig.wifi.password : nullptr;
  bool ok = WiFi.softAP(gConfig.wifi.ssid, pass, gConfig.wifi.channel, gConfig.wifi.hidden, gConfig.wifi.maxClients);

  if (!ok) {
    copyText(gConfig.wifi.ssid, sizeof(gConfig.wifi.ssid), DEFAULT_SSID);
    copyText(gConfig.wifi.password, sizeof(gConfig.wifi.password), DEFAULT_PASSWORD);
    gConfig.wifi.channel = 6;
    gConfig.wifi.hidden = 0;
    gConfig.wifi.maxClients = 4;
    saveConfig();
    WiFi.softAPdisconnect(true);
    WiFi.softAP(DEFAULT_SSID, DEFAULT_PASSWORD, 6, 0, 4);
  }

  gDns.start(53, "*", WiFi.softAPIP());
  gApPending = false;
}

static void taskApPending(uint64_t now) {
  if (!gApPending || now < gApPendingAt) return;
  gApPending = false;
  applyApConfig();
}

static void registerRoutes() {
  gServer.on("/", HTTP_GET, handleDashboard);
  gServer.on("/settings/bme280", HTTP_GET, handleBmePage);
  gServer.on("/settings/bh1750", HTTP_GET, handleBhPage);
  gServer.on("/settings/mq2", HTTP_GET, handleMqPage);
  gServer.on("/settings/soil", HTTP_GET, handleSoilPage);
  gServer.on("/settings/rain", HTTP_GET, handleRainPage);
  gServer.on("/settings/relay", HTTP_GET, handleRelayPage);
  gServer.on("/settings/wifi", HTTP_GET, handleWifiPage);

  gServer.on("/save/bme280", HTTP_POST, handleBmeSave);
  gServer.on("/save/bh1750", HTTP_POST, handleBhSave);
  gServer.on("/save/mq2", HTTP_POST, handleMqSave);
  gServer.on("/save/soil", HTTP_POST, handleSoilSave);
  gServer.on("/save/rain", HTTP_POST, handleRainSave);
  gServer.on("/save/relay", HTTP_POST, handleRelaySave);
  gServer.on("/save/wifi", HTTP_POST, handleWifiSave);

  gServer.on("/api/data", HTTP_GET, handleApiData);
  gServer.on("/api/relay", HTTP_POST, handleApiRelay);
  gServer.onNotFound(handleNotFound);
}

void setup() {
  esp_log_level_set("*", ESP_LOG_NONE);

  gBootMs = nowMs64();

  pinMode(PIN_BOOT, INPUT_PULLUP);
  pinMode(PIN_RELAY, OUTPUT);
  digitalWrite(PIN_RELAY, HIGH);

  Wire.begin(PIN_SDA, PIN_SCL, 100000);
  Wire.setClock(100000);
  Wire.setTimeOut(50);

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_MQ2, ADC_11db);
  analogSetPinAttenuation(PIN_SOIL, ADC_11db);
  analogSetPinAttenuation(PIN_RAIN, ADC_11db);

  loadConfig();

  if (gConfig.relay.mode == 2) {
    gConfig.relay.mode = 0;
    saveConfig();
  }

  gRelayOn = false;
  gRelaySafety = false;
  gRelayOnSince = 0;
  gLastRelayChange = 0;
  gRelayLastRun = 0;
  applyRelayPhysical();

  applyApConfig();
  registerRoutes();
  gServer.begin();
}

void loop() {
  uint64_t now = nowMs64();

  taskBootButton(now);
  taskApPending(now);
  taskBme(now);
  taskBh(now);
  taskMq(now);
  taskSoil(now);
  taskRain(now);
  taskRelay(now);

  gDns.processNextRequest();
  gServer.handleClient();
  yield();
}
