/*
  =====================================================================
  HAPPY & MOIST — Challenge #2 IoT · ESP32
  Monitoreo hidrometeorologico con tablero web en la WLAN
  =====================================================================
  Que hace:
    - Mide nivel (JSN-SR04T), caudal (YF-S201), temperatura y humedad
      (DHT22), presion (BMP180) y luz/radiacion (BH1750).
    - Calcula evaporacion de agua libre con Penman (1948).
    - Alerta in situ: LCD 16x2 + buzzer.
    - Tablero web embebido en el ESP32: valor actual + historico,
      con usuario/contrasena y lista blanca opcional de IPs. Sin MQTT.
    - La medicion corre en una tarea FreeRTOS aparte (nucleo 0) y el
      caudal se cuenta por interrupcion (ISR). El servidor web corre
      en el loop principal (nucleo 1), asi nunca se bloquean entre si.

  Librerias (Arduino IDE -> Administrar bibliotecas):
    - DHT sensor library (Adafruit) + Adafruit Unified Sensor
    - Adafruit BMP085 Library
    - BH1750 (Christopher Laws)
  WiFi, WebServer, Wire y LiquidCrystal ya vienen con el paquete ESP32.

  Placa: Herramientas -> Placa -> esp32 -> ESP32 Dev Module
  =====================================================================
*/

#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <DHT.h>
#include <Adafruit_BMP085.h>
#include <BH1750.h>
#include <LiquidCrystal.h>
#include <math.h>

// =====================================================================
// 1. CONFIGURACION — CAMBIA ESTO
// =====================================================================

// Red WiFi a la que se conecta el ESP32 (la misma del celular / PC)
const char* WIFI_SSID = "NOMBRE_DE_TU_RED";
const char* WIFI_PASS = "CLAVE_DE_TU_RED";

// Si no logra conectarse, crea su propia red con estos datos.
// Conectate a ella desde el celular y abre http://192.168.4.1
const char* AP_SSID = "HappyMoist";
const char* AP_PASS = "happymoist123";   // minimo 8 caracteres

// Usuario y contrasena del tablero
const char* WEB_USER = "admin";
const char* WEB_PASS = "moist2026";

// true  = valores simulados (para probar el tablero sin sensores)
// false = lectura real de los sensores
#define MODO_DEMO true

// Lista blanca de IPs. Dejala en false hasta saber la IP de tu
// celular (aparece en el Monitor Serie cada vez que abres la pagina).
const bool WHITELIST_ACTIVA = false;
IPAddress WHITELIST[] = {
  IPAddress(192, 168, 1, 50),
  IPAddress(192, 168, 1, 51)
};
const int WHITELIST_N = sizeof(WHITELIST) / sizeof(WHITELIST[0]);

// =====================================================================
// 2. PINES (igual a la hoja de cableado ESP32)
// =====================================================================
#define PIN_TRIG     17
#define PIN_ECHO     18     // con divisor 1k/2k
#define PIN_DHT       4
#define PIN_CAUDAL   16     // con divisor 1k/2k
#define PIN_BUZZER   25
#define PIN_SDA      21
#define PIN_SCL      22

#define LCD_RS 19
#define LCD_E  23
#define LCD_D4 32
#define LCD_D5 33
#define LCD_D6 26
#define LCD_D7 27

// =====================================================================
// 3. PARAMETROS DEL SISTEMA
// =====================================================================
const float ALTURA_TANQUE_CM   = 200.0;  // distancia del sensor al fondo
const float NIVEL_ALERTA_CM    = 40.0;   // nivel minimo antes de alertar
const float EVAP_ALERTA_MMDIA  = 4.0;    // evaporacion alta (mm/dia)
const float HUMEDAD_ALERTA     = 20.0;   // aire muy seco (%)
const float PRESION_ALERTA_HPA = 735.0;  // presion anomala para ~2600 m (normal ~745)
const float u2_fijo            = 2.0;    // viento asumido (m/s), FAO-56
const float ALBEDO_AGUA        = 0.08;
const float PULSOS_POR_LMIN    = 7.5;    // YF-S201: f(Hz) = 7.5 * Q(L/min)

const unsigned long MEDICION_MS  = 5000;   // cada cuanto se mide
const unsigned long HISTORIAL_MS = 10000;  // cada cuanto se guarda en el historico
                                           // (en campo: 3600000 = 1 hora)
#define HISTORIAL_MAX 120                  // registros guardados (anillo)

// =====================================================================
// 4. ESTRUCTURAS COMPARTIDAS
// =====================================================================
struct Lectura {
  unsigned long t;      // segundos desde el arranque
  float nivel;          // cm
  float caudal;         // L/min
  float temp;           // C
  float hum;            // %
  float presion;        // hPa
  float radiacion;      // W/m2
  float evap;           // mm/dia
  bool  alerta;
  bool  aNivel, aEvap, aClima;
};

Lectura actual;
Lectura historial[HISTORIAL_MAX];
int histInicio = 0;
int histCuenta = 0;

SemaphoreHandle_t mutexDatos;

// Contador de pulsos del caudalimetro, escrito desde la ISR
volatile uint32_t pulsosCaudal = 0;
portMUX_TYPE muxCaudal = portMUX_INITIALIZER_UNLOCKED;

// =====================================================================
// 5. OBJETOS
// =====================================================================
DHT dht(PIN_DHT, DHT22);
Adafruit_BMP085 bmp;
BH1750 lightMeter;
LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
WebServer server(80);

bool bmpOK = false;
bool bh1750OK = false;
bool modoAP = false;

// =====================================================================
// 6. ISR DEL CAUDALIMETRO
// =====================================================================
void IRAM_ATTR isrCaudal() {
  portENTER_CRITICAL_ISR(&muxCaudal);
  pulsosCaudal++;
  portEXIT_CRITICAL_ISR(&muxCaudal);
}

// =====================================================================
// 7. LECTURA DE SENSORES
// =====================================================================
float leerNivel() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(20);           // el JSN-SR04T prefiere pulso >= 10 us
  digitalWrite(PIN_TRIG, LOW);

  long dur = pulseIn(PIN_ECHO, HIGH, 30000);
  if (dur == 0) return NAN;        // sin eco -> sin dato (no se asume vacio)

  float dist = dur * 0.0343 / 2.0;
  if (dist < 20.0 || dist > 450.0) return NAN;   // fuera del rango util

  float nivel = ALTURA_TANQUE_CM - dist;
  if (nivel < 0) nivel = 0;
  return nivel;
}

float leerCaudal(unsigned long ventanaMs) {
  uint32_t p;
  portENTER_CRITICAL(&muxCaudal);
  p = pulsosCaudal;
  pulsosCaudal = 0;
  portEXIT_CRITICAL(&muxCaudal);
  float hz = p / (ventanaMs / 1000.0);
  return hz / PULSOS_POR_LMIN;
}

// Penman (1948), forma de flujo de masa. Devuelve mm/dia.
float calcularPenman(float T, float HR, float presion_hPa, float rad_Wm2, float u2) {
  if (isnan(T) || isnan(HR) || isnan(presion_hPa) || isnan(rad_Wm2)) return NAN;

  const float k = 0.41, z = 2.0, z0 = 0.0002, cp = 1005.0, Rd = 287.05, rhoW = 1000.0;
  float T_K = T + 273.15;
  float P   = presion_hPa * 100.0;

  float es  = 0.6108 * exp((17.27 * T) / (T + 237.3));     // kPa
  float ea  = es * (HR / 100.0);
  float de  = (es - ea) * 1000.0;                           // Pa
  float m   = 1000.0 * (4098.0 * es) / pow(T + 237.3, 2);   // Pa/K
  float lv  = (2.501 - 0.002361 * T) * 1.0e6;               // J/kg
  float gam = 0.665e-3 * P;                                 // Pa/K
  float rho = P / (Rd * T_K);                               // kg/m3
  float ga  = (k * k * u2) / pow(log(z / z0), 2);           // m/s
  float Rn  = rad_Wm2 * (1.0 - ALBEDO_AGUA);
  if (Rn < 0) Rn = 0;

  float E = (m * Rn + rho * cp * de * ga) / (lv * (m + gam)); // kg/(m2 s)
  if (E < 0) E = 0;
  return E / rhoW * 1000.0 * 86400.0;                        // mm/dia
}

// Valores simulados para probar el tablero sin hardware
void llenarDemo(Lectura &L) {
  float s = millis() / 1000.0;
  L.nivel     = 120.0 + 90.0 * sin(s / 60.0);          // sube y baja, cruza el umbral
  L.caudal    = 4.0 + 1.5 * sin(s / 23.0);
  L.temp      = 14.0 + 3.0 * sin(s / 90.0);
  L.hum       = 75.0 - 10.0 * sin(s / 70.0);
  L.presion   = 744.0 + 2.0 * sin(s / 120.0);
  L.radiacion = 250.0 + 200.0 * sin(s / 45.0);
}

// =====================================================================
// 8. LCD Y BUZZER (solo los usa la tarea de medicion)
// =====================================================================
void imprimirValor(float v, int dec) {
  if (isnan(v)) lcd.print("--");
  else lcd.print(v, dec);
}

void mostrarLCD(const Lectura &L) {
  lcd.clear();
  lcd.setCursor(0, 0);
  if (L.alerta) {
    lcd.print("ALERTA N:");
    imprimirValor(L.nivel, 0);
    lcd.print("cm");
  } else {
    lcd.print("N:");
    imprimirValor(L.nivel, 0);
    lcd.print("cm T:");
    imprimirValor(L.temp, 0);
    lcd.print("C");
  }
  lcd.setCursor(0, 1);
  lcd.print("H:");
  imprimirValor(L.hum, 0);
  lcd.print("% Ev:");
  imprimirValor(L.evap, 1);
}

void sonarAlerta() {
  for (int i = 0; i < 5; i++) {
    tone(PIN_BUZZER, 2000);
    vTaskDelay(pdMS_TO_TICKS(150));
    noTone(PIN_BUZZER);
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// =====================================================================
// 9. TAREA DE MEDICION (nucleo 0, separada del servidor web)
// =====================================================================
void tareaMedicion(void *param) {
  unsigned long ultimoHist = 0;
  unsigned long ultimaMed  = millis();

  for (;;) {
    Lectura L;
    L.t = millis() / 1000;

    unsigned long ahora = millis();
    unsigned long ventana = ahora - ultimaMed;
    ultimaMed = ahora;

    if (MODO_DEMO) {
      llenarDemo(L);
    } else {
      L.nivel  = leerNivel();
      L.caudal = leerCaudal(ventana > 0 ? ventana : MEDICION_MS);
      L.hum    = dht.readHumidity();
      L.temp   = dht.readTemperature();

      L.presion = bmpOK ? bmp.readPressure() / 100.0 : NAN;
      if (!isnan(L.presion) && (L.presion < 500.0 || L.presion > 1100.0)) L.presion = NAN;

      float lux = bh1750OK ? lightMeter.readLightLevel() : -1;
      L.radiacion = (lux >= 0) ? lux / 126.0 : NAN;
    }

    L.evap = calcularPenman(L.temp, L.hum, L.presion, L.radiacion, u2_fijo);

    // Fusion de senales. Un dato faltante (NAN) nunca dispara alerta:
    // asi un sensor caido no se confunde con una condicion real.
    L.aNivel = !isnan(L.nivel) && L.nivel <= NIVEL_ALERTA_CM;
    L.aEvap  = !isnan(L.evap)  && L.evap  >= EVAP_ALERTA_MMDIA;
    L.aClima = (!isnan(L.hum)     && L.hum     < HUMEDAD_ALERTA) ||
               (!isnan(L.presion) && L.presion < PRESION_ALERTA_HPA);
    L.alerta = L.aNivel && (L.aEvap || L.aClima);

    // Guardar en memoria compartida
    xSemaphoreTake(mutexDatos, portMAX_DELAY);
    actual = L;
    if (ahora - ultimoHist >= HISTORIAL_MS || histCuenta == 0) {
      ultimoHist = ahora;
      int pos = (histInicio + histCuenta) % HISTORIAL_MAX;
      historial[pos] = L;
      if (histCuenta < HISTORIAL_MAX) histCuenta++;
      else histInicio = (histInicio + 1) % HISTORIAL_MAX;   // borra el mas viejo
    }
    xSemaphoreGive(mutexDatos);

    // Salidas in situ
    mostrarLCD(L);
    Serial.printf("[med] N=%.1f Q=%.2f T=%.1f H=%.1f P=%.1f R=%.1f E=%.2f ALERTA=%s\n",
                  L.nivel, L.caudal, L.temp, L.hum, L.presion, L.radiacion, L.evap,
                  L.alerta ? "SI" : "NO");

    if (L.alerta) sonarAlerta();
    else noTone(PIN_BUZZER);

    vTaskDelay(pdMS_TO_TICKS(MEDICION_MS));
  }
}

// =====================================================================
// 10. SERVIDOR WEB
// =====================================================================
const char PAGINA[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html lang="es"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Happy & Moist</title>
<style>
:root{--bg:#0B1E24;--panel:#12303A;--line:#24505C;--txt:#EAF4F4;--mut:#86A6AC;--cy:#4FC3E0;--am:#F2A65A;--rd:#E4572E;--gr:#8FD19E}
*{box-sizing:border-box;margin:0;padding:0}
body{background:var(--bg);color:var(--txt);font-family:system-ui,sans-serif;padding:16px;max-width:900px;margin:auto}
h1{font-size:22px;margin-bottom:2px}.sub{color:var(--mut);font-size:13px;margin-bottom:14px}
.estado{padding:12px 14px;border-radius:6px;font-weight:600;margin-bottom:14px;background:var(--panel);border-left:4px solid var(--gr)}
.estado.alerta{border-color:var(--rd);background:#3A1A16}
.grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(140px,1fr));gap:10px;margin-bottom:16px}
.card{background:var(--panel);border:1px solid var(--line);border-radius:6px;padding:12px}
.card .k{font-size:12px;color:var(--mut)}.card .v{font-size:24px;font-weight:600;margin-top:4px}
.card .u{font-size:13px;color:var(--mut);font-weight:400}
.card.on{border-color:var(--rd)}
.box{background:var(--panel);border:1px solid var(--line);border-radius:6px;padding:12px;margin-bottom:16px}
select{background:var(--bg);color:var(--txt);border:1px solid var(--line);padding:6px;border-radius:4px;margin-bottom:8px}
canvas{width:100%;height:220px;display:block}
table{width:100%;border-collapse:collapse;font-size:12.5px}
th,td{padding:6px;border-bottom:1px solid var(--line);text-align:right}th{color:var(--mut);font-weight:500}
td:first-child,th:first-child{text-align:left}
.scroll{max-height:320px;overflow:auto}
.sd{color:var(--mut)}
</style></head><body>
<h1>Happy &amp; Moist</h1>
<div class="sub" id="sub">Conectando...</div>
<div class="estado" id="estado">Esperando datos...</div>
<div class="grid" id="grid"></div>
<div class="box">
 <select id="var"></select>
 <canvas id="cv"></canvas>
</div>
<div class="box"><div class="scroll"><table id="tabla"></table></div></div>
<script>
const V=[["nivel","Nivel","cm",1],["caudal","Caudal","L/min",2],["temp","Temperatura","°C",1],
["hum","Humedad","%",1],["presion","Presión","hPa",1],["radiacion","Radiación","W/m²",1],["evap","Evaporación","mm/día",2]];
const sel=document.getElementById('var');
V.forEach(v=>{const o=document.createElement('option');o.value=v[0];o.textContent=v[1]+' ('+v[2]+')';sel.appendChild(o);});
sel.value='nivel';let hist=[];
const f=(x,d)=>x===null?'<span class="sd">sin dato</span>':x.toFixed(d);
async function actual(){
 try{const r=await fetch('/api/actual');const d=await r.json();
 document.getElementById('sub').textContent='Modo '+d.modo+' · '+d.red+' · encendido hace '+d.t+' s';
 const e=document.getElementById('estado');
 e.className='estado'+(d.alerta?' alerta':'');
 e.textContent=d.alerta?'ALERTA CRÍTICA: nivel bajo + '+(d.aEvap?'evaporación alta':'condición climática anómala'):'Sin alerta';
 const on={nivel:d.aNivel,evap:d.aEvap,hum:d.aClima,presion:d.aClima};
 document.getElementById('grid').innerHTML=V.map(v=>'<div class="card'+(on[v[0]]?' on':'')+'"><div class="k">'+v[1]+'</div><div class="v">'+f(d[v[0]],v[3])+' <span class="u">'+v[2]+'</span></div></div>').join('');
 }catch(e){document.getElementById('sub').textContent='Sin conexión con el ESP32';}
}
async function historial(){
 try{const r=await fetch('/api/historial');hist=await r.json();dibujar();tabla();}catch(e){}
}
function dibujar(){
 const c=document.getElementById('cv'),x=c.getContext('2d'),dpr=window.devicePixelRatio||1;
 const W=c.clientWidth,H=c.clientHeight;c.width=W*dpr;c.height=H*dpr;x.scale(dpr,dpr);x.clearRect(0,0,W,H);
 const k=sel.value,pts=hist.map(h=>h[k]).filter(v=>v!==null);
 x.fillStyle='#86A6AC';x.font='12px system-ui';
 if(pts.length<2){x.fillText('Aún no hay suficientes datos',10,20);return;}
 let mn=Math.min(...pts),mx=Math.max(...pts);if(mn===mx){mn-=1;mx+=1;}
 const L=46,R=10,T=10,B=22,w=W-L-R,h=H-T-B;
 x.strokeStyle='#24505C';x.lineWidth=1;
 for(let i=0;i<=4;i++){const y=T+h*i/4;x.beginPath();x.moveTo(L,y);x.lineTo(W-R,y);x.stroke();
  x.fillText((mx-(mx-mn)*i/4).toFixed(1),4,y+4);}
 const thr={nivel:%NIVEL%,evap:%EVAP%,presion:%PRES%,hum:%HUM%}[k];
 if(thr!==undefined&&thr>=mn&&thr<=mx){const y=T+h*(1-(thr-mn)/(mx-mn));x.strokeStyle='#E4572E';x.setLineDash([5,4]);
  x.beginPath();x.moveTo(L,y);x.lineTo(W-R,y);x.stroke();x.setLineDash([]);x.fillStyle='#E4572E';x.fillText('umbral',W-R-44,y-4);}
 x.strokeStyle='#4FC3E0';x.lineWidth=2;x.beginPath();let first=true;
 hist.forEach((p,i)=>{const v=p[k];if(v===null){first=true;return;}
  const px=L+w*i/(hist.length-1),py=T+h*(1-(v-mn)/(mx-mn));
  if(first){x.moveTo(px,py);first=false;}else x.lineTo(px,py);});
 x.stroke();x.fillStyle='#86A6AC';
 x.fillText('t='+hist[0].t+' s',L,H-6);x.fillText('t='+hist[hist.length-1].t+' s',W-R-60,H-6);
}
function tabla(){
 let s='<tr><th>t (s)</th>'+V.map(v=>'<th>'+v[1]+'</th>').join('')+'<th>Alerta</th></tr>';
 hist.slice().reverse().forEach(h=>{s+='<tr><td>'+h.t+'</td>'+V.map(v=>'<td>'+f(h[v[0]],v[3])+'</td>').join('')+'<td>'+(h.alerta?'SÍ':'—')+'</td></tr>';});
 document.getElementById('tabla').innerHTML=s;
}
sel.onchange=dibujar;window.onresize=dibujar;
actual();historial();setInterval(actual,3000);setInterval(historial,10000);
</script></body></html>
)rawliteral";

String num(float v, int d) {
  if (isnan(v)) return "null";
  return String(v, d);
}

String lecturaJSON(const Lectura &L) {
  String s = "{";
  s += "\"t\":" + String(L.t);
  s += ",\"nivel\":" + num(L.nivel, 1);
  s += ",\"caudal\":" + num(L.caudal, 2);
  s += ",\"temp\":" + num(L.temp, 1);
  s += ",\"hum\":" + num(L.hum, 1);
  s += ",\"presion\":" + num(L.presion, 1);
  s += ",\"radiacion\":" + num(L.radiacion, 1);
  s += ",\"evap\":" + num(L.evap, 2);
  s += ",\"alerta\":" + String(L.alerta ? "true" : "false");
  s += ",\"aNivel\":" + String(L.aNivel ? "true" : "false");
  s += ",\"aEvap\":" + String(L.aEvap ? "true" : "false");
  s += ",\"aClima\":" + String(L.aClima ? "true" : "false");
  s += "}";
  return s;
}

// Devuelve true si la peticion puede pasar (lista blanca + login)
bool autorizado() {
  IPAddress ip = server.client().remoteIP();
  Serial.printf("[web] %s %s desde %s\n",
                server.method() == HTTP_GET ? "GET" : "POST",
                server.uri().c_str(), ip.toString().c_str());

  if (WHITELIST_ACTIVA) {
    bool ok = false;
    for (int i = 0; i < WHITELIST_N; i++) if (WHITELIST[i] == ip) { ok = true; break; }
    if (!ok) {
      server.send(403, "text/plain", "Acceso denegado: este dispositivo no esta autorizado.");
      return false;
    }
  }
  if (!server.authenticate(WEB_USER, WEB_PASS)) {
    server.requestAuthentication(BASIC_AUTH, "Happy & Moist", "Credenciales requeridas");
    return false;
  }
  return true;
}

void handleRaiz() {
  if (!autorizado()) return;
  String p = FPSTR(PAGINA);
  p.replace("%NIVEL%", String(NIVEL_ALERTA_CM, 1));
  p.replace("%EVAP%",  String(EVAP_ALERTA_MMDIA, 1));
  p.replace("%PRES%",  String(PRESION_ALERTA_HPA, 1));
  p.replace("%HUM%",   String(HUMEDAD_ALERTA, 1));
  server.send(200, "text/html; charset=utf-8", p);
}

void handleActual() {
  if (!autorizado()) return;
  xSemaphoreTake(mutexDatos, portMAX_DELAY);
  Lectura L = actual;
  xSemaphoreGive(mutexDatos);

  String s = lecturaJSON(L);
  s.remove(s.length() - 1);   // quita la } final para agregar campos
  s += ",\"modo\":\"" + String(MODO_DEMO ? "demo" : "sensores") + "\"";
  s += ",\"red\":\"" + String(modoAP ? "red propia (AP)" : WIFI_SSID) + "\"";
  s += "}";
  server.send(200, "application/json", s);
}

void handleHistorial() {
  if (!autorizado()) return;
  String s = "[";
  xSemaphoreTake(mutexDatos, portMAX_DELAY);
  for (int i = 0; i < histCuenta; i++) {
    if (i) s += ",";
    s += lecturaJSON(historial[(histInicio + i) % HISTORIAL_MAX]);
  }
  xSemaphoreGive(mutexDatos);
  s += "]";
  server.send(200, "application/json", s);
}

// =====================================================================
// 11. WIFI
// =====================================================================
void iniciarWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("Conectando a %s", WIFI_SSID);
  lcd.clear(); lcd.print("Conectando WiFi");

  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  IPAddress ip;
  if (WiFi.status() == WL_CONNECTED) {
    ip = WiFi.localIP();
    Serial.printf("Conectado. Abre http://%s\n", ip.toString().c_str());
  } else {
    modoAP = true;
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    ip = WiFi.softAPIP();
    Serial.printf("No se pudo conectar. Red propia \"%s\" (clave %s). Abre http://%s\n",
                  AP_SSID, AP_PASS, ip.toString().c_str());
  }

  lcd.clear();
  lcd.setCursor(0, 0); lcd.print(modoAP ? "Red: HappyMoist" : "WiFi conectado");
  lcd.setCursor(0, 1); lcd.print(ip.toString());
  delay(4000);
}

// =====================================================================
// 12. SETUP Y LOOP
// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_CAUDAL, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_CAUDAL), isrCaudal, FALLING);

  lcd.begin(16, 2);
  lcd.print("Happy & Moist");

  dht.begin();
  Wire.begin(PIN_SDA, PIN_SCL);
  if (!MODO_DEMO) {
    bmpOK = bmp.begin();
    bh1750OK = lightMeter.begin();
    Serial.printf("BMP180: %s | BH1750: %s\n", bmpOK ? "OK" : "NO DETECTADO",
                  bh1750OK ? "OK" : "NO DETECTADO");
  }

  mutexDatos = xSemaphoreCreateMutex();
  actual.t = 0;
  actual.nivel = actual.caudal = actual.temp = actual.hum = NAN;
  actual.presion = actual.radiacion = actual.evap = NAN;
  actual.alerta = actual.aNivel = actual.aEvap = actual.aClima = false;

  iniciarWiFi();

  server.on("/", handleRaiz);
  server.on("/api/actual", handleActual);
  server.on("/api/historial", handleHistorial);
  server.onNotFound([]() { server.send(404, "text/plain", "No encontrado"); });
  server.begin();

  // La medicion corre en su propia tarea, en el nucleo 0
  xTaskCreatePinnedToCore(tareaMedicion, "medicion", 8192, NULL, 1, NULL, 0);
}

void loop() {
  // Hilo principal: solo atiende el servidor web
  server.handleClient();
  delay(2);
}
