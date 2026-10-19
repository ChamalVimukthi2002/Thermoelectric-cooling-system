/*
 * ESP8266 TEC LQR Controller with Web Interface
 * Hardware: NodeMCU/Wemos D1 Mini, DS18B20 x2, MOSFETs, Current Sensor
 * 
 * Pins:
 * - D3 (GPIO 0):  DS18B20 Data (Requires 4.7k Pull-up) 0-cold 1-hot
 * - D5 (GPIO 14): TEC PWM Output
 * - D6 (GPIO 12): Cold Fan Output
 * - D7 (GPIO 13): Hot Fan Output
 * - A0: Current Sensor (1A = 0.5V)
 */

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// --- WiFi Configuration ---
const char* WIFI_SSID = "";
const char* WIFI_PASS = "";

// Pin Definitions
#define PIN_TEC_PWM   14 // D5
#define PIN_FAN_COLD  13 // D7
#define PIN_FAN_HOT   12 // D6
#define PIN_ONE_WIRE  0  // D3
#define PIN_CURRENT   A0

// Safety Limits
#define MAX_HOT_TEMP  60.0 // Celsius
#define MAX_CURRENT   6.0  // Amps

// Objects
OneWire oneWire(PIN_ONE_WIRE); 
DallasTemperature sensors(&oneWire);  
DeviceAddress tempDeviceAddress; 

ESP8266WebServer server(80);

// Global Variables
float tempCool = 0.0;
float tempHot = 0.0;
float targetTemp = 10.0; 
float currentAmps = 0.0;
int pwmOutput = 0;
int numberOfDevices;

// System States
bool isAutoMode = true;       
bool tecMasterEnable = true;  
bool manualFanC = false;      
bool manualFanH = false;      
int manualPWM = 0;

// Hardware Status (For feedback to web UI)
bool stateFanC = false;
bool stateFanH = false;

// --- LQR Controller Class ---
class LQRController {
  private:
    float integralError = 0.0;
    unsigned long lastTime;
    float K_p = 255.0 / 5.0; 
    float K_i = 0.5;         

  public:
    void begin() {
      lastTime = millis();
      integralError = 0.0;
    }

    int compute(float setpoint, float input) {
      unsigned long now = millis();
      float dt = (now - lastTime) / 1000.0;
      lastTime = now;

      if (dt <= 0 || dt > 1.0) dt = 0.1; 

      float error = setpoint - input; 

      if (pwmOutput > 0 && pwmOutput < 255) {
        integralError += error * dt;
      }
      
      float output = (K_p * error) + (K_i * integralError);

      if (output > 255) output = 255;
      if (output < 0) output = 0;

      return (int)output;
    }
    
    void reset() {
      integralError = 0.0;
    }
};

LQRController lqr;

// Current Sensor reading function
float current(int pin){
  int analog = 0;
  int test = 0;
  float I1;
  float R_shunt = 0.006059159991357;  
  float R3 = 985;
  float R4 = 19670;
  float Vo;
  float V_ref;
  int zero_point = 51;
  V_ref = (float)zero_point*(3.3/1023.0);

  test = analogRead(pin);
  Serial.print("  ");
  Serial.print(test);
  Serial.print("  ");

  for(int i = 0; i < 10; i++){
    analog += analogRead(pin);
  }

  analog = analog / 10;
  Vo = (float)analog*(3.3/1023.0);
  I1 = (Vo-V_ref)/(R_shunt*(R4/R3));
  return I1;
}

// --- HTML/JS Webpage String ---
const char INDEX_HTML[] PROGMEM = R"=====(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>TEC Controller Dashboard</title>
  <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
  <style>
    body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; background: #121212; color: #ffffff; margin: 0; padding: 20px; }
    h1 { text-align: center; color: #4CAF50; }
    .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); gap: 20px; }
    .card { background: #1e1e1e; padding: 20px; border-radius: 10px; box-shadow: 0 4px 8px rgba(0,0,0,0.3); }
    .stat { font-size: 1.5em; font-weight: bold; color: #4CAF50; margin: 5px 0; }
    button { background: #333; color: #fff; border: 1px solid #555; padding: 8px 15px; border-radius: 5px; cursor: pointer; margin: 5px; }
    button:hover { background: #555; }
    button.active { background: #4CAF50; color: white; border-color: #4CAF50; }
    button.danger { background: #d32f2f; border-color: #d32f2f; }
    input { padding: 8px; border-radius: 5px; border: 1px solid #555; background: #333; color: white; width: 80px; margin-right: 10px;}
    .control-group { margin-bottom: 15px; display: flex; align-items: center; justify-content: space-between; }
    canvas { max-width: 100%; max-height: 250px; }
  </style>
</head>
<body>
  <h1>TEC LQR System Dashboard</h1>
  <div style="text-align: center; margin-bottom: 20px;">Uptime: <span id="uptime">0</span>s</div>
  
  <div class="grid">
    <!-- Status Card -->
    <div class="card">
      <h3>System Readings</h3>
      <div class="control-group"><span>Cold Temp:</span> <span class="stat" id="tC">0.0 &deg;C</span></div>
      <div class="control-group"><span>Hot Temp:</span> <span class="stat" id="tH" style="color: #ff9800;">0.0 &deg;C</span></div>
      <div class="control-group"><span>Current:</span> <span class="stat" id="curr">0.0 A</span></div>
      <div class="control-group"><span>PWM Output:</span> <span class="stat" id="pwm">0 (0%)</span></div>
      <div class="control-group"><span>Hardware Status:</span> <span id="hw_status" style="color: #bbb;">Loading...</span></div>
    </div>

    <!-- Controls Card -->
    <div class="card">
      <h3>Control Panel</h3>
      <div class="control-group">
        <span>Master TEC Enable:</span>
        <div><button id="btn_tec">ON</button></div>
      </div>
      <div class="control-group">
        <span>Control Mode:</span>
        <div>
          <button id="btn_auto" onclick="cmd('mode', 1)">AUTO (LQR)</button>
          <button id="btn_man" onclick="cmd('mode', 0)">MANUAL</button>
        </div>
      </div>
      <div class="control-group" id="div_target">
        <span>Target Temp (&deg;C):</span>
        <div><input type="number" id="inp_tgt" step="0.5"><button onclick="cmd('target', document.getElementById('inp_tgt').value)">Set</button></div>
      </div>
      <div class="control-group" id="div_mpwm" style="display:none;">
        <span>Manual PWM (0-255):</span>
        <div><input type="number" id="inp_pwm" min="0" max="255"><button onclick="cmd('mpwm', document.getElementById('inp_pwm').value)">Set</button></div>
      </div>
      <div class="control-group" id="div_mfans" style="display:none;">
        <span>Manual Fans:</span>
        <div>
          <button id="btn_fc" onclick="cmd('fanc', 1)">Cold ON</button>
          <button id="btn_fh" onclick="cmd('fanh', 1)">Hot ON</button>
        </div>
      </div>
    </div>
  </div>

  <div class="grid" style="margin-top: 20px;">
    <div class="card">
      <h3>Temperatures</h3>
      <canvas id="tempChart"></canvas>
    </div>
    <div class="card">
      <h3>Current</h3>
      <canvas id="currChart"></canvas>
    </div>
  </div>

  <script>
    const ctxTemp = document.getElementById('tempChart').getContext('2d');
    const ctxCurr = document.getElementById('currChart').getContext('2d');
    
    const chartOpts = { responsive: true, animation: false, scales: { x: { display: false } }, elements: { point: { radius: 0 } } };
    
    const tempChart = new Chart(ctxTemp, {
      type: 'line',
      data: { labels: [], datasets: [
        { label: 'Cold Temp (C)', borderColor: '#2196f3', data: [] },
        { label: 'Hot Temp (C)', borderColor: '#ff9800', data: [] },
        { label: 'Target (C)', borderColor: '#4CAF50', borderDash: [5, 5], data: [] }
      ]},
      options: chartOpts
    });

    const currChart = new Chart(ctxCurr, {
      type: 'line',
      data: { labels: [], datasets: [{ label: 'Current (A)', borderColor: '#e91e63', data: [] }] },
      options: { ...chartOpts, scales: { y: { min: 0 } } }
    });

    const cmd = (action, value) => {
      fetch(`/api?cmd=${action}&val=${value}`).then(updateData);
    };

    const updateData = async () => {
      try {
        const res = await fetch('/data');
        const d = await res.json();

        document.getElementById('tC').innerHTML = d.tC.toFixed(1) + ' &deg;C';
        document.getElementById('tH').innerHTML = d.tH.toFixed(1) + ' &deg;C';
        document.getElementById('curr').innerText = d.cur.toFixed(2) + ' A';
        document.getElementById('pwm').innerText = `${d.pwm} (${Math.round((d.pwm/255)*100)}%)`;
        document.getElementById('uptime').innerText = d.up;

        document.getElementById('hw_status').innerText = `TEC: ${d.tec ? 'ON':'OFF'} | Fan C: ${d.fC ? 'ON':'OFF'} | Fan H: ${d.fH ? 'ON':'OFF'}`;

        // THIS IS THE FIX: Properly toggle the TEC button text, color, AND onclick action
        document.getElementById('btn_tec').innerText = d.tec ? 'TURN OFF TEC' : 'TURN ON TEC';
        document.getElementById('btn_tec').className = d.tec ? 'danger' : 'active';
        document.getElementById('btn_tec').setAttribute('onclick', `cmd('tec', ${d.tec ? 0 : 1})`);
        
        document.getElementById('btn_auto').className = d.auto ? 'active' : '';
        document.getElementById('btn_man').className = !d.auto ? 'active' : '';

        document.getElementById('div_target').style.display = d.auto ? 'flex' : 'none';
        document.getElementById('div_mpwm').style.display = !d.auto ? 'flex' : 'none';
        document.getElementById('div_mfans').style.display = !d.auto ? 'flex' : 'none';

        if(document.activeElement.id !== 'inp_tgt') document.getElementById('inp_tgt').value = d.tgt;
        
        if(!d.auto) {
           document.getElementById('btn_fc').innerText = d.fC ? 'Cold: ON' : 'Cold: OFF';
           document.getElementById('btn_fc').className = d.fC ? 'active' : '';
           document.getElementById('btn_fc').setAttribute('onclick', `cmd('fanc', ${d.fC ? 0 : 1})`);

           document.getElementById('btn_fh').innerText = d.fH ? 'Hot: ON' : 'Hot: OFF';
           document.getElementById('btn_fh').className = d.fH ? 'active' : '';
           document.getElementById('btn_fh').setAttribute('onclick', `cmd('fanh', ${d.fH ? 0 : 1})`);
        }

        const timeLab = new Date().toLocaleTimeString();
        if(tempChart.data.labels.length > 60) {
          tempChart.data.labels.shift();
          tempChart.data.datasets.forEach(ds => ds.data.shift());
          currChart.data.labels.shift();
          currChart.data.datasets[0].data.shift();
        }
        
        tempChart.data.labels.push(timeLab);
        tempChart.data.datasets[0].data.push(d.tC);
        tempChart.data.datasets[1].data.push(d.tH);
        tempChart.data.datasets[2].data.push(d.tgt);
        tempChart.update();

        currChart.data.labels.push(timeLab);
        currChart.data.datasets[0].data.push(d.cur);
        currChart.update();

      } catch (err) { console.error('Fetch error: ', err); }
    };

    setInterval(updateData, 1000);
    updateData(); 
  </script>
</body>
</html>
)=====";


// --- Web Server Handlers ---
void handleRoot() {
  server.send(200, "text/html", INDEX_HTML);
}

void handleData() {
  String json = "{";
  json += "\"tC\":" + String(tempCool, 1) + ",";
  json += "\"tH\":" + String(tempHot, 1) + ",";
  json += "\"cur\":" + String(currentAmps, 2) + ",";
  json += "\"pwm\":" + String(pwmOutput) + ",";
  json += "\"up\":" + String(millis() / 1000) + ",";
  json += "\"tgt\":" + String(targetTemp, 1) + ",";
  json += "\"auto\":" + String(isAutoMode ? 1 : 0) + ",";
  json += "\"tec\":" + String(tecMasterEnable ? 1 : 0) + ",";
  json += "\"fC\":" + String(stateFanC ? 1 : 0) + ",";
  json += "\"fH\":" + String(stateFanH ? 1 : 0);
  json += "}";
  
  server.send(200, "application/json", json);
}

void handleAPI() {
  if (server.hasArg("cmd") && server.hasArg("val")) {
    String cmd = server.arg("cmd");
    float val = server.arg("val").toFloat();

    if (cmd == "mode") {
      isAutoMode = (val == 1);
      if(isAutoMode) lqr.reset(); 
    } 
    else if (cmd == "target") {
      targetTemp = val;
    } 
    else if (cmd == "mpwm" && !isAutoMode) {
      manualPWM = (int)val;
      if (manualPWM > 255) manualPWM = 255;
      if (manualPWM < 0) manualPWM = 0;
    } 
    else if (cmd == "tec") {
      tecMasterEnable = (val == 1);
    } 
    else if (cmd == "fanc" && !isAutoMode) {
      manualFanC = (val == 1);
    } 
    else if (cmd == "fanh" && !isAutoMode) {
      manualFanH = (val == 1);
    }
    
    server.send(200, "text/plain", "OK");
  } else {
    server.send(400, "text/plain", "Bad Request");
  }
}

// function to print a device address
void printAddress(DeviceAddress deviceAddress) {
  for (uint8_t i = 0; i < 8; i++){
    if (deviceAddress[i] < 16) Serial.print("0");
      Serial.print(deviceAddress[i], HEX);
  }
}

void setup() {
  Serial.begin(115200);

  // Initialize GPIO
  pinMode(PIN_TEC_PWM, OUTPUT);
  pinMode(PIN_FAN_COLD, OUTPUT);
  pinMode(PIN_FAN_HOT, OUTPUT);

  analogWrite(PIN_TEC_PWM, 0);
  digitalWrite(PIN_FAN_COLD, LOW);
  digitalWrite(PIN_FAN_HOT, LOW);

  // Initialize Sensors
  sensors.begin();
  numberOfDevices = sensors.getDeviceCount();
  
  Serial.print("Locating devices...");
  Serial.print("Found ");
  Serial.print(numberOfDevices, DEC);
  Serial.println(" devices.");
  
  for(int i=0;i<numberOfDevices; i++){
    if(sensors.getAddress(tempDeviceAddress, i)){
      Serial.print("Found device ");
      Serial.print(i, DEC);
      Serial.print(" with address: ");
      printAddress(tempDeviceAddress);
      Serial.println();
    } else {
      Serial.print("Found ghost device at ");
      Serial.print(i, DEC);
      Serial.print(" but could not detect address. Check power and cabling");
    }
  }
  
  // Connect to WiFi 
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected.");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Setup Webserver Routes
  server.on("/", HTTP_GET, handleRoot);
  server.on("/data", HTTP_GET, handleData);
  server.on("/api", HTTP_GET, handleAPI);
  server.begin();
  Serial.println("Web server started.");

  lqr.begin();
}

void loop() {
  server.handleClient(); // Handle incoming web requests

  static unsigned long lastUpdate = 0;
  unsigned long now = millis();

  // Control Loop updates every 1000ms
  if (now - lastUpdate >= 1000) {
    lastUpdate = now;
    
    // 1. Read Temperatures
    sensors.requestTemperatures();
    
    // Loop through each device, print out temperature data
    for(int i=0; i < numberOfDevices; i++){
      if(sensors.getAddress(tempDeviceAddress, i)){
        float tempC = sensors.getTempC(tempDeviceAddress);
        if(i == 0){
          tempCool = tempC;
        } else {
          tempHot  = tempC;
        }
      }
    }
    
    // 2. Read Current
    currentAmps = current(PIN_CURRENT);
    
    // Cleaned up Serial Output
    Serial.print("Cold: ");
    Serial.print(tempCool);
    Serial.print(" C | Hot: ");
    Serial.print(tempHot);
    Serial.print(" C | Current: ");
    Serial.print(currentAmps);
    Serial.println(" A");
    
    // 3. Safety Overheat Check
    if (tempHot > MAX_HOT_TEMP || tempHot == -127 || currentAmps > MAX_CURRENT) {
      tecMasterEnable = false; // Force TEC off
      pwmOutput = 0;
      stateFanC = false;
      stateFanH = true; // Force hot fan ON to dissipate heat
      
      analogWrite(PIN_TEC_PWM, pwmOutput);
      digitalWrite(PIN_FAN_COLD, LOW);
      digitalWrite(PIN_FAN_HOT, HIGH);
      
      Serial.println("SAFETY FAULT: Overheat, Temp Sensor Failure, or Overcurrent!");
      return; // Skip rest of control loop
    }

    // 4. Control Logic Execution
    if (isAutoMode) {
      // AUTO (LQR) MODE
      if (tecMasterEnable) {
        pwmOutput = lqr.compute(targetTemp, tempCool);
        stateFanH = (pwmOutput > 0) || (tempHot > 35.0); // Heat dissipation or cooldown
        stateFanC = (pwmOutput > 0);                     // Only distribute cool air when TEC is running
      } else {
        pwmOutput = 0;
        lqr.reset();
        stateFanH = (tempHot > 35.0); // Keep running hot fan until cooled
        stateFanC = false;
      }
    } else {
      // MANUAL MODE
      if (tecMasterEnable) {
        pwmOutput = manualPWM;
      } else {
        pwmOutput = 0;
      }
      stateFanC = manualFanC;
      stateFanH = manualFanH;
    }

    // 5. Actuate Hardware
    analogWrite(PIN_TEC_PWM, pwmOutput);
    digitalWrite(PIN_FAN_COLD, stateFanC ? HIGH : LOW);
    digitalWrite(PIN_FAN_HOT,  stateFanH ? HIGH : LOW);
  }
}