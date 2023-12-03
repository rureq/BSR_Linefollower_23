#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

#define SENS1_PIN A4
#define SENS2_PIN A5
#define SENS3_PIN A8
#define SENS4_PIN A9
#define SENS5_PIN A10

// Replace with your network credentials
const char* ssid = "ssid";
const char* password = "password";

String sliderValue = "0";
String sliderValue2 = "0";
String sliderValue3 = "0";

// setting PWM properties
const int freq = 5000;
const int resolution = 8;

int sensor[5];
int boolSensor[5];
int threshold = 2000;

const char* PARAM_INPUT = "value";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

bool drive = 0;
double error = 0;
double prev_error = 0;
double pK = 37.5;
double dK = 181.7;
double v = 121;
double v_change = 0;
double v_out_A = 0;
double v_out_B = 0;

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>LF control panel</title>
  <style>
    html {font-family: Arial; display: inline-block; text-align: center;}
    h2 {font-size: 2.3rem;}
    p {font-size: 1.9rem;}
    body {max-width: 400px; margin:0px auto; padding-bottom: 25px;}
    .slider { -webkit-appearance: none; margin: 14px; width: 360px; height: 25px; background: #FFD65C;
      outline: none; -webkit-transition: .2s; transition: opacity .2s;}
    .slider::-webkit-slider-thumb {-webkit-appearance: none; appearance: none; width: 35px; height: 35px; background: #003249; cursor: pointer;}
    .slider::-moz-range-thumb { width: 35px; height: 35px; background: #003249; cursor: pointer; } 
    .button {
  background-color: #4CAF50; /* Green */
  border: none;
  color: white;
  padding: 15px 32px;
  text-align: center;
  text-decoration: none;
  display: inline-block;
  font-size: 16px;
}
    
  </style>
</head>
<body>
  <h2>LF control panel</h2>
  <p><span id="textSliderValue">%SLIDERVALUE% </span></p>
  <p><input type="range" onchange="updateSliderPWM(this)" id="pwmSlider" min="0" max="50" value="%SLIDERVALUE%" step="0.5" class="slider"></p>
  <p><span id="textSliderValue2">%SLIDERVALUE2% </span></p>
  <p><input type="range" onchange="updateSliderPWM2(this)" id="pwmSlider2" min="0" max="200" value="%SLIDERVALUE%" step="0.1" class="slider"></p>
  <p><span id="textSliderValue3">%SLIDERVALUE3% </span></p>
  <p><input type="range" onchange="updateSliderPWM3(this)" id="pwmSlider3" min="0" max="255" value="%SLIDERVALUE%" step="1" class="slider"></p>
  <p><button type="button" onclick="startLF(this)" class = "button">Start</button></p>
  <p><button type="button" onclick="stopLF(this)" class = "button">Stop</button></p>


<script>

function updateSliderPWM(element) {
  var sliderValue = document.getElementById("pwmSlider").value;
  document.getElementById("textSliderValue").innerHTML = sliderValue;
  console.log(sliderValue);
  var xhr = new XMLHttpRequest();
  xhr.open("GET", "/slider?value="+sliderValue, true);
  xhr.send();
}

function updateSliderPWM2(element) {
  var sliderValue2 = document.getElementById("pwmSlider2").value;
  document.getElementById("textSliderValue2").innerHTML = sliderValue2;
  console.log(sliderValue2);
  var xhr = new XMLHttpRequest();
  xhr.open("GET", "/slider2?value="+sliderValue2, true);
  xhr.send();
}

function updateSliderPWM3(element) {
  var sliderValue3 = document.getElementById("pwmSlider3").value;
  document.getElementById("textSliderValue3").innerHTML = sliderValue3;
  console.log(sliderValue3);
  var xhr = new XMLHttpRequest();
  xhr.open("GET", "/slider3?value="+sliderValue3, true);
  xhr.send();
}

function startLF(element){
  var xhr = new XMLHttpRequest();
  xhr.open("GET", "/start", true);
  xhr.send();
}

function stopLF(element){
  var xhr = new XMLHttpRequest();
  xhr.open("GET", "/stop", true);
  xhr.send();
}

</script>
</body>
</html>
)rawliteral";

// check if sensor readings exceed threshold
void calculateThreshold(){
  for(int i = 0; i < 5; i++){
    if(sensor[i] > threshold){
      boolSensor[i] = 1;
    }
    else{
      boolSensor[i] = 0;
    }
  }
}

// sensors reading
void sense(){
  sensor[0] = analogRead(SENS1_PIN);
  sensor[1] = analogRead(SENS2_PIN);
  sensor[2] = analogRead(SENS3_PIN);
  sensor[3] = analogRead(SENS4_PIN);
  sensor[4] = analogRead(SENS5_PIN);
  calculateThreshold();
}

// calculate velocity change using pid (PD in this case) 
double pid(int error){
  v_change = pK * error + dK *(error-prev_error);
  prev_error = error;
  return v_change;
}

// calculate error based on sensor readings
int calcError(){
  if((boolSensor[0] || boolSensor[1]) && ~(boolSensor[3] || boolSensor[4]) ){
    error = -boolSensor[0]*3 - boolSensor[1]*2 - boolSensor[2];
  }
  else if(boolSensor[3] || boolSensor[4] && ~(boolSensor[0] || boolSensor[1])){
    error = boolSensor[4]*3 + boolSensor[3]*2 + boolSensor[2];
  }
  else{
    error = 0;
  }
  return error;
}

// stop the robot
void stop(){
  for (int i = 0; i<4 ; i++){
    ledcWrite(i, 0);
  }
}

// main control loop
void run(){
  while(drive){
    sense();

    // if any sensor reads a line calculate new error
    if (boolSensor[0] == 1 || boolSensor[1] == 1 || boolSensor[2] == 1 || boolSensor[3] == 1 || boolSensor[4] == 1){
      error = calcError();
    }

    // calculate change of velocity 
    v_change = pid(error);
    v_out_A = v - v_change;
    v_out_B = v + v_change;

    // high speed mode when in the middle of the line
    if (boolSensor[2] == 1){
      v_out_A = 2*v;
      v_out_B = 2*v;
    }

    // stop the robot when lifted from the ground
    if (sensor[0] == 4095 && sensor[1] == 4095 && sensor[2] == 4095 && sensor[3] == 4095 && sensor[4] == 4095){
      v_out_A = 0;
      v_out_B = 0;
    }

    // set values to pwm outputs controlling the motor controller
    ledcWrite(0, v_out_A);
    ledcWrite(1, 0);
    ledcWrite(2, 0);
    ledcWrite(3, v_out_B);
  }

  stop();
}


String processor(const String& var){
  if (var == "SLIDERVALUE"){
    return sliderValue;
  }

  else if (var == "SLIDERVALUE2"){
    return sliderValue2;
  }

  else if (var == "SLIDERVALUE3"){
    return sliderValue3;
  }

  return String();
}

void setup(){
  Serial.begin(115200);
  
  // PWM configuration
  ledcSetup(0, freq, resolution);
  ledcSetup(1, freq, resolution);
  ledcSetup(2, freq, resolution);
  ledcSetup(3, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(D0, 0);
  ledcAttachPin(D1, 1);
  ledcAttachPin(D2, 2);
  ledcAttachPin(D3, 3);
 
  // connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }

  // print IP address
  Serial.println(WiFi.localIP());

  // route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html, processor);
  });

  server.on("/slider", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    if (request->hasParam(PARAM_INPUT)) {
      inputMessage = request->getParam(PARAM_INPUT)->value();
      sliderValue = inputMessage;
      pK = sliderValue.toDouble();
    }
    else {
      inputMessage = "No message sent";
    }

    Serial.print("pK: ");
    Serial.println(pK);
    request->send(200, "text/plain", "OK");
  });
  
  server.on("/slider2", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    if (request->hasParam(PARAM_INPUT)) {
      inputMessage = request->getParam(PARAM_INPUT)->value();
      sliderValue2 = inputMessage;
      dK = sliderValue2.toDouble();
    }
    else {
      inputMessage = "No message sent";
    }

    Serial.print("dK: ");
    Serial.println(dK);
    request->send(200, "text/plain", "OK");
  });

  server.on("/slider3", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    if (request->hasParam(PARAM_INPUT)) {
      inputMessage = request->getParam(PARAM_INPUT)->value();
      sliderValue3 = inputMessage;
      v = sliderValue3.toInt();
    }
    else {
      inputMessage = "No message sent";
    }

    Serial.print("v: ");
    Serial.println(v);
    request->send(200, "text/plain", "OK");
  });
    
  server.on("/start", HTTP_GET, [] (AsyncWebServerRequest *request){
    Serial.println("Started!");
    drive = 1;
  });

  server.on("/stop", HTTP_GET, [] (AsyncWebServerRequest *request){
    Serial.println("Stopped!");
    drive = 0;
  });
  
  // Start server
  server.begin();
}
  
void loop() {
  run();
}