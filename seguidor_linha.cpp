#include <Arduino.h>
#include <QTRSensors.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <WebServer.h>

#define NUMSENSORS 8

//Gerais
bool debug = false;

// Wi-Fi
const char* ssid = "Cavalo_de_Troia.exe";  // Enter SSID here
const char* password = "cobalto123";  //Enter Password here
WebServer server(91);

//QTR-SENSOR
QTRSensorsAnalog qtr((unsigned char[]) {
                      4, 36, 39, 34, 35, 32, 33, 25},
                      NUMSENSORS);

unsigned int sensorValues[NUMSENSORS];

//Potenciômetros
int potSpeed = 19;

//MotorR
int rightMotor1 = 2,
    rightMotor2 = 12,
    rightMotorPWM = 15,
    rightMaxSpeed = 255,
    rightBaseSpeed = 0,
    rightStandardSpeed = 127.5;

//MotorL
int leftMotor1 = 26,
    leftMotor2 = 14,
    leftMotorPWM = 27,
    leftMaxSpeed = 255,
    leftBaseSpeed = 0,
    leftStandardSpeed = 127.5;

//PID
double kP = 0.026,
       kI = 0.0001,
       kD = 0;

//Valores atuais de velocidade
double rightValue, leftValue;

class PID {
  public:
    double
      error,
      sample,
      lastSample,
      kP, kI, kD,
      P, I, D,
      pid,
      setPoint;

    long lastProcess;
  
  PID (double _kP, double _kI, double _kD, double _setPoint) {
    kP = _kP;
    kI = _kI;
    kD = _kD;
    setPoint = _setPoint;
  }
  
  void addNewSample(double _sample) {
    sample = _sample;
  }
  
  double process() {
    //PID
    error = setPoint - sample;
    float deltaTime = (millis() - lastProcess) / 1000.0;
    lastProcess = millis();
    
    //P
    P = error * kP;
    
    //I
    I = I + (error * kI) * deltaTime;
    
    //D
    D = (lastSample - sample) * kD / deltaTime;
    lastSample = sample;
    
    pid = P + I + D;
    
    return pid;
  }
};

PID pid(kP, kI, kD, 3500.0);

/*void sendData(){
    String data;
    data = "<!DOCTYPE html> <html>\n";
    data +="<head><meta name=\"viewport\" http-equiv=\"refresh\" content=\"5\">\n";
    data +="<title>Frankstein</title>\n";
    data +="<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\n";
    data +="body{margin-top: 50px;} h1 {color: #444444;margin: 50px auto 30px;} h2 {color: #103884;margin-bottom: 50px;} h3 {color: #444444;}\n";
    data +="p {font-size: 18px;color: #888;margin-bottom: 5px;}\n";
    data +="</style>\n";
    data +="</head>\n";
    data +="<body>\n";
    data +="<h1>Valores recebidos:</h1>\n";
    data +="<p>KP: ";
    data +=kP;
    data +="</p>";
    data +="<p>KI: ";
    data +=kI;
    data +="</p>";
    data +="<p>KD: ";
    data +=kD;
    data +="</p>";
    data +="<p>PWM-Left: ";
    data +=leftValue;
    data +="</p>";
    data +="<p>PWM-Right: ";
    data +=rightValue;
    data +="</p>";
    data +="<p>QTR-Value: ";
    data +=qtr.readLine(sensorValues);
    data +="</p>";
    server.send(200, "text/html", data);
}

void handle_args(){
    String data;
    data = "<!DOCTYPE html> <html>\n";
    data +="<head><meta name=\"viewport\" http-equiv=\"refresh\" content=\"30, width=device-width, initial-scale=1.0, user-scalable=no\">\n";
    data +="<title>Frankstein</title>\n";
    data +="<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\n";
    data +="body{margin-top: 50px;} h1 {color: #444444;margin: 50px auto 30px;} h2 {color: #103884;margin-bottom: 25px;} h3 {color: #444444;}\n";
    data +="p {font-size: 16px;color: #888;margin-bottom: 5px;}\n";
    data +="</style>\n";
    data +="</head>\n";
    data +="<body>\n";
    data +="<h1>Dados enviados:</h1>\n";
    for(int i = 0; i<server.args(); i++) {
      data +="<h2>" + server.argName(i) + ": " + server.arg(i) + "</h2>\n";
      String arg = server.argName(i);
      if(arg == "ki") {
        kI = server.arg(i).toDouble();
      } else if(arg == "kp") {
        kP = server.arg(i).toDouble();
      } else if(arg == "kd") {
        kD = server.arg(i).toDouble();
      } else if(arg == "minSpeedL") {
        leftBaseSpeed = server.arg(i).toDouble();        
      } else if(arg == "minSpeedR") {
        rightBaseSpeed = server.arg(i).toDouble();        
      } else if(arg == "maxSpeedL") {
        leftMaxSpeed = server.arg(i).toDouble();    
      } else if(arg == "maxSpeedR") {
        rightMaxSpeed = server.arg(i).toDouble();
      }
    }
    data +="</body>\n";
    data +="</html>\n";
    server.send(200, "text/html", data);
    Serial.println("------------------");
    Serial.print("Ki: ");
    Serial.println(kI);
    Serial.print("Kp: ");
    Serial.println(kP);
    Serial.print("Kd: ");
    Serial.println(kD);
    Serial.print("minSpeed: ");
    Serial.println(rightBaseSpeed);
    Serial.println(leftBaseSpeed);
    Serial.print("maxSpeed: ");
    Serial.println(rightMaxSpeed);
    Serial.println(leftMaxSpeed);
    Serial.println("------------------");
}

void wifi_Connect() {  
    Serial.print("Conectando a rede WiFi...");
    WiFi.mode(WIFI_STA);  
    WiFi.begin(ssid, password);  
    while (WiFi.status() != WL_CONNECTED) {  
        delay(500);
        Serial.print(".");
    }  
    Serial.println("WiFi conectado!");
    Serial.println(WiFi.localIP());
    server.on("/", sendData);
    server.on("/send", handle_args);
    //server.onNotFound(handle_NotFound);
    server.begin();
    Serial.println("Servidor HTTP iniciado");
    Serial.println(WiFi.localIP());
}*/

void setup() {
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);

  //calibrar o sensor
  digitalWrite(LED_BUILTIN, HIGH);
  for (int i = 0; i < 1200; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(115200);
  //wi-fi
  /*if(comunic) {
    wifi_Connect();
  }*/
}


void loop() {
  if(debug) {
    digitalWrite(rightMotor1, HIGH);
    digitalWrite(rightMotor2, LOW);
    digitalWrite(leftMotor1, HIGH);
    digitalWrite(leftMotor2, LOW);
    for(int i = 100; i<255; i++) {
      analogWrite(rightMotorPWM, i);
      analogWrite(leftMotorPWM, i);
      delay(50);
    }
    return;
  }

  /*Controle de Velocidade pelo potenciômetro
  int speed = analogRead(potSpeed);
  speed = map(speed, 0, 1023, 0, 255);
  rightBaseSpeed = speed;
  rightStandardSpeed = (rightBaseSpeed + rightMaxSpeed)/2;
  rightMaxSpeed = 2*speed;

  leftBaseSpeed = speed;
  leftStandardSpeed = (leftBaseSpeed + leftMaxSpeed)/2;
  leftMaxSpeed = 2*speed;
  Serial.print(speed);
  Serial.print('\t');*/

  //wi-fi
  /*if(comunic) {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Conexao perdida, reconectando...");
        wifi_Connect();
        return;
    }
    server.handleClient();
  }*/

  //qtrsensor
  int sensor_pos = qtr.readLine(sensorValues);
  pid.addNewSample(sensor_pos);
  double erro = pid.process();
  
  rightValue = rightStandardSpeed + erro;
  leftValue = leftStandardSpeed - erro;
  if(rightValue > rightMaxSpeed) rightValue = rightMaxSpeed;
  if(leftValue > leftMaxSpeed) leftValue = leftMaxSpeed;
  if(rightValue < rightBaseSpeed) rightValue = rightBaseSpeed;
  if(leftValue < leftBaseSpeed) leftValue = leftBaseSpeed;

  analogWrite(rightMotorPWM, rightValue);
  analogWrite(leftMotorPWM, leftValue);
  digitalWrite(rightMotor1, HIGH);
  digitalWrite(rightMotor2, LOW);
  digitalWrite(leftMotor1, HIGH);
  digitalWrite(leftMotor2, LOW);
}
