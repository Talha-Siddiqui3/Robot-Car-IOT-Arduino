#include <WebSocketsServer.h>
//#include <ESP8266HTTPClient.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
//#include <vector>
#include <Servo.h>
//using namespace std;
#include <NewPingESP8266.h>
#define MAX_DISTANCE 200
#define TRIG_PIN 16
#define ECHO_PIN 5
NewPingESP8266 sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);

const char* ssid = "PUPU";
const char* password = "pakistan9";
Servo myservo;
int speed = 0;
int calibratedSpeedForLeftMotorForward = 0;
int calibratedSpeedForLeftMotorBackward = 0;
WebSocketsServer webSocket = WebSocketsServer(81);
ESP8266WebServer server(80);
int motorLeft = 14;
int motorRight = 12;
int rightMotorBackwards = 4;
int rightMotorForwards = 0;
int leftMotorBackwards = 13;
int leftMotorForwards = 15;
int echoPin = 5;
int trigPin = 16;
int fir = A0;
int lir = 1;
int rir = 3;
long distanceToObstacle = 0;
bool obstacleAvoidModeAuto = false;
bool alreadyForward = false;
String url = "http://192.168.0.109:8080/?distance=";
bool AnyRequest = false;
bool AlreadySetRxTx = false;
//String WebPage = "<!DOCTYPE html><html><style>input[type=\"text\"]{width: 90%; height: 3vh;}input[type=\"button\"]{width: 9%; height: 3.6vh;}.rxd{height: 90vh;}textarea{width: 99%; height: 100%; resize: none;}</style><script>var Socket;function start(){Socket=new WebSocket('ws://' + window.location.hostname + ':81/'); Socket.onmessage=function(evt){document.getElementById(\"rxConsole\").value =evt.data;}}function enterpressed(){Socket.send(document.getElementById(\"txbuff\").value); document.getElementById(\"txbuff\").value=\"\";}</script><body onload=\"javascript:start();\"> <div><input class=\"txd\" type=\"text\" id=\"txbuff\" onkeydown=\"if(event.keyCode==13) enterpressed();\"><input class=\"txd\" type=\"button\" onclick=\"enterpressed();\" value=\"Send\" > </div><br><div class=\"rxd\"> <textarea id=\"rxConsole\" readonly></textarea> </div></body></html>";
//uint8_t numRecieved;


/*template<typename T>
  vector <T>
  split(const T &str, const T &delimiters) {
  vector <T> v;
  typename T::size_type start = 0;
  auto pos = str.find_first_of(delimiters, start);
  while (pos != T::npos) {
    if (pos != start) // ignore empty tokens
    v.emplace_back(str, start, pos - start);
    start = pos + 1;
    pos = str.find_first_of(delimiters, start);
  }
  if (start < str.length()) // ignore trailing delimiter
  v.emplace_back(str, start, str.length() - start); // add what's left of the string
  return v;
  }
*/
/*
  String getValue(String data, char separator, int index)
  {
    int found = 0;
    int strIndex[] = { 0, -1 };
    int maxIndex = data.length() - 1;

    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
  }
*/


void setup() {
 // Serial.begin(9600);
  delay(10);
  myservo.attach(2);
  myservo.write(92);
  pinMode(motorLeft, OUTPUT);
  pinMode(motorRight, OUTPUT);
  digitalWrite(motorLeft, HIGH);
  digitalWrite(motorRight, HIGH);
  pinMode(rightMotorForwards, OUTPUT);
  pinMode(rightMotorBackwards, OUTPUT);
  pinMode(leftMotorForwards, OUTPUT);
  pinMode(leftMotorBackwards, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(lir, FUNCTION_0);
  pinMode(rir, FUNCTION_0);

  distanceToObstacle = readPing();


  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    //Serial.print(".");
  }
  //Serial.println("WIFI connected");
  /*server.on("/", []() {
    Serial.println("Request received from http");
    server.send(200, "text/html", WebPage);
  });

  server.begin();
*/
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
}


void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  //Serial.println("Request received from webseocket");
  if (type == WStype_TEXT) {
    //numRecieved = num;
    String requestFull = "";
    for (int i = 0; i < length; i++) {
      requestFull += (char)payload[i];
    }
    // webSocket.sendTXT(num, requestFull + String(num));
    AnyRequest = true;
    checkForRxTx();
    chooseAppropriateOperation(requestFull);
  }

}
void chooseAppropriateOperation(String requestFull) {
  int firstSemiColonPos = requestFull.indexOf(';');
  int secondSemiColonPos = requestFull.indexOf(';', firstSemiColonPos + 1);
  int thirdSemiColonPos = requestFull.indexOf(';', secondSemiColonPos + 1);
  int fourthSemiColonPos = requestFull.indexOf(';', thirdSemiColonPos + 1);
  String request = requestFull.substring(firstSemiColonPos + 1, secondSemiColonPos);
  String speedStr = requestFull.substring(secondSemiColonPos + 1, thirdSemiColonPos);
  speed = speedStr.toInt();
  if (speed > 50) {
    calibratedSpeedForLeftMotorForward = speed - 20;
    calibratedSpeedForLeftMotorBackward = speed - 35;
  }
  if (request == "ObstacleAvoidAuto" ) {
    handleAutoObstacleAvoid();
    obstacleAvoidModeAuto = true;
  }
  else {
    obstacleAvoidModeAuto = false;
    alreadyForward = false;
    handleNormalRemoteControl(request, requestFull, thirdSemiColonPos, fourthSemiColonPos);
  }
}



void loop() {
  distanceToObstacle = readPing();
  webSocket.loop();
  //server.handleClient();
  if (AnyRequest) {
    String stringifiedDistanceToObstacle=String(distanceToObstacle);
    webSocket.sendTXT(0, stringifiedDistanceToObstacle);
  }
  //HTTPClient httpclient;
  //httpclient.begin(url + String(distanceToObstacle));
  //httpclient.GET();
  //httpclient.end();
  if (obstacleAvoidModeAuto) {
    handleAutoObstacleAvoid();
  }
}


void handleAutoObstacleAvoid() {
  long distanceR = 0;
  long distanceL = 0;
  //Serial.println(distanceToObstacle);
  int firDistance = analogRead(A0);
  //int lirDistance = digitalRead(lir);
  //int rirDistance =  digitalRead(rir);
  //Serial.println(firDistance);
  //String allIrsDistances=String(firDistance)+" "+String(lirDistance)+" "+String(rirDistance);
  if (distanceToObstacle <= 19 || firDistance < 900 || digitalRead(lir) == LOW || digitalRead(rir) == LOW )
  {
    // //////("Now disablling");
    disableAll(false);
    delay(10);
    moveBackwards();
    delay(450);
    disableAll(false);
    delay(200);
    distanceR = lookRight();
    delay(200);
    distanceL = lookLeft();
    delay(200);

    if (distanceR >= distanceL)
    {
      moveRight();
      delay(350);
      disableAll(false);
    } else
    {
      moveLeft();
      delay(350);
      disableAll(false);
    }
  } else
  {
    moveForwards();
  }


  ////////Serial.println(distanceToObstacle);

}

/*
  long getDistance(){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(4);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  long distanceToObstacle = (duration/2) * 0.0343;
  ////////Serial.println(distanceToObstacle);
  if(distanceToObstacle==0){
  distanceToObstacle=250;
  }
  return distanceToObstacle;
  }
*/


long readPing() {
  int cm = sonar.ping_cm();
  if (cm == 0)
  {
    cm = 250;
  }
  return cm;
}


long lookRight()
{
  myservo.write(0);
  delay(650);
  long distanceToObstacle = readPing();
  delay(100);
  myservo.write(92);
  return distanceToObstacle;
}

long lookLeft()
{
  myservo.write(180);
  delay(650);
  long distanceToObstacle = readPing();
  delay(100);
  myservo.write(92);
  return distanceToObstacle;
}



void handleNormalRemoteControl(String request, String requestFull, int thirdSemiColonPos, int fourthSemiColonPos) {
  if (request == "Stop")  {
    disableAll(true);
  }

  else if (request == "Right")  {
    moveRight();
  }
  else if (request == "Left")  {
    moveLeft();
  }
  else if (request == "Forwards")  {
    moveForwards();
  }

  else if (request == "Backwards")  {
    moveBackwards();
  }
  else if (request == "ForwardRight")  {
    //String rightTyreSpeedStr=v[3].c_str();
    String rightTyreSpeedStr = requestFull.substring(thirdSemiColonPos + 1, fourthSemiColonPos);
    moveForwardRight(rightTyreSpeedStr.toInt());
  }
  else if (request == "ForwardLeft")  {
    //String leftTyreSpeedStr=v[3].c_str();
    String leftTyreSpeedStr = requestFull.substring(thirdSemiColonPos + 1, fourthSemiColonPos);
    moveForwardLeft(leftTyreSpeedStr.toInt());
  }
  else if (request == "BackwardRight")  {
    //String leftTyreSpeedStr=v[3].c_str();
    String leftTyreSpeedStr = requestFull.substring(thirdSemiColonPos + 1, fourthSemiColonPos);
    moveBackwardRight(leftTyreSpeedStr.toInt());
  }
  else if (request == "BackwardLeft")  {
    // String rightTyreSpeedStr=v[3].c_str();
    String rightTyreSpeedStr = requestFull.substring(thirdSemiColonPos + 1, fourthSemiColonPos);
    moveBackwardLeft(rightTyreSpeedStr.toInt());
  }
}

void checkForRxTx() {
  if (AlreadySetRxTx == false && AnyRequest == true ) {
    AlreadySetRxTx = true;
    pinMode(lir, FUNCTION_3);
    pinMode(rir, FUNCTION_3);
    pinMode(lir, INPUT);
    pinMode(rir, INPUT);
  }
}

void disableAll(boolean stop) {
  /*if(stop){
    digitalWrite(motorRight,LOW);
    digitalWrite(motorLeft,LOW);
    }*/
  digitalWrite(rightMotorForwards, LOW);
  digitalWrite(leftMotorForwards, LOW);
  digitalWrite(rightMotorBackwards, LOW);
  digitalWrite(leftMotorBackwards, LOW);
}

void moveRight() {
  disableAll(false);
  analogWrite(rightMotorBackwards, speed);
  analogWrite(leftMotorForwards, speed);
}

void moveLeft() {
  disableAll(false);
  analogWrite(rightMotorForwards, speed);
  analogWrite(leftMotorBackwards, speed);
}
void moveForwards() {
  if (alreadyForward == false) {
    alreadyForward = true;
    disableAll(false);
    analogWrite(leftMotorForwards, calibratedSpeedForLeftMotorForward);
    analogWrite(rightMotorForwards, speed);
  }


}
void moveBackwards() {
  alreadyForward = false;
  disableAll(false);
  analogWrite(leftMotorBackwards, calibratedSpeedForLeftMotorBackward);
  analogWrite(rightMotorBackwards, speed);

}
void moveForwardRight(int rightTyreSpeed) {
  disableAll(false);
  analogWrite(rightMotorForwards, rightTyreSpeed);
  analogWrite(leftMotorForwards, speed);
}
void moveForwardLeft(int leftTyreSpeed) {
  disableAll(false);
  analogWrite(rightMotorForwards, speed);
  analogWrite(leftMotorForwards, leftTyreSpeed);

}
void moveBackwardRight(int leftTyreSpeed) {
  disableAll(false);
  analogWrite(rightMotorBackwards, speed);
  analogWrite(leftMotorBackwards, leftTyreSpeed);

}
void moveBackwardLeft(int rightTyreSpeed) {
  disableAll(false);
  analogWrite(rightMotorBackwards, rightTyreSpeed);
  analogWrite(leftMotorBackwards, speed);

}
