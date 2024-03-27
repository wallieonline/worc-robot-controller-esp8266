//Needed for old arduino version
#define TCP_MSS redefined
#define LWIP_IPV6 redefined
#define LWIP_FEATURES redefined
#define LWIP_OPEN_SRC redefined

#include <ESP8266WiFi.h>
#include <espnow.h>
#include <EEPROM.h>

//Assign CPPM channel order
typedef struct rc_data_t {
  uint16_t AIL;
  uint16_t ELE;
  uint16_t THR;
  uint16_t RUD;
  uint16_t AUX1;
  uint16_t AUX2;
  uint16_t AUX3;
  uint16_t AUX4;
} rc_data_t;

//Create a struct
rc_data_t gCppm;

//Assign L293D pins
#define PWMA_OUT_PIN 5
#define PWMB_OUT_PIN 4
#define DIR1A_OUT_PIN 0
#define DIR1B_OUT_PIN 2
#define FLED_OUT_PIN 16

//Assign RC values
#define RC_NEUTRAL 1500
#define RC_MIN 960 //Signals can sometimes be lower than 1000
#define RC_MAX 2040 //Signals can sometime be higer than 2000
#define RC_DEADBAND 40
uint16_t gRcAil = 0;
uint16_t gRcEle = 0;
uint16_t gRcThr = 0;
uint16_t gRcRud = 0;
uint16_t gRcAux1 = 0;
uint16_t gRcAux2 = 0;
uint16_t gRcAux3 = 0;
uint16_t gRcAux4 = 0;
uint16_t unThrottleMin = RC_MIN;
uint16_t unThrottleMax = RC_MAX;
uint16_t unThrottleNeu = RC_NEUTRAL;
uint16_t unSteeringMin = RC_MIN;
uint16_t unSteeringMax = RC_MAX;
uint16_t unSteeringNeu = RC_NEUTRAL;

//Assign run or program mode
#define MODE_RUN_WALLIEONLINE 0
#define MODE_PROGRAM_WALLIEONLINE 1
uint8_t gMode = MODE_RUN_WALLIEONLINE;

//Assign mixer values
int gThrottle = 0;
int gSteering = 0;
int gMotorLeft = 0;
int gMotorRight = 0;

// callback function that will be executed when data is received
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  memcpy(&gCppm, incomingData, sizeof(gCppm));
  //Serial.print(gCppm.AIL); Serial.print("  ");
  //Serial.print(gCppm.ELE); Serial.print("  ");
  //Serial.print(gCppm.THR); Serial.print("  ");
  //Serial.print(gCppm.RUD); Serial.println();
}

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  pinMode(PWMA_OUT_PIN,OUTPUT);
  pinMode(PWMB_OUT_PIN,OUTPUT);
  pinMode(DIR1A_OUT_PIN,OUTPUT);
  pinMode(DIR1B_OUT_PIN,OUTPUT);
  pinMode(FLED_OUT_PIN,OUTPUT);
  delay(2000);
  WiFi.mode(WIFI_STA);
  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  //esp32 esp_now_register_recv_cb(OnDataRecv);
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop() {
  uint16_t unThrottleIn = gCppm.ELE;
  uint16_t unSteeringIn = gCppm.AIL;
  if (gCppm.AIL) gRcAil = gCppm.AIL;
  if (gCppm.ELE) gRcEle = gCppm.ELE;
  if (gCppm.THR) gRcThr = gCppm.THR;
  if (gCppm.RUD) gRcRud = gCppm.RUD;
  if (gCppm.AUX1) gRcAux1 = gCppm.AUX1;
  if (gCppm.AUX2) gRcAux2 = gCppm.AUX2;
  if (gCppm.AUX3) gRcAux3 = gCppm.AUX3;
  if (gCppm.AUX4) gRcAux4 = gCppm.AUX4;

  if (gMode == MODE_PROGRAM_WALLIEONLINE) {
    Serial.println("MODE_PROGRAM_WALLIEONLINE");
  }
  
  if (gMode == MODE_RUN_WALLIEONLINE) {
    if (unThrottleIn > unThrottleMin && unThrottleIn < unThrottleMax) {
      gThrottle = map(unThrottleIn,unThrottleMin,unThrottleMax,-255,255);
      gThrottle = constrain(gThrottle,-255,255);
      if (abs(gThrottle) <= RC_DEADBAND) {
        gThrottle = 0;
      }
    }
    if (unSteeringIn > unSteeringMin && unSteeringIn < unSteeringMax) {
      gSteering = map(unSteeringIn,unSteeringMin,unSteeringMax,-255,255);
      gSteering = constrain(gSteering,-255,255);
      if (abs(gSteering) <= RC_DEADBAND) {
        gSteering = 0;
      }
    }
    //Mix gThrottle and gSteering
    gMotorLeft = constrain((gThrottle + gSteering),-255,255);
    gMotorRight = constrain((gThrottle - gSteering),-255,255);
    if (gMotorLeft > 0) { //DIRECTION_FORWARD
      digitalWrite(DIR1A_OUT_PIN,LOW);
    } else if (gMotorLeft < 0) { //DIRECTION_REVERSE
      digitalWrite(DIR1A_OUT_PIN,HIGH);
    } else { //DIRECTION_STOP
      digitalWrite(DIR1A_OUT_PIN,LOW);
    }
    if (gMotorRight > 0) { //DIRECTION_FORWARD
      digitalWrite(DIR1B_OUT_PIN,HIGH);
    } else if (gMotorRight < 0) { //DIRECTION_REVERSE
      digitalWrite(DIR1B_OUT_PIN,LOW);
    } else { //DIRECTION_STOP
      digitalWrite(DIR1B_OUT_PIN,LOW);
    }
     analogWrite(PWMA_OUT_PIN,abs(gMotorLeft));
     analogWrite(PWMB_OUT_PIN,abs(gMotorRight));
  }
}
