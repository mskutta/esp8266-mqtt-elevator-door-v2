#ifdef ESP8266 || ESP32
  #define ISR_PREFIX ICACHE_RAM_ATTR
#else
  #define ISR_PREFIX
#endif

#include <Arduino.h>

/* WiFi */
#include <ESP8266WiFi.h> // WIFI support
#include <ArduinoOTA.h> // Updates over the air
char hostname[32] = {0};

/* mDNS */
#include <ESP8266mDNS.h>

/* WiFi Manager */
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h> 

/* I2C */
#include <Wire.h>

/* Display (SSD1306) */
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"
SSD1306AsciiWire display;
uint8_t rowHeight; // pixels per row.

/* Stepper Motor Control */
#include <Tic.h>
TicI2C tic;

// Port Expander
#include <Adafruit_MCP23008.h>
Adafruit_MCP23008 mcp;

/* MQTT */
#include <PubSubClient.h>
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);
char topic[40] = {0};
char topicDoorOpen[40] = {0};

/* Constants */
#ifdef FRONT_DOOR
const char* ESP_NAME = "elev-door-front";
#endif
#ifdef REAR_DOOR
const char* ESP_NAME = "elev-door-rear";
#endif

const int CLOSED_POSITION = 0;
const int OPEN_POSITION = 18600; // 18650

const unsigned long DOOR_DWELL_INFINITE = 600000; // 10 min
const unsigned long DOOR_DWELL_STANDARD = 5000;
const unsigned long DOOR_DWELL_SHORT = 2000;
unsigned long doorDwell = 0;

const unsigned long RUN_INTERVAL = 100;

/* Variables */
unsigned long nextRun = 0;
unsigned long currentMillis = 0;

enum class CallState {None, Up, Down};
CallState callState = CallState::None;

enum class DoorState {Unknown, Homing, Close, Closed, Closing, Open, Opening, Reopen, Reopening, Waiting};
static const char *DoorStateString[] = {"unknown", "homing", "close", "closed", "closing", "open", "opening", "reopen", "reopening", "waiting"};
DoorState doorState = DoorState::Unknown;
DoorState lastDoorState = DoorState::Unknown;

unsigned long homingTimeout = 0;
unsigned long closeTimeout = 0;
unsigned long openTimeout = 0;
unsigned long waitTimeout = 0;
bool isHomed = false;
bool lastLimitActive = false;
bool lastTripped = false;

volatile int encoderCount = 0; //This variable will increase or decrease depending on the rotation of encoder
int lastEncoderPosition = 0;

bool doorOpenReceived = false;

void setEncoderPosition(int position) {
  encoderCount = map(position, 0, 1600, 0, 600); // Encoder 600p/r, Stepper 200steps/rev * 8
}

int getEncoderPosition() {
  return map(encoderCount, 0, 600, 0, 1600); // Encoder 600p/r, Stepper 200steps/rev * 8
}

void waitDoor(unsigned long timeout) {
  display.print(F("wait: "));
  display.println(timeout);
  tic.deenergize();
  doorState = DoorState::Waiting;
  waitTimeout = millis() + timeout;
}

void home() {
  tic.energize();
  tic.setStepMode(TicStepMode::Microstep8);
  tic.setCurrentLimit(1500);
  tic.setMaxAccel(100000); // 10000 steps/sec
  tic.setMaxDecel(100000); // 10000 steps/sec
  tic.haltAndSetPosition(0); // Needed to set velocity
  tic.exitSafeStart();

  tic.setTargetVelocity(-10000000);
  
  doorState = DoorState::Homing;
  homingTimeout = millis() + 5000; // Wait 5s
}

void homingTimedOut() {
  display.println(F("Homimg Timeout"));
  display.println(F("!!! CLOSE DOOR !!!"));

  if (tic.getEnergized()) {
    tic.deenergize();
  }
  isHomed = false;
}

ISR_PREFIX void ai0() {
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if (digitalRead(D6)==LOW) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

void preCloseDoor() {
  // Allow the door to settle
  if (tic.getEnergized()) {
    tic.deenergize();
    closeTimeout = millis() + 250; // Wait 250ms
  }
  
  doorState = DoorState::Close;
}

void closeDoor() {
  tic.energize();
  tic.setStepMode(TicStepMode::Microstep8);
  tic.setCurrentLimit(900); // 750 is the minimum required to mostly open the door
  tic.setMaxSpeed(30000000); // ~11.25 revolutions (18000 steps) to open door. 2 second open time = 9000 steps/sec
  tic.setMaxAccel(100000); // 10000 steps/sec
  tic.setMaxDecel(100000); // 10000 steps/sec
  tic.haltAndSetPosition(getEncoderPosition());
  tic.setTargetPosition(CLOSED_POSITION);
  tic.exitSafeStart();
  
  doorState = DoorState::Closing;
}

void preOpenDoor(bool reopen) {
  if (tic.getEnergized()) {
    tic.deenergize();
    openTimeout = millis() + 250; // Wait 250ms
  }

  doorState = (reopen == true) ? DoorState::Reopen : DoorState::Open;
}

void openDoor(bool reopen) {
  tic.energize();
  tic.setStepMode(TicStepMode::Microstep8);
  tic.setCurrentLimit(1500);
#ifdef FRONT_DOOR
  tic.setMaxSpeed(90000000); // ~11.25 revolutions (18000 steps) to open door. 2 second open time = 9000 steps/sec
#endif
#ifdef REAR_DOOR
  if (reopen == true) {
    tic.setMaxSpeed(90000000);
  } else {
    tic.setMaxSpeed(20000000);
  }
#endif
  tic.setMaxAccel(400000); // 10000 steps/sec
  tic.setMaxDecel(700000); // 10000 steps/sec
  tic.haltAndSetPosition(getEncoderPosition());
  tic.setTargetPosition(OPEN_POSITION);
  tic.exitSafeStart();

  doorState = (reopen == true) ? DoorState::Reopening : DoorState::Opening;
}

void callUp(){
  callState = CallState::Up;
  mcp.digitalWrite(7, HIGH); // UP
  mcp.digitalWrite(5, LOW); // Down
}

void callDown(){
  callState = CallState::Down;
  mcp.digitalWrite(7, LOW); // UP
  mcp.digitalWrite(5, HIGH); // Down
}

void callNone(){
  callState = CallState::None;
  mcp.digitalWrite(7, LOW); // UP
  mcp.digitalWrite(5, LOW); // Down
}

void configModeCallback (WiFiManager *myWiFiManager) {
  display.println(F("Config Mode"));
  display.println(WiFi.softAPIP());
  display.println(myWiFiManager->getConfigPortalSSID());
}

void reconnect() {
  while (!mqtt.connected()) {
    display.println(F("MQTT Connecting..."));
    if (mqtt.connect(hostname)) {
      display.println(F("MQTT connected"));
      mqtt.subscribe(topicDoorOpen); // "elev-door-?/door/open"
    } else {
      delay(1000);
      display.print(F("."));
      ArduinoOTA.handle();
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  display.println(F("received:"));
  display.print(F("  "));
  display.println(topic);
  if(strcmp(topic, topicDoorOpen) == 0) 
  {
    doorOpenReceived = true;
  }
}

void setup() {
  /* Serial and I2C */
  Serial.begin(9600);
  Wire.begin(D2, D1); // join i2c bus with SDA=D1 and SCL=D2 of NodeMCU

  delay(1000);

  /* Display */
  display.begin(&Adafruit128x64, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x64)
  display.setFont(System5x7);
  display.setScrollMode(SCROLL_MODE_AUTO);
  display.clear();
  rowHeight = display.fontRows();

  display.println(ESP_NAME);

  /* WiFi */
  sprintf(hostname, "%s-%06X", ESP_NAME, ESP.getChipId());
  WiFiManager wifiManager;
  wifiManager.setAPCallback(configModeCallback);
  if(!wifiManager.autoConnect(hostname)) {
    display.println(F("WiFi Connect Failed"));
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(1000);
  }

  display.println(hostname);
  display.print(F("  "));
  display.println(WiFi.localIP());
  display.print(F("  "));
  display.println(WiFi.macAddress());

  /* OTA */
  ArduinoOTA.setHostname(hostname);
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    display.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    display.println(F("End"));
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    display.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    display.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      display.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      display.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      display.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      display.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      display.println("End Failed");
    }
  });
  ArduinoOTA.begin();

  // Wait to view display
  delay(2000);

  /* MQTT */

  // Discover MQTT broker via mDNS
  display.print(F("Finding MQTT Server"));
  while (MDNS.queryService("mqtt", "tcp") == 0) {
    delay(1000);
    display.print(F("."));
    ArduinoOTA.handle();
  }
  display.println();

  display.println(F("MQTT: "));
  display.print(F("  "));
  display.println(MDNS.hostname(0));
  display.print(F("  "));
  display.print(MDNS.IP(0));
  display.print(F(":"));
  display.println(MDNS.port(0));

  sprintf(topicDoorOpen, "%s/door/open", ESP_NAME);
  mqtt.setServer(MDNS.IP(0), MDNS.port(0));
  mqtt.setCallback(callback);

  // Wait to view display
  delay(2000);

  /* TIC */
  // Set the TIC product
  tic.setProduct(TicProduct::T500);
  tic.deenergize();

  /* Pins */
  
  // Note: Pins D0, D3, D4, D8 are reserved.

  // encoder
  pinMode(D5, INPUT_PULLUP);
  pinMode(D6, INPUT_PULLUP);

  // Set up interrupts
  attachInterrupt(digitalPinToInterrupt(D5), ai0, RISING);

  /* Port Expander (MCP23008) */
  mcp.begin(0); // 0x20
  
  mcp.pinMode(0, INPUT); // Limit Switch
  mcp.pullUp(0, HIGH);  // Limit Switch

  mcp.pinMode(1, INPUT); // Beam Break
  mcp.pullUp(1, HIGH);  // Beam Break

#ifdef FRONT_DOOR
  mcp.pinMode(4, INPUT); // Down Button
  mcp.pullUp(4, HIGH);  // turn on a 100K pullup internally
  
  mcp.pinMode(5, OUTPUT);  // Down Acceptance Light
  mcp.digitalWrite(5, LOW);
  
  mcp.pinMode(6, INPUT); // Up Button
  mcp.pullUp(6, HIGH);  // turn on a 100K pullup internally
  
  mcp.pinMode(7, OUTPUT);  // Up Acceptance Light
  mcp.digitalWrite(7, LOW);
#endif
#ifdef REAR_DOOR
  mcp.pinMode(6, OUTPUT);  // Door Frame
  mcp.digitalWrite(6, HIGH);
  
  mcp.pinMode(7, OUTPUT);  // Door Frame
  mcp.digitalWrite(7, HIGH);
#endif

  // Wait to view display and connect
  delay(2000);

  display.println("!!! CLOSE DOOR !!!");
}

void loop() {
  /* OTA */
  ArduinoOTA.handle();

  /* MQTT */
  if (!mqtt.connected())
  {
    reconnect();
  }
  mqtt.loop();

  /* TIC */
  tic.resetCommandTimeout();

  /* mDNS */
  MDNS.update();

  // limit switch
  bool limitActive = (mcp.digitalRead(0)==LOW);
  if (limitActive == true && limitActive != lastLimitActive) {
    if (tic.getEnergized()) {
      tic.deenergize();
    }
    setEncoderPosition(0);
    doorState = DoorState::Closed;
    isHomed = true;
    display.println(F("Limit Switch"));
  }
  lastLimitActive = limitActive;

  // Tripped State
  bool tripped = (mcp.digitalRead(1)==LOW);
  if (tripped == true && tripped != lastTripped) {
    display.println(F("Tripped"));
  }
  lastTripped = tripped;

  if (!isHomed) {
    display.invertDisplay(true);
    return;
  } else {
    display.invertDisplay(false);
  }

  // NOTE: for some reason the following (tic) needs to run at full speed for tic.getCurrentPosition to return reliable results.
  // i.e. do not add a delay before calling the below.

  // Get position info
  int currentPosition = tic.getCurrentPosition();
  int targetPosition = tic.getTargetPosition();
  
  // Run the remainder of the loop once every 100ms
  currentMillis = millis();
  if (currentMillis < nextRun) {
    return;
  }
  nextRun = currentMillis + RUN_INTERVAL;

  // Get position
  int encoderPosition = getEncoderPosition();

  // Indicate if tripped
  if (tripped) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }

  // Deenergize if stopped
  bool stopped = currentPosition == targetPosition;
  if (stopped && tic.getEnergized()) {
    tic.deenergize();
  }
  
  // Position Correction
  int positionCorrection = currentPosition - encoderPosition;

  // Messages
  bool openDoorRequested = doorOpenReceived;
  doorOpenReceived = false;

  // Handle Door States
  if (doorState == DoorState::Waiting) {
    if (millis() > waitTimeout) {
      preCloseDoor();
    }
    else if (tripped) {
      waitDoor(DOOR_DWELL_SHORT);
    }
  }
  else if (doorState == DoorState::Homing) {
    if (millis() > homingTimeout) {
      homingTimedOut();
    }
  }
  else if (doorState == DoorState::Close) {
    if (millis() > closeTimeout) {
      closeDoor();
    }
  }
  else if (doorState == DoorState::Closing) {
    if (stopped) {
      // door needs to be homed. Door should not stop before hitting limit switch
      home();
    }
    else if (tripped || // beam break - reopen
              positionCorrection < -64 || // door is being pushed - reopen
              openDoorRequested) { // open door requested
      if (positionCorrection < -64) {
        display.print(F("drift: "));
        display.println(positionCorrection);
      }
      preOpenDoor(true);
    }
    else if (positionCorrection > 64) { // door is being pulled - wait
      sprintf(topic, "%s/door/forced", ESP_NAME);
      mqtt.publish(topic, "");
      waitDoor(DOOR_DWELL_SHORT);
    }
  }
  else if (doorState == DoorState::Open || doorState == DoorState::Reopen) {
    if (millis() > openTimeout) {
#ifdef FRONT_DOOR
      doorDwell = DOOR_DWELL_INFINITE;
#endif
#ifdef REAR_DOOR
      doorDwell = DOOR_DWELL_STANDARD;
#endif
      if (doorState == DoorState::Open) {
        openDoor(false);
      } 
      else if (doorState == DoorState::Reopen) {
        openDoor(true);
      }
    }
  }
  else if (doorState == DoorState::Opening || doorState == DoorState::Reopening) {
    if (stopped || 
      encoderPosition > OPEN_POSITION) { // Door passed jam - wait

#ifdef REAR_DOOR
      mcp.digitalWrite(6, HIGH); // Turn off EL wire
      mcp.digitalWrite(7, HIGH); // Turn off EL wire
#endif
      
      if(doorState == DoorState::Reopening) {
        waitDoor(DOOR_DWELL_SHORT);
      } else {
        waitDoor(doorDwell);
      }
    }
    else if (tripped) {
      doorDwell = DOOR_DWELL_SHORT;
    }
    else if (abs(positionCorrection) > 128) { // Door is being pushed or pulled - wait
      sprintf(topic, "%s/door/forced", ESP_NAME);
      mqtt.publish(topic, "");
      waitDoor(DOOR_DWELL_SHORT);
    }
  }
  else if (doorState == DoorState::Closed) {

#ifdef FRONT_DOOR
    if (doorState != lastDoorState) { // Up Button
      callUp();
    }
#endif

    if (openDoorRequested) {
#ifdef FRONT_DOOR
      callNone();
#endif
#ifdef REAR_DOOR
      mcp.digitalWrite(6, LOW); // Turn on EL wire
      mcp.digitalWrite(7, LOW); // Turn on EL wire
#endif
      preOpenDoor(false);
    }
  }

  // Send door state
  if (doorState != lastDoorState) {
    display.print(F("door: "));
    display.println(DoorStateString[(int)doorState]);

    sprintf(topic, "%s/door/%s", ESP_NAME, DoorStateString[(int)doorState]);
    mqtt.publish(topic, "");

    lastDoorState = doorState;
  }

  lastEncoderPosition = encoderPosition;
}

