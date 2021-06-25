#include <Streaming.h>
#include <Metro.h>
#include <Bounce.h>
#include <FiniteStateMachine.h>
#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

#define WLAN_SSID       "Looney"
#define WLAN_PASS       "TinyandTooney"

#define MQTT_SERVER      "broker.hivemq.com"
#define MQTT_SERVERPORT  1883

// D4 is builtin LED
#define PIN_LED D4
#define PIN_MOTOR D1
#define PIN_DOOR D6
#define PIN_PIR D2

Bounce door = Bounce( PIN_DOOR, 1 );
Bounce motion = Bounce( PIN_PIR, 1 );

String stateNames[] = {
  "Closed",
  "In Only",
  "Open",
};

void goToState(byte state);
void closeDoor();
void openDoor();
void openDoorOnMotion();

const uint32_t inOnlyOpenDuration = 60000; // ms; door remains open this long in InOnly/motion mode.

State closed = State(NULL, closeDoor, NULL);
State inOnly = State(NULL, openDoorOnMotion, NULL);
State open = State(NULL, openDoor, NULL);

FSM sM = FSM(closed); // initialize state machine

WiFiClient client;

Adafruit_MQTT_Client mqtt(&client, MQTT_SERVER, MQTT_SERVERPORT);

Adafruit_MQTT_Publish pubMotion = Adafruit_MQTT_Publish(&mqtt, "magister/catdoor/motion");
Adafruit_MQTT_Publish pubDoor = Adafruit_MQTT_Publish(&mqtt, "magister/catdoor/door");

Adafruit_MQTT_Subscribe subMode = Adafruit_MQTT_Subscribe(&mqtt, "magister/catdoor/mode");

void MQTT_connect();
void setup() {
  delay(1000);

  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_MOTOR, OUTPUT);
  pinMode(PIN_DOOR, INPUT_PULLUP);
  pinMode(PIN_PIR, INPUT);

  Serial.begin(115200);
  Serial << endl << endl;
  Serial << "Setup. begin." << endl;

  // close the door
  closeDoor();

  digitalWrite(PIN_LED, LOW); // boot indicator

  Serial << "Connecting to: " << WLAN_SSID << endl;
  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  WiFi.begin(WLAN_SSID, WLAN_PASS);

  // start with a manual sync.
  sensorUpdate(10);
  sM.transitionTo(closed);

  // Setup MQTT subscription for instructions.
  mqtt.subscribe(&subMode);

  Serial << "WiFi connecting " << endl;
  byte retries = 20;
  boolean ledState = false;
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);  // wait
    if (retries-- == 0) while (1);  // basically die and wait for WDT to reset me
    Serial.print(".");
    ledState = !ledState;
    digitalWrite(PIN_LED, ledState); // blink while waiting to connect
  }
  Serial.println();

  Serial.println("WiFi connected");
  Serial.println("IP address: "); Serial.println(WiFi.localIP());

  digitalWrite(PIN_LED, HIGH); // all clear
  Serial << "Setup. complete." << endl;
}

void loop() {
  MQTT_connect();
  yield();

  getStatus();
  yield();

  sensorUpdate(2);
  yield();

  sM.update();
  yield();

  static Metro pubIntervalDoor = 11000UL;
  if ( pubIntervalDoor.check() ) {
    pubIntervalDoor.reset();
    if ( door.read() == LOW) pubDoor.publish(1);
    else pubDoor.publish(0);
  }
  static Metro pubIntervalMotion = 3000UL;
  if ( pubIntervalMotion.check() ) {
    pubIntervalMotion.reset();
    if ( motion.read() == HIGH) pubMotion.publish(1);
    else pubMotion.publish(0);
  }

}

void getStatus() {
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(1000))) {

    if ( subscription == &subMode ) {
      String msg = (char *)subMode.lastread;
      if ( msg.length() > 1 ) {
        Serial << "getStatus.  error.  Got weird mode '" << msg << "'." << endl;
        return;
      }
      byte newMode = constrain(msg.toInt(), 0, sizeof(stateNames));

      goToState(newMode);
    }

  }
}

void goToState(byte mode) {
  mode = constrain(mode, 0, sizeof(stateNames));
  Serial << "goToState.  " << stateNames[mode] << endl;
  switch ( mode ) {
    case 0: sM.transitionTo(closed); break;
    case 1: sM.transitionTo(inOnly); break;
    case 2: sM.transitionTo(open); break;
  }
}

// door.read() == LOW -> door is open
// door.read() == HIGH -> door is closed
// motion.read() == LOW -> no motion
// motion.read() == HIGH -> motion
void sensorUpdate(uint32_t dur) {
  Metro waitfor(dur);
  waitfor.reset();

  while ( ! waitfor.check() ) {
    motion.update();
    door.update();
    yield();
  }

  digitalWrite(PIN_LED, door.read());
}

// Door
// HIGH==Closed
// LOW==Open
void pulseDoorMotor(uint32_t dur) {
  digitalWrite(PIN_MOTOR, HIGH);    // motor on
  sensorUpdate(dur);
  digitalWrite(PIN_MOTOR, LOW);    // motor off
  sensorUpdate(20);
}
void openDoor() {
  if ( door.read() == HIGH ) {
//    pulseDoorMotor(40);
    while ( door.read() == HIGH ) pulseDoorMotor(8);

    pubDoor.publish(true);
    Serial << "openDoor: door open." << endl;
  }
}
void closeDoor() {
  if ( door.read() == LOW ) {
//    pulseDoorMotor(90);
    while ( door.read() == LOW ) pulseDoorMotor(8);

    pubDoor.publish(false);
    Serial << "closeDoor: door closed." << endl;
  }
}

void openDoorOnMotion() {
  // track motion and time
  static uint32_t last = millis();
  uint32_t now = millis();
  static boolean doorState = false;
  
  // motion?
  if ( motion.read() == HIGH ) {
    // open the door
    openDoor();
    last = now;
    // note we have motion
    if( !doorState ) {
      pubMotion.publish(true);
      Serial << "openDoorOnMotion: motion=1... " << now << endl;
      doorState = true;
    }
  }

  // ready to close
  if ( (now - last) > inOnlyOpenDuration ) {
    // open the door
    closeDoor();
    // note we have no motion
    if( doorState ) {
      pubMotion.publish(false);
      Serial << "openDoorOnMotion: motion=0... " << now << endl;
      doorState = false;
    }
  }
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
    retries--;
    if (retries == 0) {
      // basically die and wait for WDT to reset me
      while (1);
    }
  }
  Serial.println("MQTT Connected!");
}
