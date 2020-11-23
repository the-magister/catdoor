#include <Streaming.h>
#include <Metro.h>
#include <Bounce.h>
#include <FiniteStateMachine.h>
#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <ESP_EEPROM.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

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

struct State_t {
  // don't bother to commit these to EEPROM when changed
  boolean motion;
  boolean door;
  // commit these to EEPROM when changed
  byte mode;
  uint32_t forDur;
  uint32_t motionDur;
};
State_t state;

String stateNames[] = {
  "Closed",
  "In Only",
  "Open",
  "Open then Closed",
  "Open then In Only"
};

void goToState(byte state);
void closeDoor();
void inOnlyUpdate();
void openDoor();
void openThenClosedUpdate() ;
void openThenInOnlyUpdate();

State closed = State(closeDoor, NULL, NULL);
State inOnly = State(closeDoor, inOnlyUpdate, NULL);
State open = State(openDoor, NULL, NULL);
State openThenClosed = State(openDoor, openThenClosedUpdate, NULL);
State openThenInOnly = State(openDoor, openThenInOnlyUpdate, NULL);

FSM sM = FSM(inOnly); // initialize state machine

WiFiClient client;

Adafruit_MQTT_Client mqtt(&client, MQTT_SERVER, MQTT_SERVERPORT);

Adafruit_MQTT_Publish pubMotion = Adafruit_MQTT_Publish(&mqtt, "magister/catdoor/motion");
Adafruit_MQTT_Publish pubDoor = Adafruit_MQTT_Publish(&mqtt, "magister/catdoor/door");
Adafruit_MQTT_Publish pubMode = Adafruit_MQTT_Publish(&mqtt, "magister/catdoor/mode");
Adafruit_MQTT_Publish pubForDur = Adafruit_MQTT_Publish(&mqtt, "magister/catdoor/doorDur");
Adafruit_MQTT_Publish pubMotionDur = Adafruit_MQTT_Publish(&mqtt, "magister/catdoor/motionDur");

Adafruit_MQTT_Subscribe subMotion = Adafruit_MQTT_Subscribe(&mqtt, "magister/catdoor/motion");
Adafruit_MQTT_Subscribe subDoor = Adafruit_MQTT_Subscribe(&mqtt, "magister/catdoor/door");
Adafruit_MQTT_Subscribe subMode = Adafruit_MQTT_Subscribe(&mqtt, "magister/catdoor/mode");
Adafruit_MQTT_Subscribe subForDur = Adafruit_MQTT_Subscribe(&mqtt, "magister/catdoor/doorDur");
Adafruit_MQTT_Subscribe subMotionDur = Adafruit_MQTT_Subscribe(&mqtt, "magister/catdoor/motionDur");

void MQTT_connect();

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

void setup() {
  delay(1000);

  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_MOTOR, OUTPUT);
  pinMode(PIN_DOOR, INPUT_PULLUP);
  pinMode(PIN_PIR, INPUT);

  Serial.begin(115200);
  Serial << endl << endl;
  Serial << "Setup. begin." << endl;

  digitalWrite(PIN_LED, LOW); // boot indicator

  Serial << "Connecting to: " << WLAN_SSID << endl;
  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  WiFi.begin(WLAN_SSID, WLAN_PASS);

  // start with a manual sync.
  doorDelay(10);

  Serial.println("Initializing EEPROM...");
  EEPROM.begin(512);

  EEPROM.get(0, state);
  if ( state.mode >= sizeof(stateNames) || false ) {
    Serial << "Bad EEPROM! Bootstrapping" << endl;

    state.motion = false;
    state.door = false;
    state.mode = 1; // in only
    state.forDur = 3600; // sec
    state.motionDur = 30; // sec

    EEPROM.put(0, state);
    EEPROM.commit();
  }
  goToState(state.mode);

  // Setup MQTT subscription for instructions.
  mqtt.subscribe(&subDoor);
  mqtt.subscribe(&subMotion);
  mqtt.subscribe(&subMode);
  mqtt.subscribe(&subForDur);
  mqtt.subscribe(&subMotionDur);

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

  Serial << "starting time client." << endl;
  timeClient.begin();
  long offset = -7L * 60L * 60L;
  Serial << "offset. " << offset << endl;
  timeClient.setTimeOffset(offset);

  digitalWrite(PIN_LED, HIGH); // all clear
  Serial << "Setup. complete." << endl;
}

void loop() {
  MQTT_connect();
  yield();

  timeClient.update();
  yield();

  getStatus();
  yield();

  updateMotion();
  yield();

  sM.update();
  yield();

  sendStatus();
  yield();

  doorDelay(2);
  yield();
}

void getStatus() {

  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(1000))) {
    if ( subscription == &subDoor ) {
      String msg = (char *)subDoor.lastread;
      boolean newDoor = msg.toInt();

      if ( newDoor != state.door ) {
        Serial << "getStatus.  door=" << newDoor << endl;
        if ( newDoor == false ) closeDoor();
        else openDoor();
      }
    }

    if ( subscription == &subMotion ) {
      String msg = (char *)subMotion.lastread;
      boolean newMotion = msg.toInt();

      if ( newMotion != state.motion ) {
        Serial << "getStatus.  motion=" << newMotion << endl;
        state.motion = newMotion;
      }
    }

    if ( subscription == &subMode ) {
      String msg = (char *)subMode.lastread;
      if( msg.length() > 1 ) {
        Serial << "getStatus.  error.  Got weird mode '" << msg << "'." << endl;
        return;
      }
      byte newMode = constrain(msg.toInt(), 0, sizeof(stateNames));

      if ( newMode != state.mode ) {
        Serial << "getStatus.  mode=" << msg << " " << newMode << endl;
        state.mode = newMode;
        goToState(state.mode);

        EEPROM.put(0, state);
        EEPROM.commit();
      }
    }

    if ( subscription == &subForDur ) {
      String msg = (char *)subForDur.lastread;
      uint32_t newForDur = msg.toInt() * 60UL; // min -> sec

      if ( newForDur != state.forDur ) {
        Serial << "getStatus.  forDur=" << msg << " " << newForDur << endl;
        state.forDur = newForDur;

        EEPROM.put(0, state);
        EEPROM.commit();
      }
    }

    if ( subscription == &subMotionDur ) {
      String msg = (char *)subMotionDur.lastread;
      uint32_t newMotionDur = msg.toInt();

      if ( newMotionDur != state.motionDur ) {
        Serial << "getStatus.  motionDur=" << msg << " " << newMotionDur << endl;
        state.motionDur = newMotionDur;
        
        EEPROM.put(0, state);
        EEPROM.commit();
      }
    }

  }
}

void sendStatus() {
  static boolean motion, door;
  static byte mode;
  static uint32_t forDur, motionDur;

  static byte next = 0;

  // prioritize if changed
  if ( motion != state.motion ) next = 0;
  if ( door != state.door ) next = 1;
  if ( mode != state.mode ) next = 2;
  if ( forDur != state.forDur ) next = 3;
  if ( motionDur != state.motionDur ) next = 4;

  static Metro sendInterval(1UL * 1000UL); // take care to avoid throttling
  if ( sendInterval.check() ) {
    switch ( next ) {
      case 0: pubMotion.publish(state.motion); motion = state.motion; break;
      case 1: pubDoor.publish(state.door); door = state.door; break;
      case 2: pubMode.publish(state.mode); mode = state.mode; break;
      case 3: pubForDur.publish((uint32_t)(state.forDur / 60UL)); forDur = state.forDur; break; // sec -> min
      case 4: pubMotionDur.publish(state.motionDur); motionDur = state.motionDur; break;
    }
    // rotate through
    if ( ++next > 4 ) next = 0;
  }
}

void goToState(byte mode) {
  mode = constrain(mode, 0, sizeof(stateNames));
  state.mode = mode;
  Serial << "goToState.  " << stateNames[mode] << endl;
  switch ( mode ) {
    case 0: sM.transitionTo(closed); break;
    case 1: sM.transitionTo(inOnly); break;
    case 2: sM.transitionTo(open); break;
    case 3: sM.transitionTo(openThenClosed); break;
    case 4: sM.transitionTo(openThenInOnly); break;
  }
}

void doorDelay(uint32_t dur) {
  Metro waitfor(dur);
  waitfor.reset();

  while ( ! waitfor.check() ) {
    if ( door.update() ) digitalWrite(PIN_LED, door.read());
    yield();
  }
}
void pulseDoorMotor(uint32_t dur) {
  digitalWrite(PIN_MOTOR, HIGH);    // turn the LED off by making the voltage LOW
  doorDelay(dur);
  digitalWrite(PIN_MOTOR, LOW);    // turn the LED off by making the voltage LOW
  doorDelay(20);  
}
uint32_t lastOpen;
void openDoor() {
  pulseDoorMotor(40);
  while ( door.read() == HIGH ) pulseDoorMotor(8);
  
  state.door = true;
  lastOpen = timeClient.getEpochTime();
  Serial << "door open." << endl;
}
void closeDoor() {
  if( state.door == true && door.read() == LOW ) pulseDoorMotor(90);
  state.door = false;
  Serial << "door closed." << endl;  
}

String getDayTime() {
  String tdc;

  int day = timeClient.getDay();
  switch (day) {
    case 0: tdc = "Sun "; break;
    case 1: tdc = "Mon "; break;
    case 2: tdc = "Tue "; break;
    case 3: tdc = "Wed "; break;
    case 4: tdc = "Thu "; break;
    case 6: tdc = "Fri "; break;
    case 7: tdc = "Sat "; break;
  }
  tdc += timeClient.getFormattedTime();

  return ( tdc );
}

uint32_t lastMotion;
void updateMotion() {
  if ( motion.update() && motion.read() ) {
    lastMotion = timeClient.getEpochTime();
  }

  state.motion = motion.read();
  digitalWrite(PIN_LED, !state.motion); // LED is on when LOW
}

void inOnlyUpdate() {

  static boolean haveMotion = false;

  uint32_t now = timeClient.getEpochTime();

  if ( (now - lastMotion) <= state.motionDur && !haveMotion ) {
    haveMotion = true;
    Serial << "openMotion: motion. ";
    openDoor();
  }

  if ( (now - lastMotion) > state.motionDur && haveMotion ) {
    haveMotion = false;
    Serial << "openMotion: no motion. ";
    closeDoor();
  }

}

void openThenInOnlyUpdate() {
  uint32_t now = timeClient.getEpochTime();
  if ( (now - lastOpen) > state.forDur ) {
    Serial << "openThenInOnlyUpdate. duration expired=" << state.forDur << endl;
    goToState(1); // in only
  }
}

void openThenClosedUpdate() {
  uint32_t now = timeClient.getEpochTime();
  if ( (now - lastOpen) > state.forDur ) {
    Serial << "openThenClosedUpdate. duration expired=" << state.forDur << endl;
    goToState(0); // closed
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
