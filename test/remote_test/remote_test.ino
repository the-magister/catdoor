#include <Streaming.h>
#include <Metro.h>
#include <Bounce.h>
#include <FiniteStateMachine.h>
#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <ESP_EEPROM.h>

#define WLAN_SSID       "Looney"
#define WLAN_PASS       "TinyandTooney"

#define MQTT_SERVER      "broker.hivemq.com"
#define MQTT_SERVERPORT  1883

unsigned long openForDuration = 10UL * 1000UL;
unsigned long openMotionDuration = 20UL * 1000UL;

#define PIN_PIR D3
#define PIN_RELAY D1

Bounce motion = Bounce( PIN_PIR, 1 );

void closeDoor();
void openMotion();
void openDoor();
void openFor() ;
void goToClosed();
void goToInOnly();

State closed = State(closeDoor, NULL, NULL);
State inOnly = State(closeDoor, openMotion, NULL);
State open = State(openDoor, NULL, NULL);
State openThenClosed = State(openDoor, openFor, goToClosed);
State openThenInOnly = State(openDoor, openFor, goToInOnly);

FSM sM = FSM(inOnly); //initialize state machine

WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, MQTT_SERVER, MQTT_SERVERPORT);

Adafruit_MQTT_Publish pubMotion = Adafruit_MQTT_Publish(&mqtt, "magister/catdoor/motion");
Adafruit_MQTT_Publish pubDoor = Adafruit_MQTT_Publish(&mqtt, "magister/catdoor/door");

Adafruit_MQTT_Subscribe subMode = Adafruit_MQTT_Subscribe(&mqtt, "magister/catdoor/mode");
Adafruit_MQTT_Subscribe subForDur = Adafruit_MQTT_Subscribe(&mqtt, "magister/catdoor/doorDur");
Adafruit_MQTT_Subscribe subMotionDur = Adafruit_MQTT_Subscribe(&mqtt, "magister/catdoor/motionDur");

void MQTT_connect();

const int EEPROM_start = 0;

void setup() {
  pinMode(PIN_PIR, INPUT);
  pinMode(PIN_RELAY, OUTPUT);
  pinMode(BUILTIN_LED, OUTPUT);

  delay(300);
  Serial.begin(115200);
  Serial << endl << endl;
  Serial << "Setup. begin." << endl;

  Serial << "Connecting to: " << WLAN_SSID << endl;

  WiFi.begin(WLAN_SSID, WLAN_PASS);

  Serial.println("Initializing EEPROM...");

  EEPROM.begin(512);
  EEPROM.get(0, openForDuration);
  EEPROM.get(sizeof(openForDuration), openMotionDuration);
  String mode;
  EEPROM.get(sizeof(openForDuration) + sizeof(openMotionDuration), mode);
  Serial << "openForDur = " << openForDuration << endl;
  Serial << "openMotionDur = " << openMotionDuration << endl;
  Serial << "mode = " << mode << endl;

  if ( mode.equals("closed") ) {
    sM.transitionTo(closed);
  } else if ( mode.equals("inOnly") ) {
    sM.transitionTo(inOnly);
  } else if ( mode.equals("open") ) {
    sM.transitionTo(open);
  } else if ( mode.equals("openForThenClosed") ) {
    sM.transitionTo(openThenClosed);
  } else if ( mode.equals("openForThenInOnly") ) {
    sM.transitionTo(openThenInOnly);
  }

  // Setup MQTT subscription for instructions.
  mqtt.subscribe(&subMode);
  mqtt.subscribe(&subForDur);
  mqtt.subscribe(&subMotionDur);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.println("WiFi connected");
  Serial.println("IP address: "); Serial.println(WiFi.localIP());

  Serial << "Setup. complete." << endl;
}

void msgUpdates() {

  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(1000))) {
    if (subscription == &subMode) {
      String msg = (char *)subMode.lastread;
      Serial << "msgUpdates. mode: " << msg << endl;
      if ( msg.equals("closed") ) {
        sM.transitionTo(closed);
      } else if ( msg.equals("inOnly") ) {
        sM.transitionTo(inOnly);
      } else if ( msg.equals("open") ) {
        sM.transitionTo(open);
      } else if ( msg.equals("openForThenClosed") ) {
        sM.transitionTo(openThenClosed);
      } else if ( msg.equals("openForThenInOnly") ) {
        sM.transitionTo(openThenInOnly);
      }
      EEPROM.put(sizeof(openForDuration) + sizeof(openMotionDuration), msg);
      EEPROM.commit();
    }

    if (subscription == &subForDur) {
      String msg = (char *)subForDur.lastread;
      unsigned long dur = msg.toInt();
      openForDuration = dur * 60UL * 60UL * 1000UL; // h -> ms
      EEPROM.put(0, openForDuration);
      EEPROM.commit();
      Serial << "doorUpdate. forDuration: " << dur << " hr (" << openForDuration << " ms)" << endl;
    }

    if (subscription == &subMotionDur) {
      String msg = (char *)subMotionDur.lastread;
      unsigned long dur = msg.toInt();
      openMotionDuration = dur * 1000UL; // sec -> ms
      EEPROM.put(sizeof(openForDuration), openMotionDuration);
      EEPROM.commit();
      Serial << "doorUpdate. motionDur: " << dur << " sec (" << openMotionDuration << " ms)" << endl;
    }
  }

}

void loop() {
  MQTT_connect();
  yield();
  
  msgUpdates();
  yield();

  sM.update();
  yield();

  static Metro keepAlive(30UL * 1000UL);
  if ( keepAlive.check() && !mqtt.ping() ) mqtt.disconnect();
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

void closeDoor() {
  digitalWrite(PIN_RELAY, LOW);
  Serial << "shutting door." << endl;

  if (! pubDoor.publish(0) )  Serial << "MQTT Failed!" << endl;
}

void openDoor() {
  digitalWrite(PIN_RELAY, HIGH);
  Serial << "opening door." << endl;

  if (! pubDoor.publish(1) ) Serial << "MQTT Failed!" << endl;
}

void exitThisState() {
  State currState = sM.getCurrentState();
  currState.exit();
}

void openMotion() {

  static boolean haveMotion = false;
  static unsigned long lastMotion;

  if ( motion.update() && motion.read() ) {
    lastMotion = millis();
    haveMotion = true;

    Serial << "openMotion: motion. ";
    openDoor();

    if (! pubMotion.publish(1) )  Serial << "MQTT Failed!" << endl;

  }

  if ( haveMotion && (millis() - lastMotion) > openMotionDuration  ) {
    haveMotion = false;

    Serial << "openMotion: no motion. ";
    closeDoor();

    if (! pubMotion.publish(0) ) Serial << "MQTT Failed!" << endl;

  }

  digitalWrite(BUILTIN_LED, !motion.read());

}

void openFor() {
  if ( sM.timeInCurrentState() <= openForDuration ) return;

  Serial << "openFor.  duration=" << openForDuration << endl;

  exitThisState();
}
void goToClosed() {
  sM.transitionTo(closed);
}
void goToInOnly() {
  sM.transitionTo(inOnly);
}
