#include <Streaming.h>
#include <Metro.h>
#include <Bounce.h>
#include <FiniteStateMachine.h>
#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

#define WLAN_SSID       "Looney"
#define WLAN_PASS       "TinyandTooney"

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883                   // use 8883 for SSL
#define AIO_USERNAME  "magister"
#define AIO_KEY       "aio_pPTq93Pfx3NXjHEqnqzwOX9KF7WA"

unsigned long openForDuration = 10UL * 1000UL;
unsigned long openMotionDuration = 20UL * 1000UL;

#define PIN_PIR D3
#define PIN_RELAY D1

Bounce motion = Bounce( PIN_PIR, 5 );

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
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

Adafruit_MQTT_Publish pubMotion = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/catdoor.motion");
Adafruit_MQTT_Publish pubDoor = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/catdoor.door");
Adafruit_MQTT_Subscribe subMode = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/catdoor.mode");

void MQTT_connect();

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
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.println("WiFi connected");
  Serial.println("IP address: "); Serial.println(WiFi.localIP());

  // Setup MQTT subscription for onoff feed.
  mqtt.subscribe(&subMode);

  Serial << "Setup. complete." << endl;
}


void loop() {
  MQTT_connect();

  sM.update();

  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(5000))) {
    if (subscription == &subMode) {
      Serial.print(F("Got: "));
      Serial.println((char *)subMode.lastread);
    }
  }

  if (! mqtt.ping()) {
    mqtt.disconnect();
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
