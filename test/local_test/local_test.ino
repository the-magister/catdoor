#include <Streaming.h>
#include <Metro.h>
#include <Bounce.h>
#include <FiniteStateMachine.h>

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

void setup() {
  pinMode(PIN_PIR, INPUT);
  pinMode(PIN_RELAY, OUTPUT);
  pinMode(BUILTIN_LED, OUTPUT);

  delay(300);
  Serial.begin(115200);
  Serial << endl << endl;
  Serial << "Setup. complete." << endl;
}


void loop() {

  sM.update();

}

void closeDoor() {
  digitalWrite(PIN_RELAY, LOW);
  Serial << "shutting door." << endl;
}

void openDoor() {
  digitalWrite(PIN_RELAY, HIGH);
  Serial << "opening door." << endl;
}

void exitThisState() {
  State currState = sM.getCurrentState();
  currState.exit();
}

void openMotion() {

  static boolean haveMotion = false;
  static unsigned long lastMotion;

  static Metro heartbeat(5000);
  if ( heartbeat.check() ) Serial << "..." << endl;

  if ( motion.update() && motion.read() ) {
    lastMotion = millis();
    haveMotion = true;

    Serial << "openMotion: motion. ";
    openDoor();
  }

  if ( haveMotion && (millis() - lastMotion) > openMotionDuration  ) {
    haveMotion = false;

    Serial << "openMotion: no motion. ";
    closeDoor();
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
