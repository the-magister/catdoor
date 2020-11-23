#include <Streaming.h>
#include <Metro.h>
#include <Bounce.h>

/*
  Blink

  Turns an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the UNO, MEGA and ZERO
  it is atached to digital pin 13, on MKR1000 on pin 6. PIN_LED is set to
  the correct LED pin independent of which board is used.
  If you want to know what pin the on-board LED is connected to on your Arduino
  model, check the Technical Specs of your board at:
  https://www.arduino.cc/en/Main/Products

  modified 8 May 2014
  by Scott Fitzgerald
  modified 2 Sep 2016
  by Arturo Guadalupi
  modified 8 Sep 2016
  by Colby Newman

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/Blink
*/

// D4 is builtin LED
#define PIN_LED D4
#define PIN_MOTOR D1
#define PIN_DOOR D6
#define PIN_PIR D2

Bounce doorSwitch = Bounce( PIN_DOOR, 1 );
Bounce motionDet = Bounce( PIN_PIR, 1 );

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin PIN_LED as an output.
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_MOTOR, OUTPUT);
  pinMode(PIN_DOOR, INPUT_PULLUP);
  pinMode(PIN_PIR, INPUT);

  delay(300);
  Serial.begin(115200);
  Serial << endl << endl;

  // start with a manual sync.
  myDelay(100);

  Serial << "Open door..." << endl;
  openDoor();
  myDelay(500);
  Serial << "Close door..." << endl;
  closeDoor();
  myDelay(50);

  Serial << "Startup complete." << endl;
}

void pulseDoorMotor(uint32_t dur) {
  digitalWrite(PIN_MOTOR, HIGH);    // turn the LED off by making the voltage LOW
  myDelay(dur);
  digitalWrite(PIN_MOTOR, LOW);    // turn the LED off by making the voltage LOW
  myDelay(20);  
}
void openDoor() {
  pulseDoorMotor(40);
  while ( doorSwitch.read() == HIGH ) pulseDoorMotor(8);
}
void closeDoor() {
  pulseDoorMotor(90);
}

// the loop function runs over and over again forever
void loop() {
  myDelay(1);

  if( motionDet.update() ) {
    Serial << "Motion: " << motionDet.read() << endl;
    if( motionDet.read() == HIGH ) openDoor();
    if( motionDet.read() == LOW ) closeDoor(); 
  }
}

void myDelay(uint32_t dur) {
  Metro waitfor(dur);
  waitfor.reset();

  while ( ! waitfor.check() ) {
    if ( doorSwitch.update() ) digitalWrite(PIN_LED, doorSwitch.read());
    yield();
  }
}
