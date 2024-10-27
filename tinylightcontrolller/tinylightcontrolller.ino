//BEFORE FLASHING, ENSURE THAT THE ATTINY85 IS FLASHED TO 8MHZ
//to do so, set the lfuse bit to E2 using this: sudo avrdude -p t85 -c usbtiny -U lfuse:w:0xE2:m


#include <SoftwareSerial.h>    

#define RxD 0
#define TxD 1

SoftwareSerial blueToothSerial(RxD,TxD);
int relayPin = 4;
int state = HIGH;
int cmd = 0;
 
void setup() { 
  pinMode(RxD, INPUT);
  pinMode(TxD, OUTPUT);
  setupBlueToothConnection();
  pinMode(relayPin,OUTPUT);
  digitalWrite(relayPin, state); //initial state
} 

void loop() {
  if (blueToothSerial.available()) {
    cmd = blueToothSerial.read();
    if (cmd == '0') state = LOW;
    else if (cmd == '1') state = HIGH;
    else if (cmd == '2') state = state == HIGH ? state = LOW : state = HIGH;
    digitalWrite(relayPin, state);
  }
}
 
void setupBlueToothConnection() { //dont mess w this
  blueToothSerial.begin(9600); //init
  blueToothSerial.print("\r\n+STWMOD=0\r\n"); //set the bluetooth work in slave mode
  blueToothSerial.print("\r\n+STNA=MARC_HC06\r\n"); //set bt name
  blueToothSerial.print("\r\n+STOAUT=1\r\n"); // Permit Paired device to connect 
  delay(2000); // This delay is required.
  blueToothSerial.print("bluetooth connected!\n");
  delay(2000); // This delay is required.
  blueToothSerial.flush();
}
