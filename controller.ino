#include <SPI.h>

#include <nRF24L01.h>

#include <RF24.h>


RF24 radio(7, 8); //CE, CSN


int joystick[3];

const byte address[6] = "00001"; 

//The address value can be changed to 5 strings, and the transmitter and the receiver must have the same address.

int inputPinX = A0;

int inputPinX2 = A1;


void setup() {

  Serial.begin(9600);

  radio.begin();

  radio.openWritingPipe(address); //Set address

  radio.setPALevel(RF24_PA_MAX); 

  //Sets the power level. If the distance between modules is close, set to minimum.  


  
  

  radio.stopListening();  //transmitter..

  pinMode(inputPinX, INPUT);

  pinMode(inputPinX2,INPUT);

}

void loop() {

  joystick[1] = analogRead(inputPinX)*0.9+1026; //value of ESC

  joystick[2] = analogRead(inputPinX2)*0.9+1034;    //value of servo

  radio.write(&joystick, sizeof(joystick) ); 

  //Send to receiver

}

