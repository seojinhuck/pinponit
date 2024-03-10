#include <SPI.h>

#include <RF24.h>

#include <Servo.h>

#define MAX_SIGNAL 1900

#define MIN_SIGNAL 1100

byte MOTOR_PIN = 9;

byte MOTOR2_PIN = 10;


RF24 radio(7, 8); //CE, CSN

Servo esc_motor;

Servo esc_motor2;


int joystick[3];

const byte address[6] = "00001"; 

//The address value can be changed to 5 strings, and the transmitter and the receiver must have the same address.

int motor;

int motor2;


void setup() {

  Serial.begin(9600);
  

  Serial.println("Program begin...");

  radio.begin();

  radio.openReadingPipe(0, address);

  radio.setPALevel(RF24_PA_MAX);

  esc_motor.attach(MOTOR_PIN);

  esc_motor2.attach(MOTOR2_PIN);

  radio.startListening();

  


  esc_motor.writeMicroseconds(MAX_SIGNAL);

  delay(100);

  esc_motor.writeMicroseconds(MIN_SIGNAL);

  esc_motor2.writeMicroseconds(MAX_SIGNAL);

  delay(100);

  esc_motor2.writeMicroseconds(MIN_SIGNAL);
}


void loop() {

  if (radio.available()) {

    

    radio.read(&joystick, sizeof(joystick));

    

    char buffer[20];

    sprintf(buffer , "X : %d, X2 :  %d", joystick[1], joystick[2]);

    Serial.println(buffer);

    

    motor = joystick[1];

    motor2 = joystick[2];

    esc_motor2.writeMicroseconds(motor2);

    esc_motor.writeMicroseconds(motor);

  }

}

