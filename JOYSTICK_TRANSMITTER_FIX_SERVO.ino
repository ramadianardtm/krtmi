/*Arduino JOYSTICK CONTROLLED CAR (TRANSMITTER)
          
YOU HAVE TO INSTALL THE RF24 LIBRARY BEFORE UPLOADING THE CODE
   https://github.com/tmrh20/RF24/      
*/
#include <PS2X_lib.h>  //for v1.6
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

//PIN PUSH BUTTON
int buttonPin1 = A1;
int buttonPin2 = A2;
int buttonState1 = 0;
int buttonState2 = 0;

int data[2];
RF24 radio(7,8); // CE, CSN

const byte address[6] = "00001";
char xyData[32] = "";
String xAxis, yAxis;

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  pinMode(buttonPin1, INPUT_PULLUP);
  pinMode(buttonPin2, INPUT_PULLUP);
  radio.stopListening();
}
void loop() {
  //Kodingan Lama 
  xAxis = analogRead(A0); // Read Joysticks X-axis
  yAxis = analogRead(A1); // Read Joysticks Y-axis
  // X value
  xAxis.toCharArray(xyData, 5); // Put the String (X Value) into a character array
  radio.write(&xyData, sizeof(xyData)); // Send the array data (X value) to the other NRF24L01 modile
  // Y value
  yAxis.toCharArray(xyData, 5);
  radio.write(&xyData, sizeof(xyData));

  buttonState1 = digitalRead(buttonPin1);
  if(buttonState1 == HIGH){
    data[1] = 170;
    radio.write(data, sizeof(data));
  }if(buttonState1 == LOW){
    data[1] = 20;
    radio.write(data, sizeof(data));
  }

  buttonState2 = digitalRead(buttonPin2);
   if (buttonState2 == HIGH)
  {
    data[0] = 170;
    radio.write(data, sizeof(data));
  }
    if (buttonState2 == LOW)
  {
    data[0] = 20;
    radio.write(data, sizeof(data));
  }
  Serial.println(data[0]);
  Serial.println(data[1]);
  delay(20);

}
  
