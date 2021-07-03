#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(9, 10); // CE, CSN
const byte address[6] = "00001";

void setup() {
    pinMode(6, OUTPUT);
    Serial.begin(9600);
    radio.begin();
    radio.openReadingPipe(0, address);   //Setting the address at which we will receive the data
    radio.setPALevel(RF24_PA_MIN);       //You can set this as minimum or maximum depending on the distance between the transmitter and receiver.
    radio.startListening();              //This sets the module as receiver
}
void loop()
{
    if (radio.available())              //Looking for the data.
    {
        char data[32] = {0};                 //Saving the incoming data
        radio.read(&data, sizeof(data));    //Reading the data
        Serial.print("Received Data: ");
        for (int i = 0; i < 32; i++){
            Serial.print(data[i], HEX);
        }
        Serial.println("");
    }
delay(5);
}
