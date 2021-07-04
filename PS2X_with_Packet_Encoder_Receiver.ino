#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>


namespace krtmi {

namespace crc8 {
    using crc_t = uint_fast8_t;

    const crc_t crc_table[256] = {
        0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15, 0x38, 0x3f, 0x36, 0x31, 0x24, 0x23, 0x2a, 0x2d,
        0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65, 0x48, 0x4f, 0x46, 0x41, 0x54, 0x53, 0x5a, 0x5d,
        0xe0, 0xe7, 0xee, 0xe9, 0xfc, 0xfb, 0xf2, 0xf5, 0xd8, 0xdf, 0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd,
        0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85, 0xa8, 0xaf, 0xa6, 0xa1, 0xb4, 0xb3, 0xba, 0xbd,
        0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2, 0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea,
        0xb7, 0xb0, 0xb9, 0xbe, 0xab, 0xac, 0xa5, 0xa2, 0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a,
        0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32, 0x1f, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0d, 0x0a,
        0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42, 0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a,
        0x89, 0x8e, 0x87, 0x80, 0x95, 0x92, 0x9b, 0x9c, 0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4,
        0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec, 0xc1, 0xc6, 0xcf, 0xc8, 0xdd, 0xda, 0xd3, 0xd4,
        0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c, 0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44,
        0x19, 0x1e, 0x17, 0x10, 0x05, 0x02, 0x0b, 0x0c, 0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34,
        0x4e, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5c, 0x5b, 0x76, 0x71, 0x78, 0x7f, 0x6a, 0x6d, 0x64, 0x63,
        0x3e, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b, 0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13,
        0xae, 0xa9, 0xa0, 0xa7, 0xb2, 0xb5, 0xbc, 0xbb, 0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83,
        0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb, 0xe6, 0xe1, 0xe8, 0xef, 0xfa, 0xfd, 0xf4, 0xf3
    };

    inline crc_t crc_init(void)
    {
        return 0x00;
    }

    crc_t crc_update(crc_t crc, const void *data, size_t data_len)
    {
        const unsigned char *d = (const unsigned char *)data;
        unsigned int tbl_idx;

        while (data_len--) {
            tbl_idx = crc ^ *d;
            crc = crc_table[tbl_idx] & 0xff;
            d++;
        }
        return crc & 0xff;
    }

    inline crc_t crc_finalize(crc_t crc)
    {
        return crc;
    }

}

class PS2XPacket {
    public:
        // public constant value
        static const uint8_t kStartFlag         = 0x7E;
        static const uint8_t kStopFlag          = 0x7D;
        static const uint8_t kPacketSize        = 9;
        static const uint8_t kDataSize          = 6; 

    private:
        // private constant value
        static const uint8_t kStartIndex        = 0;
        static const uint8_t kStopIndex         = kPacketSize - 1;
        static const uint8_t kCRCIndex          = kPacketSize - 2;
        static const uint8_t kDataStartIndex    = 1;
        static const uint8_t kRightAnalogX      = 3;
        static const uint8_t kRightAnalogY      = 4;
        static const uint8_t kLeftAnalogX       = 5;
        static const uint8_t kLeftAnalogY       = 6;  


        /*  packet consist of

            |  1-byte | 2-byte  | 1-byte  | 1-byte  | 1-byte | 1-byte  |  1-byte  |  1-byte  |
            |  START  | buttons | right-x | right-y | left-x | right-y |   crc8   |   STOP   | 
        

            buttons payload is a 2-byte where every bit is a representation of a button press

            | SQUARE | CROSS | CIRCLE | TRIANGLE | R1 | L1 | R2 | L2 | PAD_LEFT | PAD_DOWN | PAD_RIGHT | PAD_UP | START | R3 | L3 | SELECT |
        */
        uint8_t packet[kPacketSize] = {0};  
        uint16_t* button_states = (uint16_t*) &packet[kDataStartIndex];

    public:  
        PS2XPacket() = default;
        inline bool GetButtonState( uint16_t button ){
            if (button >= 0x0001 && button <= 0x8000 ){
                return *button_states & button ? true : false;
            }
            return false;
        }

        inline bool SetButtonState( uint16_t button ){
            if (button >= 0x0001 && button <= 0x8000 ){
                *button_states = *button_states | button;
                return true;
            }
            return false;
        }

        inline bool ClearButtonState( uint16_t button ){
            if (button >= 0x0001 && button <= 0x8000 ){
                *button_states = *button_states & ~(button);
                return true;
            }
            return false;
        }

        inline uint8_t GetAnalogValue( uint8_t analog_type){
            analog_type -= 2;
            if (analog_type >=3 && analog_type <= 6){
                return packet[analog_type];
            } 
            return 0;            
        }

        inline bool SetAnalogValue( uint8_t analog_type, uint8_t value ){
            analog_type -= 2;
            if (analog_type >=3 && analog_type <= 6){
                packet[analog_type] = value;
                return true;
            } 
            return false;
            
        }

        inline uint8_t* GetPacket(){
            return packet;
        }

        inline bool ParsePacket(uint8_t* packet_in, size_t size){
            if (packet_in != nullptr &&
                size == kPacketSize &&
                packet_in[kStartIndex] == kStartFlag &&
                packet_in[kStopIndex] == kStopFlag
            ){  
                crc8::crc_t crc = crc8::crc_init();
                crc = crc8::crc_update(crc, &packet_in[kDataStartIndex], kDataSize);
                crc = crc8::crc_finalize(crc);
                
                if (packet_in[kCRCIndex] != crc){
                    return false;
                }

                memcpy(this->packet, packet_in, kPacketSize);
                return true;
            }
            return false;
        }

        void Clear(){
            *button_states = 0;
        }

        void Finalize(){
            
            packet[kStartIndex] = kStartFlag;             // set start flag

            crc8::crc_t crc = crc8::crc_init();
            crc = crc8::crc_update(crc, &packet[kDataStartIndex], kDataSize);
            crc = crc8::crc_finalize(crc);

            packet[kCRCIndex] = crc; // set stop flag
            packet[kStopIndex]  = kStopFlag; // set stop flag
        }

        void Print(){
#ifdef ARDUINO
            Serial.print("PS2X Controller State: ");
            for (int i = 0; i < 16; i++){

                bool state = (*button_states >> i) & 1U;
                Serial.print(state, DEC);
            }
            Serial.println();
            Serial.print("Analog Right X: ");
            Serial.println(packet[kRightAnalogX], DEC);
            Serial.print("Analog Right Y: ");
            Serial.println(packet[kRightAnalogY], DEC);
            Serial.print("Analog Left X: ");
            Serial.println(packet[kLeftAnalogX], DEC);
            Serial.print("Analog Left Y: ");
            Serial.println(packet[kLeftAnalogY], DEC);            
#else
            printf("PS2X Controller State: ");
            for (int i = 0; i < 16; i++){

                bool state = (*button_states >> i) & 1U;
                PDEBUGMESSAGE("%d", state);
            }
            printf("\n");
            printf("Analog Right X: %d\n", packet[kRightAnalogX]);
            printf("Analog Right Y: %d\n", packet[kRightAnalogY]);
            printf("Analog Left X: %d\n", packet[kLeftAnalogX]);
            printf("Analog Left Y: %d\n", packet[kLeftAnalogY]);
#endif
        }
};

} // namespace krtmi

// nRF24L01 configuration
RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001";

krtmi::PS2XPacket ps2x_packet;
void setup() {
    pinMode(6, OUTPUT);
    Serial.begin(9600);
    if (!radio.begin()) {
        Serial.println(F("radio hardware is not responding!!"));
        while (1) {} // hold in infinite loop
    }
    radio.openReadingPipe(0, address);   //Setting the address at which we will receive the data
    radio.setPALevel(RF24_PA_MIN);       //You can set this as minimum or maximum depending on the distance between the transmitter and receiver.
    radio.startListening();              //This sets the module as receiver
}
void loop()
{
    bool data_ready = false;
    if (radio.available())              //Looking for the data.
    {
        Serial.print("Receive Payload Size: ");  
        Serial.println(radio.getDynamicPayloadSize());      
        char data[krtmi::PS2XPacket::kPacketSize] = {0};    //Saving the incoming data
        radio.read(&data, sizeof(data));    //Reading the data
        Serial.print("Received Data: ");
        for (int i = 0; i < sizeof(data); i++){
            Serial.print(data[i], HEX);
        }
        Serial.println("");
        
        // parsing data 
        data_ready = ps2x_packet.ParsePacket(data, sizeof(data));
        Serial.print("Parsing Data : ");
        Serial.println(data_ready);
        if (data_ready){
            ps2x_packet.Print();
        } else {
            Serial.print("Failed parsing data.");
        }
    }
delay(5);
}
