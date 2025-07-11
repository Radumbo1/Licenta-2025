#include <RadioLib.h>

SX1276 radio = new Module(LORA_IRQ_DUMB, LORA_IRQ, RADIOLIB_NC, RADIOLIB_NC,
    SPI1, SPISettings(200000, MSBFIRST, SPI_MODE0));

int state = RADIOLIB_ERR_NONE;
bool operationDone = false, transmitFlag = false, LEDstatus = false, receiveValid = false;
uint8_t msgTransmit[32];
uint8_t receivedMsg[64];

// save transmission states between loops
int transmissionState = RADIOLIB_ERR_NONE;

float freq_list[8] = {868.1, 867.1, 868.5, 868.3, 867.3, 867.5, 867.9, 867.7};
const int num_freqs = sizeof(freq_list) / sizeof(freq_list[0]);
int freq_cnt = 0;

void setFlag(void);

// interrupt based stuff..!
void setFlag(void) {
    operationDone = true;
}

unsigned long blinkLED = 0;
bool led = true;

void blink_led(int s){
    if(millis() - blinkLED > s){
        blinkLED = millis();
        digitalWrite(LED_BUILTIN, led);
        led = !led;
    }
}


class LoRaRadio {
public:
    LoRaRadio(){}

    void initModule(){
        int status = radio.begin(868.1);
        if (status == RADIOLIB_ERR_NONE)
            Serial.println("LoRa module initialized.");
        else {
            Serial.println("LoRa init failed");
            Serial.println(status);
            while (1) {
                Serial.print(".");
                delay(1000);
            }
        }
    }

    void resetModule(){
        // code straight from a github issue
        // Reset Modem (thank you sandeepmistry and associates)
        pinMode(LORA_IRQ_DUMB, OUTPUT);
        digitalWrite(LORA_IRQ_DUMB, LOW);

        // Hardware reset
        pinMode(LORA_BOOT0, OUTPUT);
        digitalWrite(LORA_BOOT0, LOW);

        pinMode(LORA_RESET, OUTPUT);
        digitalWrite(LORA_RESET, HIGH);
        delay(200);
        digitalWrite(LORA_RESET, LOW);
        delay(200);
        digitalWrite(LORA_RESET, HIGH);
        delay(50);
    }

    void setSettings(){
        radio.setFrequency(868.1);
        radio.setSpreadingFactor(7);
        radio.setBandwidth(250.0);
        radio.setCodingRate(5);
        radio.setOutputPower(14); 
        radio.setPreambleLength(8);
        radio.setCRC(true); 
        radio.setSyncWord(0x45);
        Serial.println("radio settings init done");
    }
    void setSettings2(){
        radio.setFrequency(868.1);
        radio.setSpreadingFactor(8);
        radio.setBandwidth(125.0);
        radio.setCodingRate(5);
        radio.setOutputPower(14); 
        radio.setPreambleLength(8);
        radio.setCRC(true);
        radio.setSyncWord(0x45);
        Serial.println("radio settings init done");
    }

    void sendData(uint8_t *str, size_t size) {
        Serial.print("sizeof(str) = "); Serial.println(sizeof(str));
        float str_size = size;
        int subSize = ceil(str_size / 7);
        freq_cnt = 0;
    
        Serial.print("str_Size: ");
        Serial.println(str_size);
        Serial.print("subSize: ");
        Serial.println(subSize);
    
        //cycle through freq and divide the message into subSize chunks
        uint8_t* sizedChunk = (uint8_t*)calloc(subSize, sizeof(uint8_t));
        int cnt = 0;
        for (int i = 0; i < 7; ++i) {
            Serial.print("Cnt: \t\t");
            Serial.println(cnt);

            Serial.print("Chunk:\t\t");
            for(int j=0;j<subSize; j++){
                sizedChunk[j] = str[j+cnt];
                Serial.print(sizedChunk[j]);
                Serial.print(" ");
            }
            Serial.println();

            Serial.print("Freq:\t\t");
            Serial.println(freq_list[freq_cnt]);
    
            radio.setFrequency(freq_list[freq_cnt]);
            transmissionState = radio.startTransmit(sizedChunk, subSize);
            if (transmissionState == RADIOLIB_ERR_NONE)
                Serial.println("Chunk transmit OK.\n");
            else {
                Serial.print(F("failed chunk transmit, code "));
                Serial.println(transmissionState);
            }
    
            // delay transmission of each chunk by some value
            // to make sure we can receive every packet on the receiver side
            delay(100);
    
            freq_cnt++;
            cnt += subSize;
            if (freq_cnt > 6) freq_cnt = 0;
        }
        free(sizedChunk);
        transmitFlag = true;
    }


    void displayData(byte* str, int len) {
        // packet was successfully received
        Serial.println(F("Received packet!"));

        Serial.print("Length: \t");
        Serial.println(len);
    
        // print data of the packet
        Serial.print(F("Data:\t\t"));
        // Serial.println(str);
        for(int i=0;i<len;i++){
            Serial.print(str[i]);
            Serial.print(" ");
        }
        Serial.println();
    
        // print RSSI (Received Signal Strength Indicator)
        Serial.print(F("RSSI:\t\t"));
        Serial.print(radio.getRSSI());
        Serial.println(F(" dBm"));
    
        // print SNR (Signal-to-Noise Ratio)
        Serial.print(F("SNR:\t\t"));
        Serial.print(radio.getSNR());
        Serial.println(F(" dB"));

        // print channel
        Serial.print("Frequency: \t");
        Serial.println(freq_list[freq_cnt]);

        // print whole str array
        Serial.print("Array: \t\t");
        for(int i=0;i<20;i++){
            Serial.print(str[i]);
            Serial.print(" ");
        }
        Serial.println();

        Serial.print("\n");
    }
};