#include <Arduino.h>
#include <RadioLib.h>
#include "can.h"
#include "LoraRadio.h"
#include "aes.h"

unsigned long startTime = millis(), currentTime = 0;

CAN can;
LoRaRadio lora;


byte aes_key[] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                   0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F }; // 16 bytes for AES-128
byte aes_iv[]  = { 0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7,
                   0xA8, 0xA9, 0xAA, 0xAB, 0xAC, 0xAD, 0xAE, 0xAF }; // 16 bytes IV for CBC

///////////////////////////////////////////////////////TEST VARIABLES
unsigned long lastSimUpdate = 0;
uint8_t simValue = 0;
bool simBit = false;
uint8_t msgTransmitSim[41];
///////////////////////////////////////////////////////TEST VARIABLES

// struct the easy acces to data
struct TelemetryData
{
    uint8_t cooling;
    uint8_t brake_sens;
    uint8_t steer_sens;
    uint8_t acc1;
    uint8_t acc2;
    uint16_t wheel_spin_1;
    uint16_t wheel_spin_2;
    uint8_t motor_temp_1;
    uint8_t motor_temp_2;
    uint8_t control_temp_1;
    uint8_t control_temp_2;
    uint8_t serial_throttle_1;
    uint8_t serial_throttle_2;
    float inputV1_p;
    float inputV2_p;
    float phasecurrent1;
    float phasecurrent2;
    uint8_t susp_travel_FL;
    uint8_t susp_travel_FR;
    uint8_t susp_travel_BL;
    uint8_t susp_travel_BR;
    uint8_t ECU_control;
    uint16_t VRef_precharge;
    uint16_t meas_precharge;
    uint16_t current_sensor;
    uint16_t LV_state_of_charge;

    //for packed bytes
    uint8_t ventValue;
    uint8_t pumpValue;
    float flowValue;

    uint8_t brakeEngaged;
    uint8_t soundPlaying;
    uint8_t R2D_Button_State;
    uint8_t airPlusValue;
    uint8_t airMinusValue;
    uint8_t prechgValue;
    uint8_t SDC_END;
    uint8_t Measure_Digital;

    uint8_t IMU_Speed;
    float accelX_IMU;
    float accelY_IMU;
    float pitch_IMU;
    float roll_IMU;
    float yaw_IMU;

    uint16_t rawTemp1;
    uint16_t rawTemp2;
};

TelemetryData dataToSend;

////////////////////////////////////////////////////////////////////////////////TEST FUNCTIONS
void simulateTelemetryData() {
    // all analog fields 0-100-0
    for (int i = 1; i < 27; ++i) {
        msgTransmitSim[i] = simValue;
    }

    // toggle all bits ON or OFF every cycle
    if(simBit == 1){
        msgTransmitSim[0]  = 255; // ventValue, pumpValue, flowValue (all bits)
        msgTransmitSim[27] = 255; // all ECU control bits
    } else {
        msgTransmitSim[0]  = 0;   // ventValue, pumpValue, flowValue (all bits)
        msgTransmitSim[27] = 0;   // all ECU control bits
    }
    msgTransmitSim[28] = simValue; // VRef_precharge
    msgTransmitSim[29] = simValue; // meas_precharge
    msgTransmitSim[30] = simValue; // current_sensor
    msgTransmitSim[31] = simValue; // LV_state_of_charge

    msgTransmitSim[32] = simValue; // RPM_Speed (km/h)

    msgTransmitSim[33] = (int8_t)(simBit ? simValue : -simValue); // accelX_int
    msgTransmitSim[34] = simValue % 100;                          // accelX_frac

    msgTransmitSim[35] = (int8_t)(simBit ? -simValue : simValue); // accelY_int
    msgTransmitSim[36] = (simValue * 2) % 100;                    // accelY_frac

    msgTransmitSim[37] = (int8_t)(simValue / 2);                  // roll_int
    msgTransmitSim[38] = (simValue * 3) % 100;                    // roll_frac

    msgTransmitSim[39] = (int8_t)(-simValue / 2);                 // pitch_int
    msgTransmitSim[40] = (simValue * 4) % 100;  
}
////////////////////////////////////////////////////////////////////////////////TEST FUNCTIONS

float unpackFlowLps(uint8_t packed) {
    float max_lps = 10.0f / 60.0f;
    return packed * (max_lps / 255.0f);
}

void setup()
{
    Serial.begin(9600);
    while (!Serial)
        delay(50);
    Serial.println("serial initialized");

    // reset module for consistency
    lora.resetModule();

    // initialize the lora module
    SPI1.begin();
    lora.initModule();

    // set settings for lora modulation
    lora.setSettings2();

    // set the function that will be called
    // when new packet is received (both can and lora)
    radio.setDio0Action(setFlag, RISING);
    Serial.println("interrupt funcs set");

    // init can module
    can.init();

}

void loop()
{
    currentTime = millis();
    blink_led(1000);

    // // process ISO-TP receive only
    // can.isotp.receive(&can.receivedMsg);

    // if(can.receivedMsg.tp_state == ISOTP_FINISHED) {
    //     Serial.println();
    //     Serial.print("Received msg: ");
    //     for(int i=0;i<can.receivedMsg.len;i++) {
    //         Serial.print(can.receivedMsg.Buffer[i]);
    //         Serial.print(" ");
    //     }
    //     Serial.println();
    
    //     // AES CTR ENCRYPTION
    //     struct AES_ctx ctx;
    //     AES_init_ctx_iv(&ctx, aes_key, aes_iv);
    //     uint8_t encryptedCAN[can.receivedMsg.len];
    //     memcpy(encryptedCAN, can.receivedMsg.Buffer, can.receivedMsg.len);
    //     AES_CTR_xcrypt_buffer(&ctx, encryptedCAN, can.receivedMsg.len);
    
    //     lora.sendData(encryptedCAN, can.receivedMsg.len);
    //     transmitFlag = true;
    
    //     can.receivedMsg.tp_state = ISOTP_IDLE;
    // }
    // if(!digitalRead(INTCAN)) can.receive_normal_message();

    // send a message every 3s, with AES-128 CTR excryption
    if(currentTime - startTime >= 3000) {
        if (simValue >= 100) {
            simValue = 0;
        } else {
            simValue++;
            simBit = !simBit; // toggle all bits every cycle
        }
        simulateTelemetryData();
        //AES CTR ENCRYPTION
        struct AES_ctx ctx;
        AES_init_ctx_iv(&ctx, aes_key, aes_iv);

        // uint8_t encryptedMsg[sizeof(msgTransmit)];
        // memcpy(encryptedMsg, msgTransmit, sizeof(msgTransmit)); ------FOR CAN DATA

        uint8_t encryptedMsg[sizeof(msgTransmitSim)];
        memcpy(encryptedMsg, msgTransmitSim, sizeof(msgTransmitSim));

        Serial.print("Original: ");
        for (size_t i = 0; i < sizeof(msgTransmitSim); i++) {
            Serial.print(msgTransmitSim[i]);
            Serial.print(" ");
        }
        Serial.println();

        AES_CTR_xcrypt_buffer(&ctx, encryptedMsg, sizeof(encryptedMsg));

        Serial.print("Encrypted: ");
        for (size_t i = 0; i < sizeof(encryptedMsg); i++) {
            Serial.print(encryptedMsg[i]);
            Serial.print(" ");
        }
        Serial.println();

        lora.sendData(encryptedMsg, sizeof(encryptedMsg));
        startTime = currentTime;
        // radio.sleep();
    }

    // interrupt based handling
    // when flag is set, that means data was sent/received
    if (operationDone)
    {
        // reset flag
        operationDone = false;

        if (transmitFlag)
        {
            // the previous operation was transmission
            if (transmissionState == RADIOLIB_ERR_NONE)
            {
                // packet was successfully sent
                Serial.println(F("package transmission finished!"));
            }
            else
            {
                Serial.print(F("failed, code "));
                Serial.println(transmissionState);
            }

            transmitFlag = false;
        }
    }
}

