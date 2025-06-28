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
    uint8_t inputV1_p_intr;
    uint8_t inputV1_p_frac;
    uint8_t inputV2_p_intr;
    uint8_t inputV2_p_frac;
    uint8_t phasecurrent1_p_intr;
    uint8_t phasecurrent1_p_frac;
    uint8_t phasecurrent2_p_intr;
    uint8_t phasecurrent2_p_frac;
    uint8_t susp_travel_FL;
    uint8_t susp_travel_FR;
    uint8_t susp_travel_BL;
    uint8_t susp_travel_BR;
    uint8_t ECU_control;
    uint8_t VRef_precharge;
    uint8_t meas_precharge;
    uint8_t current_sensor;
    uint8_t LV_state_of_charge;
};

TelemetryData dataToSend;

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

    //TEST ONLY
    randomSeed(millis());
}

void loop()
{
    currentTime = millis();
    blink_led(1000);

    // poll the CAN line 24/7 (blocking ew)
    // when message is received immediately send it over LoRa
    // can.isotp.receive(&can.receivedMsg);

    // if(can.receivedMsg.tp_state == ISOTP_FINISHED) {
    //     Serial.println();
    //     Serial.print("Received msg: ");
    //     for(int i=0;i<can.receivedMsg.len;i++) {
    //         Serial.print(can.receivedMsg.Buffer[i]);
    //         Serial.print(" ");
    //     }
    //     Serial.println();
    //     // lora.sendDataAck2(can.receivedMsg.Buffer, can.receivedMsg.len);
    //     lora.sendData(can.receivedMsg.Buffer, can.receivedMsg.len);
    //     transmitFlag = true;
    // }

    // send a message every 3s, with AES-128 CTR excryption
    if(currentTime - startTime >= 3000) {
        for(int i=0; i<sizeof(msgTransmit)/sizeof(uint8_t); i++){
            msgTransmit[i] = random(0,255);
        }

        // --- AES CTR ENCRYPTION ---
        struct AES_ctx ctx;
        AES_init_ctx_iv(&ctx, aes_key, aes_iv);

        // Make a copy to avoid modifying the original if needed
        uint8_t encryptedMsg[sizeof(msgTransmit)];
        memcpy(encryptedMsg, msgTransmit, sizeof(msgTransmit));

                // Print original data---------------------------------------------------------------------------DELETE
        Serial.print("Original: ");
        for (size_t i = 0; i < sizeof(msgTransmit); i++) {
            Serial.print(msgTransmit[i]);
            Serial.print(" ");
        }
        Serial.println();
        //----------------------------------------------------------------------------------------------DELETE

        AES_CTR_xcrypt_buffer(&ctx, encryptedMsg, sizeof(encryptedMsg));

                // Print encrypted data----------------------------------------------------------------------------DELETE
        Serial.print("Encrypted: ");
        for (size_t i = 0; i < sizeof(encryptedMsg); i++) {
            Serial.print(encryptedMsg[i]);
            Serial.print(" ");
        }
        Serial.println();
        //----------------------------------------------------------------------------------------------DELETE

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

