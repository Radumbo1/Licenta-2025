#include <Arduino.h>
#include <RadioLib.h>
#include "can.h"
#include "LoraRadio.h"
#include "aes.h"

unsigned long startTime = millis(), currentTime = 0;

CAN can;
LoRaRadio lora;

byte aes_key[] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                   0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F };
byte aes_iv[]  = { 0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7,
                   0xA8, 0xA9, 0xAA, 0xAB, 0xAC, 0xAD, 0xAE, 0xAF };


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

    //extracted boolean/packed fields
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

//////////////////////////////////////////////////////////////////TEST FUNCTIONS

void simulateTelemetryData() {
    // analog-like fields 0-100-0
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


void convertToInfluxDB(void){
    dataToSend.cooling = receivedMsg[0];                // vent/pump/flow packed
    dataToSend.brake_sens = receivedMsg[1];             // valueBrake
    dataToSend.steer_sens = receivedMsg[2];             // valueSteering
    dataToSend.acc1 = receivedMsg[3];                   // pos1
    dataToSend.acc2 = receivedMsg[4];                   // pos2

    // Wheel RPMs
    dataToSend.wheel_spin_1 = (receivedMsg[5]) | (receivedMsg[6] << 8); // controllerDataL.RPMController
    dataToSend.wheel_spin_2 = (receivedMsg[7]) | (receivedMsg[8] << 8); // controllerDataR.RPMController

    // Temperatures
    dataToSend.motor_temp_1 = receivedMsg[9];           // controllerDataL.tempMotor
    dataToSend.motor_temp_2 = receivedMsg[10];          // controllerDataR.tempMotor
    dataToSend.control_temp_1 = receivedMsg[11];        // controllerDataL.tempController
    dataToSend.control_temp_2 = receivedMsg[12];        // controllerDataR.tempController

    // Throttle voltages (truncated integer values)
    dataToSend.serial_throttle_1 = (uint8_t)(receivedMsg[13] | (receivedMsg[14] << 8)); // controllerDataL.throttle
    dataToSend.serial_throttle_2 = (uint8_t)(receivedMsg[15] | (receivedMsg[16] << 8)); // controllerDataR.throttle

    // Supply voltages (scaled int16_t, divide by 100)
    int16_t inputV1_raw = (int16_t)(receivedMsg[17] | (receivedMsg[18] << 8));
    int16_t inputV2_raw = (int16_t)(receivedMsg[19] | (receivedMsg[20] << 8));
    dataToSend.inputV1_p = inputV1_raw / 100.0f;
    dataToSend.inputV2_p = inputV2_raw / 100.0f;

    // Phase currents (scaled int16_t, divide by 100)
    int16_t phase1_raw = (int16_t)(receivedMsg[21] | (receivedMsg[22] << 8));
    int16_t phase2_raw = (int16_t)(receivedMsg[23] | (receivedMsg[24] << 8));
    dataToSend.phasecurrent1 = phase1_raw / 100.0f;
    dataToSend.phasecurrent2 = phase2_raw / 100.0f;

    // Suspension travel
    dataToSend.susp_travel_FL = receivedMsg[25];
    dataToSend.susp_travel_FR = receivedMsg[26];
    dataToSend.susp_travel_BL = receivedMsg[27];
    dataToSend.susp_travel_BR = receivedMsg[28];

    // Packed ECU control bits
    dataToSend.ECU_control = receivedMsg[29];
    dataToSend.VRef_precharge = (uint16_t)(receivedMsg[30] | (receivedMsg[31] << 8));
    dataToSend.current_sensor = (uint16_t)(receivedMsg[32] | (receivedMsg[33] << 8));
    dataToSend.meas_precharge = (uint16_t)(receivedMsg[49] | (receivedMsg[50] << 8));
    dataToSend.LV_state_of_charge = receivedMsg[51]; // state of charge in %

    // Wheel speed (km/h)
    dataToSend.IMU_Speed = receivedMsg[34];

    dataToSend.accelX_IMU = (int16_t)(receivedMsg[35] | (receivedMsg[36] << 8)) / 32768.0f * 16.0f; // accelX in g

    dataToSend.accelY_IMU = (int16_t)(receivedMsg[37] | (receivedMsg[38] << 8)) / 32768.0f * 16.0f; // accelY in g

    dataToSend.roll_IMU = (int16_t)(receivedMsg[39] | (receivedMsg[40] << 8)) / 32768.0f * 180.0f; // roll in degrees

    dataToSend.pitch_IMU = (int16_t)(receivedMsg[41] | (receivedMsg[42] << 8)) / 32768.0f * 180.0f; // pitch in degrees

    dataToSend.rawTemp1 = (uint16_t)(receivedMsg[43] | (receivedMsg[44] << 8));
    dataToSend.rawTemp2 = (uint16_t)(receivedMsg[45] | (receivedMsg[46] << 8));

    dataToSend.yaw_IMU = (int16_t)(receivedMsg[47] | (receivedMsg[48] << 8)) / 32768.0f * 180.0f; // yaw in degrees


    // bit-unpack logic
    dataToSend.ventValue        = (receivedMsg[0] >> 7) & 0x01;
    dataToSend.pumpValue        = (receivedMsg[0] >> 6) & 0x01;
    dataToSend.flowValue        = unpackFlowLps((receivedMsg[0] & 0x3F)<<2); // unpacked flow in L/s

    dataToSend.brakeEngaged     = (receivedMsg[29] >> 7) & 0x01;
    dataToSend.soundPlaying     = (receivedMsg[29] >> 6) & 0x01;
    dataToSend.R2D_Button_State = (receivedMsg[29] >> 5) & 0x01;
    dataToSend.airPlusValue     = (receivedMsg[29] >> 4) & 0x01;
    dataToSend.airMinusValue    = (receivedMsg[29] >> 3) & 0x01;
    dataToSend.prechgValue      = (receivedMsg[29] >> 2) & 0x01;
    dataToSend.SDC_END          = (receivedMsg[29] >> 1) & 0x01;
    dataToSend.Measure_Digital  = (receivedMsg[29] >> 0) & 0x01;
}

void sendToInfluxDB()
{
    // telemetry received array to struct
    convertToInfluxDB();

    // formatting the data in line protocol for InfluxDB
    Serial.println("START INFLUX");
    Serial.print("telemetry_data,location=test,device=senzori ");

    Serial.print("cooling="); Serial.print(dataToSend.cooling); Serial.print(",");
    Serial.print("brake_sens="); Serial.print(dataToSend.brake_sens); Serial.print(",");
    Serial.print("steer_sens="); Serial.print(dataToSend.steer_sens); Serial.print(",");
    Serial.print("acc1="); Serial.print(dataToSend.acc1); Serial.print(",");
    Serial.print("acc2="); Serial.print(dataToSend.acc2); Serial.print(",");
    Serial.print("wheel_spin_1="); Serial.print(dataToSend.wheel_spin_1); Serial.print(",");
    Serial.print("wheel_spin_2="); Serial.print(dataToSend.wheel_spin_2); Serial.print(",");
    Serial.print("motor_temp_1="); Serial.print(dataToSend.motor_temp_1); Serial.print(",");
    Serial.print("motor_temp_2="); Serial.print(dataToSend.motor_temp_2); Serial.print(",");
    Serial.print("control_temp_1="); Serial.print(dataToSend.control_temp_1); Serial.print(",");
    Serial.print("control_temp_2="); Serial.print(dataToSend.control_temp_2); Serial.print(",");
    Serial.print("serial_throttle_1="); Serial.print(dataToSend.serial_throttle_1); Serial.print(",");
    Serial.print("serial_throttle_2="); Serial.print(dataToSend.serial_throttle_2); Serial.print(",");
    Serial.print("inputV1_p="); Serial.print(dataToSend.inputV1_p); Serial.print(",");
    Serial.print("inputV2_p="); Serial.print(dataToSend.inputV2_p); Serial.print(",");
    Serial.print("phasecurrent1="); Serial.print(dataToSend.phasecurrent1); Serial.print(",");
    Serial.print("phasecurrent2="); Serial.print(dataToSend.phasecurrent2); Serial.print(",");
    Serial.print("susp_travel_FL="); Serial.print(dataToSend.susp_travel_FL); Serial.print(",");
    Serial.print("susp_travel_FR="); Serial.print(dataToSend.susp_travel_FR); Serial.print(",");
    Serial.print("susp_travel_BL="); Serial.print(dataToSend.susp_travel_BL); Serial.print(",");
    Serial.print("susp_travel_BR="); Serial.print(dataToSend.susp_travel_BR); Serial.print(",");
    Serial.print("ECU_control="); Serial.print(dataToSend.ECU_control); Serial.print(",");
    Serial.print("VRef_precharge="); Serial.print(dataToSend.VRef_precharge); Serial.print(",");
    Serial.print("meas_precharge="); Serial.print(dataToSend.meas_precharge); Serial.print(",");
    Serial.print("current_sensor="); Serial.print(dataToSend.current_sensor); Serial.print(",");
    Serial.print("LV_state_of_charge="); Serial.print(dataToSend.LV_state_of_charge); Serial.print(",");

    Serial.print("ventValue="); Serial.print(dataToSend.ventValue); Serial.print(",");
    Serial.print("pumpValue="); Serial.print(dataToSend.pumpValue); Serial.print(",");
    Serial.print("flowValue="); Serial.print(dataToSend.flowValue); Serial.print(",");
    Serial.print("brakeEngaged="); Serial.print(dataToSend.brakeEngaged); Serial.print(",");
    Serial.print("soundPlaying="); Serial.print(dataToSend.soundPlaying); Serial.print(",");
    Serial.print("R2D_Button_State="); Serial.print(dataToSend.R2D_Button_State); Serial.print(",");
    Serial.print("airPlusValue="); Serial.print(dataToSend.airPlusValue); Serial.print(",");
    Serial.print("airMinusValue="); Serial.print(dataToSend.airMinusValue); Serial.print(",");
    Serial.print("prechgValue="); Serial.print(dataToSend.prechgValue); Serial.print(",");
    Serial.print("SDC_END="); Serial.print(dataToSend.SDC_END); Serial.print(",");
    Serial.print("Measure_Digital="); Serial.print(dataToSend.Measure_Digital); Serial.print(",");

    Serial.print("accelX="); Serial.print(dataToSend.accelX_IMU); Serial.print(",");
    Serial.print("accelY_="); Serial.print(dataToSend.accelY_IMU); Serial.print(",");
    Serial.print("roll="); Serial.print(dataToSend.roll_IMU); Serial.print(",");
    Serial.print("pitch="); Serial.print(dataToSend.pitch_IMU); Serial.print(",");
    Serial.print("yaw="); Serial.print(dataToSend.yaw_IMU); Serial.print(",");
    Serial.print("IMU_Speed="); Serial.print(dataToSend.IMU_Speed); Serial.print(",");
    Serial.print("rawTemp1="); Serial.print(dataToSend.rawTemp1); Serial.print(",");
    Serial.print("rawTemp2="); Serial.print(dataToSend.rawTemp2);


    Serial.println(" ");
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



    // start listening for LoRa packets on this node
    Serial.println("starting to listen :)");
    freq_cnt = 0;
    radio.setFrequency(freq_list[freq_cnt]);
    state = radio.startReceive();
    if (state == RADIOLIB_ERR_NONE)
    {
        Serial.println(F("success!"));
        Serial.print("Freq: \t\t");
        Serial.println(freq_list[freq_cnt]);
    }
    else
    {
        Serial.print(F("failed, code "));
        Serial.println(state);
        while (true)
        {
            delay(10);
        }
    }
}

void loop()
{

    currentTime = millis();
    blink_led(1000);

    // receiver code here
    // data has already been sent over serial to be displayed
    if (receiveValid)
    {
        size_t received_len = 32; // or however many bytes you actually receive

        Serial.print("Encrypted: ");
        for (size_t i = 0; i < received_len; i++) {
            Serial.print(receivedMsg[i]);
            Serial.print(" ");
        }
        Serial.println();

        struct AES_ctx ctx;
        uint8_t ctr_iv[16];
        memcpy(ctr_iv, aes_iv, 16); // must match the IV/nonce used by transmitter

        AES_init_ctx_iv(&ctx, aes_key, ctr_iv);

        AES_CTR_xcrypt_buffer(&ctx, receivedMsg, received_len);

        TelemetryData* receivedData = (TelemetryData*)receivedMsg;

        Serial.print("Decrypted: ");
        for (size_t i = 0; i < received_len; i++) {
            Serial.print(receivedMsg[i]);
            Serial.print(" ");
        }
        Serial.println();

        memcpy(&dataToSend, receivedData, sizeof(TelemetryData));
        receiveValid = false;
        Serial.println("\nStarting data processing.");

        // process data
        sendToInfluxDB();
        memset(receivedMsg, 0, sizeof(receivedMsg));
    }

    // interrupt based handling
    // when flag is set, that means data was sent/received
    if (operationDone)
    {
        // reset flag
        operationDone = false;

        
            Serial.println("Entered operation done initial main.");
            freq_cnt = 0;
            int cnt = 0, cntM = 0;
            // module received data on channel 1
            uint8_t *str = (uint8_t *)calloc(20, sizeof(uint8_t));

            int packetLength = radio.getPacketLength();
            int state = radio.readData(str, packetLength);
            for (int i = 0; i < packetLength; i++)
            {
                receivedMsg[i + cnt] = str[i];
                cntM++;
            }
            cnt += packetLength;

            // if message seen on first channel, cycle through the rest
            if (state == RADIOLIB_ERR_NONE)
            {
                // display first chunk
                lora.displayData(str, packetLength);
                receiveValid = true;
                /*
                chunk cycler
                goes through all channels and waits for the respective chunk to arrive
                */
                for (int i = 1; i < 7; ++i)
                {
                    freq_cnt++;
                    radio.setFrequency(freq_list[freq_cnt]);
                    radio.startReceive();

                    // reset str array
                    memset(str, 0, 20 * sizeof(*str));

                    // wait until next message is received on next channel OR timeout of 100ms
                    // opDone = false when entering, interrupt will make it true
                    unsigned long timeStart = millis();
                    Serial.print("waiting for next chunk");
                    while (!operationDone && (millis() - timeStart) < 300)
                    {
                    }
                    Serial.print("\n");
                    if (!operationDone)
                        Serial.println("ERR!! Message missed.");
                    operationDone = false;

                    packetLength = radio.getPacketLength();
                    state = radio.readData(str, packetLength);
                    for (int i = 0; i < packetLength; i++)
                    {
                        receivedMsg[i + cnt] = str[i];
                        cntM++;
                    }
                    cnt += packetLength;

                    // display received chunk if everything is ok
                    if (state == RADIOLIB_ERR_NONE)
                    {
                        lora.displayData(str, packetLength);
                    }
                    else
                    {
                        Serial.print(F("failed chunk read, code "));
                        Serial.println(state);
                        receiveValid = false;
                    }
                }
            }
            else
            {
                Serial.print(F("failed chunk 1 read, code "));
                Serial.println(state);
                Serial.print(F("Freq listening: "));
                Serial.println(freq_list[freq_cnt]);
            }

            // display whole message and reset string if all chunks are received
            if (receiveValid)
            {
                Serial.println("\nFull message: ");
                for (int i = 0; i < cntM; i++)
                {
                    Serial.print(receivedMsg[i]);
                    Serial.print(" ");
                }
                Serial.println();
            }
            else
            {
                Serial.println("Message chunks invalid. Skipping this packet.");
                memset(receivedMsg, 0, sizeof(receivedMsg));
            }

            // set freq back to channel 1 and listen again for next packet
            freq_cnt = 0;
            radio.setFrequency(freq_list[freq_cnt]);
            radio.startReceive();
        
    }
}
