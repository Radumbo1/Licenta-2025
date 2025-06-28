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

// uncomment to make this the car node
//#define CAR_NODE

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

void convertToInfluxDB(void){
    // https://didatec.sharepoint.com/:x:/r/sites/ARTTUCluj-NapocaFormulaStudentTeam/_layouts/15/Doc.aspx?sourcedoc=%7BB6C6841E-A2EB-463A-ADD1-69032107CED3%7D&file=organizare%20date%20can%20ecu%20code%20variables.xlsx&wdOrigin=TEAMS-MAGLEV.teamsSdk_ns.rwc&action=default&mobileredirect=true
    dataToSend.cooling = receivedMsg[0];
    dataToSend.brake_sens = receivedMsg[1];
    dataToSend.steer_sens = receivedMsg[2];
    dataToSend.acc1 = receivedMsg[3];
    dataToSend.acc2 = receivedMsg[4];
    dataToSend.wheel_spin_1 = receivedMsg[5];
    dataToSend.wheel_spin_2 = receivedMsg[6];
    dataToSend.motor_temp_1 = receivedMsg[7];
    dataToSend.motor_temp_2 = receivedMsg[8];
    dataToSend.serial_throttle_1 = receivedMsg[9];
    dataToSend.serial_throttle_2 = receivedMsg[10];
    dataToSend.inputV1_p_intr = receivedMsg[11];
    dataToSend.inputV1_p_frac = receivedMsg[12];
    dataToSend.inputV2_p_intr = receivedMsg[13];
    dataToSend.inputV2_p_frac = receivedMsg[14];
    dataToSend.phasecurrent1_p_intr = receivedMsg[15];
    dataToSend.phasecurrent1_p_frac = receivedMsg[16];
    dataToSend.phasecurrent2_p_intr = receivedMsg[17];
    dataToSend.phasecurrent2_p_frac = receivedMsg[18];
    dataToSend.susp_travel_FL = receivedMsg[19];
    dataToSend.susp_travel_FR = receivedMsg[20];
    dataToSend.susp_travel_BL = receivedMsg[21];
    dataToSend.susp_travel_BR = receivedMsg[22];
    dataToSend.ECU_control = receivedMsg[23];
    dataToSend.VRef_precharge = receivedMsg[24];
    dataToSend.meas_precharge = receivedMsg[25];
    dataToSend.current_sensor = receivedMsg[26];
    dataToSend.LV_state_of_charge = receivedMsg[27];
}

void sendToInfluxDB()
{
    // telemetry received array to struct
    convertToInfluxDB();

    // formating the data in line protocol for InfluxDB
    Serial.println("START INFLUX");
    Serial.print("telemetry_data,location=car,device=sensor ");

    // adding the fields with the coresponding values
    Serial.print("cooling=");
    Serial.print(dataToSend.cooling);
    Serial.print(",");
    Serial.print("brake_sens=");
    Serial.print(dataToSend.brake_sens);
    Serial.print(",");
    Serial.print("steer_sens=");
    Serial.print(dataToSend.steer_sens);
    Serial.print(",");

    Serial.print("acc1=");
    Serial.print(dataToSend.acc1);
    Serial.print(",");
    Serial.print("acc2=");
    Serial.print(dataToSend.acc2);
    Serial.print(",");

    Serial.print("wheel_spin_1=");
    Serial.print(dataToSend.wheel_spin_1);
    Serial.print(",");
    Serial.print("wheel_spin_2=");
    Serial.print(dataToSend.wheel_spin_2);
    Serial.print(",");

    Serial.print("motor_temp_1=");
    Serial.print(dataToSend.motor_temp_1);
    Serial.print(",");
    Serial.print("motor_temp_2=");
    Serial.print(dataToSend.motor_temp_2);
    Serial.print(",");

    Serial.print("control_temp_1=");
    Serial.print(dataToSend.control_temp_1);
    Serial.print(",");
    Serial.print("control_temp_2=");
    Serial.print(dataToSend.control_temp_2);
    Serial.print(",");

    Serial.print("serial_throttle_1=");
    Serial.print(dataToSend.serial_throttle_1);
    Serial.print(",");
    Serial.print("serial_throttle_2=");
    Serial.print(dataToSend.serial_throttle_2);
    Serial.print(",");

    Serial.print("inputV1_p_intr=");
    Serial.print(dataToSend.inputV1_p_intr);
    Serial.print(",");
    Serial.print("inputV1_p_frac=");
    Serial.print(dataToSend.inputV1_p_frac);
    Serial.print(",");
    Serial.print("inputV2_p_intr=");
    Serial.print(dataToSend.inputV2_p_intr);
    Serial.print(",");
    Serial.print("inputV2_p_frac=");
    Serial.print(dataToSend.inputV2_p_frac);
    Serial.print(",");

    Serial.print("phasecurrent1_p_intr=");
    Serial.print(dataToSend.phasecurrent1_p_intr);
    Serial.print(",");
    Serial.print("phasecurrent1_p_frac=");
    Serial.print(dataToSend.phasecurrent1_p_frac);
    Serial.print(",");
    Serial.print("phasecurrent2_p_intr=");
    Serial.print(dataToSend.phasecurrent2_p_intr);
    Serial.print(",");
    Serial.print("phasecurrent2_p_frac=");
    Serial.print(dataToSend.phasecurrent2_p_frac);
    Serial.print(",");

    Serial.print("susp_travel_FL=");
    Serial.print(dataToSend.susp_travel_FL);
    Serial.print(",");
    Serial.print("susp_travel_FR=");
    Serial.print(dataToSend.susp_travel_FR);
    Serial.print(",");
    Serial.print("susp_travel_BL=");
    Serial.print(dataToSend.susp_travel_BL);
    Serial.print(",");
    Serial.print("susp_travel_BR=");
    Serial.print(dataToSend.susp_travel_BR);
    Serial.print(",");

    Serial.print("ECU_control=");
    Serial.print(dataToSend.ECU_control);
    Serial.print(",");
    Serial.print("VRef_precharge=");
    Serial.print(dataToSend.VRef_precharge);
    Serial.print(",");
    Serial.print("meas_precharge=");
    Serial.print(dataToSend.meas_precharge);
    Serial.print(",");
    Serial.print("current_sensor=");
    Serial.print(dataToSend.current_sensor);
    Serial.print(",");
    Serial.print("LV_state_of_charge=");
    Serial.print(dataToSend.LV_state_of_charge);

    // optional
    Serial.println(" "); // space between values and timestamp
                       // Serial.println(millis());
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

    //TEST ONLY
    randomSeed(millis());


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

            // Print encrypted data before decryption------------------------------------------------DELETE
        Serial.print("Encrypted: ");
        for (size_t i = 0; i < received_len; i++) {
            Serial.print(receivedMsg[i]);
            Serial.print(" ");
        }
        Serial.println();
        //--------------------------------------------------------------------------------------------- DELETE

        // Prepare AES context for CTR mode
        struct AES_ctx ctx;
        uint8_t ctr_iv[16];
        memcpy(ctr_iv, aes_iv, 16); // Must match the IV/nonce used by transmitter

        AES_init_ctx_iv(&ctx, aes_key, ctr_iv);

        // Decrypt in-place (CTR mode)
        AES_CTR_xcrypt_buffer(&ctx, receivedMsg, received_len);

        // Now receivedMsg contains the original struct
        TelemetryData* receivedData = (TelemetryData*)receivedMsg;

        //--------------------------------------------------------------------------DELETE
        Serial.print("Decrypted: ");
        for (size_t i = 0; i < received_len; i++) {
            Serial.print(receivedMsg[i]);
            Serial.print(" ");
        }
        Serial.println();
        //----------------------------------------------------------------------------DELETE

        // Use receivedData as needed, or copy to your struct
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
                not sure if its gonna work!!
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
                    // opDone = false when entering, interrupt will make it true (hopefully)
                    unsigned long timeStart = millis();
                    Serial.print("waiting for next chunk");
                    while (!operationDone && (millis() - timeStart) < 300)
                    {
                        // do nothing lol
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
