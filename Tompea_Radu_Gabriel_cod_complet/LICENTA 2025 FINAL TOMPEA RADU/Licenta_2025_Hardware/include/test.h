#ifndef INC_TEST_H
#define INC_TEST_H

#include <RadioLib.h>

SX1276 radio = new Module(10, 2, 9, 3);

float freq_list[] = {868.1, 868.3, 868.5, 868.7};
const int num_freqs = sizeof(freq_list) / sizeof(freq_list[0]);

void sendDataAck(const String& message) {
    int chunkSize = ceil((float)message.length() / num_freqs);
    int freqIndex = 0;
    bool ackReceived;

    for (int i = 0; i < message.length(); i += chunkSize) {
        String chunk = message.substring(i, i + chunkSize);
        Serial.print("Sending chunk: "); Serial.println(chunk);

        radio.setFrequency(freq_list[freqIndex]);
        int state = radio.transmit(chunk);
        if (state != RADIOLIB_ERR_NONE) {
            Serial.print("Transmission failed with error: ");
            Serial.println(state);
            continue;
        }

        freqIndex = (freqIndex + 1) % num_freqs;
        radio.setFrequency(freq_list[freqIndex]);
        radio.startReceive();
        Serial.println("Waiting for ACK...");

        unsigned long startTime = millis();
        ackReceived = false;
        while (millis() - startTime < 300) {
            String ack;
            if (radio.receive(ack) == RADIOLIB_ERR_NONE && ack == "ACK") {
                ackReceived = true;
                break;
            }
        }

        if (!ackReceived) {
            Serial.println("ACK timeout, resending chunk...");
            freqIndex = (freqIndex - 1 + num_freqs) % num_freqs; // Rollback frequency
            i -= chunkSize; // Retry the same chunk
        } else {
            Serial.println("ACK received, moving to next chunk.");
        }
    }
}

void receiveDataAck() {
    int freqIndex = 0;
    String receivedMessage = "";

    for (int i = 0; i < num_freqs; i++) {
        radio.setFrequency(freq_list[freqIndex]);
        radio.startReceive();
        Serial.print("Listening on frequency: "); Serial.println(freq_list[freqIndex]);

        unsigned long startTime = millis();
        bool received = false;
        String receivedChunk;

        while (millis() - startTime < 300) {
            if (radio.receive(receivedChunk) == RADIOLIB_ERR_NONE) {
                received = true;
                receivedMessage += receivedChunk;
                break;
            }
        }

        if (received) {
            Serial.println("Received chunk: " + receivedChunk);
            radio.setFrequency(freq_list[(freqIndex + 1) % num_freqs]);
            radio.transmit("ACK");
            Serial.println("ACK sent");
        } else {
            Serial.println("Timeout waiting for chunk.");
            i--; // Retry same frequency
            continue;
        }
        freqIndex = (freqIndex + 2) % num_freqs;
    }

    Serial.println("Full message received: " + receivedMessage);
}


#endif /* INC_TEST_H */
