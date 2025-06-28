#include <can.h>

CAN::CAN(){

}

void CAN::init(){
    // enable interrupt
    pinMode(INTCAN, INPUT);

    CAN0.begin(MCP_ANY, CAN_100KBPS, MCP_8MHZ);
    CAN0.setMode(MCP_NORMAL);

    receivedMsg.Buffer = (uint8_t*)calloc(MAX_MSGBUF, sizeof(uint8_t));
    transmitMsg.Buffer = (uint8_t*)calloc(MAX_MSGBUF, sizeof(uint8_t));

    receivedMsg.rx_id = TELEMETRY_ID;
    receivedMsg.tx_id = ECU_ID;
    transmitMsg.rx_id = TELEMETRY_ID;
    transmitMsg.tx_id = ECU_ID;
    transmitMsg.min_sep_time = 10;
    receivedMsg.min_sep_time = 10;
}