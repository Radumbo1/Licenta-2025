#ifndef INC_CAN_H
#define INC_CAN_H

#include <mcp_can.h>
#include <mcp_can_dfs.h>
#include <iso-tp.h>
#include <SPI.h>

// chip select pin
#define CS 7
// can interrupt pin
#define INTCAN 2

#define ECU_ID 0x1
#define TELEMETRY_ID 0x4

class CAN {
public:
    CAN();
    void init();

    MCP_CAN CAN0{CS};
    IsoTp isotp{&CAN0, INTCAN};

    Message_t receivedMsg, transmitMsg;
};

#endif /* INC_CAN_H */
