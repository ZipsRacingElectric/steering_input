 /*
 * MAIN Generated Driver File
 * 
 * @file main.c
 * 
 * @defgroup main MAIN
 * 
 * @brief This is the generated driver implementation file for the MAIN driver.
 *
 * @version MAIN Driver Version 1.0.0
*/
#include "mcc_generated_files/system/system.h"
#include "mcc_generated_files/can/can1.h"

int main(void)
{
    SYSTEM_Initialize();
    CAN1_Initialize();

    // If using interrupts in PIC18 High/Low Priority Mode you need to enable the Global High and Low Interrupts 
    // If using interrupts in PIC Mid-Range Compatibility Mode you need to enable the Global Interrupts 
    // Use the following macros to: 

    // Enable the Global Interrupts 
    //INTERRUPT_GlobalInterruptEnable(); 

    // Disable the Global Interrupts 
    //INTERRUPT_GlobalInterruptDisable(); 


    while(1) {
        // LED heartbeat on the steering board
        LED_HB_Toggle();

        if (BTN_LT_GetValue()) {
            // Testing of sending can message upon button press
            // Will change away from place holder once I test the button being pressed
            struct CAN_MSG_OBJ txCanMsg;
            // Fill the CAN_MSG_OBJ structure with your data
            txCanMsg.msgId = 0x123; // CAN message ID
            txCanMsg.field.dlc = 8; // Data Length Code: number of bytes in the message
            txCanMsg.data[0] = 0x01; // Data bytes of the CAN message
            txCanMsg.data[1] = 0x02;
            txCanMsg.data[2] = 0x03;
            txCanMsg.data[3] = 0x04;
            txCanMsg.data[4] = 0x05;
            txCanMsg.data[5] = 0x06;
            txCanMsg.data[6] = 0x07;
            txCanMsg.data[7] = 0x08;

            // Transmit the CAN message on FIFO channel 0
            enum CAN_TX_MSG_REQUEST_STATUS status = CAN1_Transmit(0, &txCanMsg);
            // Check the status of the transmission request
            if (status == CAN_TX_MSG_REQUEST_SUCCESS) {
                // The message was successfully queued for transmission
            } else {
                // The transmission request failed
            }
            
        }
    }

    return 0;    
}