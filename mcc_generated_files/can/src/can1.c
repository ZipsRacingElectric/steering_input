/**
 * CAN1 Generated Driver Source File
 * 
 * @file can1.c
 *            
 * @ingroup can_driver
 *            
 * @brief Contains the implementation file for the CAN1 driver.
 *            
 * @version CAN Driver Version 1.0.0
*/

/*
© [2024] Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms, you may use Microchip 
    software and any derivatives exclusively with Microchip products. 
    You are responsible for complying with 3rd party license terms  
    applicable to your use of 3rd party software (including open source  
    software) that may accompany Microchip software. SOFTWARE IS ?AS IS.? 
    NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS 
    SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT,  
    MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT 
    WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY 
    KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF 
    MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE 
    FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP?S 
    TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL NOT 
    EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR 
    THIS SOFTWARE.
*/

#include <string.h>
#include "../can1.h"

/**  
 * @name CAN1 Message object arbitration and control field information
 * Macros for the CAN1 Message object arbitration and control field.
 */
///@{
#define CAN_MSG_OBJ_SID_LOW_WIDTH          (0x8U) 
#define CAN_MSG_OBJ_SID_LOW_MASK           (0xFFU) 
#define CAN_MSG_OBJ_SID_HIGH_WIDTH         (0x3U) 
#define CAN_MSG_OBJ_SID_HIGH_MASK          (0x7U)  
#define CAN_MSG_OBJ_EID_LOW_WIDTH          (0x5U) 
#define CAN_MSG_OBJ_EID_LOW_MASK           (0xF8U)  
#define CAN_MSG_OBJ_EID_LOW_POSN           (0x3U)
#define CAN_MSG_OBJ_EID_MID_WIDTH          (0x8U) 
#define CAN_MSG_OBJ_EID_MID_MASK           (0xFFU)  
#define CAN_MSG_OBJ_EID_HIGH_WIDTH         (0x5U) 
#define CAN_MSG_OBJ_EID_HIGH_MASK          (0x1FU) 
#define CAN_MSG_OBJ_IDE_POSN               (0x4U)   
#define CAN_MSG_OBJ_RTR_POSN               (0x5U)     
#define CAN_MSG_OBJ_BRS_POSN               (0x6U) 
#define CAN_MSG_OBJ_FDF_POSN               (0x7U)  
#define CAN_MSG_OBJ_DLC_FIELD_MASK         (0xFU)  
#define CAN_MSG_OBJ_ID_TYPE_FIELD_MASK     (0x10U)
#define CAN_MSG_OBJ_FRAME_TYPE_FIELD_MASK  (0x20U)
#define CAN_MSG_OBJ_BRS_FIELD_MASK         (0x40U)
#define CAN_MSG_OBJ_FDF_FIELD_MASK         (0x80U)
#define CAN_STD_MSG_ID_SIZE                (CAN_MSG_OBJ_SID_LOW_WIDTH + CAN_MSG_OBJ_SID_HIGH_WIDTH)
///@}

/**
 @ingroup  can_driver
 @struct   CAN1_FIFOREG
 @brief    Defines a structure to access CAN1 FIFO Registers.
*/
struct CAN1_FIFOREG
{
    uint8_t CONL;
    uint8_t CONH;
    uint8_t CONU;
    uint8_t CONT;
    uint8_t STAL;
    uint8_t STAH;
    uint8_t STAU;
    uint8_t STAT;
    uint32_t UA;
};

// CAN1 Driver Interface 
const struct CAN_INTERFACE CAN1 = {
    .Initialize = CAN1_Initialize,
    .Deinitialize = CAN1_Deinitialize,
    .OperationModeSet = CAN1_OperationModeSet,
    .OperationModeGet = CAN1_OperationModeGet,
    .Transmit = CAN1_Transmit,
    .TransmitFIFOStatusGet = CAN1_TransmitFIFOStatusGet,
    .IsTxErrorPassive = CAN1_IsTxErrorPassive,
    .IsTxErrorWarning = CAN1_IsTxErrorWarning,
    .IsTxErrorActive = CAN1_IsTxErrorActive,
    .Receive = NULL,
    .ReceiveMessageGet = NULL,
    .ReceivedMessageCountGet = NULL,
    .ReceiveFIFOStatusGet = NULL,
    .IsRxErrorPassive = NULL,
    .IsRxErrorWarning = NULL,
    .IsRxErrorActive = NULL,
    .IsBusOff = CAN1_IsBusOff,
    .SleepMode = CAN1_Sleep,
    .InvalidMessageCallbackRegister = CAN1_InvalidMessageCallbackRegister,
    .BusWakeUpActivityCallbackRegister = CAN1_BusWakeUpActivityCallbackRegister,
    .BusErrorCallbackRegister = CAN1_BusErrorCallbackRegister,
    .ModeChangeCallbackRegister = CAN1_ModeChangeCallbackRegister,
    .SystemErrorCallbackRegister = CAN1_SystemErrorCallbackRegister,
    .TxAttemptCallbackRegister = CAN1_TxAttemptCallbackRegister,
    .RxBufferOverFlowCallbackRegister = NULL,
    .Tasks = CAN1_Tasks
};

// CAN1 Default Callback Handler
static void (*CAN1_InvalidMessageHandler)(void) = NULL;
static void (*CAN1_BusWakeUpActivityHandler)(void) = NULL;
static void (*CAN1_BusErrorHandler)(void) = NULL;
static void (*CAN1_ModeChangeHandler)(void) = NULL;
static void (*CAN1_SystemErrorHandler)(void) = NULL;
static void (*CAN1_TxAttemptHandler)(void) = NULL;

// Defining a structure to access FIFO registers.
static volatile struct CAN1_FIFOREG * const FIFO = (struct CAN1_FIFOREG *)&C1TXQCONL;

/**
 * @ingroup can_driver
 * @brief Get the DLC enum based data byte value.
 * @param [in] dlc - DLC value as described in CAN_DLC.
 * @return The data byte value coresponding to DLC.
*/
static uint8_t CAN1_DlcToDataBytesGet(const enum CAN_DLC dlc)
{
    static const uint8_t dlcByteSize[] = {0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U};
    return dlcByteSize[dlc];
}

/**
 * @ingroup can_driver
 * @brief Get the Payload size in bytes for PLSIZE register value.
 * @param [in] plsize - The value in PLSIZE register.
 * @return The number of data bytes corresponding to PLSIZE register value.
*/
static uint8_t CAN1_PLSIZEToPayloadBytesGet(uint8_t plsize)
{
    return CAN1_DlcToDataBytesGet(8U + plsize);
}

/**
 * @ingroup  can_driver
 * @brief    Check if a valid channel is configured as transmitter or not.
 * @param [in] channel - Transmit FIFO channel as described in CAN_TX_FIFO_CHANNELS.
 * @retval True - FIFO channel is configured as transmitter.
 * @retval False - FIFO channel is not configured as transmitter.
*/
static bool CAN1_IsTxChannel(uint8_t channel) 
{
    return ((channel < 4U) && (FIFO[channel].CONL & _C1FIFOCON1L_TXEN_MASK));
}

/**
 * @ingroup  can_driver
 * @brief    Returns the CAN1 transmit FIFO status.
 * @param [in] channel - Transmit FIFO channel as described in CAN_TX_FIFO_CHANNELS.
 * @return Status of the CAN1 Transmit FIFO as described in CAN_TX_FIFO_STATUS.
*/
static enum CAN_TX_FIFO_STATUS CAN1_GetTxFifoStatus(uint8_t validChannel)
{
    return (FIFO[validChannel].STAL & _C1FIFOSTA1L_TFNRFNIF_MASK);
}

/**
 * @ingroup  can_driver
 * @brief   Reads the message object from user input and updates the CAN1 TX FIFO.
 * @param [out] txFifoObj - Transmit FIFO message object.
 * @param [in] txCanMsg - Pointer to the message object of type CAN_MSG_OBJ.
 * @return None.
*/
static void CAN1_MessageWriteToFifo(uint8_t *txFifoObj, struct CAN_MSG_OBJ *txCanMsg)
{
    uint32_t msgId = txCanMsg->msgId;
 
    if ((uint8_t)CAN_FRAME_EXT == txCanMsg->field.idType)
    {
        /* message is extended identifier */
        
        txFifoObj[1] = (msgId << CAN_MSG_OBJ_EID_LOW_POSN) & CAN_MSG_OBJ_EID_LOW_MASK;
        msgId >>= CAN_MSG_OBJ_EID_LOW_WIDTH;
        
        txFifoObj[2] = msgId & CAN_MSG_OBJ_EID_MID_MASK;
        msgId >>= CAN_MSG_OBJ_EID_MID_WIDTH;
        
        txFifoObj[3] = msgId & CAN_MSG_OBJ_EID_HIGH_MASK;
        msgId >>= CAN_MSG_OBJ_EID_HIGH_WIDTH;
    }
    else
    {
        /* message is standard identifier */
        
        txFifoObj[1] = 0;
        txFifoObj[2] = 0;
        txFifoObj[3] = 0;
    }
    
    txFifoObj[0] =  msgId & CAN_MSG_OBJ_SID_LOW_MASK;
    msgId >>= CAN_MSG_OBJ_SID_LOW_WIDTH;
    
    txFifoObj[1] |= (msgId & CAN_MSG_OBJ_SID_HIGH_MASK);
      
    // DLC <3:0>, IDE <1>, RTR <1>, BRS <1>, FDF <1> 
    txFifoObj[4] = (txCanMsg->field.dlc & CAN_MSG_OBJ_DLC_FIELD_MASK) | 
                   ((uint8_t)(txCanMsg->field.idType << CAN_MSG_OBJ_IDE_POSN) & CAN_MSG_OBJ_ID_TYPE_FIELD_MASK) | 
                   ((uint8_t)(txCanMsg->field.frameType << CAN_MSG_OBJ_RTR_POSN) & CAN_MSG_OBJ_FRAME_TYPE_FIELD_MASK) | 
                   ((uint8_t)(txCanMsg->field.brs << CAN_MSG_OBJ_BRS_POSN) & CAN_MSG_OBJ_BRS_FIELD_MASK) |
                   ((uint8_t)(txCanMsg->field.formatType << CAN_MSG_OBJ_FDF_POSN) & CAN_MSG_OBJ_FDF_FIELD_MASK);

    if ((uint8_t)CAN_FRAME_DATA == txCanMsg->field.frameType)
    {
        /* Data frame message */
		
        // Copying TX message object to FIFO
        (void)memcpy(&txFifoObj[8], txCanMsg->data, CAN1_DlcToDataBytesGet(txCanMsg->field.dlc));
    }
    else
    {
        /* RTR frame message */

        // RTR frame is not supported in CAN FD. Error return type is not supported by driver yet.
    }
}

/**
 * @ingroup can_driver
 * @brief Validates transmission.
 * @param [in] channel - Transmit FIFO channel as described in CAN_TX_FIFO_CHANNELS.
 * @param [in] txCanMsg - Pointer to the message object of type CAN_MSG_OBJ.
 * @return Status of the CAN1 Transmit operation as described in CAN_TX_MSG_REQUEST_STATUS.
*/
static enum CAN_TX_MSG_REQUEST_STATUS CAN1_ValidateTransmission(uint8_t channel, struct CAN_MSG_OBJ *txCanMsg)
{
    enum CAN_TX_MSG_REQUEST_STATUS txMsgStatus = CAN_TX_MSG_REQUEST_SUCCESS;
    
    // If TX message object has BRS set
    if ((uint8_t)CAN_BRS_MODE == txCanMsg->field.brs)
    {
        txMsgStatus |= CAN_TX_MSG_REQUEST_BRS_ERROR;
    }
    
    // If TX Message object has more than 8 bytes of DLC Size
    if (txCanMsg->field.dlc > (uint8_t)DLC_8)
    {
        txMsgStatus |= CAN_TX_MSG_REQUEST_DLC_EXCEED_ERROR;
    }
       
    // If CAN TX FIFO is full
    if (CAN_TX_FIFO_FULL ==  CAN1_TransmitFIFOStatusGet(channel))
    {
        txMsgStatus |= CAN_TX_MSG_REQUEST_FIFO_FULL;
    }
    
    return txMsgStatus;
}

/**
 * @ingroup can_driver
 * @brief Configures the CAN1 TX FIFO settings.
 * @param None.
 * @return None.
*/
static void CAN1_TX_FIFO_Configuration(void)
{
    // TXQNIE disabled; TXQEIE disabled; TXATIE disabled; 
    C1TXQCONL = 0x80;
    // UINC disabled; FRESET enabled; 
    C1TXQCONH = 0x4;
    // TXPRI 0; TXAT Unlimited number of retransmission attempts; 
    C1TXQCONU = 0x60;
    // FSIZE 1; 
    C1TXQCONT = 0x0;
}

/**
 * @ingroup can_driver
 * @brief Configures the CAN1 Bit rate settings.
 * @param None.
 * @return None.
*/
static void CAN1_BitRateConfiguration(void)
{
    // SJW 0; 
    C1NBTCFGL = 0x0;
    // TSEG2 0; 
    C1NBTCFGH = 0x0;
    // TSEG1 1; 
    C1NBTCFGU = 0x1;
    // BRP 0; 
    C1NBTCFGT = 0x0;
}


void CAN1_Initialize(void)
{
    /* Enable the CAN1 module */
    C1CONHbits.ON = 1;
	
    // ABAT disabled; TXBWS No delay; 
    C1CONT = 0x4;  
	
    // RTXAT disabled; ESIGM disabled; SERR2LOM disabled; STEF disabled; TXQEN enabled; 
    C1CONU = 0x90;  
    
    // Place CAN1 module in configuration mode
    if (CAN_OP_MODE_REQUEST_SUCCESS == CAN1_OperationModeSet(CAN_CONFIGURATION_MODE))
    {
        // Initialize the CAN1 Message Memory Base Address Register with the start address of the CAN1 FIFO message object area. 
        C1FIFOBAL = (uint8_t)0x2600U; 
        C1FIFOBAH = (uint8_t)(0x2600U >> 8); 
		
        // WAKFIL enabled; WFT T11 Filter; SIDL disabled; FRZ disabled; ON enabled; 
        C1CONH = 0x87;  
	
        // DNCNT 0x0; CLKSEL0 disabled; 
        C1CONL = 0x0;  

        // Disabled CAN1 Store in Transmit Event FIFO bit
        C1CONUbits.STEF = 0;
        // Enabled CAN1 Transmit Queue bit
        C1CONUbits.TXQEN = 1;
        
        /* Configure CAN1 Bit rate settings */
        CAN1_BitRateConfiguration();        
        
        /* Configure CAN1 FIFO settings */
        CAN1_TX_FIFO_Configuration();
        
        /* Place CAN1 module in Normal Operation mode */
        (void)CAN1_OperationModeSet(CAN_NORMAL_2_0_MODE);    
    }
}

void CAN1_Deinitialize(void)
{
    /* Place CAN1 module in configuration mode */
    if(CAN_OP_MODE_REQUEST_SUCCESS == CAN1_OperationModeSet(CAN_CONFIGURATION_MODE))
    {        
        C1CONL = 0x0;  
        C1CONH = 0x7;  
        C1CONU = 0x98;  
        C1CONT = 0x4;  
           
        /* Reset bit rate settings to POR*/
        C1NBTCFGL = 0xF;
        C1NBTCFGH = 0xF;
        C1NBTCFGU = 0x3E;
        C1NBTCFGT = 0x0;
        
        /* Configure CAN1 FIFO settings */
        /* Reset TX FIFO settings to POR*/
        C1TXQCONL = 0x80;
        C1TXQCONH = 0x4;
        C1TXQCONU = 0x60;
        C1TXQCONT = 0x0;

    }
    
    /* Disable the CAN1 module */
    C1CONHbits.ON = 0;
}

enum CAN_OP_MODE_STATUS CAN1_OperationModeSet(const enum CAN_OP_MODES requestMode)
{
    enum CAN_OP_MODE_STATUS status = CAN_OP_MODE_REQUEST_SUCCESS;

    if ((CAN1_OperationModeGet() == CAN_CONFIGURATION_MODE)
        || (requestMode == CAN_DISABLE_MODE)
        || (requestMode == CAN_CONFIGURATION_MODE))
    {
        C1CONTbits.REQOP = requestMode;
        
        while (C1CONUbits.OPMOD != requestMode)
        {
            //This condition is avoiding the system error case endless loop
            if (C1INTHbits.SERRIF == 1)
            {
                status = CAN_OP_MODE_SYS_ERROR_OCCURED;
                break;
            }
        }
    }
    else
    {
        status = CAN_OP_MODE_REQUEST_FAIL;
    }

    return status;
}

enum CAN_OP_MODES CAN1_OperationModeGet(void)
{
    return C1CONUbits.OPMOD;
}

enum CAN_TX_MSG_REQUEST_STATUS CAN1_Transmit(const enum CAN_TX_FIFO_CHANNELS fifoChannel, struct CAN_MSG_OBJ *txCanMsg)
{
    uint8_t *txFifoObj;
	
    enum CAN_TX_MSG_REQUEST_STATUS status = CAN_TX_MSG_REQUEST_FIFO_FULL;
    
    status = CAN1_ValidateTransmission(fifoChannel, txCanMsg);
	
    if (CAN_TX_MSG_REQUEST_SUCCESS == status)
    {
        txFifoObj = (uint8_t *) FIFO[fifoChannel].UA;
            
        if (txFifoObj != NULL)
        {
            CAN1_MessageWriteToFifo(txFifoObj, txCanMsg);
			
            // Message Send Request
            FIFO[fifoChannel].CONH |= (_C1FIFOCON1H_TXREQ_MASK | _C1FIFOCON1H_UINC_MASK);
        }
    }
  
    return status;
}

enum CAN_TX_FIFO_STATUS CAN1_TransmitFIFOStatusGet(const enum CAN_TX_FIFO_CHANNELS fifoChannel)
{
    enum CAN_TX_FIFO_STATUS status = CAN_TX_FIFO_FULL;
    
    if (CAN1_IsTxChannel(fifoChannel)) 
    {
        status = CAN1_GetTxFifoStatus(fifoChannel);
    }
    
    return status;
}

bool CAN1_IsTxErrorPassive(void)
{
    return C1TRECUbits.TXBP;
}

bool CAN1_IsTxErrorWarning(void)
{
    return C1TRECUbits.TXWARN;
}

bool CAN1_IsTxErrorActive(void)
{
    bool errorState = false;
	
    if(C1TRECHbits.TERRCNT < 128U) 
    {
        errorState = true;
    }
    
    return errorState;
}

bool CAN1_IsBusOff(void)
{
    return C1TRECUbits.TXBO;
}

void CAN1_Sleep(void)
{
    C1INTHbits.WAKIF = 0;
    C1INTTbits.WAKIE = 1;
	
    // CAN Information Interrupt Enable bit
    PIE0bits.CANIE = 1;  
    
    // put the module in disable mode
    (void)CAN1_OperationModeSet(CAN_DISABLE_MODE);
}

void CAN1_InvalidMessageCallbackRegister(void (*handler)(void))
{
    if(handler != NULL)
    {
        CAN1_InvalidMessageHandler = handler;
    }
}

void CAN1_BusWakeUpActivityCallbackRegister(void (*handler)(void))
{
    if(handler != NULL)
    {
        CAN1_BusWakeUpActivityHandler = handler;
    }
}

void CAN1_BusErrorCallbackRegister(void (*handler)(void))
{
    if(handler != NULL)
    {
        CAN1_BusErrorHandler = handler;
    }
}

void CAN1_ModeChangeCallbackRegister(void (*handler)(void))
{
    if(handler != NULL)
    {
        CAN1_ModeChangeHandler = handler;
    }
}

void CAN1_SystemErrorCallbackRegister(void (*handler)(void))
{
    if(handler != NULL)
    {
        CAN1_SystemErrorHandler = handler;
    }
}

void CAN1_TxAttemptCallbackRegister(void (*handler)(void))
{
    if(handler != NULL)
    {
        CAN1_TxAttemptHandler = handler;
    }
}


void CAN1_InformationISR(void)
{
    // Bus Wake-up Activity Interrupt 
    if(1 == C1INTHbits.WAKIF)
    {
        if(CAN1_BusWakeUpActivityHandler != NULL)
        {
            CAN1_BusWakeUpActivityHandler();
        }
        
        C1INTHbits.WAKIF = 0;
    }
    
    PIR0bits.CANIF = 0;
}

void CAN1_Tasks(void)
{
	
    if(1 == C1INTHbits.IVMIF)
    {
        if(CAN1_InvalidMessageHandler != NULL)
        {
            CAN1_InvalidMessageHandler();
        }
       
        C1INTHbits.IVMIF = 0;
    }  
	
    if(1 == C1INTHbits.CERRIF)
    {
        if(CAN1_BusErrorHandler != NULL)
        {
            CAN1_BusErrorHandler();
        }
       
        C1INTHbits.CERRIF = 0;
    }  
	
    if(1 == C1INTLbits.MODIF)
    {
        if(CAN1_ModeChangeHandler != NULL)
        {
            CAN1_ModeChangeHandler();
        }
       
        C1INTLbits.MODIF = 0;
    }  
	
    if(1 == C1INTHbits.SERRIF)
    {
        if(CAN1_SystemErrorHandler != NULL)
        {
            CAN1_SystemErrorHandler();
        }
       
        C1INTHbits.SERRIF = 0;
    }  
        
    if(1 == C1TXQSTALbits.TXATIF)
    {
	    if(CAN1_TxAttemptHandler != NULL)
        {
            CAN1_TxAttemptHandler();
        }
		
		C1TXQSTALbits.TXATIF = 0;
    }
}





