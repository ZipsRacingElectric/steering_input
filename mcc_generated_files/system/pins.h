/**
 * Generated Pins header File
 * 
 * @file pins.h
 * 
 * @defgroup  pinsdriver Pins Driver
 * 
 * @brief This is generated driver header for pins. 
 *        This header file provides APIs for all pins selected in the GUI.
 *
 * @version Driver Version  3.1.0
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

#ifndef PINS_H
#define PINS_H

#include <xc.h>

#define INPUT   1
#define OUTPUT  0

#define HIGH    1
#define LOW     0

#define ANALOG      1
#define DIGITAL     0

#define PULL_UP_ENABLED      1
#define PULL_UP_DISABLED     0

// get/set RA0 aliases
#define BTN_LT_TRIS                 TRISAbits.TRISA0
#define BTN_LT_LAT                  LATAbits.LATA0
#define BTN_LT_PORT                 PORTAbits.RA0
#define BTN_LT_WPU                  WPUAbits.WPUA0
#define BTN_LT_OD                   ODCONAbits.ODCA0
#define BTN_LT_ANS                  ANSELAbits.ANSELA0
#define BTN_LT_SetHigh()            do { LATAbits.LATA0 = 1; } while(0)
#define BTN_LT_SetLow()             do { LATAbits.LATA0 = 0; } while(0)
#define BTN_LT_Toggle()             do { LATAbits.LATA0 = ~LATAbits.LATA0; } while(0)
#define BTN_LT_GetValue()           PORTAbits.RA0
#define BTN_LT_SetDigitalInput()    do { TRISAbits.TRISA0 = 1; } while(0)
#define BTN_LT_SetDigitalOutput()   do { TRISAbits.TRISA0 = 0; } while(0)
#define BTN_LT_SetPullup()          do { WPUAbits.WPUA0 = 1; } while(0)
#define BTN_LT_ResetPullup()        do { WPUAbits.WPUA0 = 0; } while(0)
#define BTN_LT_SetPushPull()        do { ODCONAbits.ODCA0 = 0; } while(0)
#define BTN_LT_SetOpenDrain()       do { ODCONAbits.ODCA0 = 1; } while(0)
#define BTN_LT_SetAnalogMode()      do { ANSELAbits.ANSELA0 = 1; } while(0)
#define BTN_LT_SetDigitalMode()     do { ANSELAbits.ANSELA0 = 0; } while(0)

// get/set RA1 aliases
#define BTN_RT_TRIS                 TRISAbits.TRISA1
#define BTN_RT_LAT                  LATAbits.LATA1
#define BTN_RT_PORT                 PORTAbits.RA1
#define BTN_RT_WPU                  WPUAbits.WPUA1
#define BTN_RT_OD                   ODCONAbits.ODCA1
#define BTN_RT_ANS                  ANSELAbits.ANSELA1
#define BTN_RT_SetHigh()            do { LATAbits.LATA1 = 1; } while(0)
#define BTN_RT_SetLow()             do { LATAbits.LATA1 = 0; } while(0)
#define BTN_RT_Toggle()             do { LATAbits.LATA1 = ~LATAbits.LATA1; } while(0)
#define BTN_RT_GetValue()           PORTAbits.RA1
#define BTN_RT_SetDigitalInput()    do { TRISAbits.TRISA1 = 1; } while(0)
#define BTN_RT_SetDigitalOutput()   do { TRISAbits.TRISA1 = 0; } while(0)
#define BTN_RT_SetPullup()          do { WPUAbits.WPUA1 = 1; } while(0)
#define BTN_RT_ResetPullup()        do { WPUAbits.WPUA1 = 0; } while(0)
#define BTN_RT_SetPushPull()        do { ODCONAbits.ODCA1 = 0; } while(0)
#define BTN_RT_SetOpenDrain()       do { ODCONAbits.ODCA1 = 1; } while(0)
#define BTN_RT_SetAnalogMode()      do { ANSELAbits.ANSELA1 = 1; } while(0)
#define BTN_RT_SetDigitalMode()     do { ANSELAbits.ANSELA1 = 0; } while(0)

// get/set RA5 aliases
#define LED_HB_TRIS                 TRISAbits.TRISA5
#define LED_HB_LAT                  LATAbits.LATA5
#define LED_HB_PORT                 PORTAbits.RA5
#define LED_HB_WPU                  WPUAbits.WPUA5
#define LED_HB_OD                   ODCONAbits.ODCA5
#define LED_HB_ANS                  ANSELAbits.ANSELA5
#define LED_HB_SetHigh()            do { LATAbits.LATA5 = 1; } while(0)
#define LED_HB_SetLow()             do { LATAbits.LATA5 = 0; } while(0)
#define LED_HB_Toggle()             do { LATAbits.LATA5 = ~LATAbits.LATA5; } while(0)
#define LED_HB_GetValue()           PORTAbits.RA5
#define LED_HB_SetDigitalInput()    do { TRISAbits.TRISA5 = 1; } while(0)
#define LED_HB_SetDigitalOutput()   do { TRISAbits.TRISA5 = 0; } while(0)
#define LED_HB_SetPullup()          do { WPUAbits.WPUA5 = 1; } while(0)
#define LED_HB_ResetPullup()        do { WPUAbits.WPUA5 = 0; } while(0)
#define LED_HB_SetPushPull()        do { ODCONAbits.ODCA5 = 0; } while(0)
#define LED_HB_SetOpenDrain()       do { ODCONAbits.ODCA5 = 1; } while(0)
#define LED_HB_SetAnalogMode()      do { ANSELAbits.ANSELA5 = 1; } while(0)
#define LED_HB_SetDigitalMode()     do { ANSELAbits.ANSELA5 = 0; } while(0)

// get/set RB0 aliases
#define IO_RB0_TRIS                 TRISBbits.TRISB0
#define IO_RB0_LAT                  LATBbits.LATB0
#define IO_RB0_PORT                 PORTBbits.RB0
#define IO_RB0_WPU                  WPUBbits.WPUB0
#define IO_RB0_OD                   ODCONBbits.ODCB0
#define IO_RB0_ANS                  ANSELBbits.ANSELB0
#define IO_RB0_SetHigh()            do { LATBbits.LATB0 = 1; } while(0)
#define IO_RB0_SetLow()             do { LATBbits.LATB0 = 0; } while(0)
#define IO_RB0_Toggle()             do { LATBbits.LATB0 = ~LATBbits.LATB0; } while(0)
#define IO_RB0_GetValue()           PORTBbits.RB0
#define IO_RB0_SetDigitalInput()    do { TRISBbits.TRISB0 = 1; } while(0)
#define IO_RB0_SetDigitalOutput()   do { TRISBbits.TRISB0 = 0; } while(0)
#define IO_RB0_SetPullup()          do { WPUBbits.WPUB0 = 1; } while(0)
#define IO_RB0_ResetPullup()        do { WPUBbits.WPUB0 = 0; } while(0)
#define IO_RB0_SetPushPull()        do { ODCONBbits.ODCB0 = 0; } while(0)
#define IO_RB0_SetOpenDrain()       do { ODCONBbits.ODCB0 = 1; } while(0)
#define IO_RB0_SetAnalogMode()      do { ANSELBbits.ANSELB0 = 1; } while(0)
#define IO_RB0_SetDigitalMode()     do { ANSELBbits.ANSELB0 = 0; } while(0)

// get/set RB1 aliases
#define IO_RB1_TRIS                 TRISBbits.TRISB1
#define IO_RB1_LAT                  LATBbits.LATB1
#define IO_RB1_PORT                 PORTBbits.RB1
#define IO_RB1_WPU                  WPUBbits.WPUB1
#define IO_RB1_OD                   ODCONBbits.ODCB1
#define IO_RB1_ANS                  ANSELBbits.ANSELB1
#define IO_RB1_SetHigh()            do { LATBbits.LATB1 = 1; } while(0)
#define IO_RB1_SetLow()             do { LATBbits.LATB1 = 0; } while(0)
#define IO_RB1_Toggle()             do { LATBbits.LATB1 = ~LATBbits.LATB1; } while(0)
#define IO_RB1_GetValue()           PORTBbits.RB1
#define IO_RB1_SetDigitalInput()    do { TRISBbits.TRISB1 = 1; } while(0)
#define IO_RB1_SetDigitalOutput()   do { TRISBbits.TRISB1 = 0; } while(0)
#define IO_RB1_SetPullup()          do { WPUBbits.WPUB1 = 1; } while(0)
#define IO_RB1_ResetPullup()        do { WPUBbits.WPUB1 = 0; } while(0)
#define IO_RB1_SetPushPull()        do { ODCONBbits.ODCB1 = 0; } while(0)
#define IO_RB1_SetOpenDrain()       do { ODCONBbits.ODCB1 = 1; } while(0)
#define IO_RB1_SetAnalogMode()      do { ANSELBbits.ANSELB1 = 1; } while(0)
#define IO_RB1_SetDigitalMode()     do { ANSELBbits.ANSELB1 = 0; } while(0)

/**
 * @ingroup  pinsdriver
 * @brief GPIO and peripheral I/O initialization
 * @param none
 * @return none
 */
void PIN_MANAGER_Initialize (void);

/**
 * @ingroup  pinsdriver
 * @brief Interrupt on Change Handling routine
 * @param none
 * @return none
 */
void PIN_MANAGER_IOC(void);


#endif // PINS_H
/**
 End of File
*/