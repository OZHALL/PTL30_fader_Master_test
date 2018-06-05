/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs 

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs  - 1.45
        Device            :  PIC16F18855
        Driver Version    :  2.00
    The generated drivers are tested against the following:
        Compiler          :  XC8 1.35
        MPLAB             :  MPLAB X 3.40
*/

/*
    (c) 2016 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
*/
/*
 2017-06-29 ozh this is working:  read fader 0 & change the level of all 8 fader LEDs
 2017-07-04 ozh this is working:  Master fader controls speed.  Count goes a full 16 bits.
 2017-07-04 ozh this is working:  I2C 100KHz.  uP clockspeed 16Mhz on Master & 4 Mhz on Slave
 2017-07-09 ozh I upped the clockspeed to 32Mhz.  Had to juggle some times in the delay routine
 *              with a 32x multiplier the counts were overrunning the "int" type variables
 *              as a result, the total blinkyloop was finishing before the slave, and they did not sync
 2017-07-09 ozh upped the I2C 400KHz.  It's working (with slave @ 16 MHz). 
 */
#include "mcc_generated_files/mcc.h"


/* BEGIN
 copied from PLT30_fader_led_test1 
 */
typedef uint16_t adc_result_t;

typedef enum
{
    FADER0 = 0x0,
    FADER1 = 0x1,
    FADER2 = 0x2,
    FADER3 = 0x3,
    FADER4 = 0x4,
    POT =  0x4,
    FADER5 = 0x5,
    FADER6 = 0x6,
    FADER7 = 0x7,      
} adcc_channel_t;

void delay(int);   
 // master clock frequency adjust.  1x = 1MHz   8x = 8MHz  16=16MHz
const int cMstrClkAdjust = 32;
void delay(int delaytime) {     
    long counter = 0;
    long adjustedDelaytime=delaytime*cMstrClkAdjust;
    if (0<delaytime)
        for (counter = 0; counter<adjustedDelaytime; counter++);   
    //__delay_ms(delaytime);     // this should be the proper way to do this
 } 

void blinkyLoop (int maxLoops){
    int loopCount=maxLoops;
    int delayTime;
    
    ODCONB &= 0xE0;  // not open drain
    ODCONC &= 0x1F;  // not open drain    
    while(loopCount>0){
        delayTime=loopCount*400;    

        // brightest
        PORTB |= 0x1F; // bottom 5 bits of PORTB
        PORTC |= 0xE0; // top 3 bits of PORTC all led's on activate all

        delay(delayTime);
        // deactivate all led's
        PORTB &= 0xE0; // deactivate all led's 
        PORTC &= 0x1F; // deactivate all led's 

        delay(delayTime);
        
        loopCount--;
    }
    PORTB |= 0x1F; // bottom 5 bits of PORTB
    PORTC |= 0xE0; // top 3 bits of PORTC all led's on activate all
    delay(5000); // extra delay on the Master side - this appears to be required!
    return;
}

/* END
 copied from PLT30_fader_led_test1 
 */
/*
                         Main application
 */
void MYI2C_Write2LEDBytes(uint8_t slaveDeviceAddress, uint8_t MSBWriteByte,uint8_t LSBWriteByte);
void main(void)
{
    uint8_t I2C_ADDRESS_FADELED0 =  0x10;  // assume only 1 device for now
    uint16_t iCounter=0; // counter
    uint16_t iMSByte;
    uint16_t iLSByte;
    int faderValue;
    uint8_t fader8bitValue;
    uint8_t prevFader8bitValue;
    
    // initialize the device
    SYSTEM_Initialize();

    // When using interrupts, you need to set the Global and Peripheral Interrupt Enable bits
    // Use the following macros to:

    // Enable the Global Interrupts
    INTERRUPT_GlobalInterruptEnable();

    // Enable the Peripheral Interrupts
    INTERRUPT_PeripheralInterruptEnable();

    // Disable the Global Interrupts
    //INTERRUPT_GlobalInterruptDisable();

    // Disable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptDisable();
    

     
    // get value from fader
    faderValue=ADCC_GetSingleConversion(FADER0);  // not POT
    prevFader8bitValue=faderValue>>2; // convert 10 bit to 8 bit   
    
    blinkyLoop(10);
    //Clear_WDT(); // clear watchdog timer, until i figure  out how to shut it off 
    
    while (1) {
            // get value from fader
            if((0==iCounter%4)) // only get it once every 4 loops
            {
                faderValue=ADCC_GetSingleConversion(FADER0);  // not POT
                fader8bitValue=faderValue>>2;
            }   

            
            // show change locally:  this will NOT be in the Teensy code
            if (faderValue> 640){
                // brightest
                PORTB |= 0x1F; // bottom 5 bits of PORTB
                PORTC |= 0xE0; // top 3 bits of PORTC all led's on activate all
                ODCONB &= 0xE0;  // not open drain
                ODCONC &= 0x1F;  // not open drain
            }else{
                if (faderValue > 320) {
                    // Pullups are NOT active on OUTPUT pins!!!
                    // activate pullup
                    //WPUC = 0xFF;
                    PORTB |= 0x1F; // bottom 5 bits of PORTB
                    PORTC |= 0xE0; // top 3 bits of PORTC all led's on activate all
                    ODCONB |= 0x1F; //  open drain combines with external pullup
                    ODCONC |= 0xE0; //  open drain combines with external pullup
                }else{
                    // deactivate pullup
                    //WPUC = 0x00;
                    PORTB &= 0xE0; // deactivate all led's 
                    PORTC &= 0x1F; // deactivate all led's 
                    ODCONB &= 0xE0;  // not open drain
                    ODCONC &= 0x1F;  // not open drain
                }
            }
            // now send to slave
            //if (prevFader8bitValue != fader8bitValue)
            //{
            iMSByte = iCounter;
            iMSByte =  iMSByte>>8;
            iLSByte = iCounter&0xFF;
            MYI2C_Write2LEDBytes(I2C_ADDRESS_FADELED0,iMSByte,iLSByte);
            //}
            prevFader8bitValue=fader8bitValue;
            //delay(); 
            //Clear_WDT(); // clear watchdog timer, until i figure  out how to shut it off
            iCounter++;    // 16 bits auto rollover
            if (fader8bitValue<252)
                delay(1*(255-fader8bitValue)); // invert so that higher is faster
    } 
 }

void MYI2C_Write2LEDBytes(uint8_t slaveDeviceAddress,uint8_t MSBWriteByte,uint8_t LSBWriteByte)
{
   //<code>
        #define SLAVE_I2C_GENERIC_RETRY_MAX     100
        
        // initialize the module
        //I2C1_Initialize();  done in system_initialize())
           

        // write to Fader/LED Device
        
        uint16_t        dataAddress;
        uint8_t         sourceData[16] = {  0xA0, 0xA1, 0xA2, 0xA3, 
                                            0xA4, 0xA5, 0xA6, 0xA7, 
                                            0xA8, 0xA9, 0xAA, 0xAB, 
                                            0xAC, 0xAD, 0xAE, 0xAF }; 
        uint8_t         *pData;
        uint16_t        nCount;

        uint8_t         writeBuffer[3];
        uint8_t         *pD;
        uint16_t        counter, timeOut;
        uint8_t         pointerByte = 0b00100000;    // 7 bit address + 0 for Write
        
        I2C1_MESSAGE_STATUS status = I2C1_MESSAGE_PENDING;

        dataAddress = 0x00;             // starting Fader Submodule address 
        pD = sourceData;                // initialize the source of the data
        nCount = 1;                     // number of byte pairs to write

        // signal I2C traffic
        // TODO convert RC1 back to Analog In for the slave module
        PORTC |= 0x02; // RC1 bit of PORTC led activate
        
        for (counter = 0; counter < nCount; counter++)
        {
            // build the write buffer first
            // starting address of the EEPROM memory
            writeBuffer[0] = pointerByte;                        // pointer byte
            // data to be written
            writeBuffer[1] = (MSBWriteByte);            // high address
            writeBuffer[2] = (LSBWriteByte);            // low address

            // Now it is possible that the slave device will be slow.
            // As a work around on these slaves, the application can
            // retry sending the transaction
            timeOut = 0;
            while(status != I2C1_MESSAGE_FAIL)
            {
                // write one byte to EEPROM (3 is the number of bytes to write)
                I2C1_MasterWrite(  writeBuffer,
                                        3,
                                        slaveDeviceAddress,
                                        &status);

                // wait for the message to be sent or status has changed.
                while(status == I2C1_MESSAGE_PENDING);

                if (status == I2C1_MESSAGE_COMPLETE)
                    break;

                // if status is  I2C1_MESSAGE_ADDRESS_NO_ACK,
                //               or I2C1_DATA_NO_ACK,
                // The device may be busy and needs more time for the last
                // write so we can retry writing the data, this is why we
                // use a while loop here

                // check for max retry and skip this byte
                if (timeOut == SLAVE_I2C_GENERIC_RETRY_MAX)
                    break;
                else
                    timeOut++;
            }

            if (status == I2C1_MESSAGE_FAIL)
            {
                break;
            }
            dataAddress++;
        }
        PORTC &= 0xFD; // deactivate all led's 
        //delay(1000);

    //</code>    
}
        
/**
 End of File
*/