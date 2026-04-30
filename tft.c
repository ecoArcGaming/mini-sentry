/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include "tft.h"

//==============================================================
// write8_a0()
// writes an 8-bit value to the TFT with the D/C line low
//
// Arguments:
//      data - 8-bit value
//==============================================================
void write8_a0(uint8 data)
{   while(SPIM_1_GetTxBufferSize() > 0); 
    CyDelayUs(5);
	DC_Write(0x00); 						        // set DC line low
    SPIM_1_WriteTxData(data);         		        // send data to transmit buffer
    while (!(SPIM_1_ReadTxStatus() & 0x01)){};	    // wait for data to be sent
}

//==============================================================
// write8_a1()
// writes an 8-bit value to the TFT with the D/C line high
//
// Arguments:
//      data - 8-bit value
//==============================================================
void write8_a1(uint8 data)
{   while(SPIM_1_GetTxBufferSize() > 0); 
    CyDelayUs(5);
	DC_Write(0x01); 						        // set DC line high
    SPIM_1_WriteTxData(data);                       // send data to transmit buffer
    while (!(SPIM_1_ReadTxStatus() & 0x01)){};	    // wait for data to be sent
}

//==============================================================
// writeM8_a1()
// writes multiple bytes to the TFT with the D/C line high
//
// Arguments:
//      pData - pointer to an array of 8-bit data values
//      N - the size of the array *pData
//==============================================================
void writeM8_a1(uint8 *pData, int N)
{   while(SPIM_1_GetTxBufferSize() > 0); 
    CyDelayUs(5);
	DC_Write(0x01); 						        // set DC line high
    int i;
    for (i=0; i<N; i++)
    {
        SPIM_1_WriteTxData(pData[i]);               // send data to transmit buffer
        while (!(SPIM_1_ReadTxStatus() & 0x01)){};  // wait for data to be sent
    }
}

//==============================================================
// read8_a1()
// reads an 8-bit value to the TFT with the D/C line high
//==============================================================
uint8 read8_a1(void)
{
		return 0x00;                                     // read function not implemented
                                   // read function not implemented
};

//==============================================================
// readM8_a1()
// reads multple 8-bit values from the TFT with the D/C line high
//
// Arguments:
//      pData - an array where the read values we be stored
//      N - the number of values that will be read (also size of 
//          the array pData)
//==============================================================
void  readM8_a1(uint8 *pData, int N)
{
	 int i;
    for (i = 0; i < N; i++)
    {
        pData[i] = 0x00; 
    }                                    // read function not implemented
}

//==============================================================
// tftStart()
// this function must be called to initializes the TFT
//==============================================================
void tftStart(void)
{
    RESET_Write(1);
    CyDelay(5);
    RESET_Write(0);
    CyDelay(20);
    RESET_Write(1);
    CyDelay(150);

    // Software Reset
    write8_a0(0x01);
    CyDelay(5);

    // Power Control 1
    write8_a0(0xC0);
    write8_a1(0x21);

    // Power Control 2
    write8_a0(0xC1);
    write8_a1(0x10);

    // VCOM Control 1
    write8_a0(0xC5);
    write8_a1(0x31);
    write8_a1(0x3C);

    // VCOM Control 2
    write8_a0(0xC7);
    write8_a1(0xC0);

    // Display Function Control
    write8_a0(0xB6);
    write8_a1(0x0A);
    write8_a1(0x82);
    write8_a1(0x27);
    write8_a1(0x00);

    // Memory Access Control
    write8_a0(0x36);
    write8_a1(0x20);

    // Pixel Format
    write8_a0(0x3A);
    write8_a1(0x55);

    // Sleep Out
    write8_a0(0x11);
    CyDelay(120);

    // Normal Display Mode
    write8_a0(0x13);

    // Display On
    write8_a0(0x29);
    CyDelay(250);
}


/* [] END OF FILE */

