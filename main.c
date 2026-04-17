#include "project.h"
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"
#include <stdio.h>
/* 
 * Assuming standard 16-bit RGB565 color definitions
 */
#define RED   0xF800
#define BLUE  0x001F
#define MLX_I2C_ADDR 0x33

// Allocate memory for the sensor data ~20kb RAM
static uint16_t eeMLX90640[832];    // Requires 832 words for EEPROM data
static uint16_t mlx90640Frame[834]; // Requires 834 words for frame data
static float mlx90640Image[768];    // Requires 768 floats for the 32x24 image
paramsMLX90640 mlx90640;

void TFT_SendCommand(uint8_t cmd)
{
    DC_Write(0); // Set Data/Command pin to Command
    SPIM_1_ClearTxBuffer(); // Ensure FIFO is empty
    SPIM_1_ReadTxStatus();  // Dummy read to clear stale "DONE" flags
    
    SPIM_1_WriteTxData(cmd);
    
    // Wait for this specific transfer to complete
    while(!(SPIM_1_ReadTxStatus() & SPIM_1_STS_SPI_DONE)); 
}

void TFT_SendData(uint8_t data)
{
    DC_Write(1); // Set Data/Command pin to Data
    SPIM_1_ClearTxBuffer(); 
    SPIM_1_ReadTxStatus();  
    
    SPIM_1_WriteTxData(data);
    
    while(!(SPIM_1_ReadTxStatus() & SPIM_1_STS_SPI_DONE));
}


void TFT_Init(void)
{
    //Hardware Reset 
    RESET_Write(1);  
    CyDelay(5);      
    RESET_Write(0);  
    CyDelay(20);    
    RESET_Write(1);  
    CyDelay(150);    
    // Software Reset
    TFT_SendCommand(0x01); 
    CyDelay(5);          

    // Power Control 1
    TFT_SendCommand(0xC0); 
    TFT_SendData(0x21);    // Sets GVDD level to 4.50V

    // 3. Power Control 2
    TFT_SendCommand(0xC1); 
    TFT_SendData(0x10);    // Sets the factor used in the step-up circuits 

    // VCOM Control 
    TFT_SendCommand(0xC5); 
    TFT_SendData(0x31);    // VMH value 
    TFT_SendData(0x3C);    // VML value 

    TFT_SendCommand(0xC7); 
    TFT_SendData(0xC0);    // Default VCOM offset parameter 

    // Display Function 
    TFT_SendCommand(0xB6); 
    TFT_SendData(0x0A);    // PTG [1:0] and PT [1:0] defaults
    TFT_SendData(0x82);    // REV, GS, SS, SM, ISC defaults
    TFT_SendData(0x27);    // NL [5:0] default
    TFT_SendData(0x00);
    
    //  Memory Access 
    TFT_SendCommand(0x36); // MAC register
    TFT_SendData(0x28);    // Row/Column exchange for Landscape

    // Pixel Format Set
    TFT_SendCommand(0x3A); // PIXSET: Sets the RGB and MCU interface format 
    TFT_SendData(0x55);    // 16-bit/pixel (RGB 5-6-5) format 

    // Exit Sleep Mode
    TFT_SendCommand(0x11); // SLPOUT command (Standard hex 0x11)
    CyDelay(120);          // It takes 120msec to become Sleep Out mode after command is issued 

    // Turn on the Display
    TFT_SendCommand(0x13); // NORON: Normal Display Mode On 
    TFT_SendCommand(0x29); // Display ON command 
}


/* 
 * Helper function to map a float value between min and max 
 * to a 16-bit RGB565 color gradient (Blue -> Green -> Red).
 */
uint16_t GetThermalColor(float value, float min, float max)
{
    // Prevent division by zero if the whole frame is the exact same temperature
    if (max == min) return 0x0000; // Black
    
    // Normalize the value between 0.0 and 1.0
    float ratio = (value - min) / (max - min);
    
    uint8_t r = 0, g = 0, b = 0;
    
    //  simple gradient
    if (ratio < 0.5f) {
        //  Blue (0) to Green (0.5)
        b = (uint8_t)(255.0f * (1.0f - (ratio * 2.0f)));
        g = (uint8_t)(255.0f * (ratio * 2.0f));
    } else {
        // Green (0.5) to Red (1.0)
        g = (uint8_t)(255.0f * (1.0f - ((ratio - 0.5f) * 2.0f)));
        r = (uint8_t)(255.0f * ((ratio - 0.5f) * 2.0f));
    }
    
    // 16-bit RGB565
    // Red: 5 bits, Green: 6 bits, Blue: 5 bits
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

/*
 * process the 768 thermal pixels and draw them 
 * on the ILI9341 TFT display as 10x10 blocks.
 */
void Update_TFT_Image(float* thermalImage)
{
    float minTemp = thermalImage[0];
    float maxTemp = thermalImage[0];
    uint16_t x, y, scaleX, scaleY;
    uint16_t color;
    
    // Find the coldest and hottest pixels 
    for (int i = 0; i < 768; i++) 
    {
        if (thermalImage[i] < minTemp) minTemp = thermalImage[i];
        if (thermalImage[i] > maxTemp) maxTemp = thermalImage[i];
    }

    // Set full 320x240 screen
    // CASET (Column Address Set) command 0x2A
    TFT_SendCommand(0x2A); 
    TFT_SendData(0x00); TFT_SendData(0x00); // Start Column: 0
    TFT_SendData(0x01); TFT_SendData(0x3F); // End Column: 319 (0x013F)

    // PASET (Page Address Set) command 0x2B
    TFT_SendCommand(0x2B); 
    TFT_SendData(0x00); TFT_SendData(0x00); // Start Page (Row): 0
    TFT_SendData(0x00); TFT_SendData(0xEF); // End Page (Row): 239 (0x00EF)

    // prepare for pixel data
    TFT_SendCommand(0x2C); 
    
    // Data/Command = High 
    DC_Write(1); 
    
    //  Loop through thermal camera array
    for(y = 0; y < 24; y++)
    {
        for(scaleY = 0; scaleY < 10; scaleY++) 
        {
            for(x = 0; x < 32; x++)
            {
                // calculate color for 10x10 block
                float tempValue = thermalImage[y * 32 + x];
                color = GetThermalColor(tempValue, minTemp, maxTemp);
                
                // Pre-calculate the bytes 
                uint8_t highByte = color >> 8;
                uint8_t lowByte = color & 0xFF;
                
                for(scaleX = 0; scaleX < 10; scaleX++)
                {
                    // Wait FIFO is full
                    while(SPIM_1_GetTxBufferSize() >= 4); 
                    
                    SPIM_1_WriteTxData(highByte);
                    
                    while(SPIM_1_GetTxBufferSize() >= 4); 
                    
                    SPIM_1_WriteTxData(lowByte);
                }
            }
        }
    }
    
    while(SPIM_1_GetTxBufferSize() > 0);
    CyDelayUs(10);
}

// define functions required by MLX90640_I2C_Driver.h
void MLX90640_I2CInit(void)
{
    I2C_MASTER_1_Start();
}

void MLX90640_I2CFreqSet(int freq)
{ 
    (void)freq; // frequency is set in PSoC design
}
int MLX90640_I2CRead(uint8_t slaveAddr, uint16_t startAddress, uint16_t nMemAddressRead, uint16_t *data)
{
    // start
    if (I2C_MASTER_1_MasterSendStart(slaveAddr, 0) != I2C_MASTER_1_MSTR_NO_ERROR) return -1;
    
     //  memory address (MSB first)
    I2C_MASTER_1_MasterWriteByte(startAddress >> 8);
    I2C_MASTER_1_MasterWriteByte(startAddress & 0xFF);
    
    // restart
    if (I2C_MASTER_1_MasterSendRestart(slaveAddr, 1) != I2C_MASTER_1_MSTR_NO_ERROR) return -1;
    
    // read words
    for (uint16_t i = 0; i < nMemAddressRead; i++) 
    {
        // Read MSB and send ACK
        uint16_t highByte = I2C_MASTER_1_MasterReadByte(I2C_MASTER_1_ACK_DATA);
        
        uint16_t lowByte;
        if (i == (nMemAddressRead - 1)) {
            //  last byte,  send NAK to stop
            lowByte = I2C_MASTER_1_MasterReadByte(I2C_MASTER_1_NAK_DATA);
        } else {
            // send ACK to keep reading
            lowByte = I2C_MASTER_1_MasterReadByte(I2C_MASTER_1_ACK_DATA);
        }
        
        // Reconstruct the 16-bit word
        data[i] = (highByte << 8) | lowByte;
    }
    
    //stop
    I2C_MASTER_1_MasterSendStop();
    return 0; 
}

int MLX90640_I2CWrite(uint8_t slaveAddr, uint16_t writeAddress, uint16_t data)
{
    // start
    if (I2C_MASTER_1_MasterSendStart(slaveAddr, 0) != I2C_MASTER_1_MSTR_NO_ERROR) return -1;
    
    //  memory address (MSB first)
    I2C_MASTER_1_MasterWriteByte(writeAddress >> 8);
    I2C_MASTER_1_MasterWriteByte(writeAddress & 0xFF);
    
    // 16-bit data (MSB first)
    I2C_MASTER_1_MasterWriteByte(data >> 8);
    I2C_MASTER_1_MasterWriteByte(data & 0xFF);
    
    // stop
    I2C_MASTER_1_MasterSendStop();
    
    // Read back the data to verify (as required by the API)
    uint16_t dataCheck;
    if (MLX90640_I2CRead(slaveAddr, writeAddress, 1, &dataCheck) != 0) 
    {
        return -1; // read error
    }
    
    if (dataCheck != data) 
    {
        return -2; // mismatch data
    }
    
    return 0; // Success
}
int MLX90640_I2CGeneralReset(void)
{
    if (I2C_MASTER_1_MasterSendStart(0x00, 0) != I2C_MASTER_1_MSTR_NO_ERROR) return -1;
    
    // I2C reset command 
    I2C_MASTER_1_MasterWriteByte(0x06);
    I2C_MASTER_1_MasterSendStop();
    
    return 0; 
}

void TFT_diagnostic(){
    // Set the drawing window to the whole screen
    TFT_SendCommand(0x2A); 
    TFT_SendData(0x00); TFT_SendData(0x00);
    TFT_SendData(0x01); TFT_SendData(0x3F);
    TFT_SendCommand(0x2B); 
    TFT_SendData(0x00); TFT_SendData(0x00);
    TFT_SendData(0x00); TFT_SendData(0xEF);
    
    TFT_SendCommand(0x2C); // RAM Write
    
    // Fill 320x240 with RED (0xF800)
    for(uint32_t i = 0; i < 76800; i++) {
        TFT_SendData(0xF8); // High Byte (Red)
        TFT_SendData(0x00); // Low Byte
    }
}
int main(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */
    
    // Start Hardware Blocks
 
    SPIM_1_Start();
    LED_Write(1); 
    TFT_Init(); 
    // TFT_diagnostic();
    // Init I2C Driver
    MLX90640_I2CInit();
    
    // Set refresh rate of 2 Hz, must be < 16Hz  
    MLX90640_SetRefreshRate(MLX_I2C_ADDR, 0x02);
    
    // Extract calibration parameter
    // I2C clock <= 400kHz for safety here
    MLX90640_DumpEE(MLX_I2C_ADDR, eeMLX90640);
    MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
    
    for(;;)
    {
        // get frame data
        int status = MLX90640_GetFrameData(MLX_I2C_ADDR, mlx90640Frame);
        
        if (status >= 0) 
        {
            MLX90640_GetImage(mlx90640Frame, &mlx90640, mlx90640Image);  
            Update_TFT_Image(mlx90640Image); 
        }
    }
}