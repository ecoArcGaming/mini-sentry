#include "project.h"
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"
#include <stdio.h>
#include "GUI.h"
#include <math.h> // Add this to the top of your file with the other includes

// DELTA DRAWING STATE
static float prevTemps[768] = {-100.0f}; 

//  scaling and UI offsets
#define SCALE 8
#define OFFSET_X 0 
#define OFFSET_Y 40 
/* * standard 16-bit RGB565 color definitions
 */
#define RED   0xF800
#define BLUE  0x001F
#define MLX_I2C_ADDR 0x33
#define CENTER_X 15.5
#define CENTER_Y 11.5
#define TARGET_CUTOFF 10.0
#define K_PAN 6.0
#define K_TILT 6.0
#define MAX_PWM_PAN 2000 // us 
#define MAX_PWM_TILT 1916
#define MIN_PWM_PAN 1000
#define MIN_PWM_TILT 1084

// Allocate memory for the sensor data, ~20kb RAM
static uint16_t eeMLX90640[832];    
static uint16_t mlx90640Frame[834]; 
static float mlx90640Image[768];    
paramsMLX90640 mlx90640;


// state tracking, neutral defaults for locations, all white for frame data
static uint16 current_pan_pwm = 1500;  
static uint16 current_tilt_pwm = 1500;
//static uint16_t prevColors[768] = {0xFFFF}; 


void TFT_SendCommand(uint8_t cmd)
{
    DC_Write(0); 
    SPIM_1_ClearTxBuffer(); 
    SPIM_1_ReadTxStatus();  
    
    SPIM_1_WriteTxData(cmd);
    while(!(SPIM_1_ReadTxStatus() & SPIM_1_STS_SPI_DONE)); 
}

void TFT_SendData(uint8_t data)
{
    DC_Write(1); 
    SPIM_1_ClearTxBuffer(); 
    SPIM_1_ReadTxStatus();  
    
    SPIM_1_WriteTxData(data);
    while(!(SPIM_1_ReadTxStatus() & SPIM_1_STS_SPI_DONE));
}

// Helper function to set a specific drawing window
void TFT_SetWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    TFT_SendCommand(0x2A); // CASET
    TFT_SendData(x0 >> 8); TFT_SendData(x0 & 0xFF);
    TFT_SendData(x1 >> 8); TFT_SendData(x1 & 0xFF);

    TFT_SendCommand(0x2B); // PASET
    TFT_SendData(y0 >> 8); TFT_SendData(y0 & 0xFF);
    TFT_SendData(y1 >> 8); TFT_SendData(y1 & 0xFF);

    TFT_SendCommand(0x2C); // RAMWR
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

    // Power Control 
    TFT_SendCommand(0xC0); 
    TFT_SendData(0x21);    
    TFT_SendCommand(0xC1); 
    TFT_SendData(0x10);    

    // VCOM Control 
    TFT_SendCommand(0xC5); 
    TFT_SendData(0x31);    
    TFT_SendData(0x3C);    

    TFT_SendCommand(0xC7); 
    TFT_SendData(0xC0);    

    // Display Function 
    TFT_SendCommand(0xB6); 
    TFT_SendData(0x0A);    
    TFT_SendData(0x82);    
    TFT_SendData(0x27);    
    TFT_SendData(0x00);
    
    //  Memory Access 
    TFT_SendCommand(0x36); 
    TFT_SendData(0x20);     // Landscape, RGB order

    // Pixel Format Set
    TFT_SendCommand(0x3A); 
    TFT_SendData(0x55);    

    // Exit Sleep Mode
    TFT_SendCommand(0x11); 
    CyDelay(120);          

    // Turn on the Display
    TFT_SendCommand(0x13); 
    TFT_SendCommand(0x29); 
}

uint16_t GetThermalColor(float value, float min, float max)
{
    if (max == min) return 0x0000; // Black
    float ratio = (value - min) / (max - min);
    uint8_t r = 0, g = 0, b = 0;
    
    if (ratio < 0.5f) {
        b = (uint8_t)(255.0f * (1.0f - (ratio * 2.0f)));
        g = (uint8_t)(255.0f * (ratio * 2.0f));
    } else {
        g = (uint8_t)(255.0f * (1.0f - ((ratio - 0.5f) * 2.0f)));
        r = (uint8_t)(255.0f * ((ratio - 0.5f) * 2.0f));
    }
    return ((b & 0xF8) << 8) | ((g & 0xFC) << 3) | (r >> 3);
}
void Update_TFT_Image(float* thermalImage, float minTemp, float maxTemp)
{
    uint16_t x, y;
    uint16_t color;

    for(y = 0; y < 24; y++)
    {
        for(x = 0; x < 32; x++)
        {
            int index = y * 32 + x;
            float tempValue = thermalImage[index];
            
            // Only redraw if the temperature changed by more than 1 degree
            if (fabsf(tempValue - prevTemps[index]) > 1.0f) 
            {
                // Update the memory map
                prevTemps[index] = tempValue; 
                color = GetThermalColor(tempValue, minTemp, maxTemp);
                
                // Calculate actual screen coordinates with the UI offset
                uint16_t screen_X = (x * SCALE) + OFFSET_X;
                uint16_t screen_Y = (y * SCALE) + OFFSET_Y;
                
                // Open a window exactly the size of our new block (8x8)
                TFT_SetWindow(screen_X, screen_Y, screen_X + (SCALE - 1), screen_Y + (SCALE - 1));
                
                DC_Write(1); 
                uint8_t hi = color >> 8;
                uint8_t low = color & 0xFF;
                
                // Stream 64 pixels (8x8) to fill the block
                for(int p = 0; p < (SCALE * SCALE); p++) 
                {
                    while(SPIM_1_GetTxBufferSize() >= 4); 
                    SPIM_1_WriteTxData(hi);
                    
                    while(SPIM_1_GetTxBufferSize() >= 4); 
                    SPIM_1_WriteTxData(low);
                }
                
               
                while(SPIM_1_GetTxBufferSize() > 0);
                CyDelayUs(2); 
            }
        }
    }
}

// MLX90640 I2C API Functions
void MLX90640_I2CInit(void) { I2C_MASTER_1_Start(); }
void MLX90640_I2CFreqSet(int freq) { (void)freq; }

int MLX90640_I2CRead(uint8_t slaveAddr, uint16_t startAddress, uint16_t nMemAddressRead, uint16_t *data)
{
    if (I2C_MASTER_1_MasterSendStart(slaveAddr, 0) != I2C_MASTER_1_MSTR_NO_ERROR) return -1;
    
    I2C_MASTER_1_MasterWriteByte(startAddress >> 8);
    I2C_MASTER_1_MasterWriteByte(startAddress & 0xFF);
    
    if (I2C_MASTER_1_MasterSendRestart(slaveAddr, 1) != I2C_MASTER_1_MSTR_NO_ERROR) return -1;
    
    for (uint16_t i = 0; i < nMemAddressRead; i++) 
    {
        uint16_t highByte = I2C_MASTER_1_MasterReadByte(I2C_MASTER_1_ACK_DATA);
        uint16_t lowByte;
        
        if (i == (nMemAddressRead - 1)) {
            lowByte = I2C_MASTER_1_MasterReadByte(I2C_MASTER_1_NAK_DATA);
        } else {
            lowByte = I2C_MASTER_1_MasterReadByte(I2C_MASTER_1_ACK_DATA);
        }
        data[i] = (highByte << 8) | lowByte;
    }
    
    I2C_MASTER_1_MasterSendStop();
    return 0; 
}

int MLX90640_I2CWrite(uint8_t slaveAddr, uint16_t writeAddress, uint16_t data)
{
    if (I2C_MASTER_1_MasterSendStart(slaveAddr, 0) != I2C_MASTER_1_MSTR_NO_ERROR) return -1;
    
    I2C_MASTER_1_MasterWriteByte(writeAddress >> 8);
    I2C_MASTER_1_MasterWriteByte(writeAddress & 0xFF);
    I2C_MASTER_1_MasterWriteByte(data >> 8);
    I2C_MASTER_1_MasterWriteByte(data & 0xFF);
    
    I2C_MASTER_1_MasterSendStop();
    
    uint16_t dataCheck;
    if (MLX90640_I2CRead(slaveAddr, writeAddress, 1, &dataCheck) != 0) return -1; 
    if (dataCheck != data) return -2; 
    return 0; 
}

int MLX90640_I2CGeneralReset(void)
{
    if (I2C_MASTER_1_MasterSendStart(0x00, 0) != I2C_MASTER_1_MSTR_NO_ERROR) return -1;
    I2C_MASTER_1_MasterWriteByte(0x06);
    I2C_MASTER_1_MasterSendStop();
    return 0; 
}

void Track_Target(float minTemp, float maxTemp, uint16_t target_x, uint16_t target_y) 
{
    if (maxTemp - minTemp < TARGET_CUTOFF) return;
    
    float error_x = target_x - CENTER_X; 
    float error_y = target_y - CENTER_Y;
  
    current_pan_pwm  += (int16)(error_x * K_PAN);
    current_tilt_pwm += (int16)(error_y * K_TILT);

    if (current_pan_pwm > MAX_PWM_PAN) current_pan_pwm = MAX_PWM_PAN;
    if (current_pan_pwm < MIN_PWM_PAN) current_pan_pwm = MIN_PWM_PAN;
    if (current_tilt_pwm > MAX_PWM_TILT) current_tilt_pwm = MAX_PWM_TILT;
    if (current_tilt_pwm < MIN_PWM_TILT) current_tilt_pwm = MIN_PWM_TILT;
    
    PWM_Pan_WriteCompare(current_pan_pwm);
    PWM_Tilt_WriteCompare(current_tilt_pwm);
}

int main(void)
{
    CyGlobalIntEnable; 
    
    // Start Hardware Blocks
    PWM_Pan_Start();
    PWM_Tilt_Start();
    SPIM_1_Start();
    LED_Write(1); 
    
    // Initialize Display
    TFT_Init(); 
    
    // Initialize emWin GUI 
    GUI_Init(); 
    
   
    GUI_SetFont(&GUI_Font16_ASCII);
    GUI_SetColor(GUI_WHITE);
    GUI_SetTextMode(GUI_TM_TRANS); // Transparent background so it doesn't overwrite thermal data
    GUI_DispStringAt("TRACKING ACTIVE", 10, 10); 
    
    // Init Camera
    MLX90640_I2CInit();
    MLX90640_SetRefreshRate(MLX_I2C_ADDR, 0x04);
    
    MLX90640_DumpEE(MLX_I2C_ADDR, eeMLX90640);
    MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
    
    for(;;)
    {
        int status = MLX90640_GetFrameData(MLX_I2C_ADDR, mlx90640Frame);
        
        if (status >= 0) 
        {
            MLX90640_GetImage(mlx90640Frame, &mlx90640, mlx90640Image);  
            float minTemp = mlx90640Image[0];
            float maxTemp = mlx90640Image[0];
            uint16_t max_x = 0;
            uint16_t max_y = 0;
            
            for (int i = 0; i < 768; i++) 
            {
                if (mlx90640Image[i] < minTemp) minTemp = mlx90640Image[i];
                if (mlx90640Image[i] > maxTemp) {
                    maxTemp = mlx90640Image[i];
                    max_x = i % 32; 
                    max_y = i / 32;
                }
            }
            
            // Track_Target(minTemp, maxTemp, max_x, max_y);            
            
            Update_TFT_Image(mlx90640Image, minTemp, maxTemp); 
        }
    }
}