#include "project.h"
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"
#include <stdio.h>
#include "GUI.h"
#include "BUTTON.h"
#include "WM.h"
#include "TEXT.h" // Required for the TEXT widget
#include <math.h>

// DELTA DRAWING STATE
static float prevTemps[768] = {-100.0f}; 

//  scaling and UI offsets
#define SCALE 8
#define OFFSET_X 0 
#define OFFSET_Y 40 

/* * standard 16-bit RGB565 color definitions */
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

#define TOUCH_X_MIN 300
#define TOUCH_X_MAX 3800
#define TOUCH_Y_MIN 300
#define TOUCH_Y_MAX 3800

#define LCD_WIDTH  240
#define LCD_HEIGHT 320

// XPT2046 Command Bytes (12-bit mode, differential)
#define CMD_READ_X 0xD0
#define CMD_READ_Y 0x90
// Allocate memory for the sensor data, ~20kb RAM
static uint16_t eeMLX90640[832];    
static uint16_t mlx90640Frame[834]; 
static float mlx90640Image[768];    
paramsMLX90640 mlx90640;

// state tracking, neutral defaults for locations, all white for frame data
static uint16 current_pan_pwm = 1500;  
static uint16 current_tilt_pwm = 1500;

// Global Handle for the Text Widget ---
TEXT_Handle hTxtMode;

// 1. Create a volatile flag for the ISR to talk to the main loop
volatile uint8_t touch_flag = 0;

CY_ISR(Touch_ISR_Handler)
{
    T_INT_ClearInterrupt();
    touch_flag = 1; 
}
void TFT_Wait(void)
{
    while(SPIM_1_GetTxBufferSize() > 0); 
    CyDelayUs(5); 
}
void TFT_SendCommand(uint8_t cmd)
{
    TFT_Wait();
    DC_Write(0); 
    SPIM_1_ClearTxBuffer(); 
    SPIM_1_ReadTxStatus();  
    
    SPIM_1_WriteTxData(cmd);
    while(!(SPIM_1_ReadTxStatus() & SPIM_1_STS_SPI_DONE)); 
}

void TFT_SendData(uint8_t data)
{   
    TFT_Wait();
    DC_Write(1); 
    SPIM_1_ClearTxBuffer(); 
    SPIM_1_ReadTxStatus();  
    
    SPIM_1_WriteTxData(data);
    while(!(SPIM_1_ReadTxStatus() & SPIM_1_STS_SPI_DONE));
}

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
                prevTemps[index] = tempValue; 
                color = GetThermalColor(tempValue, minTemp, maxTemp);
                
                uint16_t screen_X = (x * SCALE) + OFFSET_X;
                uint16_t screen_Y = (y * SCALE) + OFFSET_Y;
                
                TFT_SetWindow(screen_X, screen_Y, screen_X + (SCALE - 1), screen_Y + (SCALE - 1));
                
                DC_Write(1); 
                uint8_t hi = color >> 8;
                uint8_t low = color & 0xFF;
                
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

//  events on the background window 
static void _cbBackground(WM_MESSAGE * pMsg) 
{
    int NCode, Id;

    switch (pMsg->MsgId) 
    {  
        case WM_NOTIFY_PARENT:
            Id    = WM_GetId(pMsg->hWinSrc);
            NCode = pMsg->Data.v;

            if (NCode == WM_NOTIFICATION_RELEASED) 
            {
                // Change the text 
                switch (Id) 
                {
                    case GUI_ID_BUTTON0:
                        TEXT_SetText(hTxtMode, "Mode: Auto");
                        break;
                    case GUI_ID_BUTTON1:
                        TEXT_SetText(hTxtMode, "Mode: Zone");
                        break;
                    case GUI_ID_BUTTON2:
                        TEXT_SetText(hTxtMode, "Mode: Audio");
                        break;
                }
            }
            break;
            
        default:
            WM_DefaultProc(pMsg);
            break;
    }
}
static uint16_t Touch_Read_Axis(uint8_t command)
{
    uint16_t data = 0;
    uint8_t msb, lsb;
    CS_1_Write(0); 
    // 2. Send the Command Byte
    SPIM_2_ClearRxBuffer();
    SPIM_2_WriteTxData(command);
    while(!(SPIM_2_ReadTxStatus() & SPIM_2_STS_SPI_DONE));
    SPIM_2_ReadRxData(); // Clear the dummy read from the buffer

    // 3. Send 0x00 to clock in the Most Significant Byte (MSB)
    SPIM_2_WriteTxData(0x00);
    while(!(SPIM_2_ReadTxStatus() & SPIM_2_STS_SPI_DONE));
    msb = SPIM_2_ReadRxData();

    // 4. Send 0x00 to clock in the Least Significant Byte (LSB)
    SPIM_2_WriteTxData(0x00);
    while(!(SPIM_2_ReadTxStatus() & SPIM_2_STS_SPI_DONE));
    lsb = SPIM_2_ReadRxData();
     CS_1_Write(1); 
    // 6. Combine the bytes. 
    // The XPT2046 sends a 12-bit value left-aligned across the 16 bits.
    data = ((msb << 8) | lsb) >> 3;
    
    return (data & 0x0FFF); // Mask to ensure it's strictly 12 bits
}

// The main function called by your loop when the interrupt flag fires
void Update_Touch_State(void)
{
    GUI_PID_STATE touchState;
    uint16_t raw_x, raw_y;
    int32_t mapped_x, mapped_y;

    // Read the physical pin state to ensure we are still touching (Debounce check)
    if (T_INT_Read() == 0) 
    {
        // Read raw 12-bit ADC values from the touch controller
        raw_x = Touch_Read_Axis(CMD_READ_X);
        raw_y = Touch_Read_Axis(CMD_READ_Y);

        // Map the raw ADC values (0-4095) to screen coordinates (320x240)
        mapped_x = ((int32_t)(raw_x - TOUCH_X_MIN) * LCD_WIDTH) / (TOUCH_X_MAX - TOUCH_X_MIN);
        mapped_y = ((int32_t)(raw_y - TOUCH_Y_MIN) * LCD_HEIGHT) / (TOUCH_Y_MAX - TOUCH_Y_MIN);

        // Clamp the mapped values so they don't draw outside the screen bounds
        if (mapped_x < 0) mapped_x = 0;
        if (mapped_x >= LCD_WIDTH) mapped_x = LCD_WIDTH - 1;
        
        if (mapped_y < 0) mapped_y = 0;
        if (mapped_y >= LCD_HEIGHT) mapped_y = LCD_HEIGHT - 1;

        // Invert axes if necessary 
        // (Uncomment these if your touch moves left when you drag right, or up when dragging down)
        // mapped_x = (LCD_WIDTH - 1) - mapped_x;
        mapped_y = (LCD_HEIGHT - 1) - mapped_y;

        // Swap axes if necessary 
        // (If your touch X is moving the Y axis, you need to swap them depending on screen rotation)
        // int32_t temp = mapped_x;
        // mapped_x = mapped_y;
        //mapped_y = temp;

        // Feed the pressed state and coordinates to emWin
        touchState.x = mapped_x;
        touchState.y = mapped_y;
        touchState.Pressed = 1;
    } 
    else 
    {
        // Screen is not currently being touched
        touchState.x = -1;
        touchState.y = -1;
        touchState.Pressed = 0;
    }

    GUI_TOUCH_StoreStateEx(&touchState);
}
int main(void)
{
    CyGlobalIntEnable; 
    
    // Start Hardware Blocks
    PWM_Pan_Start();
    PWM_Tilt_Start();
    SPIM_1_Start();
    SPIM_2_Start();
    LED_Write(1); 
    
    CS_1_Write(0);
    Touch_Read_Axis(CMD_READ_X); // 0xD0 has PD0=0, enabling PENIRQ
    CS_1_Write(1);
    
    // Initialize Display
    TFT_Init(); 
    
    // Initialize emWin GUI 
    GUI_Init(); 
    
    // Attach the callback to the background window, attach ISR to vector
    T_ISR_StartEx(Touch_ISR_Handler); 
    WM_SetCallback(WM_HBKWIN, _cbBackground);
    
    GUI_SetFont(&GUI_Font16_ASCII);
    GUI_SetColor(GUI_WHITE);
    
    // x0, y0, xSize, ySize, hParent, Flags, ExFlags, Id, pText
    hTxtMode = TEXT_CreateEx(10, 10, 150, 20, WM_HBKWIN, WM_CF_SHOW, 0, GUI_ID_TEXT0, "Mode: Auto");
    TEXT_SetTextColor(hTxtMode, GUI_WHITE);
    TEXT_SetFont(hTxtMode, &GUI_Font16_ASCII);
    TEXT_SetBkColor(hTxtMode, GUI_BLACK);
    
    // Create Buttons
    BUTTON_Handle hBtnAuto, hBtnZone, hBtnAudio;
    
    hBtnAuto = BUTTON_Create(0,   270, 80, 40, GUI_ID_BUTTON0, WM_CF_SHOW);
    BUTTON_SetText(hBtnAuto, "Auto");

    hBtnZone = BUTTON_Create(80,  270, 80, 40, GUI_ID_BUTTON1, WM_CF_SHOW);
    BUTTON_SetText(hBtnZone, "Zone");

    hBtnAudio = BUTTON_Create(160, 270, 80, 40, GUI_ID_BUTTON2, WM_CF_SHOW);
    BUTTON_SetText(hBtnAudio, "Audio");
    
    // Init Camera
    MLX90640_I2CInit();
    MLX90640_SetRefreshRate(MLX_I2C_ADDR, 0x04);
    
    MLX90640_DumpEE(MLX_I2C_ADDR, eeMLX90640);
    MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
    
    for(;;)
    {       
      
        int status = MLX90640_GetFrameData(MLX_I2C_ADDR, mlx90640Frame);
        if (T_INT_Read() == 0 || touch_flag == 1) 
        {
            // Mask the interrupt so SPI traffic doesn't trigger the ISR 
            T_ISR_Disable();
            Update_Touch_State();   // SPI touch read here
            T_ISR_ClearPending();
            T_ISR_Enable();
        }
       
        
      
        GUI_Exec(); // EmWin processes the touch states here
        
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