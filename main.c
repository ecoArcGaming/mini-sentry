#include "project.h"
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"
#include <stdio.h>
#include "GUI.h"
#include "BUTTON.h"
#include "WM.h"
#include "TEXT.h" 
#include <math.h>

// buffer for previous frame
static float prevTemps[768] = {-100.0f}; 

// offsets for the camera 
#define SCALE 8
#define OFFSET_X 0 
#define OFFSET_Y 40 

/* * 16-bit RGB color  */
#define RED   0xF800
#define BLUE  0x001F

#define MLX_I2C_ADDR 0x33

// the feedback system
#define CENTER_X 15.5
#define CENTER_Y 11.5
#define TARGET_CUTOFF 15.0
#define K_PAN 6.0
#define K_TILT 6.0
#define MAX_PWM_PAN 2000 // us 
#define MAX_PWM_TILT 2500
#define MIN_PWM_PAN 1000
#define MIN_PWM_TILT 1200
#define MIDDLE_PWM_PAN 1500
#define MIDDLE_PWM_TILT 1800

// touch screen
#define TOUCH_X_MIN 300
#define TOUCH_X_MAX 3800
#define TOUCH_Y_MIN 300
#define TOUCH_Y_MAX 3800
// TFT 
#define LCD_WIDTH  240
#define LCD_HEIGHT 320

// XPT2046 cmds
#define CMD_READ_X 0xD0
#define CMD_READ_Y 0x90

//  memory for the sensor data, ~20kb RAM
static uint16_t eeMLX90640[832];    
static uint16_t mlx90640Frame[834]; 
static float mlx90640Image[768];    
paramsMLX90640 mlx90640;

// state tracking, neutral defaults for locations, all white for frame data
static uint16 current_pan_pwm = 1500;  
static uint16 current_tilt_pwm = 1500;

typedef struct {
    float pos;
    float vel;
    float p00, p01, p11;
    float q_pos, q_vel;
    float r; 
} KalmanFilter;

KalmanFilter KALMAN_PAN;
KalmanFilter KALMAN_TILT;

// text widget
TEXT_Handle hTxtMode;

// flags for ISR
volatile uint8_t touch_flag = 0;
volatile enum Mode{
    AUTO, 
    STATION,
    AUDIO
};
volatile uint8_t spi_tft_busy = 0; // like a semaphore for two SPIs
volatile enum Mode curMode = AUTO;

// station mode stuff
#define DT 0.25
#define ALERT_TEMP_CUTOFF 120.0f 
#define BOX_SIZE 10             // bounding box
#define MAX_BOX_X (32 - BOX_SIZE) // 
#define MAX_BOX_Y (24 - BOX_SIZE) // 
static uint8_t box_x = 11; //  centered
static uint8_t box_y = 7;

// flag for Button ISR
volatile uint8_t control_axis = 0; // 0 = X-axis, 1 = Y-axis

// redraw if the box moves
void Force_TFT_Redraw(void)
{
    for(int i = 0; i < 768; i++) 
    {
        prevTemps[i] = -100.0f;
    }
}

CY_ISR(Button_ISR_Handler)
{
    Pin_Button_ClearInterrupt(); 
    control_axis = !control_axis; // Toggle 0 and 1
}

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
    spi_tft_busy = 1;
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
    spi_tft_busy = 0;
}

void Update_TFT_Image_Box(float* thermalImage, float minTemp, float maxTemp)
{
    uint16_t x, y;
    uint16_t color;
    spi_tft_busy = 1;

    for(y = 0; y < 24; y++)
    {
        for(x = 0; x < 32; x++)
        {
            int index = y * 32 + x;
            float tempValue = thermalImage[index];
            
            // Check if this pixel is on the 10x10 box border
            uint8_t is_border = 0;
            if (curMode == STATION) 
            {
                if (x >= box_x && x < box_x + BOX_SIZE && y >= box_y && y < box_y + BOX_SIZE) 
                {
                    // on the edge of square
                    if (x == box_x || x == box_x + (BOX_SIZE - 1) || y == box_y || y == box_y + (BOX_SIZE - 1)) {
                        is_border = 1;
                    }
                }
            }
            
            
            if (fabsf(tempValue - prevTemps[index]) > 1.0f) 
            {
                prevTemps[index] = tempValue; 
                
                // Override color for the bounding box
                if (is_border) {
                    color = RED; // actually blue
                } else {
                    color = GetThermalColor(tempValue, minTemp, maxTemp);
                }
                
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
    spi_tft_busy = 0;
}

// MLX90640 I2C API Functions
void MLX90640_I2CInit(void) { I2C_MASTER_1_Start(); }

// this is set in the schematic
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
    I2C_MASTER_1_MasterWriteByte(0x06); // reset cmd
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

void Update_Kalman(KalmanFilter* axis,  float target){
    axis->pos = axis->pos + DT *  axis->vel;
    axis->p00 = axis->p00 + 2 * axis->p01 + axis->p11 + axis->q_pos;
    axis->p01 = axis->p01 + axis->p11;    
    axis->p11 = axis->p11 + axis->q_vel;
    
    float error = target - axis->pos;
    float S = axis->p00 + axis->r;
    float k0 = axis->p00 / S;
    float k1 = axis->p01 / S;
    // apply kalman gain
    axis->pos += (error * k0);
    axis->vel += (error * k1);
    float p00_temp = axis->p00;
    axis->p00 = p00_temp - (k0 * p00_temp);
    float p01_temp = axis->p01;
    axis->p01 = p01_temp - (k0 * p01_temp);
    axis->p11 -= k1 * p01_temp; 
}

void Track_Target_Kalman(float minTemp, float maxTemp, uint16_t target_x, uint16_t target_y) 
{
    if (maxTemp - minTemp < TARGET_CUTOFF) return;
    
    Update_Kalman(&KALMAN_PAN,  target_x);
    Update_Kalman(&KALMAN_TILT,  target_y);
    float smoothed_error_x = KALMAN_PAN.pos - CENTER_X; 
    float smoothed_error_y = KALMAN_TILT.pos - CENTER_Y;
  
    current_pan_pwm  += (int16)(smoothed_error_x * K_PAN);
    current_tilt_pwm += (int16)(smoothed_error_y * K_TILT);
    

    if (current_pan_pwm > MAX_PWM_PAN) current_pan_pwm = MAX_PWM_PAN;
    if (current_pan_pwm < MIN_PWM_PAN) current_pan_pwm = MIN_PWM_PAN;
    if (current_tilt_pwm > MAX_PWM_TILT) current_tilt_pwm = MAX_PWM_TILT;
    if (current_tilt_pwm < MIN_PWM_TILT) current_tilt_pwm = MIN_PWM_TILT;
    
    PWM_Pan_WriteCompare(current_pan_pwm);
    PWM_Tilt_WriteCompare(current_tilt_pwm);
}

void Init_Kalman(){
    
    KALMAN_PAN.pos = CENTER_X;
    KALMAN_TILT.pos = CENTER_Y;
    
    KALMAN_PAN.r = 5.0f;
    KALMAN_TILT.r = 5.0f;
    
    KALMAN_PAN.q_pos = 0.5f;
    KALMAN_PAN.q_vel = 0.1f;
    
    KALMAN_TILT.q_pos = 0.1f;
    KALMAN_TILT.q_vel = 0.01f;
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
                // Change the text and state, also update and debounce
                switch (Id) 
                {
                    case GUI_ID_BUTTON0:
                        TEXT_SetText(hTxtMode, "Mode: Auto");
                        curMode = AUTO;
                        Force_TFT_Redraw();
                       //  CyDelay(50);
                        break;
                    case GUI_ID_BUTTON1:
                        TEXT_SetText(hTxtMode, "Mode: Zone");
                        curMode = STATION;
                        Force_TFT_Redraw();
                        // CyDelay(50);
                        break;
                    case GUI_ID_BUTTON2:
                        TEXT_SetText(hTxtMode, "Mode: Audio");
                        curMode = AUDIO;
                        Force_TFT_Redraw();
                        //  CyDelay(50);
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
   
    SPIM_2_ClearRxBuffer();
    SPIM_2_WriteTxData(command);
    while(!(SPIM_2_ReadTxStatus() & SPIM_2_STS_SPI_DONE));
    SPIM_2_ReadRxData(); // Clear the dummy read from the buffer

    //MSB
    SPIM_2_WriteTxData(0x00);
    while(!(SPIM_2_ReadTxStatus() & SPIM_2_STS_SPI_DONE));
    msb = SPIM_2_ReadRxData();

    // LSB
    SPIM_2_WriteTxData(0x00);
    while(!(SPIM_2_ReadTxStatus() & SPIM_2_STS_SPI_DONE));
    lsb = SPIM_2_ReadRxData();
     CS_1_Write(1); 
    // Combine the bytes. 
    // 12-bit value left-aligned across the 16 bits.
    data = ((msb << 8) | lsb) >> 3;
    
    return (data & 0x0FFF); // Mask to ensure it's strictly 12 bits
}

void Update_Touch_State(void)
{
    GUI_PID_STATE touchState;
    uint16_t raw_x, raw_y;
    int32_t mapped_x, mapped_y;

    if (T_INT_Read() == 0) 
    {
        // 12-bit ADC values 
        raw_x = Touch_Read_Axis(CMD_READ_X);
        raw_y = Touch_Read_Axis(CMD_READ_Y);

        //  ADC values  to screen coordinates 
        mapped_x = ((int32_t)(raw_x - TOUCH_X_MIN) * LCD_WIDTH) / (TOUCH_X_MAX - TOUCH_X_MIN);
        mapped_y = ((int32_t)(raw_y - TOUCH_Y_MIN) * LCD_HEIGHT) / (TOUCH_Y_MAX - TOUCH_Y_MIN);

        // Clamp the mapped values 
        if (mapped_x < 0) mapped_x = 0;
        if (mapped_x >= LCD_WIDTH) mapped_x = LCD_WIDTH - 1;
        
        if (mapped_y < 0) mapped_y = 0;
        if (mapped_y >= LCD_HEIGHT) mapped_y = LCD_HEIGHT - 1;

        // Invert axes 
        mapped_y = (LCD_HEIGHT - 1) - mapped_y;

        touchState.x = mapped_x;
        touchState.y = mapped_y;
        touchState.Pressed = 1;
    } 
    else 
    {
        // Screen is not currently being touched, debounce
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
    PWM_ALERT_Start();
    PWM_ALERT_WriteCompare(0); 

    SPIM_1_Start();
    SPIM_2_Start();
    
    ADC_DelSig_1_Start();               // start the ADC_DelSig_1
    ADC_DelSig_1_StartConvert();        // start the ADC_DelSig_1 conversion
    LED_Write(1); 
    
    // ISRs
    T_ISR_StartEx(Touch_ISR_Handler); 
    Button_ISR_StartEx(Button_ISR_Handler); 
    // enable 
    Touch_Read_Axis(CMD_READ_X); // enable pen interrupt
    // Init states of Kalman filters
    Init_Kalman();
    // Initialize Display
    TFT_Init(); 
    
    // Initialize emWin GUI 
    GUI_Init(); 
    
    // Attach the callback to the background window
    WM_SetCallback(WM_HBKWIN, _cbBackground);
    
    // change the UI 
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
        if (curMode == STATION)
        {
            // Read Potentiometer
            uint16_t adc_val = 0;
            if( ADC_DelSig_1_IsEndConversion(ADC_DelSig_1_WAIT_FOR_RESULT) )
            {
                adc_val = ADC_DelSig_1_GetResult16();
            }
            if (adc_val > 4095) adc_val = 4095;

            // Update box coordinates
            uint8_t old_box_x = box_x;
            uint8_t old_box_y = box_y;
            
            // check what button ISR did
            if (control_axis == 0) {
                // Map 0-4095 to 0-22
                box_x = (uint8_t)(((uint32_t)adc_val * MAX_BOX_X) / 4095);
            } else {
                // Map 0-4095 to 0-14
                box_y = (uint8_t)(((uint32_t)adc_val * MAX_BOX_Y) / 4095);
            }

            // If the box moved, force the screen to redraw 
            if (box_x != old_box_x || box_y != old_box_y) {
                Force_TFT_Redraw();
            }
        }
        
        int status = MLX90640_GetFrameData(MLX_I2C_ADDR, mlx90640Frame);
        if ((T_INT_Read() == 0 || touch_flag == 1) && !spi_tft_busy) 
        {
            // temp disable ISR 
            T_ISR_Disable();
            Update_Touch_State();   // SPI touch read here
            T_ISR_ClearPending();
            T_ISR_Enable();
            touch_flag = 0;
        }
        
        GUI_Exec(); 
        CyDelay(50); // debounce? 
        
        if (status >= 0)  // the camera returned a frame
        {
            MLX90640_GetImage(mlx90640Frame, &mlx90640, mlx90640Image);  
            float minTemp = mlx90640Image[0];
            float maxTemp = mlx90640Image[0];
            uint16_t max_x = 0;
            uint16_t max_y = 0;
            
            // compute the heat point and coordinates
            for (int i = 0; i < 768; i++) 
            {
                if (mlx90640Image[i] < minTemp) minTemp = mlx90640Image[i];
                if (mlx90640Image[i] > maxTemp) {
                    maxTemp = mlx90640Image[i];
                    max_x = i % 32; 
                    max_y = i / 32;
                }
            }
            
            // mode switching 
            if (curMode == AUTO){
                Track_Target(minTemp, maxTemp, max_x, max_y);     
                // Track_Target_Kalman(minTemp, maxTemp, max_x, max_y);
                Update_TFT_Image(mlx90640Image, minTemp, maxTemp); 
            }
            
            if (curMode == STATION)
            {   
                Update_TFT_Image_Box(mlx90640Image, minTemp, maxTemp); 

                uint8_t target_detected = 0;
                
                // Scan only the pixels inside the 10x10 box
                for (int y = box_y; y < box_y + BOX_SIZE; y++) 
                {
                    for (int x = box_x; x < box_x + BOX_SIZE; x++) 
                    {
                        int index = y * 32 + x;
                        if (mlx90640Image[index] > ALERT_TEMP_CUTOFF) 
                        {
                            target_detected = 1;
                            break; 
                        }
                    }
                    if (target_detected) break;
                }

                if (target_detected) {
                    PWM_ALERT_WriteCompare(30); //  alert ON
                } else {
                    PWM_ALERT_WriteCompare(0);  //  alert OFF
                }
            }
            else 
            {
                PWM_ALERT_WriteCompare(0); //  alert is off 
            }
            
        }
    }
}