//******************************************************************************
//! \file           LCD_Task.c
//! \brief          LCD_Task is a TI RTOS Application Thread that provides an
//! \brief          interface for the ILI9341 LCD touchpanel from Adafruit.
//
// ****************************************************************************
// includes
// ****************************************************************************

#include <xdc/std.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <stdint.h>

#include <string.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>

#include "LCD_Task.h"
#include "ili9341.h"
#include "Adafruit_GFX.h"
#include "Adafruit_FT6206.h"
#include "board.h"
#include "sensor.h"
#include "bsp_spi.h"
//#include "sensortag.h"

// ****************************************************************************
// defines
// ****************************************************************************

//! \brief Motion sensor triggered
#define LCDTASK_MOTION_SENSED_EVENT 0x0001

//! \brief Motion sensor no longer sees movement
#define LCDTASK_MOTION_GONE_EVENT 0x0002


//! \brief Size of stack created for LCD RTOS task
#define LCDTASK_STACK_SIZE 700

//! \brief Task priority for LCD RTOS task
#define LCDTASK_PRIORITY 3

//! \brief Max number of presses accepted for a keycode
#define LCD_MAX_KEYLENGTH 10

#define LCDTASK_UNINIT_STATE        0
#define LCDTASK_OFF_STATE           1
#define LCDTASK_TURNING_ON_STATE    2
#define LCDTASK_ON_STATE            3
#define LCDTASK_WAIT_STATE          4

//*****************************************************************************
// globals
//*****************************************************************************
// Key press buffer
static uint8_t keybuf[LCD_MAX_KEYLENGTH] = {0};
static uint8_t length;
                
// Track the motion input state
static uint8_t motion_state = 0;

//! \brief ICall ID for stack which will be sending LCD messages
//!
//static uint32_t stackServiceID = 0x0000;

//! \brief RTOS task structure for LCD task
//!
static Task_Struct lcdTaskStruct;

//! \brief Allocated memory block for LCD task's stack
//!
Char lcdTaskStack[LCDTASK_STACK_SIZE];


//! \brief LCD thread ICall Semaphore.
//!
//ICall_Semaphore lcdSem = NULL; //ZH
Semaphore_Handle lcdSem = NULL;

//! \brief LCD ICall Application Entity ID.
//!
//ICall_EntityID lcdAppEntityID = 0;

//! \brief Task pending events
//!
static uint16_t LCDTask_events = 0;

//! \brief Event flags for capturing Task-related events from ISR context
//!
static uint16_t MOTION_ISR_EVENT_FLAGS = 0;

static uint8_t LCDTask_State = LCDTASK_UNINIT_STATE;

// Global pin resources
static PIN_State pinGpioState;
static PIN_Handle hlGpioPin;
extern PIN_Handle hGpioPin;


//! \brief PIN Config for Mrdy and Srdy signals
static PIN_Config lcdMotionPinsCfg[] =
{  
    Board_LCD_MOTION | PIN_GPIO_OUTPUT_DIS | PIN_INPUT_EN | PIN_PULLDOWN,
//    Board_LCD_DC | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL |  PIN_DRVSTR_MIN,    
//    Board_LCD_PWR | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL |  PIN_DRVSTR_MIN,   
//    Board_LCD_CS | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL |  PIN_DRVSTR_MIN,   
    PIN_TERMINATE
};

//*****************************************************************************
// function prototypes
//*****************************************************************************

//! \brief      LCD main event processing loop.
//!
static void LCDTask_process(void);



// -----------------------------------------------------------------------------
//! \brief      Initialization for the LCD Thread
//!
//! \return     void
// -----------------------------------------------------------------------------
static void LCDTask_inititializeTask(void)
{

    LCDTask_events = 0;
    LCDTask_State = LCDTASK_OFF_STATE;
    
    // Handling of buttons, relay and MPU interrupt
    hlGpioPin = PIN_open(&pinGpioState, lcdMotionPinsCfg);
//    if (hGpioPin == 0) {
//        //HWREGB(GPIO_BASE+GPIO_O_DOUT3_0+Board_LCD_PWR) = 1;
//        //HWREGB(GPIO_BASE+GPIO_O_DOUT3_0+Board_LCD_PWR) = 0;
//    } else {
        PIN_registerIntCb(hlGpioPin, LCD_WakeupPinHwiFxn);
//    
//        // Enable IRQ
        PIN_setConfig(hlGpioPin, PIN_BM_IRQ, Board_LCD_MOTION | PIN_IRQ_BOTHEDGES);
//        // Enable wakeup
        //PIN_setConfig(hlGpioPin, PINCC26XX_BM_WAKEUP, Board_LCD_MOTION | PINCC26XX_WAKEUP_POSEDGE);
        
//        motion_state = PIN_getInputValue(Board_LCD_MOTION);
//        // Init SPI Bus
//        bspSpiOpen();
        // Init LCD Variables
        ILI9341_init(hGpioPin);
//    }
    
}

static void LCDTask_turnOnLCD(void) {
  
    // Turn on LCD
    PIN_setOutputValue(hGpioPin, Board_LCD_PWR, Board_LCD_PWR_ON);
    delay_ms(250);
    // Initialize LCD
    //bspSpiOpen();
    ILI9341_setup();
    ILI9341_fillScreen(ILI9341_BLACK);
    
    Adafruit_GFX_setTextColorBg(ILI9341_WHITE,ILI9341_BLACK);
    Adafruit_GFX_setTextSize(4);
    Adafruit_GFX_setCursor(45*1+5*4*0, 38*1+8*4*0);
    Adafruit_GFX_print("1");
    Adafruit_GFX_setCursor(45*2+5*4*1, 38*1+8*4*0);
    Adafruit_GFX_print("2");
    Adafruit_GFX_setCursor(45*3+5*4*2, 38*1+8*4*0);
    Adafruit_GFX_print("3");
    Adafruit_GFX_setCursor(45*1+5*4*0, 38*2+8*4*1);
    Adafruit_GFX_print("4");
    Adafruit_GFX_setCursor(45*2+5*4*1, 38*2+8*4*1);
    Adafruit_GFX_print("5");
    Adafruit_GFX_setCursor(45*3+5*4*2, 38*2+8*4*1);
    Adafruit_GFX_print("6");
    Adafruit_GFX_setCursor(45*1+5*4*0, 38*3+8*4*2);
    Adafruit_GFX_print("7");
    Adafruit_GFX_setCursor(45*2+5*4*1, 38*3+8*4*2);
    Adafruit_GFX_print("8");
    Adafruit_GFX_setCursor(45*3+5*4*2, 38*3+8*4*2);
    Adafruit_GFX_print("9");
    Adafruit_GFX_setCursor(45*2+5*4*1, 38*4+8*4*3);
    Adafruit_GFX_print("0");
    
    Adafruit_FT6206_setup(FT6206_DEFAULT_THRESSHOLD);
}

static void LCDTask_turnOffLCD(void) {
    // Turn off LCD
    PIN_setOutputValue(hGpioPin, Board_LCD_PWR, Board_LCD_PWR_OFF);
}

static int8_t LCDTask_getTouchPoint(void) {
    TS_Point pnt = Adafruit_FT6206_getPoint();
    uint16_t x,y;
    x = pnt.x;
    y = pnt.y;
    if ((x > 22) && (x < 66)) {
        if ((y > 19) && (y < 70)) {
            return 1;
        }
    }
    return -1;
}

// -----------------------------------------------------------------------------
//! \brief      LCD main event processing loop.
//!
//! \return     void
// -----------------------------------------------------------------------------
static void LCDTask_process(void)
{ 
    uint16_t timeout_counter = 0;
    uint8_t last_motion_state = 0;
    /* Forever loop */
    for (;; )
    {
        if (LCDTask_State == LCDTASK_OFF_STATE) {
            Semaphore_pend(lcdSem, BIOS_WAIT_FOREVER);
        }
        
    
        // Capture the ISR events flags now within a critical section.  
        // We do this to avoid possible race conditions where the ISR is 
        // modifying the event mask while the task is read/writing it.
        //UInt hwiKey = Hwi_disable(); UInt taskKey = Task_disable();
        
        LCDTask_events = MOTION_ISR_EVENT_FLAGS;
        
        MOTION_ISR_EVENT_FLAGS = 0;
        
        //Task_restore(taskKey); Hwi_restore(hwiKey);
//        motion_state = PIN_getInputValue(Board_LCD_MOTION);
//        if (motion_state != last_motion_state) {
//          if (motion_state) {  
//            LCDTask_events = LCDTASK_MOTION_SENSED_EVENT;
//          } else {
//            LCDTask_events = LCDTASK_MOTION_GONE_EVENT;
//          }
//          last_motion_state = motion_state;
//        } 
        if(LCDTask_events & LCDTASK_MOTION_SENSED_EVENT)
        {
            if (LCDTask_State == LCDTASK_OFF_STATE) {
                LCDTask_turnOnLCD();
                LCDTask_State = LCDTASK_TURNING_ON_STATE;
            }
            // If only a start to motion was detected,
            // Then we want to disable the timeout
            timeout_counter = 0;
        }
        if (LCDTask_events & LCDTASK_MOTION_GONE_EVENT) {
            LCDTask_State = LCDTASK_WAIT_STATE;
            // Since we've sensed no more motion start timeout
            // Start/reset count down to power off
            timeout_counter = 10;
        }
        
        if (LCDTask_State > LCDTASK_OFF_STATE) {
            // Check for presses
//            int8_t touch = LCDTask_getTouchPoint();
//            
//            if (touch>-1) {
//                // Add some sort of feedback
//                
//                // Check to see if this is a command press
//                if (touch > 9) {
//                    // Process command
//                    if (touch == 10) {
//                        // Lock the door
//                        // sendLockRequest();
//                    } else if (touch == 11) {
//                        // Process keycode (enter)
//                        if (length > 0) {
//                            // Send keycode for checking
//                            // checkKeyCode(keybuf,length);
//                            // Reset buffer
//                            length = 0;
//                        }
//                    }
//                } else {
//                    // Add touch to buffer
//                    if (length < LCD_MAX_KEYLENGTH) {
//                        keybuf[length++] = touch;
//                    }
//                }
//            }
            // Check for timeout
            if (timeout_counter > 0) {
                timeout_counter--;
                if (timeout_counter==0) {
                    LCDTask_State = LCDTASK_OFF_STATE;
                    LCDTask_turnOffLCD();
                }
            }
            delay_ms(100);
        } 
    }
}

// -----------------------------------------------------------------------------
//! \brief      LCD Task function called from within LCDTask_Fxn
//!
//! \return     void
// -----------------------------------------------------------------------------
void LCDTask_task(void)
{
    // Initialize application
    LCDTask_inititializeTask();

    // No return from TestProfile2 process
    LCDTask_process();
}


// -----------------------------------------------------------------------------
// Exported Functions


// -----------------------------------------------------------------------------
//! \brief      LCD task entry point.
//!
//! \return     void
// -----------------------------------------------------------------------------
Void LCDTask_Fxn(UArg a0, UArg a1)
{
    LCDTask_task();
}

// -----------------------------------------------------------------------------
//! \brief      Task creation function for LCD
//!
//! \return     void
// -----------------------------------------------------------------------------
void LCDTask_createTask(void)
{
    memset(&lcdTaskStack, 0xDD, sizeof(lcdTaskStack));

    // Configure and create the LCD task.
    Task_Params lcdTaskParams;
    Task_Params_init(&lcdTaskParams);
    lcdTaskParams.stack = lcdTaskStack;
    lcdTaskParams.stackSize = LCDTASK_STACK_SIZE;
    lcdTaskParams.priority = LCDTASK_PRIORITY;

    Task_construct(&lcdTaskStruct, LCDTask_Fxn, &lcdTaskParams, NULL);
}


// -----------------------------------------------------------------------------
//! \brief      This is a HWI function handler for the motion sensor pin. 
//!             
//!
//! \param[in]  hPin - PIN Handle
//! \param[in]  pinId - ID of pin that triggered HWI
//!
//! \return     void
// -----------------------------------------------------------------------------
void LCD_WakeupPinHwiFxn(PIN_Handle hPin, PIN_Id pinId)
{
    // The pin driver does not currently support returning whether the int
    // was neg or pos edge so we must use a variable to keep track of state. 
    // If the physical state of the pin was used then a very quick toggle of
    // of motion could be missed.
    motion_state ^= 1;
    
    if(motion_state == 0) {
        MOTION_ISR_EVENT_FLAGS |= LCDTASK_MOTION_GONE_EVENT;
    } else {
        MOTION_ISR_EVENT_FLAGS |= LCDTASK_MOTION_SENSED_EVENT;
    }
    Semaphore_post(lcdSem);
}
