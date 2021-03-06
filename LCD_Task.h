//******************************************************************************
//! \file           LCD_Task.h
//! \brief          LCD_Task is a TI RTOS Application Thread that provides an
//! \brief          interface for the ILI9341 LCD touchpanel from Adafruit.
//
//******************************************************************************
#ifndef _LCDTASK_H
#define _LCDTASK_H

#ifdef __cplusplus
extern "C"
{
#endif

// ****************************************************************************
// includes
// ****************************************************************************
#include <ti/sysbios/knl/Semaphore.h>
// ****************************************************************************
// defines
// ****************************************************************************




//*****************************************************************************
// globals
//*****************************************************************************
extern Semaphore_Handle lcdSem;
//extern ICall_Semaphore lcdSem;
//*****************************************************************************
// function prototypes
//*****************************************************************************

// -----------------------------------------------------------------------------
//! \brief      LCD task creation function
//!
//! \return     void
// -----------------------------------------------------------------------------
Void LCDTask_createTask(void);
void LCD_WakeupPinHwiFxn(PIN_Handle hPin, PIN_Id pinId);


#ifdef __cplusplus
{
#endif // extern "C"

#endif // end of _LCDTASK_H definition
