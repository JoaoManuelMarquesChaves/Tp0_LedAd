/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;
S_ADCResults ADCResults;
// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;

    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{
    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {   
            bool appInitialized = true;
            if (appInitialized)
            {
                lcd_init();             //initialisation du lcd
                lcd_bl_on();            //alumer le bl
                printf_lcd("Tp0 Led+AD 2022-23");
                lcd_gotoxy(1,2 );      // ecrire sur la deuxieme ligne
                printf_lcd("Joao Marques Chaves");

                BSP_InitADC10();     //initialisation de l'adc 
                LED_ON_OFF(FLAG_ON); // appel de la fonction pour alumer les LED
                DRV_TMR0_Start();
                
                appData.state = APP_STATE_SERVICE_TASKS;
            }
            appData.state = APP_STATE_WAIT;
            break;
        }
        case APP_STATE_WAIT:
        {
            break;
        }
        case APP_STATE_SERVICE_TASKS:
        {
            ADCResults = BSP_ReadAllADC(); //recupere les valeur des pot
            lcd_gotoxy(1,3);               // ecrire sur la troisime ligne
            printf_lcd("Ch0 %4d Ch1 %4d",ADCResults.Chan0,ADCResults.Chan1);
            LED_ON_OFF(FLAG_OFF);   //appel de la fonction pour eteindre les Led
            Chenilard();            //appel de la fonction chenillard
            appData.state = APP_STATE_WAIT;
            break;
        }
        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}
  void APP_UpdateState(APP_STATES NewState)
    {
        appData.state = NewState;
    }
  // fonctiom qui permet d'alumer ou eteindre les led
  void LED_ON_OFF(STATE_FLAG State_Flag)
  {
    uint8_t FLAG_LED = FLAG_OFF;
    FLAG_LED = State_Flag;
      if(FLAG_LED == FLAG_ON)
      {
        BSP_LEDOn(BSP_LED_0);
        BSP_LEDOn(BSP_LED_1);
        BSP_LEDOn(BSP_LED_2);
        BSP_LEDOn(BSP_LED_3);
        BSP_LEDOn(BSP_LED_4);
        BSP_LEDOn(BSP_LED_5);
        BSP_LEDOn(BSP_LED_6);
        BSP_LEDOn(BSP_LED_7);
      }
      if(FLAG_LED == FLAG_OFF)
      {
        BSP_LEDOff(BSP_LED_0);
        BSP_LEDOff(BSP_LED_1);
        BSP_LEDOff(BSP_LED_2);
        BSP_LEDOff(BSP_LED_3);
        BSP_LEDOff(BSP_LED_4);
        BSP_LEDOff(BSP_LED_5);
        BSP_LEDOff(BSP_LED_6);
        BSP_LEDOff(BSP_LED_7);
      }
  }
  //fonction qui permet d'efectuer un chenilard
  void Chenilard(void)
  {
    static uint8_t Chillard_Etat=0;
    switch(Chillard_Etat)
      {
          case 0:
              BSP_LEDOff(BSP_LED_6);
              BSP_LEDOn(BSP_LED_0);
              Chillard_Etat =1;
          break;
          case 1:
              BSP_LEDOff(BSP_LED_0);
              BSP_LEDOn(BSP_LED_1);
              Chillard_Etat =2;
          break;
          case 2:
              BSP_LEDOff(BSP_LED_1);
              BSP_LEDOn(BSP_LED_2);
              Chillard_Etat =3;
          break;
          case 3:
              BSP_LEDOff(BSP_LED_2);
              BSP_LEDOn(BSP_LED_3);
              Chillard_Etat =4;
          break;
          case 4:
              BSP_LEDOff(BSP_LED_3);
              BSP_LEDOn(BSP_LED_4);
              Chillard_Etat =5;
          break;
          case 5:
              BSP_LEDOff(BSP_LED_4);
              BSP_LEDOn(BSP_LED_5);
              Chillard_Etat =6;
          break;
          case 6:
              BSP_LEDOff(BSP_LED_5);
              BSP_LEDOn(BSP_LED_6);
              Chillard_Etat =7;
          break;
          case 7:
              BSP_LEDOff(BSP_LED_6);
              BSP_LEDOn(BSP_LED_7);
              Chillard_Etat =0;
          break;
          default:
          break;
      }
  }
/*******************************************************************************
 End of File
 */
