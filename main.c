/******************************************************************************
 * File Name:   main.c
 *
 * Description: This is the source code for the PSoC 4 Direction Detection
 * using Quadrature Decoder application for ModusToolbox.
 *
 * Related Document: See README.md
 *
 *
 *******************************************************************************
 * Copyright 2023, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 *******************************************************************************/

/*******************************************************************************
 * Include Header files
 ********************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"
#include <stdio.h>

/*******************************************************************************
 * Macros
 ********************************************************************************/
#define MID_COUNT           (0x8000u)   /* for 16 bit counter*/
#define DELAY               (500u)      /* 500 msec */
                                        /* Delay should be more than the period
                                        of input signal to the quadrature
                                        decoder */
#define ROTATION    1   /* CLOCKWISE - 1, ANTI_CLOCKWISE - 2, NO_ROTATION - 0 */
#define TCPWM_PWM_VAL       (999UL)     /* Counter value*/

/*******************************************************************************
 * Function Name: main
 ********************************************************************************
 * Summary:
 *  System entrance point. This function performs
 *    1.Initializes the BSP.
 *    2.To detect the direction of rotation, quadrature decoder counter is monitored.
 *    3.The result is shown using LED.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  int
 *
 *******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    uint16_t count      = MID_COUNT;
    uint16_t count_prev  = MID_COUNT;

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize with config set in peripheral and enable QuadDec */
    Cy_TCPWM_QuadDec_Init(QuadDec_HW, QuadDec_NUM, &QuadDec_config);
    Cy_TCPWM_Enable_Multiple(QuadDec_HW, QuadDec_MASK);

    /* Start QuadDec */
    Cy_TCPWM_TriggerReloadOrIndex(QuadDec_HW, QuadDec_MASK);

    /* Configure two PWM for generating quadrature encoded signals*/
    /* Initialize with config set in peripheral and enable both the PWMs*/
    Cy_TCPWM_PWM_Init(PWM_phiA_HW, PWM_phiA_NUM, &PWM_phiA_config);
    Cy_TCPWM_PWM_Init(PWM_phiB_HW, PWM_phiB_NUM, &PWM_phiB_config);
    Cy_TCPWM_Enable_Multiple(PWM_phiA_HW, PWM_phiA_MASK | PWM_phiB_MASK);

    /* Start both the PWM Peripherals */
    /* Both PWM peripherals will be triggered simultaneously. Counter value
     * will emulate 90 degrees phase shift.
     */

#if (ROTATION == 1)

    /* Set the counter value to generate phase shift */
    Cy_TCPWM_PWM_SetCounter(PWM_phiA_HW,PWM_phiA_NUM,TCPWM_PWM_VAL);
    /* Trigger a software start on the TCPWM instance. This is required when
     * no other hardware input signal is connected to the peripheral to act as
     * a trigger source.
     */
    Cy_TCPWM_TriggerStart(PWM_phiA_HW, PWM_phiA_MASK | PWM_phiB_MASK);

#elif (ROTATION == 2)

    /* Set the counter value to generate phase shift */
    Cy_TCPWM_PWM_SetCounter(PWM_phiB_HW,PWM_phiB_NUM,TCPWM_PWM_VAL);
    /* Trigger a software start on the TCPWM instance. This is required when
     * no other hardware input signal is connected to the peripheral to act as
     * a trigger source.
     */
    Cy_TCPWM_TriggerStart(PWM_phiA_HW, PWM_phiA_MASK | PWM_phiB_MASK);

#endif

    for(;;)
    {
        count = Cy_TCPWM_QuadDec_GetCounter(QuadDec_HW, QuadDec_NUM);
    /* For clockwise rotation count value increases and for anti-clockwise
     * counter value decreases.
     */
        if(count > count_prev)       /* Condition for clockwise rotation */
        {
            Cy_GPIO_Clr(LED1_PORT, LED1_NUM);   /* LED 1 ON  */
            Cy_GPIO_Set(LED2_PORT, LED2_NUM);  /* LED 2 OFF */
            Cy_SysLib_Delay(DELAY);
        }
        else if(count < count_prev)  /* Condition for anti-clockwise rotation */
        {
            Cy_GPIO_Set(LED1_PORT, LED1_NUM);   /* LED 1 OFF */
            Cy_GPIO_Clr(LED2_PORT, LED2_NUM);  /* LED 2 ON  */
            Cy_SysLib_Delay(DELAY);
        }
        else                        /* No rotation */
        {
            Cy_GPIO_Set(LED1_PORT, LED1_NUM);   /* LED 1 OFF */
            Cy_GPIO_Set(LED2_PORT, LED2_NUM);  /* LED 2 OFF */
        }
        /* Update count_prev value */
        count_prev = count;
    }
}
/* [] END OF FILE */
