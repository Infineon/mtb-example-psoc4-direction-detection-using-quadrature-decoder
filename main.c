/******************************************************************************
 * File Name:   main.c
 *
 * Description: This is the source code for the PSOC 4 Direction Detection
 * using Quadrature Decoder application for ModusToolbox.
 *
 * Related Document: See README.md
 *
 *
 *******************************************************************************
 * (c) 2023-2026, Infineon Technologies AG, or an affiliate of Infineon
 * Technologies AG. All rights reserved.
 * This software, associated documentation and materials ("Software") is
 * owned by Infineon Technologies AG or one of its affiliates ("Infineon")
 * and is protected by and subject to worldwide patent protection, worldwide
 * copyright laws, and international treaty provisions. Therefore, you may use
 * this Software only as provided in the license agreement accompanying the
 * software package from which you obtained this Software. If no license
 * agreement applies, then any use, reproduction, modification, translation, or
 * compilation of this Software is prohibited without the express written
 * permission of Infineon.
 *
 * Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS SOFTWARE
 * IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING, BUT NOT LIMITED TO, ALL WARRANTIES OF NON-INFRINGEMENT OF
 * THIRD-PARTY RIGHTS AND IMPLIED WARRANTIES SUCH AS WARRANTIES OF FITNESS FOR A
 * SPECIFIC USE/PURPOSE OR MERCHANTABILITY.
 * Infineon reserves the right to make changes to the Software without notice.
 * You are responsible for properly designing, programming, and testing the
 * functionality and safety of your intended application of the Software, as
 * well as complying with any legal requirements related to its use. Infineon
 * does not guarantee that the Software will be free from intrusion, data theft
 * or loss, or other breaches ("Security Breaches"), and Infineon shall have
 * no liability arising out of any Security Breaches. Unless otherwise
 * explicitly approved by Infineon, the Software may not be used in any
 * application where a failure of the Product or any consequences of the use
 * thereof can reasonably be expected to result in personal injury.
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
#define MID_COUNT         (0x8000u)     /* for 16 bit counter */
#define DELAY_500MS       (500u)        /* 500 msec
                                         * Delay should be more than the period of
                                         * input signal to the quadrature decoder
                                         */
#define CLOCKWISE         (1u)
#define COUNTER_CLOCKWISE (2u)
#define NO_ROTATION       (0u)
#define ROTATION          (CLOCKWISE)   /* Rotation direction */
#define TCPWM_PWM_VAL     (999UL)       /* Counter value*/

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
    result = cybsp_init();
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    uint16_t count = MID_COUNT;
    uint16_t count_prev = MID_COUNT;

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

#if (ROTATION == CLOCKWISE)

    /* Set the counter value to generate phase shift */
    Cy_TCPWM_PWM_SetCounter(PWM_phiA_HW, PWM_phiA_NUM, TCPWM_PWM_VAL);
    /* Trigger a software start on the TCPWM instance. This is required when
     * no other hardware input signal is connected to the peripheral to act as
     * a trigger source.
     */
    Cy_TCPWM_TriggerStart(PWM_phiA_HW, PWM_phiA_MASK | PWM_phiB_MASK);

#elif (ROTATION == COUNTER_CLOCKWISE)

    /* Set the counter value to generate phase shift */
    Cy_TCPWM_PWM_SetCounter(PWM_phiB_HW, PWM_phiB_NUM, TCPWM_PWM_VAL);
    /* Trigger a software start on the TCPWM instance. This is required when
     * no other hardware input signal is connected to the peripheral to act as
     * a trigger source.
     */
    Cy_TCPWM_TriggerStart(PWM_phiA_HW, PWM_phiA_MASK | PWM_phiB_MASK);

#endif

    for (;;)
    {
        count = Cy_TCPWM_QuadDec_GetCounter(QuadDec_HW, QuadDec_NUM);
        /* For clockwise rotation count value increases and for counter-clockwise
         * counter value decreases.
         */

        /* Correlation between LED1/LED2 state and rotation direction is
         * explained in README.md #Operation part
         */
        if (count > count_prev) /* Condition for clockwise rotation */
        {
            Cy_GPIO_Clr(LED1_PORT, LED1_NUM);
            Cy_GPIO_Set(LED2_PORT, LED2_NUM);
            Cy_SysLib_Delay(DELAY_500MS);
        }
        else if (count < count_prev) /* Condition for counter-clockwise rotation */
        {
            Cy_GPIO_Set(LED1_PORT, LED1_NUM);
            Cy_GPIO_Clr(LED2_PORT, LED2_NUM);
            Cy_SysLib_Delay(DELAY_500MS);
        }
        else /* No rotation */
        {
            Cy_GPIO_Set(LED1_PORT, LED1_NUM);
            Cy_GPIO_Set(LED2_PORT, LED2_NUM);
        }

        /* Update count_prev value */
        count_prev = count;
    }
}
/* [] END OF FILE */
