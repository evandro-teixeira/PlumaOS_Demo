/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file main.c
 * @brief PlumaOS demo application for the FRDM-KL25Z board.
 *
 * @details
 * This file contains the demo application that initializes PlumaOS, creates
 * three LED blink tasks and starts the RTOS scheduler. The example is intended
 * to demonstrate basic PlumaOS usage: task creation, task delays and simple
 * peripheral access (GPIO) via the board drivers in Library-FRDM-KL25Z.
 *
 * The three tasks implemented are:
 * - task_led_red   : toggles the red LED with a 1000 ms delay
 * - task_led_green : toggles the green LED with a 2000 ms delay
 * - task_led_blue  : toggles the blue LED with a 3000 ms delay
 *
 * @author
 * Example adapted from original Freescale demo and PlumaOS integration.
 */

#include "MKL25Z4.h"
#include "../Library-FRDM-KL25Z/externs.h"
#include "../PlumaOS/PlumaOs.h"


/** @brief LED pin definitions: (port, pin) */
#define LED_RED     GPIOB,18  /**< Red LED: port GPIOB pin 18 */
#define LED_GREEN   GPIOB,19  /**< Green LED: port GPIOB pin 19 */
#define LED_BLUE    GPIOD,1   /**< Blue LED: port GPIOD pin 1 */

/** @brief Task state identifiers used inside each LED task */
#define SETUP       0  /**< Initial setup state: configure GPIO */
#define LOOP        1  /**< Main loop state: toggle LED and delay */

/** @brief Logical level definitions for gpio_Set */
#define OFF         1  /**< LED off (active low on hardware) */
#define ON          0  /**< LED on (active low on hardware) */


void task_led_red(void);
void task_led_green(void);
void task_led_blue(void);


/** @brief Task identifiers returned by PlumaOS_TaskAdd */
uint8_t IdTaskRed = 0;    /**< Red task id */
uint8_t IdTaskGreen = 0;  /**< Green task id */
uint8_t IdTaskBlue = 0;   /**< Blue task id */


/**
 * @brief Application entry point.
 *
 * @details
 * Initializes the PlumaOS kernel, registers three LED tasks with normal priority
 * and starts the scheduler. After PlumaOS_StartScheduler() the scheduler takes
 * control and main should never return.
 *
 * @return Never returns under normal operation (returns 0 to satisfy compiler).
 */
int main(void)
{
    /* Initialize RTOS and create tasks */
    PlumaOS_Init();

    IdTaskRed     = PlumaOS_TaskAdd(task_led_red,PlumaOS_PriorityNormal,PlumaOS_TaskReady);
    IdTaskGreen   = PlumaOS_TaskAdd(task_led_green,PlumaOS_PriorityNormal,PlumaOS_TaskReady);
    IdTaskBlue    = PlumaOS_TaskAdd(task_led_blue,PlumaOS_PriorityNormal,PlumaOS_TaskReady);

    /* Start the RTOS scheduler */
    PlumaOS_StartScheduler();

    /* Fallback loop - should never be reached */
    for (;;)
    {

    }
    /* Never leave main */
    return 0;
}

/**
 * @brief Red LED task.
 *
 * @details
 * The task uses a simple state machine with a SETUP state to initialize the GPIO
 * and a LOOP state where the LED is toggled and the task delays itself using
 * PlumaOS_TaskDelay. The delay time controls the blink frequency.
 *
 * Uses:
 * - gpio_Init, gpio_Set, gpio_Toggle from Library-FRDM-KL25Z drivers
 * - PlumaOS_TaskDelay to yield the task for the specified ticks (milliseconds)
 */
void task_led_red(void)
{
    static uint8_t state = 0;

    switch(state)
    {
        case SETUP:
        {
            gpio_Init(LED_RED,OUTPUT,NO_PULL_RESISTOR);
            gpio_Set(LED_RED,OFF);
            state = 1;
        }
        break;
        case LOOP:
        default:
        {
            gpio_Toggle(LED_RED);
            PlumaOS_TaskDelay(IdTaskRed,1000);
        }
        break;
    }
}

/**
 * @brief Green LED task.
 *
 * @details
 * Same structure as task_led_red but with a different blink period.
 */
void task_led_green(void)
{
    static uint8_t state = 0;

    switch(state)
    {
        case SETUP:
        {
            gpio_Init(LED_GREEN,OUTPUT,NO_PULL_RESISTOR);
            gpio_Set(LED_GREEN,OFF);
            state = 1;
        }
        break;
        case LOOP:
        default:
        {
            gpio_Toggle(LED_GREEN);
            PlumaOS_TaskDelay(IdTaskGreen,2000);
        }
        break;
    }
}

/**
 * @brief Blue LED task.
 *
 * @details
 * Same structure as the other LED tasks; initial gpio_Set sets the LED ON
 * to demonstrate different initial conditions. Has the longest blink period.
 */
void task_led_blue(void)
{
    static uint8_t state = 0;

    switch(state)
    {
        case SETUP:
        {
            gpio_Init(LED_BLUE,OUTPUT,NO_PULL_RESISTOR);
            gpio_Set(LED_BLUE,ON);
            state = 1;
        }
        break;
        case LOOP:
        default:
        {
            gpio_Toggle(LED_BLUE);
            PlumaOS_TaskDelay(IdTaskBlue,3000);
        }
        break;
    }
}
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
