/*
 * Copyright 2016-2020 NXP
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
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
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
 * @file    MKL25Z128xxx4_Project.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL25Z4.h"
#include "fsl_debug_console.h"
/* TODO: insert other include files here. */

#include "time.h"
#include "SX1276.h"
#include "MLX90614.h"

/* TODO: insert other definitions and declarations here. */

/*
 * @brief   Application entry point.
 */
int main(void)
{
  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();

//  	/* Init FSL debug console. */
//    BOARD_InitDebugConsole();

    SIM->SCGC5 |= (1 << 9); // Clock on PORT A
    SIM->SCGC5 |= (1 << 10); // Clock on PORT B
    SIM->SCGC5 |= (1 << 11); // Clock on PORT C
    SIM->SCGC5 |= (1 << 12); // Clock on PORT D
    SIM->SCGC5 |= (1 << 13); // Clock on PORT E

    // First Come First Out Task Scheduler
    Scheduler_init();

    // SX1276
    SX1276_init();

    // I2C --- MLX90614 IR thermometer
    MLX90614_init(57600);

    float ambientTemp = 0;
    float objectTemp = 0;

    /* Force the counter to be placed into memory. */
    volatile static int i = 0 ;
    /* Enter an infinite loop, just incrementing a counter. */
    while(1)
    {
     	if ((arrival_task0_flag == 1) && (disable_task0_flag == 0)) // blinking green led to indicate "Operative-System"
     	{
     		uint32_t current_time = ticks;

     		arrival_task0_flag = 0;
     		running_task0_flag = 1;
     		queued_task0_flag = 0;

     		GPIOB->PCOR |= (1 << 19);

     		delay(50);

     		GPIOB->PSOR |= (1 << 19);

     		running_task0_flag = 0;
     		queuedTime_task0 = 0;
     		burstTime_task0 = ticks - current_time;
     	}

     	if ((arrival_task1_flag == 1) && (disable_task1_flag == 0)) // test ADC and DAC
     	{
     		uint32_t current_time = ticks;

     		arrival_task1_flag = 0;
     		running_task1_flag = 1;
     		queued_task1_flag = 0;


     		running_task1_flag = 0;
     		queuedTime_task1 = 0;
     		burstTime_task1 = ticks - current_time;
        }

     	if (arrival_task2_flag == 1 && (disable_task2_flag == 0)) // test MLX90614
     	{
     		uint32_t current_time = ticks;

     		arrival_task2_flag = 0;
     		running_task2_flag = 1;
     		queued_task2_flag = 0;

     		ambientTemp = MLX90614_getTemp(MLX90614_SCALE_CELSIUS, MLX90614_REGISTER_LINEARIZED_TAMBIENT);

     		uint8_t temp = (uint8_t)(ambientTemp);

     		SX1276_send(&temp, 1);

     		running_task2_flag = 0;
     		queuedTime_task2 = 0;
     		burstTime_task2 = ticks - current_time;
     	}

    	i++ ;
    }

    return 0 ;
}
