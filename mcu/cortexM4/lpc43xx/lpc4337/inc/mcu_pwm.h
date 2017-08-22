/* Copyright 2017, Gustavo Muro
 * All rights reserved.
 *
 * This file is part of CIAA Firmware.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef MCU_PWM_H
#define MCU_PWM_H

#include "stdint.h"

/*
 * De C:\CIAA\firmware_v2\modules\lpc4337_m4\sapi\src\sapi_pwm.c
 */

#define PWM_TOTALNUMBER   11   /* From PWM0 to PWM10 */

#define PWM_FREC          1000 /* 1Khz */
#define PWM_PERIOD        1000 /* 1000uS = 1ms*/


/*==================[inclusions]=============================================*/
#include "stdbool.h"
#include "stdint.h"

/*==================[cplusplus]==============================================*/
#ifdef __cplusplus
extern "C" {
#endif

/*==================[macros]=================================================*/

/*==================[typedef]================================================*/

/*
 * De C:\CIAA\firmware_v2\modules\lpc4337_m4\sapi\inc\sapi_peripheral_map.h
 * traigo la definicion de pinConfigLpc4337_t
 */
/* ----- Begin Pin Config Structs NXP LPC4337 ----- */

typedef struct{
   int8_t port;
   int8_t pin;
} pinConfigLpc4337_t;


/*
 * Segun : C:\CIAA\firmware_v2\modules\lpc4337_m4\sapi\src\sapi_pwm.c
 *
 * PWM3 --> CTOUT_0
 * PWM0 --> CTOUT_1
 * PWM7 --> CTOUT_2
 * PWM4 --> CTOUT_3
 * PWM9 --> CTOUT_4
 * PWM8 --> CTOUT_5
 * PWM10 --> CTOUT_6
 * PWM6 --> CTOUT_7
 * PWM2 --> CTOUT_10
 * PWM1 --> CTOUT_12
 *  PWM5 --> CTOUT_13
 */

typedef enum {PWM3, PWM0, PWM7, PWM4, PWM9, PWM8,
	          PWM10, PWM6, PWM2=10, PWM1=12, PWM5=13} mcu_gpio_pinId_enum;


/*==================[external data declaration]==============================*/

/*
 * List of ports and pins corresponding to the sct channels.
 * Each channel is asociated with a CTOUT number. Some pins, like
 * LED 1 and LCD1, have the same channel, so you can only generate 1 signal
 * for both. Because of that only one of them will be used.
 */
static pinConfigLpc4337_t SCTdataList[] =
{
/* Sct n° | port | pin | name in board */
/* CTOUT0 */ { 4 , 2 }, /* T_FIL2 */
/* CTOUT1 */ { 4 , 1 }, /* T_FIL1 */
/* CTOUT2 */ { 2 , 10 }, /* LED1 (also for LCD1) */
/* CTOUT3 */ { 4 , 3 }, /* T_FIL3 */
/* CTOUT4 */ { 2 , 12 }, /* LED3 (also for LCD3) */
/* CTOUT5 */ { 2 , 11 }, /* LED2 (also for LCD2) */
/* CTOUT6 */ { 6 , 5 }, /* GPIO2 */
/* CTOUT7 */ { 6 , 12 }, /* GPIO8 */
/* CTOUT8 */ { 1 , 3 }, /* MDC / SPI_MISO */
/* CTOUT9 */ { 1 , 4 }, /* SPI_MOSI */
/* CTOUT10 */ { 1 , 5 }, /* T_COL0 */
/* CTOUT11 */ { 0 , 0 }, /* DO NOT USE */
/* CTOUT12 */ { 7 , 5 }, /* T_COL2 */
/* CTOUT13 */ { 7 , 4 } /* T_COL1 */
};


/* Enter a pwm number, get a sct number
 * Since this module works with pwm numbers, but uses sct channels to generate
 * the signal, its necessary to connect pwm number with the SctMap_t (sAPI_PeripheralMap.h).
 * This way the user sets "pwms", while using the sct peripheral internally*/
static const uint8_t pwmMap[PWM_TOTALNUMBER] = {
   /* PWM0 */  PWM0,  /* T_FIL1 */
   /* PWM1 */  PWM1, /* T_COL2 */
   /* PWM2 */  PWM2, /* T_COL0 */
   /* PWM3 */  PWM3,  /* T_FIL2 */
   /* PWM4 */  PWM4,  /* T_FIL3 */
   /* PWM5 */  PWM5, /* T_COL1 */
   /* PWM6 */  PWM6,  /* GPIO8  */
   /* PWM7 */  PWM7,  /* LED1   */
   /* PWM8 */  PWM8,  /* LED2   */
   /* PWM9 */  PWM9,  /* LED3   */
   /* PWM10 */ PWM10   /* GPIO2  */
};



/*==================[external functions declaration]=========================*/
/**
 ** Inicializacion del Modulo de Soft de PWM
 **/
extern void mcu_pwm_init(void);

/**
 ** Permite Configurar el pin y el período en ms
 **/
extern void mcu_pwm_config(mcu_gpio_pinId_enum pin, uint32_t period);

/**
 ** Modifica la relacion entre el tiempo en que esta en alto el pin y el que esta en bajo
 **/
extern void mcu_pwm_setDutyCicle(mcu_gpio_pinId_enum pin, uint32_t duty);

/*==================[cplusplus]==============================================*/
#ifdef __cplusplus
}
#endif
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
#endif /* #ifndef MCU_PWM_H */

