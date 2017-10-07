/* Copyright 2017, Gustavo Muro
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

/** \brief PASE APP EXAMPLE
 **
 ** ejemplo de aplicaci√≥n usando CIAA Firmware
 **
 **/

/** \addtogroup
 ** @{ */
/** \addtogroup
 ** @{ */
/** \addtogroup
 ** @{ */

/*==================[inclusions]=============================================*/
#include "os.h"
#include "pase_app_example.h"
#include "bsp.h"

#include <stdio.h>

/**
 *
 * #include "mcu.h"
 */

#include "mcu_pwm.h"
#include "mcu_uart.h"
#include "mcu_gpio.h"

/*==================[macros and definitions]=================================*/
#define FIRST_START_DELAY_MS 350
#define PERIOD_MS 250

#define BAUD_RATE 115200

#define INCREMENTTICKS 0
#define CYCLEVALUE 10


#define PWM_INC 10
#define PWM_MAX_DUTY 100

#define PERIODO 2000
/*==================[internal data declaration]==============================*/

static uint32_t ContadorTiempo;
/* static uint32_t ContadorDutyCiclo; */

uint32_t ContadorDutyCiclo;

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/
/** \brief Main function
 *
 * This is the main entry point of the software.
 *
 * \returns 0
 *
 * \remarks This function never returns. Return value is only to avoid compiler
 *          warnings or errors.
 */
int main(void)
{
   /* Starts the operating system in the Application Mode 1 */
   /* This example has only one Application Mode */
   StartOS(AppMode1);

   /* StartOs shall never returns, but to avoid compiler warnings or errors
    * 0 is returned */
   return 0;
}

/** \brief Error Hook function
 *
 * This function is called from the os if an os interface (API) returns an
 * error. Is for debugging proposes. If called this function triggers a
 * ShutdownOs which ends in a while(1).
 *
 * The values:
 *    OSErrorGetServiceId
 *    OSErrorGetParam1
 *    OSErrorGetParam2
 *    OSErrorGetParam3
 *    OSErrorGetRet
 *
 * will provide you the interface, the input parameters and the returned value.
 * For more details see the OSEK specification:
 * http://portal.osek-vdx.org/files/pdf/specs/os223.pdf
 *
 */
void ErrorHook(void)
{
   ShutdownOS(0);
}

/** \brief Initial task
 *
 * This task is started automatically in the application mode 1.
 */
TASK(InitTask)
{
   /**
    * Inicializa el teclado
    */
   bsp_init();

   /**
    * La alarma ActivateKeyboardTask es activada para expirar por primera vez
    * luego de 10 ticks y luego reiteradamente cada 2 ticks es decir cada 2 ms.
    */
   SetRelAlarm(ActivateKeyboardPollingTask, 10, KEYBOARD_TASK_TIME_MS);


   mcu_pwm_config(2, 1000);
   mcu_pwm_config(5, 1000);
   mcu_pwm_config(4, 1000);

   mcu_uart_init(UART_USB);

   mcu_uart_config(UART_USB, BAUD_RATE);

   ContadorTiempo=0;

    /**
     * Debo iniciar el contador de tiempo que me dar· el TimeStamp
     * SetRelAlarm ( AlarmType <AlarmID>, TickType <increment>, TickType <cycle> )
     *
     * Parameter (In):
     *                AlarmID : Reference to the alarm element
     *                increment : Relative value in ticks
     *                cycle : Cycle value in case of cyclic alarm. In case of single alarms, cycle shall be zero.
     */

   //  SetRelAlarm(ActivateTimeCountTask, INCREMENTTICKS, CYCLEVALUE);
   SetRelAlarm(ActivateTimeCountTask, INCREMENTTICKS, CYCLEVALUE);

   ContadorDutyCiclo = 0;

   mcu_pwm_iniciar();

   TerminateTask();
}

TASK(TimeCountTask){
   ContadorTiempo=ContadorTiempo+1;
   TerminateTask();
}


TASK(Pwm){

	/*
	 *  Incremento el Duty Cycle
	 */


	ContadorDutyCiclo += PWM_INC;

	if (ContadorDutyCiclo > PWM_MAX_DUTY) {
		ContadorDutyCiclo = 0;
			}

	/*
	 * led_color_a_encender=VERDE;
	 */


	mcu_pwm_setDutyCicle((mcu_pwm_pinId_enum) 2, ContadorDutyCiclo);


}


/**
 *
 * Tarea que barre el teclado cada 2ms
 *
 * */
TASK(KeyboardPollingTask){
    bsp_keyboard_task();
    TerminateTask();
}


/**
 *
 * Tarea que analiza la ˙ltima tecla leÌda
 *
 * */
TASK(KeyboardProcessTask)
{

	int32_t tecla;

    static uint8_t mem_status_tec_1 = 0;
    static uint8_t mem_status_tec_2 = 0;

    char msg[50];

	tecla = bsp_keyboardGet();

	/**
	 * BOARD_TEC_ID_1 est· definida en bsp\edu_ciaa_nxp\inc\board.h
	 */
	if (tecla == BOARD_TEC_ID_1)
	{

		/**
		 * Procedo de acuerdo al status memorizado de TEC_1
		 *
		 */

		if (mem_status_tec_1 == 0)
		/**
		 * Est· finalizada la secuencia y se presionÛ TEC_1 ==> debe arrancar la secuencia
		 */
		{
			mem_status_tec_1 = 1;

			sprintf(msg, "TIMESTAMP: Inicio secuencia\n\r");
			uartWriteString(UART_USB, msg);
		}

		else
		/**
		 * Est· iniciada la secuencia y se presionÛ TEC_1 ==> debe finalizar la secuencia
		 */
		{
			mem_status_tec_1 = 0;
		}

	}

	/**
	 * BOARD_TEC_ID_2 est· definida en bsp\edu_ciaa_nxp\inc\board.h
	 */
	if (tecla == BOARD_TEC_ID_2)
	{

		/**
		 * Procedo de acuerdo al status memorizado de TEC_2
		 *
		 */

		if (mem_status_tec_2 == 0)
		/**
		 * Est· reanudada la secuencia y se presionÛ TEC_2 ==> debe pausar la secuencia
		 */
		{
			mem_status_tec_2 = 1;

			sprintf(msg, "TIMESTAMP: Secuencia pausada\n\r");
			uartWriteString(UART_USB, msg);
		}

		else
		/**
		 * Est· pausada la secuencia y se presionÛ TEC_2 ==> debe reanudar la secuencia
		 */
		{
			mem_status_tec_2 = 0;

			sprintf(msg, "TIMESTAMP: Secuencia reanudada\n\r");
			uartWriteString(UART_USB, msg);

		}

	}

    TerminateTask();
}


/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

