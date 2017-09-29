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
 ** ejemplo de aplicaciÃ³n usando CIAA Firmware
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
#include "mcu_pwm.h"

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

static led_color led_color_a_encender;

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/
static void eventInput1_callBack(mcu_gpio_pinId_enum id, mcu_gpio_eventTypeInput_enum evType)
{
	SetEvent(TareaInicioFin, eventoInicioFin);
}

static void eventInput2_callBack(mcu_gpio_pinId_enum id, mcu_gpio_eventTypeInput_enum evType)
{
	SetEvent(TareaPausaReinicia, eventoPausaReinicia);
}

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
   bsp_init();


   /**
    * Setea la tecla (TEC_1) , tipo de evento : Flanco Descendente y
    * la funcion de CallBack que será ejecutada
    */
   mcu_gpio_setEventInput(MCU_GPIO_PIN_ID_38,
         MCU_GPIO_EVENT_TYPE_INPUT_FALLING_EDGE,
         eventInput1_callBack);

   /**
    * Setea la tecla (TEC_2) , tipo de evento : Flanco Ascendente y
    * la funcion de CallBack que será ejecutada
    */
   mcu_gpio_setEventInput(MCU_GPIO_PIN_ID_42,
         MCU_GPIO_EVENT_TYPE_INPUT_RISING_EDGE,
         eventInput2_callBack);

   mcu_pwm_config(2, 1000);
   mcu_pwm_config(5, 1000);
   mcu_pwm_config(4, 1000);

   mcu_uart_init(UART_USB);

   mcu_uart_config(UART_USB, BAUD_RATE);

   ContadorTiempo=0;

    /**
     * Debo iniciar el contador de tiempo que me dará el TimeStamp
     * SetRelAlarm ( AlarmType <AlarmID>, TickType <increment>, TickType <cycle> )
     *
     * Parameter (In):
     *                AlarmID : Reference to the alarm element
     *                increment : Relative value in ticks
     *                cycle : Cycle value in case of cyclic alarm. In case of single alarms, cycle shall be zero.
     */

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


/*
 * TASK(InicioFin){
 *
 *	SetRelAlarm(ActivatePwm, 0, PERIODO);
 * }
 */


TASK(TareaInicioFin)
{
/*
 * La Tarea TareaInicioFin espera por un evento : "WaitEvent(eventoInicioFin)".
 * eventInput1_callBack setea este evento para TareaInicioFin. El scheduler es activado.
 * Por lo tanto TareaInicioFin es transferida del estado "waiting" al estado "ready".
 * Debido a la mayor prioridad de TareaInicioFin resulta en un cambio de tarea y TareaInicioFin
 * tiene preferencia.
 * La tarea TareaInicioFin resetea el evento : "ClearEvent(eventoInicioFin)"
 * Por lo tanto TareaInicioFin queda en waiting nuevamente a la espera de este evento nuevamente
 * y el scheduler continua con la ejecución de otra tarea
 *
 */
	WaitEvent(eventoInicioFin);
	ClearEvent(eventoInicioFin);

    bsp_ledAction(BOARD_LED_ID_1, BSP_LED_ACTION_TOGGLE);

	WaitEvent(eventoInicioFin);
}

TASK(TareaPausaReinicia)
{
	/*
	 * La Tarea TareaPausaReinicia espera por un evento : "WaitEvent(eventoPausaReinicia)".
	 * eventInput2_callBack setea este evento para TareaPausaReinicia. El scheduler es activado.
	 * Por lo tanto TareaPausaReinicia es transferida del estado "waiting" al estado "ready".
	 * Debido a la mayor prioridad de TareaPausaReinicia resulta en un cambio de tarea y TareaPausaReinicia
	 * tiene preferencia.
	 * La tarea TareaPausaReinicia resetea el evento : "ClearEvent(eventoPausaReinicia)"
	 * Por lo tanto TareaPausaReinicia queda en waiting nuevamente a la espera de este evento nuevamente
	 * y el scheduler continua con la ejecución de otra tarea
	 *
	 */

	WaitEvent(TareaPausaReinicia);
	ClearEvent(TareaPausaReinicia);

    bsp_ledAction(BOARD_LED_ID_2, BSP_LED_ACTION_TOGGLE);


	WaitEvent(eventoInicioFin);
}


/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

