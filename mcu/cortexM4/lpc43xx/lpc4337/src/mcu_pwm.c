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

#include "mcu_pwm.h"
#include "chip.h"

/*
 * Se consultó documento : How to use SCT (State Configuration Timer) as a standard PWM using LPCOpen?
 * https://community.nxp.com/thread/388765
 *
 */

extern void mcu_pwn_init(void){
	Chip_SCTPWM_Init(LPC_SCT);
}


/*
 * Se consultó documento : How to use SCT (State Configuration Timer) as a standard PWM using LPCOpen?
 * https://community.nxp.com/thread/388765
 *
 * 1. Initialize the SCT using Chip_SCTPWM_Init()
 * 2. Set up the frequency/rate of SCT using Chip_SCTPWM_SetRate()
 * 3. Configure PinMUX for the pin (SCT_OUTx) [x is the SCT_OUT pin number, see Chip User Manual SCT section for more information]
 * 4. Assign a PWM output pin to a channel using Chip_SCTPWM_SetOutPin()
 *
 */

extern void mcu_pwm_config(mcu_pwm_pinId_enum pin, uint32_t period){
	Chip_SCTPWM_Init(LPC_SCT);
	Chip_SCTPWM_SetRate(LPC_SCT, period);
	Chip_SCU_PinMux(SCTdataList[pin].port, SCTdataList[pin].pin, SCU_MODE_FUNC1, FUNC1);
	Chip_SCTPWM_SetOutPin(LPC_SCT, pin+1, pin);
}

/*
 * Se consultó documento : How to use SCT (State Configuration Timer) as a standard PWM using LPCOpen?
 * https://community.nxp.com/thread/388765
 *
 * 8. Change the duty-cycle as required using Chip_SCTPWM_SetDutyCycle()
 *
 */

extern void mcu_pwm_setDutyCicle(mcu_pwm_pinId_enum pin, uint32_t duty){

	Chip_SCTPWM_SetDutyCycle(LPC_SCT, pin + 1, Chip_SCTPWM_PercentageToTicks(LPC_SCT, duty));
}

extern void mcu_pwm_iniciar(void){
	/**
	 ** Firmware\externals\drivers\cortexM4\lpc43xx\inc\sct_pwm_18xx_43xx.h
	 **/
	Chip_SCTPWM_Start(LPC_SCT);
	}

