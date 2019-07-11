/*
 * utils.h
 *
 *  Created on: 17 θών. 2019 γ.
 *      Author: hvunt
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_

enum {
    SM_ERROR = 0xF0,
    SM_CONNECTION_ERROR,
    SM_NOT_RESPOND_ERROR,
	SM_BUSY,
    SM_OK = 0x0F
};

#define VDDA_APPLI			(3300U)
#define ADC_DELAY_CALIB_ENABLE_CPU_CYCLES  (LL_ADC_DELAY_CALIB_ENABLE_ADC_CYCLES * 32)

#endif /* INC_UTILS_H_ */
