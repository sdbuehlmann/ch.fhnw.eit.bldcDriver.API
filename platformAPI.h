/*
 * plattformAPI.h
 *
 *  Created on: Nov 29, 2017
 *      Author: simon
 */

#ifndef INC_PLATFORMAPI_H_
#define INC_PLATFORMAPI_H_

#include <stdint.h>

#include "platformAPIConfig.h"
// =============== defines ====================================================================================================
#define PLATFORM_ERROR platformError(__FILE__, __LINE__)

// =============== type definitions ===========================================================================================
typedef enum {
	phase_A, phase_B, phase_C

} Phase;

typedef enum {
	bridgeside_highside, bridgeside_lowside
} Bridgeside;

typedef enum {
	false = 0, true = 1
} Boolean;

typedef enum {
	edge_rising, edge_falling
} Edge;

typedef enum {
	adcStatus_running, adcStatus_started, adcStatus_error
} ADCStatus;

typedef enum {
	uartStatus_ok, uartStatus_tooMouchData
} UARTStatus;


typedef enum {
	controlSignalType_negative_torque, controlSignalType_positive_torque
} ControlSignalType;

typedef void (*StartDelayedCallback)(uint32_t timeToCall_us);
typedef void (*DelayedCallback)(void);

typedef enum {
	delayedCallbackStatus_ready, delayedCallbackStatus_running
} DelayedCallbackStatus;

typedef struct {
	DelayedCallback callback;
	StartDelayedCallback start;
	DelayedCallbackStatus status;
	uint32_t timestampRegistered_us;
	uint32_t timeUntilCallback_us;
} DelayedCallbackHandle;

typedef enum {
	delayedCallbackFeedback_registered, delayedCallbackFeedback_error
} DelayedCallbackFeedback;

// =============== functions ==================================================================================================
// ***to be implemented******************************************************
void startup();
void proceed();
// **************************************************************************

// --------------- serializing ------------------------------------------------------------------------------------------------
void entryNoIRQSection();
void leaveNoIRQSection();

// --------------- pwm --------------------------------------------------------------------------------------------------------
#ifdef PWM
void setPWMDutyCycle(uint8_t dutyCycle);

void enablePWM(Phase phase, Bridgeside side);
void disablePWM(Phase phase, Bridgeside side);
#endif /* PWM */

// --------------- adc --------------------------------------------------------------------------------------------------------
#ifdef ADC
// ***to be implemented******************************************************
void newData_currentPhaseA(int32_t current_mA, uint32_t range_mA);
void newData_currentPhaseB(int32_t current_mA, uint32_t range_mA);

void newData_controlSignal(uint32_t controlSignal);
void newData_mainVoltage(uint32_t mainVoltage);
void newData_calibrateEncoder(uint32_t calibrateEncoder);
// **************************************************************************

ADCStatus startMeasCurrentPhaseA();
ADCStatus startMeasCurrentPhaseB();

ADCStatus startMeasControlSignal();
ADCStatus startMeasMainVoltage();
ADCStatus startMeasEncoderCalibration();
#endif /* ADC */

// --------------- comperators ------------------------------------------------------------------------------------------------
#ifdef COMPERATORS
// ***to be implemented******************************************************
void event_comperatorSignalChanged(Phase phase, Edge edge);
// **************************************************************************

void enableComperator(Phase phase, Edge edge, Boolean enable);
Boolean isComperatorSignalHigh(Phase phase);
#endif /* COMPERATORS */

// --------------- gpio's -----------------------------------------------------------------------------------------------------
Boolean isBoardEnabled();
Boolean isNFaultFromBridgeDriver();
Boolean isNOCTWFromBridgeDriver();
Boolean isPWRGDFromBridgeDriver();
Boolean isEncoderEnabled();
Boolean isCalibrateEncoder();

ControlSignalType getControlSignalType();

void setLED(Boolean ledON);
void setEnableBridgeDriver(Boolean enable);
void setDCCalBridgeDriver(Boolean dcCal);

// --------------- systime ----------------------------------------------------------------------------------------------------
#ifdef SYSTIME
uint32_t getSystimeUs();
DelayedCallbackFeedback startDelayedCallback(uint32_t timeUntilCallback_us, DelayedCallback callback);

void wait_ms(uint32_t ms);
#endif /* SYSTIME */

// --------------- uart ------------------------------------------------------------------------------------------------------
#ifdef UART
// ***to be implemented******************************************************
void event_uartDataReceived(uint8_t *pData, uint8_t nrData);
// **************************************************************************

UARTStatus sendUartData(uint8_t *pData, uint8_t size);
#endif /* UART */

// --------------- encoder ----------------------------------------------------------------------------------------------------
#ifdef ENCODER
// ***to be implemented******************************************************
void event_rotaded180Degrees();
void newData_encoderCalibration(uint32_t encoderCal);
// **************************************************************************

void enableEncoder(Boolean enable);
uint32_t getRotadedDegreesEncoder();
void setRotadedDegreesEncoder(uint32_t rotadedDeg);
void resetRotadedDegreesEncoder();

Boolean isEncoderSignalA();
Boolean isEncoderSignalB();

void setEncoderCalibrationReferencePosition(Boolean state);
#endif /* ENCODER */

// --------------- error ----------------------------------------------------------------------------------------------------
// to be implemented:
void platformError(char file[], uint32_t line);
//--------------------

#endif /* PLATTFORMAPI_H_ */
