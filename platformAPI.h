/*
 * plattformAPI.h
 *
 *  Created on: Nov 29, 2017
 *      Author: simon
 */

#ifndef INC_PLATFORMAPI_H_
#define INC_PLATFORMAPI_H_

#include <stdint.h>

// =============== type definitions ===========================================================================================
typedef enum {
	phase_A, phase_B, phase_C

} Phase;

typedef enum {
	bridgeside_highside, bridgeside_lowside
} Bridgeside;

typedef enum {
	boolean_true, boolean_false
} Boolean;

typedef enum {
	edge_rising, edge_falling
} Edge;

typedef enum {
	adcStatus_running, adcStatus_started, adcStatus_error
} ADCStatus;

typedef enum {
	controlSignalType_negative_torque, controlSignalType_positive_torque
} ControlSignalType;

typedef (*StartDelayedCallback)(uint32_t timeToCall_us);
typedef (*DelayedCallback)(void);
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
// --------------- serializing ------------------------------------------------------------------------------------------------
void entryNonInterruptableSection();
void leaveNonInterruptableSection();

// --------------- pwm --------------------------------------------------------------------------------------------------------
#define MAX_PWM_DUTYCYCLE 255
#define MIN_PWM_DUTYCYCLE 0
void setPWMDutyCycle(uint8_t dutyCycle);

void enablePWM(Phase phase, Bridgeside side);
void disablePWM(Phase phase, Bridgeside side);

// --------------- adc --------------------------------------------------------------------------------------------------------
// to be implemented:
void newData_currentPhaseA(uint32_t nr_measurements, uint32_t *pBuffer);
void newData_currentPhaseB(uint32_t nr_measurements, uint32_t *pBuffer);

void newData_controlSignal(uint32_t controlSignal);
void newData_mainVoltage(uint32_t mainVoltage);
// ------------------

ADCStatus startMeasCurrentPhaseA(uint32_t nr_measurements, uint32_t *pBuffer);
ADCStatus startMeasCurrentPhaseB(uint32_t nr_measurements, uint32_t *pBuffer);

ADCStatus startMeasControlSignal();
ADCStatus startMeasMainVoltage();

// --------------- comperators ------------------------------------------------------------------------------------------------
// to be implemented:
void event_comperatorSignalChanged(Phase phase, Edge edge);
// -------------------
void enableComperator(Phase phase, Edge edge, Boolean enable);

Boolean isComperatorASignalHigh();
Boolean isComperatorBSignalHigh();
Boolean isComperatorCSignalHigh();

// --------------- gpio's -----------------------------------------------------------------------------------------------------
Boolean isBoardEnabled();
Boolean isNFaultFromBridgeDriver();
Boolean isNOCTWFromBridgeDriver();
Boolean isPWRGDFromBridgeDriver();

ControlSignalType getControlSignalType();

void setLED(Boolean ledON);
void setEnableBridgeDriver(Boolean enable);
void setDCCalBridgeDriver(Boolean dcCal);

// --------------- systime ----------------------------------------------------------------------------------------------------
uint32_t getSystimeUs();
DelayedCallbackFeedback startDelayedCallback(uint32_t timeUntilCallback_us);

void wait_ms(uint32_t ms);

// --------------- uart ------------------------------------------------------------------------------------------------------
// to be implemented:
uint8_t event_uartDataReceived();
// ------------------
void sendUartData(uint8_t data);

// --------------- programm flow ----------------------------------------------------------------------------------------------
// to be implemented:
void startup();
void proceed();
// -------------------

// --------------- encoder ----------------------------------------------------------------------------------------------------
// to be implemented:
void event_rotaded180Degrees();
void newData_encoderCalibration(uint32_t encoderCal);
//--------------------

void enableEncoder(Boolean enable);
uint32_t getRotadedDegreesEncoder();
void setRotadedDegreesEncoder(uint32_t rotadedDeg);
void resetRotadedDegreesEncoder();

Boolean isEncoderSignalA();
Boolean isEncoderSignalB();
Boolean isEncoderEnabled();
Boolean isCalibrateEncoder();

void setEncoderCalibrationReferencePosition(Boolean state);

// --------------- error ----------------------------------------------------------------------------------------------------
// to be implemented:
void platformError(char msg[], char file[], char line[]);
//--------------------

#endif /* PLATTFORMAPI_H_ */
