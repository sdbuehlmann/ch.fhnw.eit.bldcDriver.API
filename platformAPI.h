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
	phaseA, phaseB, phaseC

} Phase;

typedef enum {
	highside, lowside
} Bridgeside;

typedef enum {
	true, false
} Boolean;

typedef enum {
	rising, falling
} Edge;

typedef enum {
	running, started, error
} ADCStatus;

typedef enum {
	negative_torque, positive_torque
} ControlSignalType;

typedef (*DelayedCallback)(DelayedCallbackHandle handle);
typedef enum {
	registered, error, called
} DelayedCallbackStatus;
typedef struct {
	DelayedCallback callback;
	DelayedCallbackStatus status;
	uint32_t timestampRegistered_us;
	uint32_t timeUntilCallback_us;
} DelayedCallbackHandle;

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
DelayedCallbackHandle startDelayedCallback(uint32_t timeUntilCallback_us);

void wait_ms(uint32_t ms);

//========================= UART ===================================
// ----- to be implemented------------------------------------
uint8_t event_uartDataReceived();
// -----------------------------------------------------------
void sendUartData(uint8_t data);

//====================== PROGRAMM FLOW ============================
// ----- to be implemented------------------------------------
void startup();
void proceed();
void shutdown();
// -----------------------------------------------------------

//========================= ENCODER ==============================
// ----- to be implemented------------------------------------
void event_rotaded180Degrees();
void newData_encoderCalibration(uint32_t encoderCal);
//-----------------------------------------------

void enableEncoder(Boolean enable);
uint32_t getRotadedDegreesEncoder();
void setRotadedDegreesEncoder(uint32_t rotadedDeg);
void resetRotadedDegreesEncoder();

Boolean isEncoderSignalA();
Boolean isEncoderSignalB();
Boolean isEncoderEnabled();
Boolean isCalibrateEncoder();

void setEncoderCalibrationReferencePosition(Boolean state);

#endif /* PLATTFORMAPI_H_ */
