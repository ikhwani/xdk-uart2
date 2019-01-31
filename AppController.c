/*
* Licensee agrees that the example code provided to Licensee has been developed and released by Bosch solely as an example to be used as a potential reference for application development by Licensee. 
* Fitness and suitability of the example code for any use within application developed by Licensee need to be verified by Licensee on its own authority by taking appropriate state of the art actions and measures (e.g. by means of quality assurance measures).
* Licensee shall be responsible for conducting the development of its applications as well as integration of parts of the example code into such applications, taking into account the state of the art of technology and any statutory regulations and provisions applicable for such applications. Compliance with the functional system requirements and testing there of (including validation of information/data security aspects and functional safety) and release shall be solely incumbent upon Licensee. 
* For the avoidance of doubt, Licensee shall be responsible and fully liable for the applications and any distribution of such applications into the market.
* 
* 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions are 
* met:
* 
*     (1) Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer. 
* 
*     (2) Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in
*     the documentation and/or other materials provided with the
*     distribution.  
*     
*     (3)The name of the author may not be used to
*     endorse or promote products derived from this software without
*     specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR 
*  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
*  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
*  DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
*  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
*  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
*  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
*  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
*  IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
*  POSSIBILITY OF SUCH DAMAGE.
*/
/*----------------------------------------------------------------------------*/
/**
 * @ingroup APPS_LIST
 *
 * @defgroup XDK_APPLICATION_TEMPLATE XdkApplicationTemplate
 * @{
 *
 * @brief XDK Application Template
 *
 * @details XDK Application Template without any functionality.
 * Could be used as a starting point to develop new application based on XDK platform.
 *
 * @file
 **/
/* module includes ********************************************************** */

/* own header files */
#include "XdkAppInfo.h"
#undef BCDS_MODULE_ID  /* Module ID define before including Basics package*/
#define BCDS_MODULE_ID XDK_APP_MODULE_ID_APP_CONTROLLER

/* own header files */
#include "AppController.h"
#include "BSP_ExtensionPort_2.h"
#include "BCDS_UARTTransceiver.h"
#include "BCDS_MCU_UART.h"

/* system header files */
#include <stdio.h>

/* additional interface header files */
#include "BCDS_CmdProcessor.h"
#include "XDK_Utils.h"
#include "FreeRTOS.h"
#include "task.h"

/* constant definitions ***************************************************** */
#define SECONDS(x)				((portTickType) (x * 1000) / portTICK_RATE_MS)

/* local variables ********************************************************** */

static CmdProcessor_T * AppCmdProcessor;/**< Handle to store the main Command processor handle to be used by run-time event driven threads */

static xTaskHandle AppControllerHandle = NULL;/**< OS thread handle for Application controller to be used by run-time blocking threads */

/* global variables ********************************************************* */

static uint8_t UartRingBuffer[MAX_UART_RX_BUFFERSIZE];
static UARTTransceiver_T UartTransceiverInstance;

static uint8_t Uart2RingBuffer[MAX_UART_RX_BUFFERSIZE];
static UARTTransceiver_T Uart2TransceiverInstance;

/* inline functions ********************************************************* */

/* local functions ********************************************************** */

Retcode_T UartDataWrite(uint8_t *writeBuffer, uint8_t writeLength) {
	Retcode_T retVal = RETCODE_OK;
	uint32_t writeTimeout = UINT32_MAX;
	if (NULL == writeBuffer) {
		retVal = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_NULL_POINTER);
	} else {
		retVal = UARTTransceiver_WriteData(&UartTransceiverInstance,
				writeBuffer, writeLength, writeTimeout);
	}
	return retVal;
}

Retcode_T UartDataRead(UARTTransceiver_T *transceiver, uint8_t *readBuffer,
		uint8_t readlength, uint32_t *actualLength) {
	Retcode_T retVal = RETCODE_OK;
	uint32_t readTimeout = UINT32_MAX;
	if (NULL == readBuffer) {
		retVal = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_NULL_POINTER);
	} else {
		retVal = UARTTransceiver_ReadData(transceiver, readBuffer,
				readlength, actualLength, readTimeout);
	}
	return retVal;
}

static void ProcessUartRxDataBinary(void *param1, uint32_t param2) {
	bool *fp = (bool *) param1;
	bool flag = *fp;

	UARTTransceiver_T *transceiver;
	uint8_t *ringBuffer;
	if (flag == true) {
		transceiver = &UartTransceiverInstance;
		ringBuffer = UartRingBuffer;
	} else {
		transceiver = &Uart2TransceiverInstance;
		ringBuffer = Uart2RingBuffer;
	}

	Retcode_T retVal = RETCODE_OK;
	uint8_t RxCommand = 0xFF;
	uint8_t RxCommandIndex = 0xFF;

	uint32_t actualLength = 0;
	uint8_t readlength = 1;

	if (UART_RX_ERROR != param2) {
		retVal = UartDataRead(transceiver, ringBuffer, readlength, &actualLength);
		if ((RETCODE_OK == retVal) && (readlength == actualLength)) {
			RxCommandIndex = readlength - actualLength;
			RxCommand = ringBuffer[RxCommandIndex];

			if (RxCommand == 0xb5) {
				printf("\n");
			}
			printf("%02x", RxCommand);
		}
	}
}
static bool UartFrameEndCheck(uint8_t lastByte){
    BCDS_UNUSED(lastByte);
    return true;
}

static void UartTxRxCallback(struct MCU_UART_Event_S event) {
	Retcode_T retVal = RETCODE_OK;
	bool flag = true;
	if ((event.RxComplete) && (NULL != AppCmdProcessor)) {
		retVal = CmdProcessor_EnqueueFromIsr(AppCmdProcessor,
				ProcessUartRxDataBinary, &flag, UART_RX_SUCCESS);
	} else if (event.RxError) {
		retVal = CmdProcessor_EnqueueFromIsr(AppCmdProcessor,
				ProcessUartRxDataBinary, &flag, UART_RX_ERROR);
	}
	if (RETCODE_OK != retVal) {
		Retcode_RaiseErrorFromIsr(retVal);
	}
}

static void Uart2TxRxCallback(struct MCU_UART_Event_S event) {
	Retcode_T retVal = RETCODE_OK;
	bool flag = false;
	if ((event.RxComplete) && (NULL != AppCmdProcessor)) {
		retVal = CmdProcessor_EnqueueFromIsr(AppCmdProcessor,
				ProcessUartRxDataBinary, &flag, UART_RX_SUCCESS);
	} else if (event.RxError) {
		retVal = CmdProcessor_EnqueueFromIsr(AppCmdProcessor,
				ProcessUartRxDataBinary, &flag, UART_RX_ERROR);
	}
	if (RETCODE_OK != retVal) {
		Retcode_RaiseErrorFromIsr(retVal);
	}
}

void UartDriverCallBack(UART_T uart, struct MCU_UART_Event_S event) {
	Retcode_T retVal = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_INVALID_PARAM);
	if (UartTransceiverInstance.handle == uart) {
		UARTTransceiver_LoopCallback(&UartTransceiverInstance, event);
	} else {
		Retcode_RaiseError(retVal);
	}
}

void Uart2DriverCallBack(UART_T uart, struct MCU_UART_Event_S event) {
	Retcode_T retVal = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_INVALID_PARAM);
	if (Uart2TransceiverInstance.handle == uart) {
		UARTTransceiver_LoopCallback(&Uart2TransceiverInstance, event);
	} else {
		Retcode_RaiseError(retVal);
	}
}

Retcode_T Uart_Init(uint32_t baudRate, uint32_t parity, uint32_t stopbit, bool nr1) {
	Retcode_T retVal = RETCODE_OK;
	enum UARTTransceiver_UartType_E type;
	HWHandle_T UartHandle = NULL;
	uint8_t rxBuffSize;

	bool flag;
	if (nr1) {
		flag = true;
	} else {
		flag = false;
	}

	retVal = BSP_ExtensionPort_ConnectUart(flag);
	if (RETCODE_OK == retVal) {
		retVal = BSP_ExtensionPort_SetUartConfig(BSP_EXTENSIONPORT_UART_PARITY,
				parity, NULL, flag);
	}
	if (RETCODE_OK == retVal) {
		retVal = BSP_ExtensionPort_SetUartConfig(
				BSP_EXTENSIONPORT_UART_BAUDRATE, baudRate, NULL, flag);
	}
	if (RETCODE_OK == retVal) {
		retVal = BSP_ExtensionPort_SetUartConfig(
				BSP_EXTENSIONPORT_UART_STOPBITS, stopbit, NULL, flag);
	}

	if (RETCODE_OK == retVal) {
		/*Get the serial uart  handle */
		UartHandle = BSP_ExtensionPort_GetUartHandle(flag);
	}
	if (NULL == UartHandle) {
		retVal = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_NULL_POINTER);
	} else {
		if (flag == true)
		{
			retVal = MCU_UART_Initialize(UartHandle, UartDriverCallBack);
			if (RETCODE_OK == retVal) {
				type = UART_TRANSCEIVER_UART_TYPE_UART;
				rxBuffSize = (MAX_UART_RX_BUFFERSIZE - 1);
				retVal = UARTTransceiver_Initialize(&UartTransceiverInstance,
						UartHandle, UartRingBuffer, rxBuffSize, type);
			}
			printf("retVal after init UART 1: %04ld\r\n", retVal);
		}
		else
		{
			retVal = MCU_UART_Initialize(UartHandle, Uart2DriverCallBack);
			if (RETCODE_OK == retVal) {
				type = UART_TRANSCEIVER_UART_TYPE_UART;
				rxBuffSize = (MAX_UART_RX_BUFFERSIZE - 1);
				retVal = UARTTransceiver_Initialize(&Uart2TransceiverInstance,
						UartHandle, Uart2RingBuffer, rxBuffSize, type);
			}
			printf("retVal after init UART 2: %04ld\r\n", retVal);
		}
	}

	return retVal;
}

Retcode_T Uart_Enable(bool nr1) {
	Retcode_T retVal = RETCODE_OK;
	if (nr1)
	{
		retVal = BSP_ExtensionPort_EnableUart(true);
		if (RETCODE_OK == retVal) {
			retVal = UARTTransceiver_StartInAsyncMode(&UartTransceiverInstance,
					UartFrameEndCheck, UartTxRxCallback);
		}
		printf("retVal after enable UART 1: %04ld\r\n", retVal);
	}
	else
	{
		retVal = BSP_ExtensionPort_EnableUart(false);
		if (RETCODE_OK == retVal) {
			retVal = UARTTransceiver_StartInAsyncMode(&Uart2TransceiverInstance,
					UartFrameEndCheck, Uart2TxRxCallback);
		}
		printf("retVal after enable UART 2: %04ld\r\n", retVal);
	}
	return retVal;
}


/**
 * @brief Responsible for controlling application control flow.
 * Any application logic which is blocking in nature or fixed time dependent
 * can be placed here.
 *
 * @param[in] pvParameters
 * FreeRTOS task handle. Could be used if more than one thread is using this function block.
 */
static void AppControllerFire(void* pvParameters)
{
    BCDS_UNUSED(pvParameters);

//    uint8_t buffer[] = "Hello";
//    uint32_t bufferLength = sizeof(buffer)/sizeof(uint8_t);
//    UartDataWrite(buffer,bufferLength);

    /* A function that implements a task must not exit or attempt to return to
     its caller function as there is nothing to return to. */
    while (1)
    {
        /* code to implement application control flow */
        vTaskDelay(SECONDS(1));
    }
}

/**
 * @brief To enable the necessary modules for the application
 *
 * @param [in] param1
 * A generic pointer to any context data structure which will be passed to the function when it is invoked by the command processor.
 *
 * @param [in] param2
 * A generic 32 bit value  which will be passed to the function when it is invoked by the command processor..
 */
static void AppControllerEnable(void * param1, uint32_t param2)
{
    BCDS_UNUSED(param1);
    BCDS_UNUSED(param2);
    Retcode_T retcode = RETCODE_OK;

    retcode = Uart_Enable(true); // UART 1
    retcode = Uart_Enable(false); // UART 2
    if (RETCODE_OK == retcode) {
    	printf("Uart Enabling Success \r\n");
    } else {
   		printf("Uart Enabling Failed: %04ld \r\n", retcode);
    }

    if (RETCODE_OK == retcode)
    {
        if (pdPASS != xTaskCreate(AppControllerFire, (const char * const ) "AppController", TASK_STACK_SIZE_APP_CONTROLLER, NULL, TASK_PRIO_APP_CONTROLLER, &AppControllerHandle))
        {
            retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_OUT_OF_RESOURCES);
        }
    }
    if (RETCODE_OK != retcode)
    {
        printf("AppControllerEnable : Failed \r\n");
        Retcode_RaiseError(retcode);
        assert(0); /* To provide LED indication for the user */
    }
    Utils_PrintResetCause();
}

/**
 * @brief To setup the necessary modules for the application
 *
 * @param [in] param1
 * A generic pointer to any context data structure which will be passed to the function when it is invoked by the command processor.
 *
 * @param [in] param2
 * A generic 32 bit value  which will be passed to the function when it is invoked by the command processor..
 */
static void AppControllerSetup(void * param1, uint32_t param2)
{
    BCDS_UNUSED(param1);
    BCDS_UNUSED(param2);
    Retcode_T retcode = RETCODE_OK;

    retcode = BSP_ExtensionPort_Connect();

    // UART 1
    retcode = Uart_Init(UART_BAUDRATE,
    		(uint32_t) BSP_EXTENSIONPORT_UART_NO_PARITY,
    		(uint32_t) BSP_EXTENSIONPORT_UART_STOPBITS_ONE, true);
    // UART 2
    retcode = Uart_Init(UART_BAUDRATE,
    		(uint32_t) BSP_EXTENSIONPORT_UART_NO_PARITY,
    		(uint32_t) BSP_EXTENSIONPORT_UART_STOPBITS_ONE, false);
    if (RETCODE_OK == retcode) {
    	printf("Uart Initialization Success \r\n");
    } else {
   		printf("Uart Initialization Failed: %04ld \r\n", retcode);
    }

    retcode = CmdProcessor_Enqueue(AppCmdProcessor, AppControllerEnable, NULL, UINT32_C(0));
    if (RETCODE_OK != retcode)
    {
        printf("AppControllerSetup : Failed \r\n");
        Retcode_RaiseError(retcode);
        assert(0); /* To provide LED indication for the user */
    }
}

/* global functions ********************************************************* */

/** Refer interface header for description */
void AppController_Init(void * cmdProcessorHandle, uint32_t param2)
{
    BCDS_UNUSED(param2);

    Retcode_T retcode = RETCODE_OK;
    vTaskDelay(SECONDS(1));

    if (cmdProcessorHandle == NULL)
    {
        printf("AppController_Init : Command processor handle is NULL \r\n");
        retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_NULL_POINTER);
    }
    else
    {
        AppCmdProcessor = (CmdProcessor_T *) cmdProcessorHandle;
        retcode = CmdProcessor_Enqueue(AppCmdProcessor, AppControllerSetup, NULL, UINT32_C(0));
    }

    if (RETCODE_OK != retcode)
    {
        Retcode_RaiseError(retcode);
        assert(0); /* To provide LED indication for the user */
    }
}

/**@} */
/** ************************************************************************* */
