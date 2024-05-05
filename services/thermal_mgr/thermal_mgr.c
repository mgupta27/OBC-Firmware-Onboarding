#include "thermal_mgr.h"
#include "errors.h"
#include "lm75bd.h"
#include "console.h"
#include "logging.h"

#include <FreeRTOS.h>
#include <os_task.h>
#include <os_queue.h>

#include <string.h>

#define THERMAL_MGR_STACK_SIZE 256U

static TaskHandle_t thermalMgrTaskHandle;
static StaticTask_t thermalMgrTaskBuffer;
static StackType_t thermalMgrTaskStack[THERMAL_MGR_STACK_SIZE];

#define THERMAL_MGR_QUEUE_LENGTH 10U
#define THERMAL_MGR_QUEUE_ITEM_SIZE sizeof(thermal_mgr_event_t)

static QueueHandle_t thermalMgrQueueHandle;
static StaticQueue_t thermalMgrQueueBuffer;
static uint8_t thermalMgrQueueStorageArea[THERMAL_MGR_QUEUE_LENGTH * THERMAL_MGR_QUEUE_ITEM_SIZE];

static void thermalMgr(void *pvParameters);

void initThermalSystemManager(lm75bd_config_t *config) {
  memset(&thermalMgrTaskBuffer, 0, sizeof(thermalMgrTaskBuffer));
  memset(thermalMgrTaskStack, 0, sizeof(thermalMgrTaskStack));
  
  thermalMgrTaskHandle = xTaskCreateStatic(
    thermalMgr, "thermalMgr", THERMAL_MGR_STACK_SIZE,
    config, 1, thermalMgrTaskStack, &thermalMgrTaskBuffer);

  memset(&thermalMgrQueueBuffer, 0, sizeof(thermalMgrQueueBuffer));
  memset(thermalMgrQueueStorageArea, 0, sizeof(thermalMgrQueueStorageArea));

  thermalMgrQueueHandle = xQueueCreateStatic(
    THERMAL_MGR_QUEUE_LENGTH, THERMAL_MGR_QUEUE_ITEM_SIZE,
    thermalMgrQueueStorageArea, &thermalMgrQueueBuffer);
}

error_code_t thermalMgrSendEvent(thermal_mgr_event_t *event) {
  /* Check if the event is a valid argument */
  if (event == NULL) {
    return ERR_CODE_INVALID_ARG;
  }

  /* Send an event to the thermal manager queue */
  if (xQueueSend(thermalMgrQueueHandle, event, 0) != pdTRUE) {
    return ERR_CODE_INVALID_QUEUE_MSG;
  }

  return ERR_CODE_SUCCESS;
}

void osHandlerLM75BD(void) {
  /* Send an event to the thermal manager queue */
  thermal_mgr_event_t event;
  event.type = THERMAL_MGR_EVENT_OS_TRIGGERED;

  thermalMgrSendEvent(&event);
}

static void thermalMgr(void *pvParameters) {
  error_code_t errCode;
  thermal_mgr_event_t event;
  float temperature;

  while (1) {
    if (xQueueReceive(thermalMgrQueueHandle, &event, 0) != pdTRUE) {
      continue;
    }
      
    if (event.type == THERMAL_MGR_EVENT_MEASURE_TEMP_CMD) { 
      LOG_IF_ERROR_CODE(readTempLM75BD(LM75BD_OBC_I2C_ADDR, &temperature));
      
      if (errCode == ERR_CODE_SUCCESS) {
        addTemperatureTelemetry(temperature);
      }
    } 
    else if (event.type == THERMAL_MGR_EVENT_OS_TRIGGERED) {
      LOG_IF_ERROR_CODE(readTempLM75BD(LM75BD_OBC_I2C_ADDR, &temperature));

      if (errCode == ERR_CODE_SUCCESS) {
        if (temperature > LM75BD_DEFAULT_HYST_THRESH) {
          overTemperatureDetected();
        } else {
          safeOperatingConditions();
        }
      }
    }
    else {
      LOG_ERROR("Invalid event from queue.");
    }
  }
}

void addTemperatureTelemetry(float tempC) {
  printConsole("Temperature telemetry: %f deg C\n", tempC);
}

void overTemperatureDetected(void) {
  printConsole("Over temperature detected!\n");
}

void safeOperatingConditions(void) { 
  printConsole("Returned to safe operating conditions!\n");
}
