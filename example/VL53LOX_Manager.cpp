#include "VL53LOX_Manager.h"

#include <esp_log.h>
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"


static VL53L0X_Dev_t i2cHandle = {};
static VL53L0X_DEV i2cHPtr = &i2cHandle;
static VL53L0X_Error Status = VL53L0X_ERROR_NONE;
intr_handle_t h_irq = nullptr;
VL53L0X_RangingMeasurementData_t RangingMeasurementData = {};


VL53L0X_Error WaitMeasurementDataReady(VL53L0X_DEV Dev) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t NewDatReady=0;
    uint32_t LoopNb;

    // Wait until it finished
    // use timeout to avoid deadlock
    if (Status == VL53L0X_ERROR_NONE) {
        LoopNb = 0;
        do {
            Status = VL53L0X_GetMeasurementDataReady(Dev, &NewDatReady);
            if ((NewDatReady == 0x01) || Status != VL53L0X_ERROR_NONE) {
                break;
            }
            LoopNb = LoopNb + 1;
            VL53L0X_PollingDelay(Dev);
        } while (LoopNb < VL53L0X_DEFAULT_MAX_LOOP);

        if (LoopNb >= VL53L0X_DEFAULT_MAX_LOOP) {
            Status = VL53L0X_ERROR_TIME_OUT;
        }
    }

    return Status;
}

void sensorTask(void *arg) {
    ESP_LOGI("TASK", "sensor_T STARTED");
    while(1) {
        if(gpio_get_level(GPIO_NUM_18) > 0) {
            tick = (double)xTaskGetTickCount();
            sec =  tick / configTICK_RATE_HZ;
            VL53L0X_GetRangingMeasurementData(i2cHPtr, &RangingMeasurementData);
            VL53L0X_ClearInterruptMask(i2cHPtr, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
            ESP_LOGI("MesureMent", "D: %i", RangingMeasurementData.RangeMilliMeter);
        }
        //vTaskDelay(1);
    }
}

bool s_task() {
    gpio_pad_select_gpio(GPIO_NUM_18);
    gpio_set_direction(GPIO_NUM_18, GPIO_MODE_INPUT);
    xTaskCreatePinnedToCore(sensorTask, "sensor_T", 4*1024, NULL, 1, NULL, 1 );
    return true;
}

bool Init_VL53L0X() {
    VL53L0X_RangingMeasurementData_t RangingMeasurementData = {};
    VL53L0X_RangingMeasurementData_t *pRangingMeasurementData = &RangingMeasurementData;
    Status = VL53L0X_ERROR_NONE;
    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;
    VL53L0X_DeviceInfo_t devInfo = {}; 
    

    if (int_I2C_as_master(i2cHPtr, GPIO_NUM_21, GPIO_NUM_22, GPIO_PULLUP_ENABLE, GPIO_PULLUP_ENABLE, 400000, I2C_NUM_0, 0x52) == ESP_OK) {
        ESP_LOGI("I2C LOG", "Init i2c passed");

        if( VL53L0X_DataInit(i2cHPtr) == VL53L0X_ERROR_NONE) {
            ESP_LOGI("I2C", "Init passed");
            
            //g_VL53L0X_devInfor(&devInfo); // removed after inital test
            //ESP_LOGI("53 LOG", "S 0 %i", devInfo.ProductType);

            if(Status == VL53L0X_ERROR_NONE)
            {
                Status = VL53L0X_StaticInit(i2cHPtr); // Device Initialization
            }
            ESP_LOGI("53 LOG", "S 1 %i", Status);
            
            if(Status == VL53L0X_ERROR_NONE)
            {
                Status = VL53L0X_PerformRefCalibration(i2cHPtr, &VhvSettings, &PhaseCal); // Device Initialization
            }
            ESP_LOGI("53 LOG", "S 2 %i", Status);
            if(Status == VL53L0X_ERROR_NONE)
            {
                Status = VL53L0X_PerformRefSpadManagement(i2cHPtr, &refSpadCount, &isApertureSpads); // Device Initialization
            }

            ESP_LOGI("53 LOG", "S 3 %i", Status);
            if(Status == VL53L0X_ERROR_NONE)
            {
                Status = VL53L0X_SetDeviceMode(i2cHPtr, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING); // Setup RANGING MODE
            }

            Status = VL53L0X_SetGpioConfig(i2cHPtr, 0, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING, VL53L0X_GPIOFUNCTIONALITY_NEW_MEASURE_READY, VL53L0X_INTERRUPTPOLARITY_HIGH);
            vTaskDelay( 500 / portTICK_PERIOD_MS ); // For testing, clear gap on scope between init and run
            

            if(Status == VL53L0X_ERROR_NONE)
            {
                Status = VL53L0X_StartMeasurement(i2cHPtr);
            }

            s_task();

        }
    } else {
        ESP_LOGI("I2C LOG", "Init i2c failed");
    }
    return Status == VL53L0X_ERROR_NONE;
}

/*
bool g_VL53L0X_range(VL53L0X_RangingMeasurementData_t * RangingMeasurementData) {
    if (Status == VL53L0X_ERROR_NONE) {
        ESP_LOGI("I2C", "passed %i", VL53L0X_PerformSingleRangingMeasurement(i2cHPtr, RangingMeasurementData));
        ESP_LOGI("I2C", "range %i", RangingMeasurementData->RangeMilliMeter);
    }
    return Status == VL53L0X_ERROR_NONE;
}

bool g_VL53L0X_devInfor(VL53L0X_DeviceInfo_t * devInfo) {
    return VL53L0X_GetDeviceInfo(i2cHPtr, devInfo) == VL53L0X_ERROR_NONE;
}
*/


