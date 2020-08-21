#include "vl53l0x_platform.h"
#include <esp_log.h>

esp_err_t read_I2C_Multi(i2c_port_t port, uint8_t dev_address, uint8_t reg_adrs, uint8_t * data, uint32_t cnt);
// Custom wrapper funcations
esp_err_t int_I2C_as_master(VL53L0X_DEV dev, gpio_num_t sda_gpio_pin, gpio_num_t scl_gpio_pin, gpio_pullup_t sda_pullup, 
                         gpio_pullup_t scl_pullup, uint32_t clockspeed, i2c_port_t port, uint8_t address) {
    esp_err_t result;
    dev->Port = port;
    dev->I2cDevAddr = address;
    const i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_gpio_pin,
        .scl_io_num = scl_gpio_pin,
        .sda_pullup_en = sda_pullup,
        .scl_pullup_en = scl_pullup,
        .master.clk_speed = clockspeed,
    };
    result = i2c_param_config(port, &conf);
    if (result == ESP_OK) {
        result = i2c_driver_install(port, I2C_MODE_MASTER, 0, 0, 0);
    }
    return result;
}

i2c_cmd_handle_t I2C_start(uint8_t dev_address, uint8_t reg_adrs) {
    i2c_cmd_handle_t handle = i2c_cmd_link_create(); //Handle is a pointer
    i2c_master_start(handle);
    i2c_master_write_byte(handle, dev_address, true);
    i2c_master_write_byte(handle, reg_adrs, true);
    return handle;
}


i2c_cmd_handle_t I2C_start_read(uint8_t dev_address, uint8_t reg_adrs) {
    i2c_cmd_handle_t handle = i2c_cmd_link_create(); //Handle is a pointer
    i2c_master_start(handle);
    dev_address = dev_address | 1;
    i2c_master_write_byte(handle, dev_address, true);
    return handle;
}


esp_err_t I2C_end(i2c_port_t port, i2c_cmd_handle_t handle) {
    esp_err_t status;
    i2c_master_stop(handle);
    status = i2c_master_cmd_begin(port, handle, 1000 / 300);
    i2c_cmd_link_delete(handle);
    //if (status != ESP_OK) {
        //ESP_LOGI("I2C LIB", "Error %i", status);
    //}
    return status;
}

// Write I2C Master
esp_err_t write_I2C_uint8(i2c_port_t port, uint8_t dev_address, uint8_t reg_adrs, uint8_t data) {
    i2c_cmd_handle_t handle = I2C_start(dev_address, reg_adrs);
    i2c_master_write_byte(handle, data, true);
    return I2C_end(port, handle);
}

esp_err_t write_I2C_uint16(i2c_port_t port, uint8_t dev_address, uint8_t reg_adrs, uint16_t data) {
    i2c_cmd_handle_t handle = I2C_start(dev_address, reg_adrs);
    i2c_master_write_byte(handle, (data >> 8) & 0xFF, true);
    i2c_master_write_byte(handle, data & 0xFF, true);
    return I2C_end(port, handle);
}

esp_err_t write_I2C_uint32(i2c_port_t port, uint8_t dev_address, uint8_t reg_adrs, uint32_t data) {
    i2c_cmd_handle_t handle = I2C_start(dev_address, reg_adrs);
    
    i2c_master_write_byte(handle, (data >> 24) & 0xFF, true);
    i2c_master_write_byte(handle, (data >> 16) & 0xFF, true);
    i2c_master_write_byte(handle, (data >> 8) & 0xFF, true);
    i2c_master_write_byte(handle, data & 0xFF, true);

    return I2C_end(port, handle);
}

esp_err_t write_I2C_Multi(i2c_port_t port, uint8_t dev_address, uint8_t reg_adrs,uint8_t * data, uint32_t cnt) {
    i2c_cmd_handle_t handle = I2C_start(dev_address, reg_adrs);
    i2c_master_write(handle, data, cnt, true);
    return I2C_end(port, handle);
}

// Read I2C Master
esp_err_t read_I2C_uint8(i2c_port_t port, uint8_t dev_address, uint8_t reg_adrs, uint8_t * data) {
    esp_err_t status;
    i2c_cmd_handle_t handle = I2C_start(dev_address, reg_adrs);
    status = I2C_end(port, handle);

    handle = NULL;
    handle = I2C_start_read(dev_address, reg_adrs);
    if (status == ESP_OK) {
        i2c_master_read_byte(handle, data, I2C_MASTER_NACK);
        status = I2C_end(port, handle);
    }
    return status;
}

esp_err_t read_I2C_uint16(i2c_port_t port, uint8_t dev_address, uint8_t reg_adrs, uint8_t * data) {
    esp_err_t status;
    i2c_cmd_handle_t handle = I2C_start(dev_address, reg_adrs);
    status = I2C_end(port, handle);

    handle = NULL;
    handle = I2C_start_read(dev_address, reg_adrs);
    if (status == ESP_OK) {
        i2c_master_read_byte(handle, data + 1, I2C_MASTER_ACK);
        i2c_master_read_byte(handle, data, I2C_MASTER_NACK);
        status = I2C_end(port, handle);
    }
    return status;
}

esp_err_t read_I2C_uint32(i2c_port_t port, uint8_t dev_address, uint8_t reg_adrs, uint8_t * data) {
    esp_err_t status;
    i2c_cmd_handle_t handle = I2C_start(dev_address, reg_adrs);
    status = I2C_end(port, handle);

    handle = NULL;
    handle = I2C_start_read(dev_address, reg_adrs);
    if (status == ESP_OK) {
        i2c_master_read_byte(handle, data + 3, I2C_MASTER_ACK);
        i2c_master_read_byte(handle, data + 2, I2C_MASTER_ACK);
        i2c_master_read_byte(handle, data + 1, I2C_MASTER_ACK);
        i2c_master_read_byte(handle, data, I2C_MASTER_NACK);
        status = I2C_end(port, handle);
    }
    return status;
}

esp_err_t read_I2C_Multi(i2c_port_t port, uint8_t dev_address, uint8_t reg_adrs, uint8_t * data, uint32_t cnt) {
    esp_err_t status;
    i2c_cmd_handle_t handle = I2C_start(dev_address, reg_adrs);
    status = I2C_end(port, handle);

    handle = NULL;
    handle = I2C_start_read(dev_address, reg_adrs);
    if (status == ESP_OK) {
        
        --cnt;
        for (size_t x = 0; x < cnt; ++x) {
            i2c_master_read_byte(handle, data + x, I2C_MASTER_ACK);
        }
        i2c_master_read_byte(handle, data + cnt, I2C_MASTER_NACK);
        status = I2C_end(port, handle);
    }
    return status;
}
// platform funcations
VL53L0X_Error VL53L0X_LockSequenceAccess(VL53L0X_DEV Dev){
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    return Status;
}

VL53L0X_Error VL53L0X_UnlockSequenceAccess(VL53L0X_DEV Dev){
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    return Status;
}

/**
 * Writes the supplied byte buffer to the device
 * @param   Dev       Device Handle
 * @param   index     The register index
 * @param   pdata     Pointer to uint8_t buffer containing the data to be written
 * @param   count     Number of bytes in the supplied byte buffer
 * @return  VL53L0X_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L0X_Error
 */
VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count) {

    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

	if (write_I2C_Multi((i2c_port_t)Dev->Port, Dev->I2cDevAddr, index, pdata, count) != ESP_OK) {
		Status = VL53L0X_ERROR_CONTROL_INTERFACE;
    }

    return Status;
}

/**
 * Reads the requested number of bytes from the device
 * @param   Dev       Device Handle
 * @param   index     The register index
 * @param   pdata     Pointer to the uint8_t buffer to store read data
 * @param   count     Number of uint8_t's to read
 * @return  VL53L0X_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L0X_Error
 */
VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count){
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

	if (read_I2C_Multi((i2c_port_t)Dev->Port, Dev->I2cDevAddr, index, pdata, count) != ESP_OK) {
		Status = VL53L0X_ERROR_CONTROL_INTERFACE;
    }

    return Status;
}

/**
 * Write single byte register
 * @param   Dev       Device Handle
 * @param   index     The register index
 * @param   data      8 bit register data
 * @return  VL53L0X_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L0X_Error
 */
VL53L0X_Error VL53L0X_WrByte(VL53L0X_DEV Dev, uint8_t index, uint8_t data) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

	if (write_I2C_uint8((i2c_port_t)Dev->Port, Dev->I2cDevAddr, index, data) != ESP_OK) {
		Status = VL53L0X_ERROR_CONTROL_INTERFACE;
    }

    return Status;
}

/**
 * Write word register
 * @param   Dev       Device Handle
 * @param   index     The register index
 * @param   data      16 bit register data
 * @return  VL53L0X_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L0X_Error
 */
VL53L0X_Error VL53L0X_WrWord(VL53L0X_DEV Dev, uint8_t index, uint16_t data) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

	if (write_I2C_uint16((i2c_port_t)Dev->Port, Dev->I2cDevAddr, index, data) != ESP_OK) {
		Status = VL53L0X_ERROR_CONTROL_INTERFACE;
    }

    return Status;
}

/**
 * Write double word (4 byte) register
 * @param   Dev       Device Handle
 * @param   index     The register index
 * @param   data      32 bit register data
 * @return  VL53L0X_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L0X_Error
 */
VL53L0X_Error VL53L0X_WrDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t data){
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

	if (write_I2C_uint32((i2c_port_t)Dev->Port, Dev->I2cDevAddr, index, data) != ESP_OK) {
		Status = VL53L0X_ERROR_CONTROL_INTERFACE;
    }

    return Status;
}

/**
 * Read single byte register
 * @param   Dev       Device Handle
 * @param   index     The register index
 * @param   data      pointer to 8 bit data
 * @return  VL53L0X_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L0X_Error
 */
VL53L0X_Error VL53L0X_RdByte(VL53L0X_DEV Dev, uint8_t index, uint8_t *data){
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

	if (read_I2C_uint8((i2c_port_t)Dev->Port, Dev->I2cDevAddr, index, data) != ESP_OK) {
		Status = VL53L0X_ERROR_CONTROL_INTERFACE;
    }

    return Status;
}

/**
 * Read word (2byte) register
 * @param   Dev       Device Handle
 * @param   index     The register index
 * @param   data      pointer to 16 bit data
 * @return  VL53L0X_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L0X_Error
 */
VL53L0X_Error VL53L0X_RdWord(VL53L0X_DEV Dev, uint8_t index, uint16_t *data){
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

	if (read_I2C_uint16((i2c_port_t)Dev->Port, Dev->I2cDevAddr, index, (uint8_t *)data) != ESP_OK) {
		Status = VL53L0X_ERROR_CONTROL_INTERFACE;
    }

    return Status;
}

/**
 * Read dword (4byte) register
 * @param   Dev       Device Handle
 * @param   index     The register index
 * @param   data      pointer to 32 bit data
 * @return  VL53L0X_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L0X_Error
 */
VL53L0X_Error VL53L0X_RdDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t *data){
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

	if (read_I2C_uint32((i2c_port_t)Dev->Port, Dev->I2cDevAddr, index, (uint8_t *)data) != ESP_OK) {
		Status = VL53L0X_ERROR_CONTROL_INTERFACE;
    }

    return Status;
}

/**
 * Threat safe Update (read/modify/write) single byte register
 *
 * Final_reg = (Initial_reg & and_data) |or_data
 *
 * @param   Dev        Device Handle
 * @param   index      The register index
 * @param   AndData    8 bit and data
 * @param   OrData     8 bit or data
 * @return  VL53L0X_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L0X_Error
 */
VL53L0X_Error VL53L0X_UpdateByte(VL53L0X_DEV Dev, uint8_t index, uint8_t AndData, uint8_t OrData){
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t data;

    if (read_I2C_uint8((i2c_port_t)Dev->Port, Dev->I2cDevAddr, index, &data) != ESP_OK) {
        Status = VL53L0X_ERROR_CONTROL_INTERFACE;
    } else {
        data = (data & AndData) | OrData;
        if (write_I2C_uint8((i2c_port_t)Dev->Port, Dev->I2cDevAddr, index, data) != ESP_OK) {
            Status = VL53L0X_ERROR_CONTROL_INTERFACE;
        }
    }

    return Status;
}

/** @} end of VL53L0X_registerAccess_group */


/**
 * @brief execute delay in all polling API call
 *
 * A typical multi-thread or RTOs implementation is to sleep the task for some 5ms (with 100Hz max rate faster polling is not needed)
 * if nothing specific is need you can define it as an empty/void macro
 * @code
 * #define VL53L0X_PollingDelay(...) (void)0
 * @endcode
 * @param Dev       Device Handle
 * @return  VL53L0X_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L0X_Error
 */
VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV Dev) { /* usually best implemented as a real function */
    return VL53L0X_ERROR_NONE;
}

/** @} end of VL53L0X_platform_group */