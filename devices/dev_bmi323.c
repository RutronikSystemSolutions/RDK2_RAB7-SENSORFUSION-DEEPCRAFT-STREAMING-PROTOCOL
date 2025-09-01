/*
 * dev_bmi323.c
 *
 *  Created on: Aug 22, 2025
 *      Author: ROJ030
 */

#include "dev_bmi323.h"
#include "bmi323/bmi323.h"
#include "bmi323/bmi_driver.h"

#include <cyhal.h>
#include <cybsp.h>

#include "tensor_streaming_protocol/protocol.h"
#include "tensor_streaming_protocol/pb_encode.h"

#include "clock.h"

#include <stdio.h>
/**
 * Options available
 */
#define DEV_BMI323_OPTION_KEY_RATE         (1)
#define DEV_BMI323_OPTION_KEY_ACCEL_RANGE  (2)
#define DEV_BMI323_OPTION_KEY_GYRO_RANGE   (3)
#define DEV_BMI323_OPTION_KEY_STREAM_MODE  (4)

/* At 400Hz/8 = 50 Hz chunk frequency */
/* Increase this to 16 if 800 Hz mode is enabled */
#define MAX_FRAMES_IN_CHUNK   (8)

/* X Y Z */
#define AXIS_COUNT            (3)

/* Accel, Gyro */
#define SENSOR_COUNT          (2)

/* Earth's gravity in m/s^2 */
#define GRAVITY_EARTH  (9.80665f)

/*******************************************************************************
* Types
*******************************************************************************/

typedef enum {
	/* Both accelerometer and gyroscope are in the same stream.
	 * The shape of each frame will be [2,3] as
	 * Stream 0: {{ACCEL_X, ACCEL_Y, ACCEL_Z}, {GYRO_X, GYRO_Y, GYRO_Z}} */
	BMI323_MODE_STREAM_COMBINED = 0,

    /* The accelerometer and gyroscope are split in two separate streams
     * each with the shape [3] as
     * Stream 0: {ACCEL_X, ACCEL_Y, ACCEL_Z}
     * Stream 1: {GYRO_X, GYRO_Y, GYRO_Z}
     */
    BMI323_MODE_STREAM_BOTH = 3,

    /* Only the accelerometer is enabled.
     * Stream 0: {ACCEL_X, ACCEL_Y, ACCEL_Z}
     */
    BMI323_MODE_STREAM_ONLY_ACCEL = 1,

    /* Only the gyroscope is enabled.
     * Stream 0: {GYRO_X, GYRO_Y, GYRO_Z}
     */
    BMI323_MODE_STREAM_ONLY_GYRO = 2,
} stream_mode_t;

typedef struct {
    /* Hardware device */
    struct bmi3_dev sensor;

    /* Accelerator range in G. Valid values are: 2,4,8,16 */
    float accel_range;

    /* Gyroscope range in DPS. Valid values are: 125,250,500,1000,2000 */
    float gyro_range;

    /* Tick of last sample */
    clock_tick_t sample_time_tick;

    /* Used to ensure the first read */
    bool first_sample;

    /* The sample period in ticks */
    uint32_t period_tick;

    union
	{
    	struct {
			/* Converted data as meter per second squared */
			float accel_data[MAX_FRAMES_IN_CHUNK * AXIS_COUNT];

			/* Converted data as degrees per second*/
			float gyro_data[MAX_FRAMES_IN_CHUNK * AXIS_COUNT];
    	};

    	// For mode combined
    	float data_combined[MAX_FRAMES_IN_CHUNK * SENSOR_COUNT * AXIS_COUNT];
	};

    /* Number of frames collected in accel_data and gyro_data. */
    /* Cleared after each sent data-chunk. Equal or less than frames_in_chunk */
    int frames_sampled;

    /* Max number of frames in each chunk. Is less or equal to MAX_FRAMES_IN_CHUNK*/
    int frames_target;

    /* Number of frames dropped. This is cleared each data-chunk. */
    int frames_dropped;

    /* How streams are presented. */
    stream_mode_t stream_mode;
} dev_bmi323_t;

//static struct bmi3_dev bmi323 = { 0 };
_Bool bmi323_int_flag = false;

#define BMI323_IRQ_PRIORITY		5

static void bmi323_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event);
cyhal_gpio_callback_data_t bmi323_int_data =
{
		.callback = bmi323_interrupt_handler,
		.callback_arg = NULL,

};

static const struct bmi3_int_pin_config int_config =
{
		.pin_type = BMI3_INT1,
		.int_latch = BMI3_INT_LATCH_DISABLE,
		.pin_cfg[0].lvl = BMI3_INT_ACTIVE_HIGH,
		.pin_cfg[0].od = BMI3_INT_PUSH_PULL,
		.pin_cfg[0].output_en = BMI3_INT_OUTPUT_ENABLE,
		.pin_cfg[1].lvl = BMI3_INT_ACTIVE_LOW,
		.pin_cfg[1].od = BMI3_INT_PUSH_PULL,
		.pin_cfg[1].output_en = BMI3_INT_OUTPUT_DISABLE,
};

static void bmi323_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event)
{
	CY_UNUSED_PARAMETER(handler_arg);
    CY_UNUSED_PARAMETER(event);

    /*Set the interrupt global flag*/
    bmi323_int_flag = true;
}

/******************************************************************************
* Function Name: _lsb_to_mps2
********************************************************************************
* Summary:
*   This function converts lsb to meter per second squared for 16 bit accelerometer
*   at range 2G, 4G, 8G or 16G.
*/
static float _lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (GRAVITY_EARTH * val * g_range) / half_scale;
}

/*******************************************************************************
* Function Name: _lsb_to_dps
********************************************************************************
* Summary:
*   This function converts lsb to degree per second for 16 bit gyro at
*   range 125, 250, 500, 1000 or 2000dps.
*******************************************************************************/
static float _lsb_to_dps(int16_t val, float dps, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (dps / (half_scale)) * (val);
//    float tmps = (dps / (half_scale)) * (val);
//
//    tmps = tmps / 100.f;
//    return tmps;
}

static bool _config_hw(dev_bmi323_t* dev, int rate, int accel_range, int gyro_range, stream_mode_t mode)
{
    int8_t result;
    struct bmi3_map_int map_int = { 0 };

    printf("BMI323 config hw\r\n");

    dev->sample_time_tick = 0;
    dev->first_sample = true;
    dev->period_tick = CLOCK_TICK_PER_SECOND / rate;
    dev->frames_dropped = 0;
    dev->frames_sampled = 0;
    dev->accel_range = accel_range;
    dev->gyro_range = gyro_range;
    dev->stream_mode = mode;

    struct bmi3_sens_config config[SENSOR_COUNT];
    config[0].type = BMI3_ACCEL;
    config[1].type = BMI3_GYRO;

    // Get current sensor configuration
    result = bmi3_get_sensor_config(config, SENSOR_COUNT, &(dev->sensor));
    if (BMI3_OK != result)
    {
    	printf("bmi3_get_sensor_config error : %d \r\n", result);
        return false;
    }

    // Set output data rate and range based on rate and range parameters
    switch(rate) {
        case 25:
            config[0].cfg.acc.odr = BMI3_ACC_ODR_25HZ;
            config[1].cfg.gyr.odr = BMI3_GYR_ODR_25HZ;
            dev->frames_target = 1;
            break;
        case 50:
            config[0].cfg.acc.odr = BMI3_ACC_ODR_50HZ;
            config[1].cfg.gyr.odr = BMI3_GYR_ODR_50HZ;
            dev->frames_target = 1;
            break;
        case 100:
            config[0].cfg.acc.odr = BMI3_ACC_ODR_100HZ;
            config[1].cfg.gyr.odr = BMI3_GYR_ODR_100HZ;
            dev->frames_target = 2;
            break;
        case 200:
            config[0].cfg.acc.odr = BMI3_ACC_ODR_200HZ;
            config[1].cfg.gyr.odr = BMI3_GYR_ODR_200HZ;
            dev->frames_target = 4;
            break;
        case 400:
            config[0].cfg.acc.odr = BMI3_ACC_ODR_400HZ;
            config[1].cfg.gyr.odr = BMI3_GYR_ODR_400HZ;
            dev->frames_target = 8;
            break;
        case 800:
            config[0].cfg.acc.odr = BMI3_ACC_ODR_800HZ;
            config[1].cfg.gyr.odr = BMI3_GYR_ODR_800HZ;
            dev->frames_target = MAX_FRAMES_IN_CHUNK;
            break;
        default: return false;
    }

    switch(accel_range) {
        case 2: config[0].cfg.acc.range = BMI3_ACC_RANGE_2G; break;
        case 4: config[0].cfg.acc.range = BMI3_ACC_RANGE_4G; break;
        case 8: config[0].cfg.acc.range = BMI3_ACC_RANGE_8G; break;
        case 16: config[0].cfg.acc.range = BMI3_ACC_RANGE_16G; break;
        default: return false;
    }

    switch(gyro_range) {
        case 125: config[1].cfg.gyr.range = BMI3_GYR_RANGE_125DPS; break;
        case 250: config[1].cfg.gyr.range = BMI3_GYR_RANGE_250DPS; break;
        case 500: config[1].cfg.gyr.range = BMI3_GYR_RANGE_500DPS; break;
        case 1000: config[1].cfg.gyr.range = BMI3_GYR_RANGE_1000DPS; break;
        case 2000: config[1].cfg.gyr.range = BMI3_GYR_RANGE_2000DPS; break;
        default: return false;
    }

    config[0].type = BMI3_ACCEL;
    config[1].type = BMI3_GYRO;

    // The Accel bandwidth coefficient defines the 3 dB cutoff frequency in relation to the ODR.
    config[0].cfg.acc.bwp = BMI3_ACC_BW_ODR_QUARTER;

    // To enable the accelerometer set the power mode to normal mode
    // config[0].cfg.acc.acc_mode = BMI3_ACC_MODE_NORMAL;
    config[0].cfg.acc.acc_mode = BMI3_ACC_MODE_HIGH_PERF;

    // Set number of average samples for accel.
    config[0].cfg.acc.avg_num = BMI3_ACC_AVG64;
    // config[0].cfg.acc.avg_num = BMI3_ACC_AVG1;


    // The Gyroscope bandwidth coefficient defines the 3 dB cutoff frequency in relation to the ODR
	// Value   Name      Description
	//  0   odr_half   BW = gyr_odr/2
	//  1  odr_quarter BW = gyr_odr/4
    config[1].cfg.gyr.bwp = BMI3_GYR_BW_ODR_HALF;

    config[1].cfg.gyr.avg_num = BMI3_GYR_AVG4;

    // To enable the gyroscope set the power mode to normal mode
    config[1].cfg.gyr.gyr_mode = BMI3_GYR_MODE_NORMAL;

    // Set new configuration
    result = bmi3_set_sensor_config(config, SENSOR_COUNT, &(dev->sensor));
    if (result != BMI3_OK)
    {
    	printf("bmi3_set_sensor_config error\r\n");
        return false;
    }

    /* Map the FIFO full interrupt to INT1 */
	/* Note: User can map the interrupt to INT1 or INT2 */
	map_int.acc_drdy_int = BMI3_INT1;
	map_int.gyr_drdy_int = BMI3_INT1;

	/* Map the interrupt configuration */
	result = bmi323_map_interrupt(map_int, &(dev->sensor));
    if (result != BMI3_OK)
    {
    	printf("bmi323_map_interrupt error\r\n");
        return false;
    }

	/*Configure the interrupt pin outputs*/
    result = bmi3_set_int_pin_config(&int_config, &(dev->sensor));
    if (result != BMI3_OK)
    {
    	printf("bmi3_set_int_pin_config error\r\n");
        return false;
    }

    int success = 0;
    struct bmi3_sensor_data bmi_sensor_data[SENSOR_COUNT] = { 0 };
    for(int i = 0; i < 20; ++i)
    {
    	if (bmi323_int_flag)
    	{
    		uint16_t int_status = 0;
    		bmi323_int_flag = false;
    		result = bmi323_get_int1_status(&int_status, &(dev->sensor));
    		if (result != BMI3_OK)
			{
    			printf("bmi323_get_int1_status error\r\n");
				return false;
			}

    		if (int_status & BMI3_INT_STATUS_ACC_DRDY)
			{
    			result = bmi323_get_sensor_data(&bmi_sensor_data[0], 1, &(dev->sensor));
    			if (result != BMI3_OK)
				{
    				printf("bmi323_get_sensor_data 0 error\r\n");
					return false;
				}
    			else
    			{
    				success++;
    			}
			}
			if (int_status & BMI3_INT_STATUS_GYR_DRDY)
			{
				result = bmi323_get_sensor_data(&bmi_sensor_data[1], 1, &(dev->sensor));
				if (result != BMI3_OK)
				{
					printf("bmi323_get_sensor_data 1 error\r\n");
					return false;
				}
				else
				{
					success++;
				}
			}

			if(success >= 5) {
				dev->frames_sampled = 0;

				   printf("bmi323: Configured device. mode=%i, rate=%d Hz, frames/chunk=%d, gyro=%ddps, accel=%dG\r\n",
							dev->stream_mode, rate, dev->frames_target, gyro_range, accel_range);
				return true;    // OK, all good.
			}
    	}
    	cyhal_system_delay_ms(100);
    }

    printf("bmi323 timeout\r\n");
    return false;
}

static void _start_streams(protocol_t* protocol, int device, pb_ostream_t* ostream, void* arg)
{
	UNUSED(ostream);
	dev_bmi323_t* dev = (dev_bmi323_t*)arg;
	protocol_set_device_status(
			protocol,
			device,
			protocol_DeviceStatus_DEVICE_STATUS_ACTIVE,
			"Device is streaming");

    dev->first_sample = true;
    dev->sample_time_tick = clock_get_tick();
}

static void _stop_streams(protocol_t* protocol, int device, pb_ostream_t* ostream, void* arg)
{
	UNUSED(arg);
    UNUSED(ostream);

    protocol_set_device_status(
        protocol,
        device,
        protocol_DeviceStatus_DEVICE_STATUS_READY,
        "Device stopped");
}

/******************************************************************************
* Function Name: _write_payload
********************************************************************************
* Summary:
*   Writes the data into the output stream.
*
* Parameters:
*   protocol: Pointer to the protocol handle.
*   device_id: The device index.
*   stream_id: The stream index.
*   frame_count: Number of frames to write.
*   total_bytes: Total number of bytes to write (frame_count * sizeof(type) * frame_shape.flat).
*   ostream: Pointer to the output stream to write to.
*   arg: Pointer to the bmi270 device handle.
*
* Return:
*   True if data writing is successful, otherwise false.
*
*******************************************************************************/
static bool _write_payload(
    protocol_t* protocol,
    int device_id,
    int stream_id,
    int frame_count,
    int total_bytes,
    pb_ostream_t* ostream,
    void* arg)
{
    UNUSED(protocol);
    UNUSED(frame_count);
    UNUSED(protocol);

    dev_bmi323_t* dev = (dev_bmi323_t*)arg;
    float *frame;

    switch(dev->stream_mode)
    {
    case BMI323_MODE_STREAM_BOTH:
        if(stream_id == 0)
        {
            frame = dev->accel_data;
        }
        else
        {
            frame = dev->gyro_data;
        }
        return pb_write(ostream, (const pb_byte_t *)frame, total_bytes);
    case BMI323_MODE_STREAM_ONLY_ACCEL:
        return pb_write(ostream, (const pb_byte_t *)dev->accel_data, total_bytes);
    case BMI323_MODE_STREAM_ONLY_GYRO:
        return pb_write(ostream, (const pb_byte_t *)dev->gyro_data, total_bytes);
    case BMI323_MODE_STREAM_COMBINED:
    	return pb_write(ostream, (const pb_byte_t *)dev->data_combined, total_bytes);
    default:
        return false;
    }
}

static bool _read_hw(dev_bmi323_t* dev)
{
    int8_t result = 0;
    uint16_t int_status = 0;
    struct bmi3_sensor_data data[SENSOR_COUNT] = { 0 };
    struct bmi3_dev *sensor = &dev->sensor;

    data[0].type = BMI323_ACCEL;
    data[1].type = BMI323_GYRO;

    // Check first status of ISR pin (data or not?)
    if (bmi323_int_flag == 0)
    {
    	// No data available
    	return false;
    }

    result = bmi323_get_int1_status(&int_status, sensor);
    if (result != BMI323_OK)
    {
    	return false;
    }

    if (dev->stream_mode == BMI323_MODE_STREAM_COMBINED)
    {
    	// We expect both (acc and gyr to be ready)
		if (((int_status & BMI3_INT_STATUS_ACC_DRDY) == 0)
				|| ((int_status & BMI3_INT_STATUS_GYR_DRDY) == 0))
		{
			return false;
		}

		float *dest = dev->data_combined + dev->frames_sampled * AXIS_COUNT * SENSOR_COUNT;

		// Read acc.
		result = bmi323_get_sensor_data(&data[0], 1, sensor);
		if (result != BMI323_OK)
		{
			return false;
		}

		*dest++ = _lsb_to_mps2(data[0].sens_data.acc.x, dev->accel_range, sensor->resolution);
		*dest++ = _lsb_to_mps2(data[0].sens_data.acc.y, dev->accel_range, sensor->resolution);
		*dest++ = _lsb_to_mps2(data[0].sens_data.acc.z, dev->accel_range, sensor->resolution);

    	// Read gyr.
    	result = bmi323_get_sensor_data(&data[1], 1, sensor);
    	if (result != BMI323_OK)
		{
			return false;
		}

    	*dest++ = _lsb_to_dps(data[1].sens_data.gyr.x, dev->gyro_range, sensor->resolution);
		*dest++ = _lsb_to_dps(data[1].sens_data.gyr.y, dev->gyro_range, sensor->resolution);
		*dest++ = _lsb_to_dps(data[1].sens_data.gyr.z, dev->gyro_range, sensor->resolution);
    }

    if (dev->stream_mode == BMI323_MODE_STREAM_BOTH)
    {
    	// We expect both (acc and gyr to be ready)
    	if (((int_status & BMI3_INT_STATUS_ACC_DRDY) == 0)
    			|| ((int_status & BMI3_INT_STATUS_GYR_DRDY) == 0))
    	{
    		return false;
    	}

    	// Read acc.
    	result = bmi323_get_sensor_data(&data[0], 1, sensor);
    	if (result != BMI323_OK)
		{
			return false;
		}

    	float *dest = dev->accel_data + dev->frames_sampled * AXIS_COUNT;
		*dest++ = _lsb_to_mps2(data[0].sens_data.acc.x, dev->accel_range, sensor->resolution);
		*dest++ = _lsb_to_mps2(data[0].sens_data.acc.y, dev->accel_range, sensor->resolution);
		*dest++ = _lsb_to_mps2(data[0].sens_data.acc.z, dev->accel_range, sensor->resolution);

    	// Read gyr.
    	result = bmi323_get_sensor_data(&data[1], 1, sensor);
    	if (result != BMI323_OK)
		{
			return false;
		}

    	dest = dev->gyro_data + dev->frames_sampled * AXIS_COUNT;

		*dest++ = _lsb_to_dps(data[1].sens_data.gyr.x, dev->gyro_range, sensor->resolution);
		*dest++ = _lsb_to_dps(data[1].sens_data.gyr.y, dev->gyro_range, sensor->resolution);
		*dest++ = _lsb_to_dps(data[1].sens_data.gyr.z, dev->gyro_range, sensor->resolution);
    }

    if (dev->stream_mode == BMI323_MODE_STREAM_ONLY_ACCEL)
	{
		if ((int_status & BMI3_INT_STATUS_ACC_DRDY) == 0)
		{
			return false;
		}

		// Read acc.
		result = bmi323_get_sensor_data(&data[0], 1, sensor);
		if (result != BMI323_OK)
		{
			return false;
		}

		float *dest = dev->accel_data + dev->frames_sampled * AXIS_COUNT;
		*dest++ = _lsb_to_mps2(data[0].sens_data.acc.x, dev->accel_range, sensor->resolution);
		*dest++ = _lsb_to_mps2(data[0].sens_data.acc.y, dev->accel_range, sensor->resolution);
		*dest++ = _lsb_to_mps2(data[0].sens_data.acc.z, dev->accel_range, sensor->resolution);
	}

    if (dev->stream_mode == BMI323_MODE_STREAM_ONLY_GYRO)
    {
    	// We expect both (acc and gyr to be ready)
    	if ((int_status & BMI3_INT_STATUS_GYR_DRDY) == 0)
    	{
    		return false;
    	}

    	// Read gyr.
    	result = bmi323_get_sensor_data(&data[1], 1, sensor);
    	if (result != BMI323_OK)
		{
			return false;
		}

    	float *dest = dev->gyro_data + dev->frames_sampled * AXIS_COUNT;

		*dest++ = _lsb_to_dps(data[1].sens_data.gyr.x, dev->gyro_range, sensor->resolution);
		*dest++ = _lsb_to_dps(data[1].sens_data.gyr.y, dev->gyro_range, sensor->resolution);
		*dest++ = _lsb_to_dps(data[1].sens_data.gyr.z, dev->gyro_range, sensor->resolution);
    }


    dev->frames_sampled++;

    return true;
}

static void _poll_streams(protocol_t* protocol, int device, pb_ostream_t* ostream, void* arg)
{
    dev_bmi323_t* dev = (dev_bmi323_t*)arg;
    clock_tick_t current_time = clock_get_tick();

    // Reinterpret this timing as the time we wish the sample to happen at.
    clock_tick_t current_treshold = dev->sample_time_tick;

    clock_tick_t total_drift = current_time - current_treshold;

	// If we are to late we skip this frame and save time
	// Previous data package will be resent
    bool late = false;
    uint32_t drift_ms = (uint32_t)total_drift;
    if (drift_ms > 200){ // Tens of microseconds. 200 is equal to 2 ms
        late=1;
    }

    // The first sampling is now done as soon as possible.
    if(current_time >= current_treshold ) {

        // If we are late.. Skip the reading of the sensor..
        if ( late && !dev->first_sample){
            late = false;
        }
        else{
            if(!_read_hw(dev)) {
                return;
            }
            dev->first_sample = false;
        }

        /* This is updated whenever there is an _read_hw call! However since the _read_hw might fail */
        /* and when we are resending the previous data we have to force this to 1 anyway. */
        dev->frames_sampled = 1;

        /* Since we in reality dont drop any frames anymore. */
        dev->frames_dropped = 0;

        /* When we should do the next frame. */
        dev->sample_time_tick += dev->period_tick;

        /* Always send something. */
        {
            protocol_send_data_chunk(protocol, device, 0, dev->frames_sampled, dev->frames_dropped, ostream, _write_payload);

            if(dev->stream_mode == BMI323_MODE_STREAM_BOTH)
                protocol_send_data_chunk(protocol, device, 1, dev->frames_sampled, dev->frames_dropped, ostream, _write_payload);

            dev->frames_dropped = 0;
            dev->frames_sampled = 0;

        }
    }
}

/**
 * We will have 2 streams:
 * - Acceleromter
 * - Gyroscope
 */
static bool _configure_streams(protocol_t* protocol, int device, void* arg)
{
    int result;
    int rate_index;
    int rate;
	int accel_range;
	int gyro_range;
	int accel_range_index;
	int gyro_range_index;
    int mode_index;
    stream_mode_t mode;
    const char *stream0_unit;
    const char *stream1_unit;
    const char *stream0_name;
    const char *stream1_name;
    int stream0;
    int stream1;
    dev_bmi323_t* dev = (dev_bmi323_t*)arg;

    if(protocol_get_option_oneof(protocol, device, DEV_BMI323_OPTION_KEY_RATE, &rate_index) != PROTOCOL_STATUS_SUCCESS) {
        protocol_set_device_status(
            protocol,
            device,
            protocol_DeviceStatus_DEVICE_STATUS_ERROR,
            "Failed to get option frequency.");
        return true;
    }

    switch(rate_index) {
       case 0: rate = 50; break;
       case 1: rate = 100; break;
       case 2: rate = 200; break;
       case 3: rate = 400; break;
       default: return false;
    }

    if(protocol_get_option_oneof(protocol, device, DEV_BMI323_OPTION_KEY_STREAM_MODE, &mode_index) != PROTOCOL_STATUS_SUCCESS) {
        protocol_set_device_status(
            protocol,
            device,
            protocol_DeviceStatus_DEVICE_STATUS_ERROR,
            "Failed to get option stream mode.");
        return true;
    }

	switch(mode_index) {
	   case 0: mode = BMI323_MODE_STREAM_BOTH; break;
	   case 1: mode = BMI323_MODE_STREAM_ONLY_ACCEL; break;
	   case 2:mode = BMI323_MODE_STREAM_ONLY_GYRO; break;
	   case 3:mode = BMI323_MODE_STREAM_COMBINED; break;
	   default:
		   protocol_set_device_status(
				   protocol,
				   device,
				   protocol_DeviceStatus_DEVICE_STATUS_ERROR,
				   "Mode is wrong.");
		   return true;
	}

	if (protocol_get_option_oneof(protocol, device, DEV_BMI323_OPTION_KEY_ACCEL_RANGE, &accel_range_index) != PROTOCOL_STATUS_SUCCESS) {
        protocol_set_device_status(
            protocol,
            device,
            protocol_DeviceStatus_DEVICE_STATUS_ERROR,
            "Failed to get option accel range.");
        return true;
	}

	switch(accel_range_index) {
	   case 0: accel_range = 2; break;
	   case 1: accel_range = 4; break;
	   case 2: accel_range = 8; break;
	   case 3: accel_range = 16; break;
	   default:
		   protocol_set_device_status(
				   protocol,
				   device,
				   protocol_DeviceStatus_DEVICE_STATUS_ERROR,
				   "Accel range is wrong.");
		   return true;
	}

	if (protocol_get_option_oneof(protocol, device, DEV_BMI323_OPTION_KEY_GYRO_RANGE, &gyro_range_index)!= PROTOCOL_STATUS_SUCCESS) {
        protocol_set_device_status(
            protocol,
            device,
            protocol_DeviceStatus_DEVICE_STATUS_ERROR,
            "Failed to get option gyro range.");
        return true;
	}

	switch(gyro_range_index) {
	   case 0: gyro_range = 125; break;
	   case 1: gyro_range = 250; break;
	   case 2: gyro_range = 500; break;
	   case 3: gyro_range = 1000; break;
	   case 4: gyro_range = 2000; break;
	   default:
		   protocol_set_device_status(
				   protocol,
				   device,
				   protocol_DeviceStatus_DEVICE_STATUS_ERROR,
				   "Gyro range is wrong.");
		   return true;
	}

    // Clear any existing streams
    // streams depends on which mode is chosen (both, acc only, gyr only)
    if(protocol_clear_streams(protocol, device) != PROTOCOL_STATUS_SUCCESS) {
        protocol_set_device_status(
            protocol,
            device,
            protocol_DeviceStatus_DEVICE_STATUS_ERROR,
            "Failed to clear streams.");
        return true;
    }

    switch(mode_index) {
       case 0:
           stream0_unit = "m/s\xc2\xb2";         /* Meter per second squared */
           stream1_unit = "\xc2\xb0/s";         /* Degrees per second */
           stream0_name = "Accel";
           stream1_name = "Gyro";
           break;
       case 1:
           stream0_unit = "m/s\xc2\xb2";         /* Meter per second squared */
           stream1_unit = NULL;
           stream0_name = "Accel";
           stream1_name = NULL;
           break;
       case 2:
           stream0_unit = "\xc2\xb0/s";         /* Degrees per second */
           stream1_unit = NULL;
           stream0_name = "Gyro";
           stream1_name = NULL;
           break;
       case 3:
    	   stream0_unit = "m/s\xc2\xb2, \xc2\xb0/s"; /* Meter per second squared, Degrees per second */
    	   stream1_unit = NULL;
    	   stream0_name = "Combined";
    	   stream1_name = NULL;
    	   break;
       default: return false;
    }

    /* Add stream #0 */
    stream0 = protocol_add_stream(
           protocol,
           device,
           stream0_name,
           protocol_StreamDirection_STREAM_DIRECTION_OUTPUT,
           protocol_DataType_DATA_TYPE_F32,
           rate,
           1,
           stream0_unit);
    if(stream0 < 0) {
        protocol_set_device_status(
            protocol,
            device,
            protocol_DeviceStatus_DEVICE_STATUS_ERROR,
            "Failed to add streams.");
        return true;
    }

    if(mode == BMI323_MODE_STREAM_COMBINED) {
		result = protocol_add_stream_rank(
			protocol,
			device,
			stream0,
			"Sensor",
			2,
			(const char* []) { "Accel", "Gyro" });
		if(result != PROTOCOL_STATUS_SUCCESS) {
			protocol_set_device_status(
				protocol,
				device,
				protocol_DeviceStatus_DEVICE_STATUS_ERROR,
				"Failed to add streams dimension.");
			return true;
		}
	}

    /* Add optional stream #1 */
    if(stream1_name != NULL) {
        stream1 = protocol_add_stream(
           protocol,
           device,
           stream1_name,
           protocol_StreamDirection_STREAM_DIRECTION_OUTPUT,
           protocol_DataType_DATA_TYPE_F32,
           rate,
           1,
           stream1_unit);
        if(stream1 < 0) {
            protocol_set_device_status(
                protocol,
                device,
                protocol_DeviceStatus_DEVICE_STATUS_ERROR,
                "Failed to add streams.");
            return true;
        }

        result = protocol_add_stream_rank(
            protocol,
            device,
            stream1,
            "Axis",
            3,
            (const char* []) { "X", "Y", "Z" });
        if(result != PROTOCOL_STATUS_SUCCESS) {
            protocol_set_device_status(
                protocol,
                device,
                protocol_DeviceStatus_DEVICE_STATUS_ERROR,
                "Failed to add streams dimension.");
            return true;
        }
    }

    result = protocol_add_stream_rank(
        protocol,
            device,
            stream0,
            "Axis",
            3,
            (const char* []) { "X", "Y", "Z" });
    if(result != PROTOCOL_STATUS_SUCCESS) {
        protocol_set_device_status(
            protocol,
            device,
            protocol_DeviceStatus_DEVICE_STATUS_ERROR,
            "Failed to add streams dimension.");
        return true;
    }

	if(!_config_hw(dev, rate, accel_range, gyro_range, mode)) {
		protocol_set_device_status(
			protocol,
			device,
			protocol_DeviceStatus_DEVICE_STATUS_ERROR,
			"Failed to configure hardware.");
		return true;
	}

    protocol_set_device_status(
        protocol,
        device,
        protocol_DeviceStatus_DEVICE_STATUS_READY,
        "Device is ready.");

    return true;
}

static bool _init_hw(dev_bmi323_t *dev, cyhal_i2c_t* i2c)
{
	// -------------------------------
	// First, configure interrupt pin
	cy_rslt_t result = cyhal_gpio_init(ARDU_IO4, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE, false);
	if (result != CY_RSLT_SUCCESS)
	{
		printf("cyhal_gpio_init error\r\n");
		return false;
	}

	// Register callback function
	cyhal_gpio_register_callback(ARDU_IO4, &bmi323_int_data);

	// Enable rising edge interrupt events
	cyhal_gpio_enable_event(ARDU_IO4, CYHAL_GPIO_IRQ_RISE, BMI323_IRQ_PRIORITY, true);

	// -------------------------------
	// Then initialize sensor
	int8_t retval = bmi3_interface_init_i2c(&(dev->sensor), (void*)i2c);
	if(retval != BMI3_OK)
	{
		printf("bmi3_interface_init error\r\n");
		return false;
	}

	/* Initialize BMI323. */
	retval = bmi323_init(&(dev->sensor));
	if(retval != BMI3_OK)
	{
		printf("bmi323_init error\r\n");
		return false;
	}

	return true;
}

int dev_bmi323_register(protocol_t* protocol, cyhal_i2c_t* i2c)
{
    int status;

    dev_bmi323_t *dev = (dev_bmi323_t*)malloc(sizeof(dev_bmi323_t));
    if(dev == NULL)
    {
        return -1;
    }
    memset(dev, 0, sizeof(dev_bmi323_t));

    if(!_init_hw(dev, i2c))
    {
        free(dev);
        return -2;
    }

    device_manager_t manager = {
        .arg = dev,
        .configure_streams = _configure_streams,
        .start = _start_streams,
        .stop = _stop_streams,
        .poll = _poll_streams,
        .data_received = NULL /* has no input streams */
    };

    int device = protocol_add_device(
        protocol,
        protocol_DeviceType_DEVICE_TYPE_SENSOR,
        "IMU",
        "Accelerometer and Gyroscope (BMI323)",
        manager);

    if(device < 0)
    {
        return -3;
    }

    status = protocol_add_option_oneof(
        protocol,
        device,
        DEV_BMI323_OPTION_KEY_RATE,
        "Frequency",
        "Sample frequency (Hz)",
        0,
        (const char* []) { "50 Hz", "100 Hz", "200 Hz", "400 Hz" },
        4);

    if(status != PROTOCOL_STATUS_SUCCESS)
    {
        return -4;
    }

    status = protocol_add_option_oneof(
        protocol,
        device,
        DEV_BMI323_OPTION_KEY_ACCEL_RANGE,
        "Accel Range",
        "Min/Max gravity range in G",
        2,
        (const char* []) { "2 G", "4 G", "8 G", "16 G" },
        4);

    if(status != PROTOCOL_STATUS_SUCCESS)
    {
        return -5;
    }

    status = protocol_add_option_oneof(
        protocol,
        device,
        DEV_BMI323_OPTION_KEY_GYRO_RANGE,
        "Gyro Range",
        "Angular Rate Measurement Range",
        2,
        (const char* []) { "125 dps", "250 dps", "500 dps", "1000 dps", "2000 dps" },
        5);

    if(status != PROTOCOL_STATUS_SUCCESS)
    {
        return -6;
    }

     status = protocol_add_option_oneof(
        protocol,
        device,
        DEV_BMI323_OPTION_KEY_STREAM_MODE,
        "Mode",
        "Stream Configuration",
        0,
        (const char* []) { "Both", "Only Accel", "Only Gyro", "Combined" },
        4);

    if(status != PROTOCOL_STATUS_SUCCESS)
    {
        return -7;
    }

    if(!_configure_streams(protocol, device, manager.arg))
    {
        return -8;
    }

    return 0;
}
