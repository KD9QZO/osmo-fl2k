/*
 * osmo-fl2k, turns FL2000-based USB 3.0 to VGA adapters into
 * low cost DACs
 *
 * Copyright (C) 2016-2018 by Steve Markgraf <steve@steve-m.de>
 *
 * SPDX-License-Identifier: GPL-2.0+
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef FL2K_H_
#define FL2K_H_


/**
 * \defgroup fl2k_lib FL2K Library Functions
 *
 * @{
 */


#ifdef __cplusplus
extern "C" {
#endif


#include <stdint.h>
#include <osmo-fl2k_export.h>


/*! \brief Enumeration of potential errors */
enum fl2k_error {
	FL2K_SUCCESS = 0,				/*!< Equivalent to boolean \b false or successful execution */
	FL2K_TRUE = 1,					/*!< Equivalent to boolean \b true */
	FL2K_ERROR_INVALID_PARAM = -1,	/*!< Invalid parameter was specified */
	FL2K_ERROR_NO_DEVICE = -2,		/*!< No device is present */
	FL2K_ERROR_NOT_FOUND = -5,		/*!< The device was not found */
	FL2K_ERROR_BUSY = -6,			/*!< The device was busy */
	FL2K_ERROR_TIMEOUT = -7,		/*!< The operation timed out */
	FL2K_ERROR_NO_MEM = -11,		/*!< Not enough memory available */
};

/*! \brief Data structure to hold the fl2k device data */
typedef struct fl2k_data_info {
	/* information provided by library */
	void *ctx;					/*!< device context */
	uint32_t underflow_cnt;		/*!< underflows since last callback */
	uint32_t len;				/*!< buffer length */
	int using_zerocopy;			/*!< using zerocopy kernel buffers */
	int device_error;			/*!< device error happened, terminate application */

	/* filled in by application */
	int sampletype_signed;		/*!< are samples signed or unsigned? */
	char *r_buf;				/*!< pointer to red buffer */
	char *g_buf;				/*!< pointer to green buffer */
	char *b_buf;				/*!< pointer to blue buffer */
} fl2k_data_info_t;

typedef struct fl2k_dev fl2k_dev_t;



/**
 * \defgroup fl2k_xfer_buf FL2K Transfer Buffers
 *
 * \note The transfer length was chosen by the following criteria:
 * - Must be a supported resolution of the FL2000DX
 * - Must be a multiple of 61440 bytes (URB payload length), which is important for using the DAC without HSYNC/VSYNC
 *   blanking, otherwise a couple of samples are missing in between every buffer
 * - Should be smaller than 4MB in order to be allocatable by \c kmalloc() for zerocopy transfers
 *
 * @{
 */

/*! \brief Set the buffer length for the FL2K device */
#define FL2K_BUF_LEN (1280 * 1024)

/*! \brief Set the transfer length for the FL2K device */
#define FL2K_XFER_LEN (FL2K_BUF_LEN * 3)

/**
 * @}
 */



FL2K_API uint32_t fl2k_get_device_count(void);

FL2K_API const char* fl2k_get_device_name(uint32_t index);

FL2K_API int fl2k_open(fl2k_dev_t **dev, uint32_t index);

FL2K_API int fl2k_close(fl2k_dev_t *dev);



/**
 * \defgroup fl2k_config_func FL2K Configuration Functions
 *
 * @{
 */


/**
 * \brief Set the sample rate (pixel clock) for the device
 *
 * \param[in]	dev			the device handle given by \ref fl2k_open()
 * \param[in]	samp_rate	the sample rate to be set, maximum value depends on host and USB controller
 *
 * \return 0 on success, -EINVAL on invalid rate
 * \retval	0		Success
 * \retval	-EINVAL	Invalid \p samp_rate specified
 */
FL2K_API int fl2k_set_sample_rate(fl2k_dev_t *dev, uint32_t target_freq);

/**
 * \brief Get actual sample rate the device is configured to
 *
 * \param[in]	dev			the device handle given by \ref fl2k_open()
 *
 * \return 0 on error, sample rate in Hz otherwise
 * \retval	0	An error has occurred.
 */
FL2K_API uint32_t fl2k_get_sample_rate(fl2k_dev_t *dev);

/**
 * @}
 */



/**
 * \defgroup fl2k_streaming_func FL2K Streaming Functions
 *
 * @{
 */

typedef void(*fl2k_tx_cb_t)(fl2k_data_info_t *data_info);

/**
 * \brief Starts the TX thread
 *
 * \attention This function will \b block until \ref fl2k_stop_tx() is called to cancel it!
 *
 * \param[in]	dev			the device handle given by \ref fl2k_open()
 * \param[in]	cb			the \c fl2k_tx_cb_t callback function to call
 * \param[in]	ctx			user specific context to pass via the callback function
 * \param[in]	buf_num		optional buffer count: \n
 *							\p buf_num * \c FL2K_BUF_LEN = overall buffer size set to 0 for default buffer count (4)
 *
 * \return 0 on success
 */
FL2K_API int fl2k_start_tx(fl2k_dev_t *dev, fl2k_tx_cb_t cb, void *ctx, uint32_t buf_num);

/**
 * \brief Cancel all pending asynchronous operations on the device.
 *
 * \param[in]	dev			the device handle given by \ref fl2k_open()
 *
 * \return 0 on success
 */
FL2K_API int fl2k_stop_tx(fl2k_dev_t *dev);

/**
 * \brief Read 4 bytes via the FL2K I2C bus
 *
 * \param[in]	dev			the device handle given by \ref fl2k_open()
 * \param[in]	i2c_addr	address of the I2C device
 * \param[in]	reg_addr	start address of the 4 bytes to be read
 * \param[out]	data		pointer to byte array of size 4
 *
 * \return 0 on success
 *
 * \note A read operation will look like this on the bus:
 *       \code
 *       START, I2C_ADDR(W), REG_ADDR,   REP_START, I2C_ADDR(R), DATA[0], STOP
 *       START, I2C_ADDR(W), REG_ADDR+1, REP_START, I2C_ADDR(R), DATA[1], STOP
 *       START, I2C_ADDR(W), REG_ADDR+2, REP_START, I2C_ADDR(R), DATA[2], STOP
 *       START, I2C_ADDR(W), REG_ADDR+3, REP_START, I2C_ADDR(R), DATA[3], STOP
 *       \endcode
 */
FL2K_API int fl2k_i2c_read(fl2k_dev_t *dev, uint8_t i2c_addr, uint8_t reg_addr, uint8_t *data);

/**
 * \brief Write 4 bytes via the FL2K I2C bus
 *
 * \param[in]	dev			the device handle given by \ref fl2k_open()
 * \param[in]	i2c_addr	address of the I2C device
 * \param[in]	reg_addr	start address of the 4 bytes to be written
 * \param[in]	data		pointer to byte array of size 4
 *
 * \return 0 on success
 *
 * \note A write operation will look like this on the bus:
 *       \code
 *       START, I2C_ADDR(W), REG_ADDR,   DATA[0], STOP
 *       START, I2C_ADDR(W), REG_ADDR+1, DATA[1], STOP
 *       START, I2C_ADDR(W), REG_ADDR+2, DATA[2], STOP
 *       START, I2C_ADDR(W), REG_ADDR+3, DATA[3], STOP
 *       \endcode
 */
FL2K_API int fl2k_i2c_write(fl2k_dev_t *dev, uint8_t i2c_addr, uint8_t reg_addr, uint8_t *data);

/**
 * @}
 */


#ifdef __cplusplus
}
#endif


/**
 * @}
 */


#endif	/* !FL2K_H_ */
