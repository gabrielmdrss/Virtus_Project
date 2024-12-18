#ifndef DEFINES_FUNCTIONS_H
#define DEFINES_FUNCTIONS_H

/******************************************************************************
 * @file    DEFINES_FUNCTIONS.h
 * @brief   Header file containing definitions and function prototypes for the
 *          application.
 *
 * @authors  Gabriel Domingos de Medeiros, Jezreal Pereira Filgueiras
 * @date    December 2024
 * @version 1.0.0
 *
 * @note    This file includes the necessary definitions, constants, and function
 *          prototypes for the application.
 *
 * @copyright (c) 2024 VIRTUS--CC. All rights reserved.
 ******************************************************************************/



#include <math.h>         		// Math library for mathematical operations.
#include "main.h"         		// Project-specific main header file, for HAL and other declarations.

// External handles for peripherals (defined in main.c or other implementation files).
extern I2C_HandleTypeDef hi2c1; ///< I2C peripheral handle for communication (e.g., with MPU6500 and ssd1306).
extern ADC_HandleTypeDef hadc1; ///< ADC peripheral handle.
extern TIM_HandleTypeDef htim2; ///< Timer peripheral handle.

// ---------------------------- MPU6500 Registers ----------------------------

/**
 * @brief I2C address for the MPU6050/MPU6500.
 */
#define MPU6050_ADDR 0xD0

/**
 * @brief Register addresses for the MPU6500 sensor.
 * These are used for configuring or reading specific features and data.
 */
#define TEMP_OUT_H_REG     0x41  			///< High byte of temperature output register.
#define WHO_AM_I_REG       0x75  			///< Device ID register, expected to return 0x70 for MPU6500.

#define XG_OFFSET_H        (uint8_t)0x13  	///< Gyroscope X-axis offset high byte.
#define XG_OFFSET_L        (uint8_t)0x14  	///< Gyroscope X-axis offset low byte.
#define YG_OFFSET_H        (uint8_t)0x15  	///< Gyroscope Y-axis offset high byte.
#define YG_OFFSET_L        (uint8_t)0x16  	///< Gyroscope Y-axis offset low byte.
#define ZG_OFFSET_H        (uint8_t)0x17  	///< Gyroscope Z-axis offset high byte.
#define ZG_OFFSET_L        (uint8_t)0x18  	///< Gyroscope Z-axis offset low byte.

#define SMPLRT_DIV         (uint8_t)0x19  	///< Sample rate divider register.
#define CONFIG             (uint8_t)0x1A  	///< General configuration register.
#define GYRO_CONFIG        (uint8_t)0x1B  	///< Gyroscope configuration register.
#define ACCEL_CONFIG       (uint8_t)0x1C  	///< Accelerometer configuration register.
#define ACCEL_CONFIG2      (uint8_t)0x1D  	///< Secondary accelerometer configuration register.

#define WOM_THR            (uint8_t)0x1F  	///< Wake-on-motion threshold configuration.

#define INT_PIN_CFG        (uint8_t)0x37  	///< Interrupt pin configuration.
#define INT_ENABLE         (uint8_t)0x38  	///< Interrupt enable register.
#define INT_STATUS         (uint8_t)0x3A  	///< Interrupt status register.
#define DATA_ARRAY_POINTER (uint8_t)0x3B  	///< Pointer to the start of sensor data registers.

#define SIGNAL_PATH_RESET  (uint8_t)0x68  	///< Reset digital signal path.
#define ACCEL_INTEL_CTRL   (uint8_t)0x69  	///< Accelerometer intelligence control.
#define USER_CTRL          (uint8_t)0x6A  	///< User control register.
#define PWR_MGMT_1         (uint8_t)0x6B  	///< Power management register 1.
#define PWR_MGMT_2         (uint8_t)0x6C  	///< Power management register 2.

#define XA_OFFSET_H        (uint8_t)0x77  	///< Accelerometer X-axis offset high byte.
#define XA_OFFSET_L        (uint8_t)0x78  	///< Accelerometer X-axis offset low byte.
#define YA_OFFSET_H        (uint8_t)0x7A  	///< Accelerometer Y-axis offset high byte.
#define YA_OFFSET_L        (uint8_t)0x7B  	///< Accelerometer Y-axis offset low byte.
#define ZA_OFFSET_H        (uint8_t)0x7D  	///< Accelerometer Z-axis offset high byte.
#define ZA_OFFSET_L        (uint8_t)0x7E  	///< Accelerometer Z-axis offset low byte.

#define accelScalingFactor (float)2/32768  	///< Accelerometer scaling factor for ±2g range.
#define gyroScalingFactor  (float)250/32768 ///< Gyroscope scaling factor for ±250°/s range.

// ----------------------- Register Bit Definitions -----------------------

/**
 * @brief Bit definitions for various control registers in MPU6500.
 */
#define DEVICE_RESET       (uint8_t)(1 << 7) ///< Bit 7: Device reset (PWR_MGMT_1).
#define I2C_IF_DIS         (uint8_t)(1 << 4) ///< Bit 4: Disable I2C interface (USER_CTRL).
#define SIG_COND_RST       (uint8_t)(1 << 0) ///< Bit 0: Reset signal paths (USER_CTRL).
#define RAW_RDY_EN         (uint8_t)(1 << 0) ///< Bit 0: Raw data ready enable (INT_ENABLE).
#define WOM_EN             (uint8_t)(1 << 6) ///< Bit 6: Wake-on-motion enable (INT_ENABLE).
#define ACCEL_INTEL_EN     (uint8_t)(1 << 7) ///< Bit 7: Enable accelerometer intelligence (ACCEL_INTEL_CTRL).
#define ACCEL_INTEL_MODE   (uint8_t)(1 << 6) ///< Bit 6: Accelerometer intelligence mode (ACCEL_INTEL_CTRL).

// ----------------------- Ultrasonic Sensor Definitions -----------------------

/**
 * @brief GPIO pin definitions for the ultrasonic sensor.
 */
#define TRIG_PIN GPIO_PIN_6  ///< Trigger pin for the ultrasonic sensor.
#define TRIG_PORT GPIOA      ///< GPIO port for the trigger pin.
#define ECHO_PIN GPIO_PIN_7  ///< Echo pin for the ultrasonic sensor.
#define ECHO_PORT GPIOA      ///< GPIO port for the echo pin.

// ---------------------------- Button Definitions ----------------------------

/**
 * @brief GPIO pin definitions for the buttons.
 */
#define CHANGE_BUTTON GPIO_PIN_1  ///< Button for changing options/settings.
#define ENTER_BUTTON GPIO_PIN_4   ///< Button for confirming/entering a selection.

// ----------------------- MPU6500 Configuration Variables -----------------------

/**
 * @brief Raw and scaled data from the MPU6500 sensor.
 *
 * These variables store raw readings for acceleration, gyroscope, and temperature,
 * as well as their scaled values.
 */
int16_t Accel_X_RAW = 0;   ///< Raw acceleration value on the X-axis
int16_t Accel_Y_RAW = 0;   ///< Raw acceleration value on the Y-axis
int16_t Accel_Z_RAW = 0;   ///< Raw acceleration value on the Z-axis
int16_t Gyro_X_RAW = 0;    ///< Raw gyroscopic value on the X-axis
int16_t Gyro_Y_RAW = 0;    ///< Raw gyroscopic value on the Y-axis
int16_t Gyro_Z_RAW = 0;    ///< Raw gyroscopic value on the Z-axis
int16_t Temp_RAW = 0;      ///< Raw temperature value
float Ax, Ay, Az;          ///< Scaled acceleration values for X, Y, and Z axes
float gx, gy, gz;          ///< Scaled gyroscopic values for X, Y, and Z axes
float temp;                ///< Scaled temperature in degrees Celsius

// ------------------------- Kalman Filter Variables -------------------------

/**
 * @brief Variables used for the Kalman filter implementation.
 *
 * These variables manage angular rates, calibration, and calculated angles
 * for roll and pitch using the Kalman filter.
 */
float RateRoll;                      	///< Angular velocity of roll
float RatePitch;                     	///< Angular velocity of pitch
float RateYaw;                       	///< Angular velocity of yaw
float RateCalibrationRoll;           	///< Calibrated angular velocity for roll
float RateCalibrationPitch;          	///< Calibrated angular velocity for pitch
float RateCalibrationYaw;            	///< Calibrated angular velocity for yaw
int RateCalibrationNumber;           	///< Number of calibration samples
float AngleRoll;                      	///< Estimated roll angle
float AnglePitch;                     	///< Estimated pitch angle
float KalmanAngleRoll = 0;           	///< Roll angle from Kalman filter
float KalmanUncertaintyAngleRoll = 4; 	///< Uncertainty in roll angle
float KalmanAnglePitch = 0;           	///< Pitch angle from Kalman filter
float KalmanUncertaintyAnglePitch = 4; 	///< Uncertainty in pitch angle
float Kalman1DOutput[] = { 0, 0 };    	///< 1D Kalman filter output for angle estimation

// ---------------------- Timer3 Interrupt Variable ----------------------

/**
 * @brief Data ready flag for Timer3 interrupt.
 *
 * Indicates when the sensor data is ready for processing.
 */
int Data_Ready = 0;                   	///< Flag for data readiness

// --------------------- Ultrasonic Sensor Variables ---------------------

/**
 * @brief Variables related to the ultrasonic sensor operation.
 *
 * These variables include ADC readings, threshold limits, timing calculations,
 * and distance measurement.
 */
int adc_value = 0;                    	///< ADC reading for the ultrasonic sensor
int Limit_Ultrasonic = 0;             	///< Threshold for ultrasonic measurements
int intermediary_screen = 1;         	///< State for intermediate display screen
uint32_t pMillis;                     	///< Previous timestamp for timing calculations
uint32_t Value1 = 0;                  	///< Timestamp of ultrasonic pulse sending
uint32_t Value2 = 0;                  	///< Timestamp of ultrasonic echo reception
uint16_t Distance = 0;                	///< Distance calculated in centimeters
uint8_t x_distance;                   	///< Current distance for display print
uint8_t x_limit;                      	///< Limit for distance threshold for display print

// ----------------------- Low-Pass Filter Variables -----------------------

/**
 * @brief Variables for smoothing ultrasonic sensor data.
 *
 * Implements a low-pass filter to reduce noise in the ultrasonic measurements.
 */
float alpha = 0.3;                    	///< Smoothing factor (0.0 to 1.0)
uint16_t filtered_value = 0;          	///< Smoothed ADC value
uint16_t ultrasonic_filtered = 0;     	///< Smoothed ultrasonic measurement

/* Functions */

// --------------------------- Low Pass Filter Function ---------------------------

/**
 * @brief Applies a low-pass filter to smooth input data.
 *
 * @param new_value The new input value to be filtered.
 * @return uint16_t The filtered output value.
 *
 * This function applies a smoothing algorithm using an exponential
 * moving average. The smoothing factor `alpha` determines the influence
 * of the new value relative to the previous filtered value.
 */
uint16_t low_pass_filter(uint16_t new_value) {
    filtered_value = (alpha * new_value) + ((1 - alpha) * filtered_value);
    return filtered_value;
}

// -------------------------- MPU6050 Initialization ---------------------------

/**
 * @brief Initializes the MPU6050 sensor with the required settings.
 *
 * This function configures the MPU6050 sensor by:
 * - Checking the device ID through the WHO_AM_I register.
 * - Resetting the device and clearing digital filters.
 * - Selecting the best available clock source (gyro PLL).
 * - Configuring sampling and filtering parameters for the accelerometer
 *   and gyroscope.
 * - Setting output sampling rate to 100 Hz.
 *
 * @note Assumes that the I2C interface has been initialized and the
 *       `hi2c1` handle is available.
 */
void MPU6050_Init(void) {
    uint8_t check;
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x75, 1, &check, 1, 1000); // Check device ID WHO_AM_I

    if (check == 0x68) {  // 0x68 indicates the sensor is working correctly
        HAL_Delay(100);

        uint8_t Data;

        // Reset the sensor
        Data = DEVICE_RESET;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1, 1, &Data, 1, 1000);

        // Reset digital filters
        Data = 0b00000111;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SIGNAL_PATH_RESET, 1, &Data, 1, 1000);
        HAL_Delay(100);

        // Reset data registers
        Data = SIG_COND_RST;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, USER_CTRL, 1, &Data, 1, 1000);
        HAL_Delay(100);

        // Select the best available clock source (gyro PLL)
        Data = 0b001;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1, 1, &Data, 1, 1000);
        HAL_Delay(100);

        // Configure sampling and filtering parameters
        Data = 0b100;  // Fs=1kHz, gyro_BW=20Hz, temp_BW=20Hz
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, CONFIG, 1, &Data, 1, 1000);

        // Set gyroscope full-scale range to ±250°/s
        Data = 0x00;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG, 1, &Data, 1, 1000);

        // Set accelerometer full-scale range to ±2g
        Data = 0x00;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG, 1, &Data, 1, 1000);

        // Set LPF parameters for the accelerometer (Fs=1kHz, accel_BW=20Hz)
        Data = 0b100;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG2, 1, &Data, 1, 1000);

        // Set output data sampling rate to 100 Hz [1kHz/(1+9) = 100 Hz]
        Data = 9;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV, 1, &Data, 1, 1000);
    }
}


// ---------------------------- Menu Rendering Function ----------------------------

/**
 * @brief Renders the menu on an SSD1306 display.
 *
 * This function is responsible for drawing a menu interface on an SSD1306 display
 * using the I2C protocol. It displays icons and corresponding option names for the
 * current, previous, and next menu items. It also includes a selection outline and
 * a scrollbar for visual navigation feedback.
 *
 * Functionality:
 * - Clears the display by setting all pixels to white.
 * - Writes the names and icons of the previous, current, and next menu items.
 * - Draws a rectangular outline to highlight the selected menu item.
 * - Renders a scrollbar on the right side of the display, indicating the current
 *   position in the menu.
 *
 * Dependencies:
 * - The function relies on the SSD1306 driver functions:
 *   - `ssd1306_Fill(0)`: Clears the display by setting all pixels to a specific color.
 *   - `ssd1306_SetCursor`: Positions the cursor at a specified (x, y) location on the display.
 *   - `ssd1306_WriteString`: Writes a string at a specified position (x, y).
 *   - `ssd1306_DrawBitmap`: Renders a bitmap image (icon) at a specified position (x, y).
 *   - `ssd1306_DrawRectangle`: Draws a rectangle at the specified coordinates.
 *
 * @note Assumes that `menu_items` and `bitmap_icons` arrays are defined and populated
 *       with the corresponding strings and bitmap data for the menu options.
 */
void menu(void) {
    ssd1306_Fill(0); // Clear the display

    // Render the previous menu item
    ssd1306_SetCursor(25, 5);
    ssd1306_WriteString(menu_items[item_sel_previous], Font_7x10, 1);
    ssd1306_DrawBitmap(4, 2, bitmap_icons[item_sel_previous], 16, 16, 1);

    // Render the currently selected menu item
    ssd1306_SetCursor(25, 5 + 20 + 2);
    ssd1306_WriteString(menu_items[item_selected], Font_7x10, 1);
    ssd1306_DrawBitmap(4, 24, bitmap_icons[item_selected], 16, 16, 1);


    // Render the next menu item
    ssd1306_SetCursor(25, 5 + 20 + 20 + 2 + 2);
    ssd1306_WriteString(menu_items[item_sel_next], Font_7x10, 1);
    ssd1306_DrawBitmap(4, 46, bitmap_icons[item_sel_next], 16, 16, 1);

    // Draw the selection outline around the currently selected item
    ssd1306_DrawBitmap(0, 22, bitmap_item_sel_outline, 128, 21, 1);

    // Render the scrollbar on the right side
    ssd1306_DrawBitmap(128 - 8, 0, bitmap_scrollbar_background, 8, 64, 1);
    ssd1306_DrawRectangle(125, 64 / NUM_ITEMS * item_selected, 128,
                           (64 / NUM_ITEMS * item_selected + (64 / NUM_ITEMS)), 1);
    ssd1306_DrawRectangle(126, 64 / NUM_ITEMS * item_selected, 127,
                           (64 / NUM_ITEMS * item_selected + (64 / NUM_ITEMS)), 1);
}


// ---------------------------- Accelerometer Data Display Function ----------------------------

/**
 * @brief Displays accelerometer data on an SSD1306 display.
 *
 * This function checks the connection to the MPU6050 accelerometer sensor by verifying
 * its device ID. If the sensor is connected properly, it will display the current
 * acceleration values along the X, Y, and Z axes on the SSD1306 display. The accelerometer
 * values are displayed in terms of G (gravitational force) and are updated in real-time.
 * If the sensor is not detected, the function will display a message indicating that
 * the sensor is off.
 *
 * Functionality:
 * - Reads the device ID from the MPU6050 to check if the sensor is connected.
 * - If the sensor is connected, displays the current acceleration values (Ax, Ay, Az)
 *   in G units on the display.
 * - If the sensor is not connected, displays an "INERCIAL OFF" message on the screen.
 *
 * Dependencies:
 * - The function uses the SSD1306 driver functions:
 *   - `ssd1306_Fill`: Clears the display by setting all pixels to a specific color.
 *   - `ssd1306_SetCursor`: Positions the cursor at a specified (x, y) location on the display.
 *   - `ssd1306_WriteString`: Writes a string at the current cursor position.
 *   - `ssd1306_FillRectangle`: Fills a rectangular area with a specified color.
 *   - `ssd1306_DrawRectangle`: Draws a rectangle with the specified coordinates.
 * 	 - `ssd1306_UpdateScreen()` Updates the display with the buffer content.
 *
 * @note The function assumes that the accelerometer data (`Ax`, `Ay`, `Az`) has already
 *       been obtained from the sensor and is available in the variables `Ax`, `Ay`, and `Az`.
 */
void print_accel(void) {
	uint8_t check;
	// Check device ID WHO_AM_I
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x75, 1, &check, 1, 1000);
	if (check == 0x68) {

		char buffer_float[7];
		ssd1306_Fill(0); // Clear the display

		// Display header
		ssd1306_SetCursor(13, 1);
		ssd1306_WriteString("ACCELEROMETER: ", Font_7x10, 1);
		ssd1306_FillRectangle(1, 15, 128, 16, 1);
		ssd1306_DrawRectangle(1, 20, 127, 63, 1);

		// Display acceleration values
		ssd1306_SetCursor(7, 25);
		ssd1306_WriteString("ACCEL.X: ", Font_6x8, 1);
		sprintf(buffer_float, "%.1f G", Ax);
		ssd1306_WriteString(buffer_float, Font_6x8, 1);

		ssd1306_SetCursor(7, 39);
		ssd1306_WriteString("ACCEL.Y: ", Font_6x8, 1);
		sprintf(buffer_float, "%.1f G", Ay);
		ssd1306_WriteString(buffer_float, Font_6x8, 1);

		ssd1306_SetCursor(7, 53);
		ssd1306_WriteString("ACCEL.Z: ", Font_6x8, 1);
		sprintf(buffer_float, "%.1f G", Az);
		ssd1306_WriteString(buffer_float, Font_6x8, 1);

	} else {
		// Display "INERCIAL OFF" if the sensor is not detected
		ssd1306_SetCursor(5, 30);
		ssd1306_WriteString("INERCIAL OFF", Font_6x8, 1);
	}
}


// ---------------------------- Gyroscope Data Display Function ----------------------------

/**
 * @brief Displays gyroscope data on an SSD1306 display.
 *
 * This function checks the connection to the MPU6050 gyroscope sensor by verifying
 * its device ID. If the sensor is connected properly, it will display the current
 * gyroscope rates along the X, Y, and Z axes on the SSD1306 display. The gyroscope
 * values are displayed in degrees per second (°/s) and are updated in real-time.
 * If the sensor is not detected, the function will display a message indicating that
 * the sensor is off.
 *
 * Functionality:
 * - Reads the device ID from the MPU6050 to check if the sensor is connected.
 * - If the sensor is connected, displays the current gyroscope rates (RateRoll, RatePitch, RateYaw)
 *   in degrees per second on the display.
 * - If the sensor is not connected, displays an "INERCIAL OFF" message on the screen.
 *
 * Dependencies:
 * - The function uses the SSD1306 driver functions:
 *   - `ssd1306_Fill`: Clears the display by setting all pixels to white.
 *   - `ssd1306_SetCursor`: Positions the cursor at a specified (x, y) location on the display.
 *   - `ssd1306_WriteString`: Writes a string at the current cursor position.
 *   - `ssd1306_FillRectangle`: Fills a rectangular area with a specified color.
 *   - `ssd1306_DrawRectangle`: Draws a rectangle with the specified coordinates.
 *
 * @note The function assumes that the gyroscope data (`RateRoll`, `RatePitch`, `RateYaw`) has already
 *       been obtained from the sensor and is available in the variables `RateRoll`, `RatePitch`, and `RateYaw`.
 */
void print_gyro(void) {
	uint8_t check;
	// Check device ID WHO_AM_I
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x75, 1, &check, 1, 1000);
	if (check == 0x68) {

		char buffer_floats[7];
		ssd1306_Fill(0); // Clear the display

		// Display header
		ssd1306_SetCursor(35, 1);
		ssd1306_WriteString("GYROSCOPE", Font_7x10, 1);
		ssd1306_FillRectangle(1, 15, 128, 16, 1);
		ssd1306_DrawRectangle(1, 20, 127, 63, 1);

		// Display gyroscope rates
		ssd1306_SetCursor(7, 25);
		ssd1306_WriteString("GYRO X: ", Font_6x8, 1);
		sprintf(buffer_floats, "%.0f", RateRoll);
		ssd1306_WriteString(buffer_floats, Font_6x8, 1);

		ssd1306_SetCursor(7, 39);
		ssd1306_WriteString("GYRO Y: ", Font_6x8, 1);
		sprintf(buffer_floats, "%.0f", RatePitch);
		ssd1306_WriteString(buffer_floats, Font_6x8, 1);

		ssd1306_SetCursor(7, 53);
		ssd1306_WriteString("GYRO Z: ", Font_6x8, 1);
		sprintf(buffer_floats, "%.0f", RateYaw);
		ssd1306_WriteString(buffer_floats, Font_6x8, 1);

	} else {
		// Display "INERCIAL OFF" if the sensor is not detected
		ssd1306_SetCursor(5, 30);
		ssd1306_WriteString("INERCIAL OFF", Font_6x8, 1);
	}
}

// --------------------------- Calibration Function ---------------------------

/**
 * @brief Calibrates the accelerometer and gyroscope of the MPU6050 sensor.
 *
 * This function performs calibration by averaging a fixed number of raw samples
 * from the accelerometer and gyroscope. It calculates the required offset values,
 * applies scaling factors, and writes the adjusted offsets to the MPU6050 registers.
 * Progress is displayed on the screen during the calibration process.
 *
 * Dependencies:
 * - The function uses the SSD1306 driver functions:
 *   - `ssd1306_Fill`: Clears the display by setting all pixels to white.
 *   - `ssd1306_SetCursor`: Positions the cursor at a specified (x, y) location on the display.
 *   - `ssd1306_WriteString`: Writes a string at the current cursor position.
 *   - `ssd1306_FillRectangle`: Fills a rectangular area with a specified color.
 *   - `ssd1306_DrawRectangle`: Draws a rectangle with the specified coordinates.
 *
 * @note The function uses a loop to collect MAX_SAMPLE readings and calculates the
 *       average. The offset values are rounded, scaled, and multiplied by -1 before
 *       being written to the respective MPU6050 registers.
 */
void calibration(void) {

    const uint16_t MAX_SAMPLE = 1000; 										// Number of samples for offset calculation
    uint8_t check1;                   										// Temporary variable for register data
    uint8_t check2;
    uint8_t rawData[14];              										// Array to store raw sensor data

    int16_t RAW_ACCEL_X, RAW_ACCEL_Y, RAW_ACCEL_Z; 							// Raw accelerometer values
    int16_t RAW_GYRO_X, RAW_GYRO_Y, RAW_GYRO_Z;   							// Raw gyroscope values

    int32_t ACC_RAW_ACCEL_X = 0, ACC_RAW_ACCEL_Y = 0, ACC_RAW_ACCEL_Z = 0; 	// Accumulated accelerometer values
    int32_t ACC_RAW_GYRO_X = 0, ACC_RAW_GYRO_Y = 0, ACC_RAW_GYRO_Z = 0;   	// Accumulated gyroscope values

    char buffer_float[5];             										// Buffer for displaying progress percentage
    uint16_t percentual;              										// Calibration progress percentage

	//Zeroing the raw inertial sensor values
	ACC_RAW_ACCEL_X = 0;
	ACC_RAW_ACCEL_Y = 0;
	ACC_RAW_ACCEL_Z = 0;
	ACC_RAW_GYRO_X = 0;
	ACC_RAW_GYRO_Y = 0;
	ACC_RAW_GYRO_Z = 0;

	ssd1306_Fill(0);														//Clears the display

	// Loop to collect MAX_SAMPLE raw sensor data and calculate averages
	for (uint16_t contador = 0; contador <= MAX_SAMPLE; ++contador) {
		HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, DATA_ARRAY_POINTER, 1, rawData,
				14, 1000);

		// Parsing raw accelerometer and gyroscope data
		RAW_ACCEL_X = ((int16_t) rawData[0] << 8) | (rawData[1]);
		RAW_ACCEL_Y = ((int16_t) rawData[2] << 8) | (rawData[3]);
		RAW_ACCEL_Z = ((int16_t) rawData[4] << 8) | (rawData[5]);
		RAW_GYRO_X = ((int16_t) rawData[8] << 8) | (rawData[9]);
		RAW_GYRO_Y = ((int16_t) rawData[10] << 8) | (rawData[11]);
		RAW_GYRO_Z = ((int16_t) rawData[12] << 8) | (rawData[13]);

		// Accumulating sensor data
		ACC_RAW_ACCEL_X += RAW_ACCEL_X;
		ACC_RAW_ACCEL_Y += RAW_ACCEL_Y;
		ACC_RAW_ACCEL_Z += RAW_ACCEL_Z;
		ACC_RAW_GYRO_X += RAW_GYRO_X;
		ACC_RAW_GYRO_Y += RAW_GYRO_Y;
		ACC_RAW_GYRO_Z += RAW_GYRO_Z;

		// Displaying calibration progress every 1%
		if (contador % 10 == 0) {
			percentual = (contador * 100) / MAX_SAMPLE;

			ssd1306_Fill(0);
			ssd1306_SetCursor(27, 1);
			ssd1306_WriteString("CALIBRATION: ", Font_7x10, 1);
			ssd1306_FillRectangle(1, 15, 128, 17, 1);

			ssd1306_SetCursor(57, 28);
			snprintf(buffer_float, sizeof(buffer_float), "%d\n", percentual);
			ssd1306_WriteString(buffer_float, Font_7x10, 1);
			ssd1306_WriteString("%", Font_7x10, 1);
			ssd1306_DrawRectangle(11, 40, 117, 55, 1);
			ssd1306_FillRectangle(11, 40,
					(11 + (percentual * (117 - 11)) / 100), 55, 1);
			ssd1306_UpdateScreen();
		}

		HAL_Delay(15);														// Small delay for sampling
	}

	// Calculating average raw sensor values
	RAW_ACCEL_X = (ACC_RAW_ACCEL_X / MAX_SAMPLE);
	RAW_ACCEL_Y = (ACC_RAW_ACCEL_Y / MAX_SAMPLE);
	RAW_ACCEL_Z = (ACC_RAW_ACCEL_Z / MAX_SAMPLE);
	RAW_GYRO_X = (ACC_RAW_GYRO_X / MAX_SAMPLE);
	RAW_GYRO_Y = (ACC_RAW_GYRO_Y / MAX_SAMPLE);
	RAW_GYRO_Z = (ACC_RAW_GYRO_Z / MAX_SAMPLE);

	// Applying scaling factors and calculating offset values
	RAW_ACCEL_X = round(-(RAW_ACCEL_X * accelScalingFactor ) * 1024.0);
	RAW_ACCEL_Y = round(-(RAW_ACCEL_Y * accelScalingFactor ) * 1024.0);
	RAW_ACCEL_Z = round(-(1 - (RAW_ACCEL_Z * accelScalingFactor )) * 1024.0);
	RAW_GYRO_X = round(-(RAW_GYRO_X * gyroScalingFactor ) * 32.768);
	RAW_GYRO_Y = round(-(RAW_GYRO_Y * gyroScalingFactor ) * 32.768);
	RAW_GYRO_Z = round(-(RAW_GYRO_Z * gyroScalingFactor ) * 32.768);

	// Reading values ​​from accelerometer registers
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, XA_OFFSET_H, 1, &check1, 1, 1000);
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, XA_OFFSET_L, 1, &check2, 1, 1000);
	int16_t Cancel_XA_Offset = (((((uint16_t) check1 << 8) | check2) >> 1)
			+ RAW_ACCEL_X) << 1;

	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, YA_OFFSET_H, 1, &check1, 1, 1000);
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, YA_OFFSET_L, 1, &check2, 1, 1000);
	int16_t Cancel_YA_Offset = (((((uint16_t) check1 << 8) | check2) >> 1)
			+ RAW_ACCEL_Y) << 1;

	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ZA_OFFSET_H, 1, &check1, 1, 1000);
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ZA_OFFSET_L, 1, &check2, 1, 1000);
	int16_t Cancel_ZA_Offset = (((((uint16_t) check1 << 8) | check2) >> 1)
			+ RAW_ACCEL_Z) << 1;

	//Writing offset values to accelerometer registers
	check1 = Cancel_XA_Offset >> 8;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, XA_OFFSET_H, 1, &check1, 1, 1000);
	check1 = Cancel_XA_Offset & 0xFF;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, XA_OFFSET_L, 1, &check1, 1, 1000);

	check1 = Cancel_YA_Offset >> 8;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, YA_OFFSET_H, 1, &check1, 1, 1000);
	check1 = Cancel_YA_Offset & 0xFF;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, YA_OFFSET_L, 1, &check1, 1, 1000);

	check1 = Cancel_ZA_Offset >> 8;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ZA_OFFSET_H, 1, &check1, 1, 1000);
	check1 = Cancel_ZA_Offset & 0xFF;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ZA_OFFSET_L, 1, &check1, 1, 1000);

	//Writing offset values to accelerometer gyroscope
	check1 = RAW_GYRO_X >> 8;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, XG_OFFSET_H, 1, &check1, 1, 1000);
	check1 = RAW_GYRO_X & 0xFF;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, XG_OFFSET_L, 1, &check1, 1, 1000);

	check1 = RAW_GYRO_Y >> 8;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, YG_OFFSET_H, 1, &check1, 1, 1000);
	check1 = RAW_GYRO_Y & 0xFF;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, YG_OFFSET_L, 1, &check1, 1, 1000);

	check1 = RAW_GYRO_Z >> 8;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ZG_OFFSET_H, 1, &check1, 1, 1000);
	check1 = RAW_GYRO_Z & 0xFF;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ZG_OFFSET_L, 1, &check1, 1, 1000);

	current_screen = !current_screen;
}

// ----------------------------- Kalman 1D Function -----------------------------

/**
 * @brief Implements a 1D Kalman filter for state estimation.
 *
 * This function applies a simplified Kalman filter algorithm to estimate
 * the state of a 1D system. It updates the state and uncertainty based on
 * the system input and measurement. The filter assumes fixed parameters for
 * the process noise and measurement noise.
 *
 * @param KalmanState       Pointer to the current state of the system.
 * @param KalmanUncertainty Pointer to the current uncertainty (variance) of the state.
 * @param KalmanInput       Pointer to the system input (control signal or prediction).
 * @param KalmanMeasurement Pointer to the measured value of the system state.
 *
 * @note The function uses a constant Kalman gain for simplicity, which is set to 0.1.
 *       The state is predicted using the input, and then corrected using the measurement.
 *       The updated state and uncertainty are stored in the Kalman1DOutput array.
 */
void kalman_1d(float *KalmanState, float *KalmanUncertainty, float *KalmanInput,
		float *KalmanMeasurement) {

    // Predict step: Update state estimate using input
    *KalmanState = *KalmanState + 0.004 * *KalmanInput;

    // Predict step: Update uncertainty with process noise
    *KalmanUncertainty = *KalmanUncertainty + 0.004 * 0.004 * 4.0 * 4.0;

    // Compute the Kalman gain (constant for simplicity in this implementation)
    float KalmanGain = 0.1;

    // Update step: Correct the state estimate using measurement
    *KalmanState = *KalmanState + KalmanGain * (*KalmanMeasurement - *KalmanState);

    // Update step: Correct the uncertainty
    *KalmanUncertainty = (1.0 - KalmanGain) * *KalmanUncertainty;

    // Store the updated state and uncertainty in the output array
    Kalman1DOutput[0] = *KalmanState;
    Kalman1DOutput[1] = *KalmanUncertainty;
}

// --------------------------- MPU6050 Read Measures ---------------------------

/**
 * @brief Reads raw sensor data from the MPU6050 and calculates scaled values.
 *
 * This function communicates with the MPU6050 IMU sensor via I2C to read
 * accelerometer, gyroscope, and temperature data. It converts the raw values
 * into meaningful physical quantities such as acceleration (in 'g'), angular
 * rates (in °/s), and temperature (in °C).
 *
 * @note The accelerometer full-scale range (FS_SEL) is configured to ±2g,
 *       and the gyroscope full-scale range is set to ±250°/s.
 *
 * @details
 * - The function starts by checking the WHO_AM_I register (0x75) to validate the device ID.
 * - Raw data for accelerometer, gyroscope, and temperature is read from the corresponding registers.
 * - The raw values are then scaled to obtain physical units.
 * - Roll and pitch angles are calculated based on accelerometer values using trigonometric functions.
 *
 *
 * @note This function assumes the MPU6050 has already been initialized and
 *       the I2C communication is properly set up.
 */
void MPU6050_Read_Measures(void) {

	uint8_t check;

	// Check device ID by reading WHO_AM_I register (0x75)
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x75, 1, &check, 1, 1000);

	// Verify MPU6050 device ID (0x68)
	if (check == 0x68) {
		uint8_t Rec_Data[14];

		// Read 14 bytes starting from ACCEL_XOUT_H (0x3B)
		HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x3B, 1, Rec_Data, 14, 1000);

		// Extract raw accelerometer data
		Accel_X_RAW = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
		Accel_Y_RAW = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
		Accel_Z_RAW = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);

		// Extract raw temperature data
		Temp_RAW = (int16_t) (Rec_Data[6] << 8 | Rec_Data[7]);

		// Extract raw gyroscope data
		Gyro_X_RAW = (int16_t) (Rec_Data[8] << 8 | Rec_Data[9]);
		Gyro_Y_RAW = (int16_t) (Rec_Data[10] << 8 | Rec_Data[11]);
		Gyro_Z_RAW = (int16_t) (Rec_Data[12] << 8 | Rec_Data[13]);

		 /*** Convert raw values to physical units ***/

		// Accelerometer: Convert to g (FS_SEL = 0, divisor = 16384)
		Ax = (float) Accel_X_RAW / 16384.0;
		Ay = (float) Accel_Y_RAW / 16384.0;
		Az = (float) Accel_Z_RAW / 16384.0;

		// Gyroscope: Convert to °/s (Full-scale = ±250°/s, divisor = 131)
		RateRoll = (float) Gyro_X_RAW / 131.0;
		RatePitch = (float) Gyro_Y_RAW / 131.0;
		RateYaw = (float) Gyro_Z_RAW / 131.0;

		// Temperature: Convert to °C (according to datasheet formula)
		temp = ((float) Temp_RAW) / 333.87 + 21.0;

		// Roll angle (degrees)
		AngleRoll = atan(Ay / sqrt(Ax * Ax + Az * Az)) * 1 / (3.142 / 180.0);

		// Pitch angle (degrees)
		AnglePitch = -atan(Ax / sqrt(Ay * Ay + Az * Az)) * 1 / (3.142 / 180.0);
	}
}

// --------------------------- Kalman Filter Display ---------------------------

/**
 * @brief Displays the filtered Roll and Pitch angles using the Kalman filter.
 *
 * This function uses the Kalman filter to compute the Roll and Pitch angles
 * based on input rates and angles, and then displays the results on the SSD1306
 * OLED display. The display includes a title ("KALMAN FILTER") at the top,
 * separated by a horizontal bar, and the filtered Roll and Pitch values shown below.
 *
 * @note This function assumes the Kalman filter is already implemented and that
 *       `kalman_1d()` is properly defined to process 1D Kalman filtering.
 *
 * @details
 * - The function calls `kalman_1d()` twice: once for Roll and once for Pitch.
 * - The filtered outputs (angle and uncertainty) are updated to global variables
 *   `KalmanAngleRoll`, `KalmanUncertaintyAngleRoll`, `KalmanAnglePitch`, and
 *   `KalmanUncertaintyAnglePitch`.
 * - The Roll and Pitch values are displayed on the SSD1306 OLED display using
 *   the provided font and format.
 * - The display layout includes:
 *   - A title "KALMAN FILTER" at the top.
 *   - A horizontal bar to separate the title from the values.
 *   - The Roll value displayed first, followed by the Pitch value.
 *
 * Dependencies:
 * - The function uses the SSD1306 driver functions:
 *   - `ssd1306_Fill`: Clears the display by setting all pixels to white.
 *   - `ssd1306_SetCursor`: Positions the cursor at a specified (x, y) location on the display.
 *   - `ssd1306_WriteString`: Writes a string at the current cursor position.
 *   - `ssd1306_FillRectangle`: Fills a rectangular area with a specified color.
 *   - `ssd1306_DrawRectangle`: Draws a rectangle with the specified coordinates.
 *
 */
void print_kalman(void) {

	// Update Roll angle using Kalman filter
	kalman_1d(&KalmanAngleRoll, &KalmanUncertaintyAngleRoll, &RateRoll,
			&AngleRoll);
	KalmanAngleRoll = Kalman1DOutput[0];
	KalmanUncertaintyAngleRoll = Kalman1DOutput[1];

	// Update Pitch angle using Kalman filter
	kalman_1d(&KalmanAnglePitch, &KalmanUncertaintyAnglePitch, &RatePitch,
			&AnglePitch);
	KalmanAnglePitch = Kalman1DOutput[0];
	KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

	char buffer_float[7]; 									// Buffer to hold formatted string values

	// --------------------------- Display Setup ---------------------------
	ssd1306_Fill(0); 										// Clear the display buffer
	ssd1306_SetCursor(14, 1);								// Set cursor for title
	ssd1306_WriteString("KALMAN FILTER: ", Font_7x10, 1);	// Write title

	// Draw horizontal bar under the title
	ssd1306_FillRectangle(1, 15, 128, 16, 1);

	//Draw the Container that haves the values
	ssd1306_DrawRectangle(1, 20, 127, 63, 1);

	// Display Roll Value
	ssd1306_SetCursor(7, 25); 								// Position cursor for Roll value
	ssd1306_WriteString("ROLL: ", Font_6x8, 1);
	sprintf(buffer_float, "%.1f", KalmanAngleRoll);
	ssd1306_WriteString(buffer_float, Font_7x10, 1);

	// Display Pitch Value
	ssd1306_SetCursor(7, 39); 								// Position cursor for Pitch value
	ssd1306_WriteString("PITCH: ", Font_6x8, 1);
	sprintf(buffer_float, "%.1f", KalmanAnglePitch);
	ssd1306_WriteString(buffer_float, Font_7x10, 1);
}

// --------------------------- Ultrasonic Sensor Measurement ---------------------------

/**
 * @brief Measures the distance using an ultrasonic sensor.
 *
 * This function triggers the ultrasonic sensor by sending a pulse to the TRIG pin
 * and waits for the echo signal on the ECHO pin. The time between the TRIG signal and
 * the ECHO response is measured and converted to a distance value using the speed of sound.
 * The result is stored in the global variable `Distance`.
 *
 * @note The function assumes that the timer `htim2` is configured and running, and
 *       the TRIG and ECHO pins are properly connected and initialized.
 *
 * @details
 * - The TRIG pin is set high for 10 µs, then set low to trigger the ultrasonic sensor.
 * - The function waits for the echo signal to go high (indicating the start of the pulse)
 *   and records the timer counter value (`Value1`).
 * - It then waits for the echo signal to go low (indicating the end of the pulse)
 *   and records the second timer counter value (`Value2`).
 * - The distance is calculated using the formula:
 *   \[
 *   \text{Distance} = \frac{(\text{Value2} - \text{Value1}) \times 0.034}{2}
 *   \]
 *   where 0.034 cm/µs is the speed of sound, and the division by 2 accounts for
 *   the round-trip of the sound wave.
 *
 *
 * @note The timeout mechanism ensures that the function does not get stuck in
 *       an infinite loop if the echo signal is not received within the expected time.
 *
 * Dependencies:
 * - `HAL_TIM_GET_COUNTER()` and `__HAL_TIM_SET_COUNTER()` for reading and resetting the timer.
 * - `HAL_GPIO_ReadPin()` and `HAL_GPIO_WritePin()` to control and read the GPIO pins.
 * - `HAL_GetTick()` for tracking time to avoid infinite loops.
 *
 */
void ultrassonico(void) {

	// Trigger the ultrasonic sensor by sending a pulse
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET); 	// Set TRIG pin HIGH
	__HAL_TIM_SET_COUNTER(&htim2, 0);						// Reset timer counter
	while (__HAL_TIM_GET_COUNTER (&htim2) < 10)
		;  													// Wait for 10 microseconds
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET); // Set TRIG pin LOW

	// Record the time when the echo pin goes high
	pMillis = HAL_GetTick(); 								// Store current time for timeout check
	// Wait for echo pin to go HIGH
	while (!(HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN))
			&& pMillis + 10 > HAL_GetTick())
		;

	// Capture timer value when echo pin goes HIGH
	Value1 = __HAL_TIM_GET_COUNTER(&htim2);

	// Wait for the echo pin to go low
	pMillis = HAL_GetTick(); 								// Store current time for timeout check
	// wait for the echo pin to go low
	while ((HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN))
			&& pMillis + 50 > HAL_GetTick())
		; 													// Wait for echo pin to go LOW

	// Capture timer value when echo pin goes LOW
	Value2 = __HAL_TIM_GET_COUNTER(&htim2);

	// Calculate distance based on the time difference between Value1 and Value2
	Distance = (Value2 - Value1) * 0.034 / 2;				// Distance in centimeters
}

// --------------------------- Ultrasonic Sensor Menu Display ---------------------------

/**
 * @brief Displays a user interface for configuring and monitoring ultrasonic sensor readings.
 *
 * This function implements a two-stage interface for interacting with the ultrasonic sensor:
 *
 * 1. **Configuration Stage:**
 *    Allows the user to set a distance threshold (limit) using a potentiometer. The value ranges
 *    from 5 cm to 200 cm. The selected threshold is displayed on the SSD1306 OLED display.
 *
 * 2. **Monitoring Stage:**
 *    Continuously measures the distance using the ultrasonic sensor and displays it both as a
 *    numerical value (in centimeters) and as a dynamic horizontal bar. A vertical line indicates
 *    the configured threshold. If the measured distance surpass the threshold, an alert is triggered
 *    by setting a GPIO pin (e.g., turning on a Buzzer).
 *
 * @note This function assumes that the SSD1306 OLED driver, ultrasonic sensor, and ADC (for potentiometer)
 *       are properly initialized and configured before calling this function.
 *
 * Dependencies:
 * - `ultrassonico()` updates the global variable `Distance` with the current distance reading.
 * - `low_pass_filter()` filters raw ADC and distance readings to reduce noise.
 * - HAL ADC and GPIO libraries for reading the potentiometer and handling the ENTER button and GPIO outputs.
 *
 * @details
 * - The function starts in the configuration stage, where the potentiometer value is read using the ADC.
 *   A low-pass filter is applied to stabilize the readings, and the resulting value is mapped to the range 5–200 cm.
 * - The user presses the ENTER button to exit the configuration stage and proceed to the monitoring stage.
 * - In the monitoring stage, the ultrasonic sensor measures the distance, and the value is displayed dynamically
 *   on the OLED display. The display updates a horizontal bar representing the current distance and a vertical line
 *   for the threshold. A GPIO pin is toggled based on whether the measured distance exceeds the threshold.
 *
 * @note The display uses the SSD1306 OLED driver functions:
 *   - `ssd1306_Fill`: Clears the display by setting all pixels to specific color.
 *   - `ssd1306_SetCursor`: Positions the cursor at a specified (x, y) location on the display.
 *   - `ssd1306_WriteString`: Writes a string at the current cursor position.
 *   - `ssd1306_FillRectangle`: Fills a rectangular area with a specified color.
 *   - `ssd1306_DrawRectangle`: Draws a rectangle with the specified coordinates.
 *
 */
void print_ultrassonico(void) {

	char buffer_float[7];							// Buffer to store formatted strings for display output

	// --------------------------- Configuration Stage ---------------------------
	while (intermediary_screen) {
		ssd1306_Fill(0); 							// Clear the display buffer
		ssd1306_SetCursor(27, 1);
		ssd1306_WriteString("ULTRASONIC: ", Font_7x10, 1);
		ssd1306_FillRectangle(1, 15, 128, 16, 1);	// Draw header rectangle
		ssd1306_DrawRectangle(1, 20, 127, 63, 1);	// Draw main display rectangle

		// Read potentiometer value using ADC
		HAL_ADC_Start(&hadc1);
		uint16_t raw_value = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);

		// Apply low-pass filter to smooth ADC readings
		uint16_t filtered_value = low_pass_filter(raw_value);

		// Map potentiometer value to range 5–200 cm
		Limit_Ultrasonic = 5 + ((filtered_value * 195) / 4095);

		// Display the threshold value
		ssd1306_SetCursor(9, 35);
		ssd1306_WriteString("Limit Value: ", Font_6x8, 1);
		sprintf(buffer_float, "%dcm", Limit_Ultrasonic);
		ssd1306_WriteString(buffer_float, Font_6x8, 1);

		// Check if ENTER button is pressed
		if (!HAL_GPIO_ReadPin(GPIOA, ENTER_BUTTON)) {
			intermediary_screen = 0;				// Exit configuration stage
			HAL_Delay(200);							// Debounce delay
		}

		ssd1306_UpdateScreen();						// Update the OLED display
	}

	// --------------------------- Monitoring Stage ---------------------------
	ssd1306_Fill(0); 								// Clear the display buffer
	ssd1306_SetCursor(27, 1);
	ssd1306_WriteString("ULTRASONIC: ", Font_7x10, 1);
	ssd1306_FillRectangle(1, 15, 128, 16, 1);
	ssd1306_DrawRectangle(1, 20, 127, 63, 1);

	// Measure distance using ultrasonic sensor
	ultrassonico();

	// Apply low-pass filter to smooth readings from the ultrasonic sensor
	uint16_t filtered_distance = low_pass_filter(Distance);

	// Display the measured distance
	ssd1306_SetCursor(5, 22);
	if (filtered_distance > 200) {
		filtered_distance = 200;
		ssd1306_WriteString("Dist= +200cm", Font_6x8, 1);

	} else if (filtered_distance < 5) {
		filtered_distance = 5;
		ssd1306_WriteString("Dist= -5cm", Font_6x8, 1);
	} else {
		ssd1306_WriteString("Dist= ", Font_6x8, 1);
		sprintf(buffer_float, "%d", filtered_distance);
		ssd1306_WriteString(buffer_float, Font_7x10, 1);
		ssd1306_WriteString("cm", Font_7x10, 1);
	}

	// Draws the line separating the distance values ​​from the horizontal distance bar
	ssd1306_DrawRectangle(1, 33, 127, 33, 1);

	// Condition if the number has 2 or 3 digits to help with formatting
	if (Limit_Ultrasonic >= 100) {
		ssd1306_DrawRectangle(95, 20, 127, 33, 1);
		ssd1306_SetCursor(97, 22);
	} else {
		ssd1306_DrawRectangle(99, 20, 127, 33, 1);
		ssd1306_SetCursor(102, 22);
	}

	// Write the threshold number
	sprintf(buffer_float, "%dcm", Limit_Ultrasonic);
	ssd1306_WriteString(buffer_float, Font_6x8, 1);

	// Draw Distance marker
	x_distance = (uint8_t) (((Distance - 5) * 128) / (200 - 5)) + 1;
	ssd1306_FillRectangle(1, 33, x_distance, 63, 1);

	// Draw threshold marker
	x_limit = (uint8_t) (((Limit_Ultrasonic - 5) * 128) / (200 - 5)) + 1;

	// Color inversion condition for better visualization
	if (x_distance > x_limit)
		ssd1306_DrawRectangle(x_limit, 33, x_limit + 1, 63, 0);
	else
		ssd1306_DrawRectangle(x_limit, 33, x_limit + 1, 63, 1);

	// Alert condition: activate GPIO if distance exceeds threshold
	if (x_distance <= x_limit)
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

}

#endif /*DEFINES_FUNCTIONS_H*/
