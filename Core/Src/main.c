/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include <math.h>
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define DEG2RAD 3.14159265f / 180.0f
#define RAD2DEG 180.0f / 3.14159265f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

float sigma = DEG2RAD * 0.5f;   // 0.5 deg/s

// Pre-allocated matrices for CMSIS-DSP (flat arrays)
static float F_flat[16];
static float P_flat[16];
static float Q_flat[16];
static float temp1_flat[16];
static float temp2_flat[16];
static float F_T_flat[16];

static float H_flat[12];
static float HP_flat[12];
static float S_flat[9];
static float R_flat[9];
static float HP_T_flat[12];
static float S_inv_flat[9];
static float PHt_flat[12];
static float K_flat[12];
static float KH_flat[16];
static float I_KH_flat[16];
static float P_new_flat[16];

// CMSIS-DSP matrix instances
static arm_matrix_instance_f32 F_mat = { 4, 4, F_flat };
static arm_matrix_instance_f32 P_mat = { 4, 4, P_flat };

static arm_matrix_instance_f32 temp1_mat = { 4, 4, temp1_flat };
static arm_matrix_instance_f32 temp2_mat = { 4, 4, temp2_flat };
static arm_matrix_instance_f32 F_T_mat = { 4, 4, F_T_flat };


// test the github
static arm_matrix_instance_f32 H_mat = { 3, 4, H_flat };
static arm_matrix_instance_f32 HP_mat = { 3, 4, HP_flat };
static arm_matrix_instance_f32 S_mat = { 3, 3, S_flat };
static arm_matrix_instance_f32 HP_T_mat = { 4, 3, HP_T_flat };
static arm_matrix_instance_f32 S_inv_mat = { 3, 3, S_inv_flat };
static arm_matrix_instance_f32 H_T_mat = { 4, 3, PHt_flat };
static arm_matrix_instance_f32 PHt_mat = { 4, 3, PHt_flat };
static arm_matrix_instance_f32 K_mat = { 4, 3, K_flat };
static arm_matrix_instance_f32 KH_mat = { 4, 4, KH_flat };
static arm_matrix_instance_f32 I_KH_mat = { 4, 4, I_KH_flat };
static arm_matrix_instance_f32 P_new_mat = { 4, 4, P_new_flat };

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	uint8_t data[2];
	uint8_t buf[200];

	// Raw sensor values
	int16_t AccX, AccY, AccZ;
	int16_t GyroX, GyroY, GyroZ;

	// Converted values
	float Ax_g, Ay_g, Az_g;
	float Gx_dps, Gy_dps, Gz_dps;

	// Gyro offsets
	float Gx_offset = 0, Gy_offset = 0, Gz_offset = 0;

	// Timing
	uint32_t start_time, execution_time;
	float dt;

	// Quaternion State
	float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

	// Output angles
	float Roll_q, Pitch_q, Yaw_q;

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_I2C1_Init();
	MX_USB_DEVICE_Init();

	/* USER CODE BEGIN 2 */
#define MPU_ADDR (0x68 << 1)

	uint8_t config;

	// Sample Rate = 200Hz
	config = 0x04;
	HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, 0x19, I2C_MEMADD_SIZE_8BIT, &config, 1,
			100);

	// Gyroscope Range = ±500 dps
	config = 0x08;
	HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, 0x1B, I2C_MEMADD_SIZE_8BIT, &config, 1,
			100);

	// Accelerometer Range = ±4g
	config = 0x08;
	HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, 0x1C, I2C_MEMADD_SIZE_8BIT, &config, 1,
			100);

	// Digital Low Pass Filter = 184Hz
	config = 0x01;
	HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, 0x1A, I2C_MEMADD_SIZE_8BIT, &config, 1,
			100);

	HAL_Delay(100);

	// -------- Gyroscope Calibration --------
	int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
	uint16_t samples = 0;
	uint32_t calib_start = HAL_GetTick();

	while ((HAL_GetTick() - calib_start) < 10000) {
		HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, 0x43, I2C_MEMADD_SIZE_8BIT, data, 2,
				100);
		GyroX = (data[0] << 8) | data[1];
		HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, 0x45, I2C_MEMADD_SIZE_8BIT, data, 2,
				100);
		GyroY = (data[0] << 8) | data[1];
		HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, 0x47, I2C_MEMADD_SIZE_8BIT, data, 2,
				100);
		GyroZ = (data[0] << 8) | data[1];

		gx_sum += GyroX;
		gy_sum += GyroY;
		gz_sum += GyroZ;
		samples++;
		HAL_Delay(5);
	}

	Gx_offset = (float) gx_sum / samples;
	Gy_offset = (float) gy_sum / samples;
	Gz_offset = (float) gz_sum / samples;

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		start_time = HAL_GetTick();

		//================ READ ACC =================
		HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, 0x3B, I2C_MEMADD_SIZE_8BIT, data, 2,
				100);
		AccX = (data[0] << 8) | data[1];

		HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, 0x3D, I2C_MEMADD_SIZE_8BIT, data, 2,
				100);
		AccY = (data[0] << 8) | data[1];

		HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, 0x3F, I2C_MEMADD_SIZE_8BIT, data, 2,
				100);
		AccZ = (data[0] << 8) | data[1];

		//================ READ GYRO =================
		HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, 0x43, I2C_MEMADD_SIZE_8BIT, data, 2,
				100);
		GyroX = (data[0] << 8) | data[1];

		HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, 0x45, I2C_MEMADD_SIZE_8BIT, data, 2,
				100);
		GyroY = (data[0] << 8) | data[1];

		HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, 0x47, I2C_MEMADD_SIZE_8BIT, data, 2,
				100);
		GyroZ = (data[0] << 8) | data[1];

		//================ CONVERSION =================
		Ax_g = AccX / 8192.0f;
		Ay_g = AccY / 8192.0f;
		Az_g = AccZ / 8192.0f;

		Gx_dps = (GyroX - Gx_offset) / 65.5f;
		Gy_dps = (GyroY - Gy_offset) / 65.5f;
		Gz_dps = (GyroZ - Gz_offset) / 65.5f;

		//================ DT ==============================
		execution_time = HAL_GetTick() - start_time;
		while (execution_time < 5) {
			execution_time = HAL_GetTick() - start_time;
		}
		dt = execution_time / 1000.0f;

		//================ GYRO TO RAD ======================
		float gx = Gx_dps * 0.0174533f;
		float gy = Gy_dps * 0.0174533f;
		float gz = Gz_dps * 0.0174533f;

		//================ KALMAN PREDICTION =================
		// Quaternion update using gyroscope data
		float qw = q0, qx = q1, qy = q2, qz = q3;

		q0 = qw - 0.5f * dt * gx * qx - 0.5f * dt * gy * qy
				- 0.5f * dt * gz * qz;
		q1 = qx + 0.5f * dt * gx * qw - 0.5f * dt * gy * qz
				+ 0.5f * dt * gz * qy;
		q2 = qy + 0.5f * dt * gx * qz + 0.5f * dt * gy * qw
				- 0.5f * dt * gz * qx;
		q3 = qz - 0.5f * dt * gx * qy + 0.5f * dt * gy * qx
				+ 0.5f * dt * gz * qw;

		// State transition matrix F
		float hdt = 0.5f * dt;
		F_flat[0] = 1.0f;
		F_flat[1] = -hdt * gx;
		F_flat[2] = -hdt * gy;
		F_flat[3] = -hdt * gz;
		F_flat[4] = hdt * gx;
		F_flat[5] = 1.0f;
		F_flat[6] = hdt * gz;
		F_flat[7] = -hdt * gy;
		F_flat[8] = hdt * gy;
		F_flat[9] = -hdt * gz;
		F_flat[10] = 1.0f;
		F_flat[11] = hdt * gx;
		F_flat[12] = hdt * gz;
		F_flat[13] = hdt * gy;
		F_flat[14] = -hdt * gx;
		F_flat[15] = 1.0f;

		// Process noise covariance Q
		float q_var = sigma * sigma * dt * dt;
		arm_fill_f32(0.0f, Q_flat, 16);
		Q_flat[0] = q_var;
		Q_flat[5] = q_var;
		Q_flat[10] = q_var;
		Q_flat[15] = q_var;

		// Covariance prediction: P = F * P * F^T + Q
		arm_mat_mult_f32(&F_mat, &P_mat, &temp1_mat);
		arm_mat_trans_f32(&F_mat, &F_T_mat);
		arm_mat_mult_f32(&temp1_mat, &F_T_mat, &temp2_mat);
		arm_add_f32(temp2_flat, Q_flat, P_flat, 16);

		//================ KALMAN UPDATE =================
		// Measurement vector from accelerometer
		float z[3];
		float acc_norm = sqrtf(Ax_g * Ax_g + Ay_g * Ay_g + Az_g * Az_g);
		if (acc_norm > 0.000001f) {
			z[0] = Ax_g / acc_norm;
			z[1] = Ay_g / acc_norm;
			z[2] = Az_g / acc_norm;
		}

		// Predicted measurement from quaternion
		float h[3];
		h[0] = 2.0f * (q1 * q3 - q0 * q2);
		h[1] = 2.0f * (q2 * q3 + q0 * q1);
		h[2] = 1.0f - 2.0f * (q1 * q1 + q2 * q2);

		// Measurement noise covariance R
		arm_fill_f32(0.0f, R_flat, 9);
		R_flat[0] = 0.01f;
		R_flat[4] = 0.01f;
		R_flat[8] = 0.01f;

		// Jacobian matrix H
		H_flat[0] = -2 * q2;
		H_flat[1] = 2 * q3;
		H_flat[2] = -2 * q0;
		H_flat[3] = 2 * q1;
		H_flat[4] = 2 * q1;
		H_flat[5] = 2 * q0;
		H_flat[6] = 2 * q3;
		H_flat[7] = 2 * q2;
		H_flat[8] = 0.0f;
		H_flat[9] = -4 * q1;
		H_flat[10] = -4 * q2;
		H_flat[11] = 0.0f;

		// Innovation
		float v[3];
		v[0] = z[0] - h[0];
		v[1] = z[1] - h[1];
		v[2] = z[2] - h[2];

		// Innovation covariance S = H*P*H^T + R
		arm_mat_mult_f32(&H_mat, &P_mat, &HP_mat);
		arm_mat_trans_f32(&HP_mat, &HP_T_mat);
		arm_mat_mult_f32(&HP_mat, &HP_T_mat, &S_mat);
		arm_add_f32(S_flat, R_flat, S_flat, 9);

		// Inverse of S
		arm_mat_inverse_f32(&S_mat, &S_inv_mat);

		// Kalman gain K = P * H^T * S_inv
		arm_mat_trans_f32(&H_mat, &H_T_mat);
		arm_mat_mult_f32(&P_mat, &H_T_mat, &PHt_mat);
		arm_mat_mult_f32(&PHt_mat, &S_inv_mat, &K_mat);

		// State update
		float Kv[4];
		arm_matrix_instance_f32 v_mat = { 3, 1, v };
		arm_matrix_instance_f32 Kv_mat = { 4, 1, Kv };
		arm_mat_mult_f32(&K_mat, &v_mat, &Kv_mat);

		q0 += Kv[0];
		q1 += Kv[1];
		q2 += Kv[2];
		q3 += Kv[3];

		// Covariance update
		arm_mat_mult_f32(&K_mat, &H_mat, &KH_mat);

		// I_KH = I - KH
		float I_flat[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };
		arm_sub_f32(I_flat, KH_flat, I_KH_flat, 16);

		// P = (I - K*H) * P
		arm_mat_mult_f32(&I_KH_mat, &P_mat, &P_new_mat);
		arm_copy_f32(P_new_flat, P_flat, 16);

		//================ CONVERT TO EULER ANGLES =================
		Roll_q = atan2f(2.0f * (q0 * q1 + q2 * q3),
				1.0f - 2.0f * (q1 * q1 + q2 * q2));
		Roll_q = Roll_q * RAD2DEG;

		float t = 2.0f * (q0 * q2 - q3 * q1);
		if (t > 1.0f)
			t = 1.0f;
		if (t < -1.0f)
			t = -1.0f;
		Pitch_q = asinf(t) * RAD2DEG;
		Yaw_q = atan2f(2.0f * (q0 * q3 + q1 * q2),
				1.0f - 2.0f * (q2 * q2 + q3 * q3));
		Yaw_q = Yaw_q * RAD2DEG;

		// Send to USB
		snprintf((char*) buf, sizeof(buf), "Roll %.2f Pitch %.2f Yaw %.2f\r\n",
				Roll_q, Pitch_q, Yaw_q);
		CDC_Transmit_FS(buf, strlen((char*) buf));

		/* USER CODE END WHILE */
		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 25;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 400000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	__disable_irq();
	while (1) {
	}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {
}
#endif /* USE_FULL_ASSERT */
