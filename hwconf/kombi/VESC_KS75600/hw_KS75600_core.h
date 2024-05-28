/*
	Copyright 2022 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#ifndef HW_KS75600_CORE_H_
#define HW_KS75600_CORE_H_

#ifdef HWKS75600
  #define HW_NAME					"KS75600"
#elif defined(HWKS75600_01) // 0.1 mOhm shunts
  #define HW_NAME					"KS75600_01"
#else
  #error "Must define hardware type"
#endif

// HW properties
#define HW_HAS_3_SHUNTS //三分流电阻
#define HW_HAS_PHASE_FILTERS //相线电压滤波器（模拟开关控制的一个滤波电阻）
#define HW_HAS_PHASE_SHUNTS//相线分流电阻采样模式//TODO测试这个，因为str500没有这个

// Macros
#define LED_GREEN_GPIO			GPIOC
#define LED_GREEN_PIN			  5
#define LED_RED_GPIO			  GPIOA
#define LED_RED_PIN				  15

#define LED_GREEN_ON()			palSetPad(LED_GREEN_GPIO, LED_GREEN_PIN)
#define LED_GREEN_OFF()			palClearPad(LED_GREEN_GPIO, LED_GREEN_PIN)
#define LED_RED_ON()			  palSetPad(LED_RED_GPIO, LED_RED_PIN)
#define LED_RED_OFF()			  palClearPad(LED_RED_GPIO, LED_RED_PIN)

#define PHASE_FILTER_GPIO		  GPIOC
#define PHASE_FILTER_PIN		  9
#define PHASE_FILTER_ON()		  palSetPad(PHASE_FILTER_GPIO, PHASE_FILTER_PIN)
#define PHASE_FILTER_OFF()		palClearPad(PHASE_FILTER_GPIO, PHASE_FILTER_PIN)

//#define AUX_GPIO				GPIOA
//#define AUX_PIN					15
//#define AUX_ON()				palSetPad(AUX_GPIO, AUX_PIN)
//#define AUX_OFF()				palClearPad(AUX_GPIO, AUX_PIN)

#define CURRENT_FILTER_ON()		palSetPad(GPIOC, 12)//TODO测试这个，因为str500没有这个
#define CURRENT_FILTER_OFF()	palClearPad(GPIOC, 12)//TODO测试这个，因为str500没有这个

/*
 * ADC Vector
 *
 * 0  (1):	IN0		SENS1
 * 1  (2):	IN1		SENS2
 * 2  (3):	IN2		SENS3
 * 3  (1):	IN10	CURR1
 * 4  (2):	IN11	CURR2
 * 5  (3):	IN12	CURR3
 * 6  (1):	IN5		ADC_EXT1
 * 7  (2):	IN6		ADC_EXT2
 * 8  (3):	IN3		TEMP_MOS
 * 9  (1):	IN14	TEMP_MOTOR
 * 10 (2):	IN15	ADC_EXT3
 * 11 (3):	IN13	AN_IN
 * 12 (1):	Vrefint
 * 13 (2):	IN0		SENS1
 * 14 (3):	IN1		SENS2
 * 15 (1):  IN8		TEMP_MOS_2
 * 16 (2):  IN9		TEMP_MOS_3
 * 17 (3):  IN3		SENS3
 */

#define HW_ADC_CHANNELS			18
//#define HW_ADC_CHANNELS_EXTRA	8
#define HW_ADC_INJ_CHANNELS		3
#define HW_ADC_NBR_CONV			6

// ADC Indexes
#define ADC_IND_SENS1			3
#define ADC_IND_SENS2			4
#define ADC_IND_SENS3			5
#define ADC_IND_CURR1			0
#define ADC_IND_CURR2			1
#define ADC_IND_CURR3			2
#define ADC_IND_VIN_SENS		11
#define ADC_IND_EXT				6
#define ADC_IND_EXT2			7
#define ADC_IND_EXT3			10
#define ADC_IND_TEMP_MOS		8
#define ADC_IND_TEMP_MOS_2		15
#define ADC_IND_TEMP_MOS_3		16
#define ADC_IND_TEMP_MOTOR		9
#define ADC_IND_VREFINT			12
// ADC macros and settings

// Component parameters (can be overridden)
#ifndef V_REG
#define V_REG					3.3
#endif
#ifndef VIN_R1
#define VIN_R1					56000.0
#endif
#ifndef VIN_R2
#define VIN_R2					2200.0
#endif
#ifndef CURRENT_AMP_GAIN
#define CURRENT_AMP_GAIN		0.6622516556291391//放大器前面有一个5.1k和10k分压，放大器只做跟随
#endif
#ifndef CURRENT_SHUNT_RES
#ifdef HWKS75600
#define CURRENT_SHUNT_RES		(0.004)
//500A-2000mv的转换关系，在5V的传感器上转换比为0.004v/a（相当于0.004欧电阻）
#else
#define CURRENT_SHUNT_RES		(0.0001 / 4.0)
#endif
#endif

// Input voltage
#define GET_INPUT_VOLTAGE()		((V_REG / 4095.0) * (float)ADC_Value[ADC_IND_VIN_SENS] * ((VIN_R1 + VIN_R2) / VIN_R2))

// NTC Termistors
//TODO NTC参数后面要调整，包括电机和mos的
#define NTC_RES(adc_val)		((4095.0 * 10000.0) / adc_val - 10000.0)
#define NTC_TEMP(adc_ind)		hw100_400_get_temp()

#define NTC_RES_MOTOR(adc_val)	(10000.0 / ((4095.0 / (float)adc_val) - 1.0)) // Motor temp sensor on low side
#define NTC_TEMP_MOTOR(beta)	(1.0 / ((logf(NTC_RES_MOTOR(ADC_Value[ADC_IND_TEMP_MOTOR]) / 10000.0) / beta) + (1.0 / 298.15)) - 273.15)

#define NTC_TEMP_MOS1()			(1.0 / ((logf(NTC_RES(ADC_Value[ADC_IND_TEMP_MOS]) / 10000.0) / 3380.0) + (1.0 / 298.15)) - 273.15)
#define NTC_TEMP_MOS2()			(1.0 / ((logf(NTC_RES(ADC_Value[ADC_IND_TEMP_MOS_2]) / 10000.0) / 3380.0) + (1.0 / 298.15)) - 273.15)
#define NTC_TEMP_MOS3()			(1.0 / ((logf(NTC_RES(ADC_Value[ADC_IND_TEMP_MOS_3]) / 10000.0) / 3380.0) + (1.0 / 298.15)) - 273.15)

// Voltage on ADC channel
#define ADC_VOLTS(ch)			((float)ADC_Value[ch] / 4096.0 * V_REG)

// COMM-port ADC GPIOs
#define HW_ADC_EXT_GPIO			GPIOB
#define HW_ADC_EXT_PIN			0
#define HW_ADC_EXT2_GPIO		GPIOB
#define HW_ADC_EXT2_PIN			1

// UART Peripheral
#define HW_UART_DEV				  SD3
#define HW_UART_GPIO_AF			GPIO_AF_USART3
#define HW_UART_TX_PORT			GPIOB
#define HW_UART_TX_PIN			10
#define HW_UART_RX_PORT			GPIOB
#define HW_UART_RX_PIN			11

// Permanent UART Peripheral (SWD/ESP)
#define HW_UART_P_BAUD			115200
#define HW_UART_P_DEV			  SD4
#define HW_UART_P_GPIO_AF		GPIO_AF_UART4
#define HW_UART_P_TX_PORT		GPIOC
#define HW_UART_P_TX_PIN		10
#define HW_UART_P_RX_PORT		GPIOC
#define HW_UART_P_RX_PIN		11

// ICU Peripheral for servo decoding
#define HW_USE_SERVO_TIM4
#define HW_ICU_TIMER			  TIM4
#define HW_ICU_TIM_CLK_EN()		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE)
#define HW_ICU_DEV				  ICUD4
#define HW_ICU_CHANNEL			ICU_CHANNEL_1
#define HW_ICU_GPIO_AF			GPIO_AF_TIM4
#define HW_ICU_GPIO				  GPIOB
#define HW_ICU_PIN				  6

// I2C Peripheral
#define HW_I2C_DEV				  I2CD2
#define HW_I2C_GPIO_AF			GPIO_AF_I2C2
#define HW_I2C_SCL_PORT			GPIOB
#define HW_I2C_SCL_PIN			10
#define HW_I2C_SDA_PORT			GPIOB
#define HW_I2C_SDA_PIN			11

// Hall/encoder pins
#define HW_HALL_ENC_GPIO1		GPIOC
#define HW_HALL_ENC_PIN1		6
#define HW_HALL_ENC_GPIO2		GPIOC
#define HW_HALL_ENC_PIN2		7
#define HW_HALL_ENC_GPIO3		GPIOC
#define HW_HALL_ENC_PIN3		8
#define HW_ENC_TIM				  TIM3
#define HW_ENC_TIM_AF			  GPIO_AF_TIM3
#define HW_ENC_TIM_CLK_EN()		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE)
#define HW_ENC_EXTI_PORTSRC		EXTI_PortSourceGPIOC
#define HW_ENC_EXTI_PINSRC		EXTI_PinSource8
#define HW_ENC_EXTI_CH			EXTI9_5_IRQn
#define HW_ENC_EXTI_LINE		EXTI_Line8
#define HW_ENC_EXTI_ISR_VEC		EXTI9_5_IRQHandler
#define HW_ENC_TIM_ISR_CH		TIM3_IRQn
#define HW_ENC_TIM_ISR_VEC		TIM3_IRQHandler

// SPI pins
#define HW_SPI_DEV				  SPID1
#define HW_SPI_GPIO_AF			GPIO_AF_SPI1
#define HW_SPI_PORT_NSS			GPIOA
#define HW_SPI_PIN_NSS			4
#define HW_SPI_PORT_SCK			GPIOA
#define HW_SPI_PIN_SCK			5
#define HW_SPI_PORT_MOSI		GPIOA
#define HW_SPI_PIN_MOSI			7
#define HW_SPI_PORT_MISO		GPIOA
#define HW_SPI_PIN_MISO			6

// IMU
// #define LSM6DS3_NSS_GPIO		GPIOC
// #define LSM6DS3_NSS_PIN			13
// #define LSM6DS3_SCK_GPIO		GPIOB
// #define LSM6DS3_SCK_PIN			12
// #define LSM6DS3_MOSI_GPIO		GPIOB
// #define LSM6DS3_MOSI_PIN		3
// #define LSM6DS3_MISO_GPIO		GPIOB
// #define LSM6DS3_MISO_PIN		4
// #define BMI160_SPI_PORT_NSS		GPIOC
// #define BMI160_SPI_PIN_NSS		13
// #define BMI160_SPI_PORT_SCK		GPIOB
// #define BMI160_SPI_PIN_SCK		12
// #define BMI160_SPI_PORT_MOSI	GPIOB
// #define BMI160_SPI_PIN_MOSI		3
// #define BMI160_SPI_PORT_MISO	GPIOB
// #define BMI160_SPI_PIN_MISO		4

// Measurement macros
#define ADC_V_L1				ADC_Value[ADC_IND_SENS1]
#define ADC_V_L2				ADC_Value[ADC_IND_SENS2]
#define ADC_V_L3				ADC_Value[ADC_IND_SENS3]
#define ADC_V_ZERO				(ADC_Value[ADC_IND_VIN_SENS] / 2)

// Macros
#define READ_HALL1()			palReadPad(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1)
#define READ_HALL2()			palReadPad(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2)
#define READ_HALL3()			palReadPad(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3)

// Override dead time. See the stm32f4 reference manual for calculating this value.
#define HW_DEAD_TIME_NSEC		1000.0

// Default setting overrides
#ifndef MCCONF_L_MIN_VOLTAGE
#define MCCONF_L_MIN_VOLTAGE			12.0		// Minimum input voltage
#endif
#ifndef MCCONF_L_MAX_VOLTAGE
#define MCCONF_L_MAX_VOLTAGE			72.0	// Maximum input voltage
#endif
#ifndef MCCONF_DEFAULT_MOTOR_TYPE
#define MCCONF_DEFAULT_MOTOR_TYPE		MOTOR_TYPE_FOC
#endif
#ifndef MCCONF_FOC_F_ZV
#define MCCONF_FOC_F_ZV					30000.0
#endif
#ifndef MCCONF_L_MAX_ABS_CURRENT
#define MCCONF_L_MAX_ABS_CURRENT		420.0	// The maximum absolute current above which a fault is generated
#endif
#ifndef MCCONF_FOC_SAMPLE_V0_V7
#define MCCONF_FOC_SAMPLE_V0_V7			false	// Run control loop in both v0 and v7 (requires phase shunts)
#endif
#ifndef MCCONF_L_IN_CURRENT_MAX
#define MCCONF_L_IN_CURRENT_MAX			250.0	// Input current limit in Amperes (Upper)
#endif
#ifndef MCCONF_L_IN_CURRENT_MIN
#define MCCONF_L_IN_CURRENT_MIN			-200.0	// Input current limit in Amperes (Lower)
#endif
#ifndef APPCONF_SHUTDOWN_MODE
#define APPCONF_SHUTDOWN_MODE			SHUTDOWN_MODE_ALWAYS_ON
#endif

// Setting limits
#ifdef HWKS75600
#define HW_LIM_CURRENT			-500.0, 500.0
#define HW_LIM_CURRENT_IN		-500.0, 500.0
#define HW_LIM_CURRENT_ABS		0.0, 720.0
#else
#define HW_LIM_CURRENT			-400.0, 400.0
#define HW_LIM_CURRENT_IN		-400.0, 400.0
#define HW_LIM_CURRENT_ABS		0.0, 480.0
#endif
#define HW_LIM_VIN				11.0, 72.0
#define HW_LIM_ERPM				-200e3, 200e3
#define HW_LIM_DUTY_MIN			0.0, 0.1
#define HW_LIM_DUTY_MAX			0.0, 0.99
#define HW_LIM_TEMP_FET			-40.0, 110.0

float hwks75600_get_temp(void);

#endif /* HW_KS75600_CORE_H_ */
