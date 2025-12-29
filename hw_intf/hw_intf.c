#include "spi.h"
#include "tim.h"
#include "i2c.h"
#include "usart.h"

#include "OpenDrone_FC_HwIntf.h"
#include "OpenDrone_FC_Define.h"
#include "mpu6050.h"
#include "hmc5883l.h"
#include "qmc5883l.h"
#include "bmp280.h"
#include "icm42688.h"
#include "esc_dshot.h"
#include "nrf24l01.h"

#define APP_TIM 						htim3

#ifdef USE_SERIAL_DEBUG
#define UART_DEBUG  					huart1
#endif

#ifdef USE_NRF24L01
#define NRF24L01_SPI                 	hspi2
#define NRF24L01_GPIO_PORT_CS         	GPIOB
#define NRF24L01_GPIO_PIN_CS      		GPIO_PIN_12
#define NRF24L01_GPIO_PORT_CE         	GPIOA
#define NRF24L01_GPIO_PIN_CE      		GPIO_PIN_8
#define NRF24L01_GPIO_PORT_IRQ        	GPIOC
#define NRF24L01_GPIO_PIN_IRQ         	GPIO_PIN_9
#endif

#ifdef USE_SX1278
#define SX1278_SPI                 		hspi2
#define SX1278_GPIO_PORT_CS         	GPIOB
#define SX1278_GPIO_PIN_CS      		GPIO_PIN_4
#define SX1278_GPIO_PORT_RST         	GPIOB
#define SX1278_GPIO_PIN_RST     		GPIO_PIN_5
#define SX1278_GPIO_PORT_IRQ            GPIOB
#define SX1278_GPIO_PIN_IRQ         	GPIO_PIN_2
#endif

#ifdef USE_MPU6050
#define I2C_ADDR_MPU6050   				(MPU6050_I2C_ADDR<<1)
#define MPU6050_I2C  					hi2c2
#endif

#ifdef USE_ICM42688
#define ICM42688_SPI  					hspi1
#define ICM42688_GPIO_PORT_CS         	GPIOA
#define ICM42688_GPIO_PIN_CS      		GPIO_PIN_4
#endif

#ifdef USE_HMC5883L
#define HMC5883L_I2C  					hi2c2
#endif

#ifdef USE_QMC5883L
#define I2C_ADDR_QMC5883L				(QMC5883L_I2C_ADDR<<1)
#define QMC5883L_I2C  					hi2c2
#endif

#ifdef USE_BMP280
#define BMP280_I2C                          hi2c2
#define I2C_ADDR_BMP280                     (BMP280_I2C_ADDR_0<<1)
#endif

#ifdef USE_ESC_DSHOT
#define FL_ESC_DSHOT_TIM                htim3
#define FL_ESC_DSHOT_TIM_CHNL           TIM_CHANNEL_3

#define FR_ESC_DSHOT_TIM                htim5
#define FR_ESC_DSHOT_TIM_CHNL           TIM_CHANNEL_3

#define BL_ESC_DSHOT_TIM                htim5
#define BL_ESC_DSHOT_TIM_CHNL           TIM_CHANNEL_4

#define BR_ESC_DSHOT_TIM                htim3
#define BR_ESC_DSHOT_TIM_CHNL           TIM_CHANNEL_4
#endif

uint32_t hw_intf_get_time_us(void)
{
	return __HAL_TIM_GET_COUNTER(&htim2);
}

void hw_intf_delay_ms(uint32_t time_ms)
{
	HAL_Delay(time_ms);
}

#ifdef USE_SERIAL_DEBUG
err_code_t hw_intf_uart_debug_send(uint8_t *log_buf, uint16_t len)
{
	HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)log_buf, len, 100);

	return ERR_CODE_SUCCESS;
}
#endif

#ifdef USE_NRF24L01
nrf24l01_status_t hw_intf_nrf24l01_spi_send(uint8_t *buf_send, uint16_t len)
{
	HAL_SPI_Transmit(&NRF24L01_SPI, buf_send, len, 100);

	return ERR_CODE_SUCCESS;
}

nrf24l01_status_t hw_intf_nrf24l01_spi_recv(uint8_t *buf_recv, uint16_t len)
{
	HAL_SPI_Receive(&NRF24L01_SPI, buf_recv, len, 100);

	return ERR_CODE_SUCCESS;
}

nrf24l01_status_t hw_intf_nrf24l01_set_cs(uint8_t level)
{
	HAL_GPIO_WritePin(NRF24L01_GPIO_PORT_CS, NRF24L01_GPIO_PIN_CS, level);

	return ERR_CODE_SUCCESS;
}

nrf24l01_status_t hw_intf_nrf24l01_set_ce(uint8_t level)
{
	HAL_GPIO_WritePin(NRF24L01_GPIO_PORT_CE, NRF24L01_GPIO_PIN_CE, level);

	return ERR_CODE_SUCCESS;
}

nrf24l01_status_t hw_intf_nrf24l01_get_irq(uint8_t *level)
{
	*level = HAL_GPIO_ReadPin(NRF24L01_GPIO_PORT_IRQ, NRF24L01_GPIO_PIN_IRQ);

	return ERR_CODE_SUCCESS;
}
#endif

#ifdef USE_SX1278
err_code_t hw_intf_sx1278_spi_send(uint8_t *buf_send, uint16_t len)
{
	HAL_SPI_Transmit(&SX1278_SPI, buf_send, len, 100);

	return ERR_CODE_SUCCESS;
}

err_code_t hw_intf_sx1278_spi_recv(uint8_t *buf_recv, uint16_t len)
{
	HAL_SPI_Receive(&SX1278_SPI, buf_recv, len, 100);

	return ERR_CODE_SUCCESS;
}

err_code_t hw_intf_sx1278_set_cs(uint8_t level)
{
	HAL_GPIO_WritePin(SX1278_GPIO_PORT_CS, SX1278_GPIO_PIN_CS, level);

	return ERR_CODE_SUCCESS;
}

err_code_t hw_intf_sx1278_set_rst(uint8_t level)
{
	HAL_GPIO_WritePin(SX1278_GPIO_PORT_RST, SX1278_GPIO_PIN_RST, level);

	return ERR_CODE_SUCCESS;
}

err_code_t hw_intf_sx1278_get_irq(uint8_t *level)
{
	*level = HAL_GPIO_ReadPin(SX1278_GPIO_PORT_IRQ, SX1278_GPIO_PIN_IRQ);

	return ERR_CODE_SUCCESS;
}
#endif

#ifdef USE_MPU6050
err_code_t hw_intf_mpu6050_i2c_send(uint8_t reg_addr, uint8_t *buf, uint16_t len)
{
	uint8_t buf_send[len + 1];
	buf_send[0] = reg_addr;
	for (uint8_t i = 0; i < len; i++)
	{
		buf_send[i + 1] = buf[i];
	}

	HAL_I2C_Master_Transmit(&MPU6050_I2C, I2C_ADDR_MPU6050, buf_send, len + 1, 100);

	return ERR_CODE_SUCCESS;
}

err_code_t hw_intf_mpu6050_i2c_recv(uint8_t reg_addr, uint8_t *buf, uint16_t len)
{
	uint8_t buffer[1];
	buffer[0] = reg_addr;

	HAL_I2C_Master_Transmit(&MPU6050_I2C, I2C_ADDR_MPU6050, buffer, 1, 100);
	HAL_I2C_Master_Receive(&MPU6050_I2C, I2C_ADDR_MPU6050, buf, len, 100);

	return ERR_CODE_SUCCESS;
}
#endif

#ifdef USE_ICM42688
icm42688_status_t hw_intf_icm42688_spi_send(uint8_t *buf_send, uint16_t len)
{
	HAL_SPI_Transmit(&ICM42688_SPI, buf_send, len, 100);

	return ERR_CODE_SUCCESS;
}

icm42688_status_t hw_intf_icm42688_spi_recv(uint8_t *buf_recv, uint16_t len)
{
	HAL_SPI_Receive(&ICM42688_SPI, buf_recv, len, 100);

	return ERR_CODE_SUCCESS;
}

icm42688_status_t hw_intf_icm42688_set_cs(uint8_t level)
{
	HAL_GPIO_WritePin(ICM42688_GPIO_PORT_CS, ICM42688_GPIO_PIN_CS, level);

	return ERR_CODE_SUCCESS;
}
#endif

#ifdef USE_HMC5883L
err_code_t hw_intf_hmc5883l_i2c_send(uint8_t reg_addr, uint8_t *buf, uint16_t len)
{
	uint8_t buf_send[len + 1];
	buf_send[0] = reg_addr;
	for (uint8_t i = 0; i < len; i++)
	{
		buf_send[i + 1] = buf[i];
	}

	HAL_I2C_Master_Transmit(&HMC5883L_I2C, HMC5883L_I2C_ADDR, buf_send, len + 1, 100);

	return ERR_CODE_SUCCESS;
}

err_code_t hw_intf_hmc5883l_i2c_recv(uint8_t reg_addr, uint8_t *buf, uint16_t len)
{
	uint8_t buffer[1];
	buffer[0] = reg_addr;

	HAL_I2C_Master_Transmit(&HMC5883L_I2C, HMC5883L_I2C_ADDR, buffer, 1, 100);
	HAL_I2C_Master_Receive(&HMC5883L_I2C, HMC5883L_I2C_ADDR, buf, len, 100);

	return ERR_CODE_SUCCESS;
}
#endif

#ifdef USE_QMC5883L
qmc5883l_status_t hw_intf_qmc5883l_i2c_send(uint8_t reg_addr, uint8_t *buf, uint16_t len)
{
	uint8_t buf_send[len + 1];
	buf_send[0] = reg_addr;
	for (uint8_t i = 0; i < len; i++)
	{
		buf_send[i + 1] = buf[i];
	}

	HAL_I2C_Master_Transmit(&QMC5883L_I2C, I2C_ADDR_QMC5883L, buf_send, len + 1, 100);

	return ERR_CODE_SUCCESS;
}

qmc5883l_status_t hw_intf_qmc5883l_i2c_recv(uint8_t reg_addr, uint8_t *buf, uint16_t len)
{
	uint8_t buffer[1];
	buffer[0] = reg_addr;

	HAL_I2C_Master_Transmit(&QMC5883L_I2C, I2C_ADDR_QMC5883L, buffer, 1, 100);
	HAL_I2C_Master_Receive(&QMC5883L_I2C, I2C_ADDR_QMC5883L, buf, len, 100);

	return ERR_CODE_SUCCESS;
}
#endif

#ifdef USE_BMP280
bmp280_status_t hw_intf_bmp280_i2c_send(uint8_t reg_addr, uint8_t *buf, uint16_t len)
{
    uint8_t buf_send[len + 1];
    buf_send[0] = reg_addr;
    for (uint8_t i = 0; i < len; i++)
    {
        buf_send[i + 1] = buf[i];
    }

    HAL_I2C_Master_Transmit(&BMP280_I2C, I2C_ADDR_BMP280, buf_send, len + 1, 100);

    return ERR_CODE_SUCCESS;
}

bmp280_status_t hw_intf_bmp280_i2c_recv(uint8_t reg_addr, uint8_t *buf, uint16_t len)
{
    uint8_t buffer[1];
    buffer[0] = reg_addr;

    HAL_I2C_Master_Transmit(&BMP280_I2C, I2C_ADDR_BMP280, buffer, 1, 100);
    HAL_I2C_Master_Receive(&BMP280_I2C, I2C_ADDR_BMP280, buf, len, 100);

    return ERR_CODE_SUCCESS;
}
#endif

#ifdef USE_ESC_DSHOT
esc_dshot_status_t hw_intf_fl_esc_dshot_send_dma(uint32_t *packet_dma)
{
	HAL_TIM_PWM_Start_DMA(&FL_ESC_DSHOT_TIM, FL_ESC_DSHOT_TIM_CHNL, packet_dma, DSHOT_DMA_BUFFER);
	return ERR_CODE_SUCCESS;
}

esc_dshot_status_t hw_intf_fr_esc_dshot_send_dma(uint32_t *packet_dma)
{
	HAL_TIM_PWM_Start_DMA(&FR_ESC_DSHOT_TIM, FR_ESC_DSHOT_TIM_CHNL, packet_dma, DSHOT_DMA_BUFFER);
	return ERR_CODE_SUCCESS;
}

esc_dshot_status_t hw_intf_bl_esc_dshot_send_dma(uint32_t *packet_dma)
{
	HAL_TIM_PWM_Start_DMA(&BL_ESC_DSHOT_TIM, BL_ESC_DSHOT_TIM_CHNL, packet_dma, DSHOT_DMA_BUFFER);
	return ERR_CODE_SUCCESS;
}

esc_dshot_status_t hw_intf_br_esc_dshot_send_dma(uint32_t *packet_dma)
{
	HAL_TIM_PWM_Start_DMA(&BR_ESC_DSHOT_TIM, BR_ESC_DSHOT_TIM_CHNL, packet_dma, DSHOT_DMA_BUFFER);
	return ERR_CODE_SUCCESS;
}
#endif
