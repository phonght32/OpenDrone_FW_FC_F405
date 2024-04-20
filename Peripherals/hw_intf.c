#include "spi.h"
#include "tim.h"
#include "i2c.h"

#include "hw_intf.h"
#include "mpu6050.h"

#define APP_TIM 						htim1

#ifdef USE_NRF24L01
#define NRF24L01_SPI                 	hspi2
#define NRF24L01_GPIO_PORT_CS         	GPIOB
#define NRF24L01_GPIO_PIN_CS      		GPIO_PIN_14
#define NRF24L01_GPIO_PORT_CE         	GPIOB
#define NRF24L01_GPIO_PIN_CE      		GPIO_PIN_5
#define NRF24L01_GPIO_PORT_IRQ        	GPIOB
#define NRF24L01_GPIO_PIN_IRQ         	GPIO_PIN_2
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
#define MPU6050_I2C  					hi2c1
#endif

uint32_t hw_intf_get_time_us(void)
{
	return __HAL_TIM_GET_COUNTER(&APP_TIM);
}

void hw_intf_delay_ms(uint32_t time_ms)
{
	HAL_Delay(time_ms);
}

#ifdef USE_NRF24L01
err_code_t hw_intf_nrf24l01_spi_send(uint8_t *buf_send, uint16_t len)
{
	HAL_SPI_Transmit(&NRF24L01_SPI, buf_send, len, 100);

	return ERR_CODE_SUCCESS;
}

err_code_t hw_intf_nrf24l01_spi_recv(uint8_t *buf_recv, uint16_t len)
{
	HAL_SPI_Receive(&NRF24L01_SPI, buf_recv, len, 100);

	return ERR_CODE_SUCCESS;
}

err_code_t hw_intf_nrf24l01_set_cs(uint8_t level)
{
	HAL_GPIO_WritePin(NRF24L01_GPIO_PORT_CS, NRF24L01_GPIO_PIN_CS, level);

	return ERR_CODE_SUCCESS;
}

err_code_t hw_intf_nrf24l01_set_ce(uint8_t level)
{
	HAL_GPIO_WritePin(NRF24L01_GPIO_PORT_CE, NRF24L01_GPIO_PIN_CE, level);

	return ERR_CODE_SUCCESS;
}

err_code_t hw_intf_nrf24l01_get_irq(uint8_t *level)
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

	HAL_I2C_Master_Transmit(&MPU6050_I2C, MPU6050_I2C_ADDR, buf_send, len + 1, 100);

	return ERR_CODE_SUCCESS;
}

err_code_t hw_intf_mpu6050_i2c_recv(uint8_t reg_addr, uint8_t *buf, uint16_t len)
{
	uint8_t buffer[1];
	buffer[0] = reg_addr;

	HAL_I2C_Master_Transmit(&MPU6050_I2C, MPU6050_I2C_ADDR, buffer, 1, 100);
	HAL_I2C_Master_Receive(&MPU6050_I2C, MPU6050_I2C_ADDR, buf, len, 100);

	return ERR_CODE_SUCCESS;
}
#endif
