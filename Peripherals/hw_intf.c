#include "spi.h"
#include "tim.h"

#include "hw_intf.h"

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
