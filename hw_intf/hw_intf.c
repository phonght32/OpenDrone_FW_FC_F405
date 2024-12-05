#include "spi.h"
#include "tim.h"
#include "i2c.h"
#include "usart.h"

#include "OpenDrone_FC_HwIntf.h"
#include "mpu6050.h"
#include "hmc5883l.h"
#include "qmc5883l.h"
#include "esc_dshot.h"

#define APP_TIM 						htim3

#ifdef USE_SERIAL_DEBUG
#define UART_DEBUG  					huart1
#endif

#ifdef USE_NRF24L01
#define NRF24L01_SPI                 	hspi2
#define NRF24L01_GPIO_PORT_CS         	GPIOB
#define NRF24L01_GPIO_PIN_CS      		GPIO_PIN_12
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
#define I2C_ADDR_MPU6050   				(MPU6050_I2C_ADDR<<1)
#define MPU6050_I2C  					hi2c2
#endif

#ifdef USE_HMC5883L
#define HMC5883L_I2C  					hi2c2
#endif

#ifdef USE_QMC5883L
#define I2C_ADDR_QMC5883L				(QMC5883L_I2C_ADDR<<1)
#define QMC5883L_I2C  					hi2c2
#endif

#ifdef USE_ESC_DSHOT
#define FL_ESC_DSHOT_TIM  				htim1
#define FL_ESC_DSHOT_TIM_CHNL 			TIM_CHANNEL_2
#define FL_ESC_DSHOT_TIM_DMA_ID 		TIM_DMA_ID_CC2
#define FL_ESC_DSHOT_TIM_DMA_CC			TIM_DMA_CC2
#define FL_ESC_DSHOT_TIM_CCR   			CCR2

#define FR_ESC_DSHOT_TIM  				htim1
#define FR_ESC_DSHOT_TIM_CHNL   		TIM_CHANNEL_3
#define FR_ESC_DSHOT_TIM_DMA_ID 		TIM_DMA_ID_CC3
#define FR_ESC_DSHOT_TIM_DMA_CC			TIM_DMA_CC3
#define FR_ESC_DSHOT_TIM_CCR   			CCR3

#define BL_ESC_DSHOT_TIM  				htim2
#define BL_ESC_DSHOT_TIM_CHNL   		TIM_CHANNEL_4
#define BL_ESC_DSHOT_TIM_DMA_ID 		TIM_DMA_ID_CC4
#define BL_ESC_DSHOT_TIM_DMA_CC			TIM_DMA_CC4
#define BL_ESC_DSHOT_TIM_CCR   			CCR4

#define BR_ESC_DSHOT_TIM  				htim2
#define BR_ESC_DSHOT_TIM_CHNL   		TIM_CHANNEL_3
#define BR_ESC_DSHOT_TIM_DMA_ID 		TIM_DMA_ID_CC3
#define BR_ESC_DSHOT_TIM_DMA_CC			TIM_DMA_CC3
#define BR_ESC_DSHOT_TIM_CCR   			CCR3
#endif

uint32_t hw_intf_get_time_us(void)
{
	return HAL_GetTick() * 1000;
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
err_code_t hw_intf_qmc5883l_i2c_send(uint8_t reg_addr, uint8_t *buf, uint16_t len)
{
	uint8_t buf_send[len + 1];
	buf_send[0] = reg_addr;
	for (uint8_t i = 0; i < len; i++)
	{
		buf_send[i + 1] = buf[i];
	}

	HAL_I2C_Master_Transmit(&QMC5883L_I2C, QMC5883L_I2C_ADDR, buf_send, len + 1, 100);

	return ERR_CODE_SUCCESS;
}

err_code_t hw_intf_qmc5883l_i2c_recv(uint8_t reg_addr, uint8_t *buf, uint16_t len)
{
	uint8_t buffer[1];
	buffer[0] = reg_addr;

	HAL_I2C_Master_Transmit(&QMC5883L_I2C, QMC5883L_I2C_ADDR, buffer, 1, 100);
	HAL_I2C_Master_Receive(&QMC5883L_I2C, QMC5883L_I2C_ADDR, buf, len, 100);

	return ERR_CODE_SUCCESS;
}
#endif

#ifdef USE_ESC_DSHOT
err_code_t hw_intf_fl_esc_dshot_set_auto_reload(uint32_t auto_reload)
{
	__HAL_TIM_SET_AUTORELOAD(&FL_ESC_DSHOT_TIM, auto_reload);

	return ERR_CODE_SUCCESS;
}

err_code_t hw_intf_fr_esc_dshot_set_auto_reload(uint32_t auto_reload)
{
	__HAL_TIM_SET_AUTORELOAD(&FR_ESC_DSHOT_TIM, auto_reload);

	return ERR_CODE_SUCCESS;
}

err_code_t hw_intf_bl_esc_dshot_set_auto_reload(uint32_t auto_reload)
{
	__HAL_TIM_SET_AUTORELOAD(&BL_ESC_DSHOT_TIM, auto_reload);

	return ERR_CODE_SUCCESS;
}

err_code_t hw_intf_br_esc_dshot_set_auto_reload(uint32_t auto_reload)
{
	__HAL_TIM_SET_AUTORELOAD(&BR_ESC_DSHOT_TIM, auto_reload);

	return ERR_CODE_SUCCESS;
}

err_code_t hw_intf_fl_esc_dshot_send_dma(uint32_t *packet_dma)
{
	HAL_DMA_Start((&FL_ESC_DSHOT_TIM)->hdma[FL_ESC_DSHOT_TIM_DMA_ID], (uint32_t)packet_dma, (uint32_t)(&FL_ESC_DSHOT_TIM)->Instance->FL_ESC_DSHOT_TIM_CCR, DSHOT_DMA_BUFFER);
	__HAL_TIM_ENABLE_DMA(&FL_ESC_DSHOT_TIM, FL_ESC_DSHOT_TIM_DMA_CC);

	return ERR_CODE_SUCCESS;
}

err_code_t hw_intf_fr_esc_dshot_send_dma(uint32_t *packet_dma)
{
	HAL_DMA_Start((&FR_ESC_DSHOT_TIM)->hdma[FR_ESC_DSHOT_TIM_DMA_ID], (uint32_t)packet_dma, (uint32_t)(&FR_ESC_DSHOT_TIM)->Instance->FR_ESC_DSHOT_TIM_CCR, DSHOT_DMA_BUFFER);
	__HAL_TIM_ENABLE_DMA(&FR_ESC_DSHOT_TIM, FR_ESC_DSHOT_TIM_DMA_CC);

	return ERR_CODE_SUCCESS;
}

err_code_t hw_intf_bl_esc_dshot_send_dma(uint32_t *packet_dma)
{
	HAL_DMA_Start((&BL_ESC_DSHOT_TIM)->hdma[BL_ESC_DSHOT_TIM_DMA_ID], (uint32_t)packet_dma, (uint32_t)(&BL_ESC_DSHOT_TIM)->Instance->BL_ESC_DSHOT_TIM_CCR, DSHOT_DMA_BUFFER);
	__HAL_TIM_ENABLE_DMA(&BL_ESC_DSHOT_TIM, BL_ESC_DSHOT_TIM_DMA_CC);

	return ERR_CODE_SUCCESS;
}

err_code_t hw_intf_br_esc_dshot_send_dma(uint32_t *packet_dma)
{
	HAL_DMA_Start((&BR_ESC_DSHOT_TIM)->hdma[BR_ESC_DSHOT_TIM_DMA_ID], (uint32_t)packet_dma, (uint32_t)(&BR_ESC_DSHOT_TIM)->Instance->BR_ESC_DSHOT_TIM_CCR, DSHOT_DMA_BUFFER);
	__HAL_TIM_ENABLE_DMA(&BR_ESC_DSHOT_TIM, BR_ESC_DSHOT_TIM_DMA_CC);

	return ERR_CODE_SUCCESS;
}

err_code_t hw_intf_esc_dshot_start(void)
{
	HAL_TIM_PWM_Start(&FL_ESC_DSHOT_TIM, FL_ESC_DSHOT_TIM_CHNL);
	HAL_TIM_PWM_Start(&FR_ESC_DSHOT_TIM, FR_ESC_DSHOT_TIM_CHNL);
	HAL_TIM_PWM_Start(&BL_ESC_DSHOT_TIM, BL_ESC_DSHOT_TIM_CHNL);
	HAL_TIM_PWM_Start(&BR_ESC_DSHOT_TIM, BR_ESC_DSHOT_TIM_CHNL);

	return ERR_CODE_SUCCESS;
}
#endif
