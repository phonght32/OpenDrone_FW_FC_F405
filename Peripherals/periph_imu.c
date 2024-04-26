#include "hw.h"
#include "hw_intf.h"
#include "periph_imu.h"
#include "mpu6050.h"
#include "hmc5883l.h"

#ifdef USE_MPU6050
mpu6050_handle_t mpu6050_handle;
#endif

#ifdef USE_HMC5883L
hmc5883l_handle_t hmc5883l_handle;
#endif

err_code_t periph_imu_init(void)
{
#ifdef USE_MPU6050
	mpu6050_handle = mpu6050_init();
	mpu6050_cfg_t mpu6050_cfg = {
		.clksel 		= CONFIG_MPU6050_CLKSEL,
		.dlpf_cfg 		= CONFIG_MPU6050_DLPF,
		.sleep_mode 	= CONFIG_MPU6050_SLEEP_MODE,
		.gfs_sel 		= CONFIG_MPU6050_GFS,
		.afs_sel 		= CONFIG_MPU6050_AFS,
		.accel_bias_x 	= 0,
		.accel_bias_y 	= 0,
		.accel_bias_z 	= 0,
		.gyro_bias_x 	= 0,
		.gyro_bias_y 	= 0,
		.gyro_bias_z 	= 0,
		.i2c_send 		= hw_intf_mpu6050_i2c_send,
		.i2c_recv 		= hw_intf_mpu6050_i2c_recv,
		.delay 			= HAL_Delay
	};
	mpu6050_set_config(mpu6050_handle, mpu6050_cfg);
	mpu6050_config(mpu6050_handle);
#endif

#ifdef USE_HMC5883L
	hmc5883l_handle = hmc5883l_init();
	hmc5883l_cfg_t hmc5883l_cfg = {
		.range 		= CONFIG_HMC5883L_RANGE,
		.opr_mode 	= CONFIG_HMC5883L_OPR_MODE,
		.data_rate 	= CONFIG_HMC5883L_DATA_RATE,
		.samples 	= CONFIG_HMC5883L_SAMPLES,
		.mag_bias_x = 0,
		.mag_bias_y = 0,
		.mag_bias_z = 0,
		.i2c_send 	= hw_intf_hmc5883l_i2c_send,
		.i2c_recv 	= hw_intf_hmc5883l_i2c_recv,
		.delay 		= HAL_Delay
	};
	hmc5883l_set_config(hmc5883l_handle, hmc5883l_cfg);
	hmc5883l_config(hmc5883l_handle);
#endif

	return ERR_CODE_SUCCESS;
}