#include "hw.h"
#include "hw_intf.h"
#include "periph_esc.h"
#include "esc_dshot.h"

#ifdef USE_ESC_DSHOT
esc_dshot_handle_t fl_esc_dshot_handle;
esc_dshot_handle_t fr_esc_dshot_handle;
esc_dshot_handle_t bl_esc_dshot_handle;
esc_dshot_handle_t br_esc_dshot_handle;
#endif

err_code_t periph_esc_init(void)
{
#ifdef USE_ESC_DSHOT
	fl_esc_dshot_handle = esc_dshot_init();
	fr_esc_dshot_handle = esc_dshot_init();
	bl_esc_dshot_handle = esc_dshot_init();
	br_esc_dshot_handle = esc_dshot_init();

	esc_dshot_cfg_t esc_dshot_cfg = {
		.tim_freq 	= CONFIG_ESC_DSHOT_TIM_FREQ,
		.dshot_type = CONFIG_ESC_DSHOT_TYPE,
	};

	esc_dshot_set_config(fl_esc_dshot_handle, esc_dshot_cfg);
	esc_dshot_set_config(fr_esc_dshot_handle, esc_dshot_cfg);
	esc_dshot_set_config(bl_esc_dshot_handle, esc_dshot_cfg);
	esc_dshot_set_config(br_esc_dshot_handle, esc_dshot_cfg);

	esc_dshot_config(fl_esc_dshot_handle);
	esc_dshot_config(fr_esc_dshot_handle);
	esc_dshot_config(bl_esc_dshot_handle);
	esc_dshot_config(br_esc_dshot_handle);
#endif
	return ERR_CODE_SUCCESS;
}