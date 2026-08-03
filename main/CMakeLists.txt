idf_component_register(
    SRCS
        "main.c"
        "esc_pwm.c"
        "esc_controller.c"
        "bluetooth_spp.c"
	"command_handler.c"
	"esc_protocol.c"
	"esc_dshot.c" 
	"dshot_rmt.c"
	"dshot_rmt_encoder.c"
    INCLUDE_DIRS "."
    REQUIRES 
        bt 
        nvs_flash
        esp_driver_ledc
	esp_driver_rmt
)