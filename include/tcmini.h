#ifndef TCMINI_H
#define TCMINI_H

#include <stdbool.h>
#include <stdint.h>

#define TCMINI_PIN_SCL     3
#define TCMINI_PIN_SDA    14
#define TCMINI_PIN_IRQ    23

enum TCMINI_PIN_MAPPING{
    kTCMINI_PIN_MAPPING_NONE = 0,
    kTCMINI_PIN_MAPPING_DPDN1 = 1,
    kTCMINI_PIN_MAPPING_DPDN2 = 2,
    kTCMINI_PIN_MAPPING_SBU = 4
};

typedef void (*t_tcmini_vdm_cb)(uint32_t *data, uint8_t cnt);

int tcmini_init();
void tcmini_deinit();
void tcmini_task();

bool tcmini_is_device_connected();
void tcmini_register_vdm_cb(t_tcmini_vdm_cb cb);

int tcmini_vmd_send(const uint32_t *data, uint8_t cnt);
int tcmini_vdm_apple_perform_action(bool exit, 
                                    bool persist, 
                                    bool exit_conflicting, 
                                    enum TCMINI_PIN_MAPPING mapping, 
                                    uint16_t action_id, 
                                    uint32_t arguments[], 
                                    uint8_t arguments_len);
int tcmini_vmd_apple_send_reboot();
int tcmini_vmd_apple_send_dfu();
int tcmini_vmd_apple_send_map_uart(enum TCMINI_PIN_MAPPING mapping);

#endif // TCMINI_H