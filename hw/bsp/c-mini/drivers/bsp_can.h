#pragma once

#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
#include "FreeRTOS.h"
#include "bsp.h"
typedef enum {
  BSP_CAN_1,
  BSP_CAN_2,
  BSP_CAN_3,
  BSP_CAN_4,
  BSP_CAN_NUM,
  BSP_CAN_ERR,
} bsp_can_t;

#define BSP_CAN_BASE_NUM (BSP_CAN_2 + 1)
#define BSP_CAN_EXT_NUM (BSP_CAN_NUM - BSP_CAN_BASE_NUM)

typedef enum {
  CAN_RX_MSG_CALLBACK,
  CAN_TX_CPLT_CALLBACK,
  BSP_CAN_CB_NUM
} bsp_can_callback_t;

typedef enum {
  CAN_FORMAT_STD_DATA,
  CAN_FORMAT_EXT_DATA,
  CAN_FORMAT_STD_REMOTE,
  CAN_FORMAT_EXT_REMOTE,
} bsp_can_format_t;

void bsp_can_init(void);
bsp_status_t bsp_can_register_callback(
    bsp_can_t can, bsp_can_callback_t type,
    void (*callback)(bsp_can_t can, uint32_t id, uint8_t *data, void *arg),
    void *callback_arg);
bsp_status_t bsp_can_trans_packet(bsp_can_t can, bsp_can_format_t format,
                                  uint32_t id, uint8_t *data);
bsp_status_t bsp_cantouart_get_msg(bsp_can_t can, uint8_t *data);
#ifdef __cplusplus
}
#endif
