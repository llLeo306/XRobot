#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "bsp_usb.h"
#include "usbd_cdc_if.h"

uint8_t usb_tx_buf[512];
uint8_t usb_rx_buf[512];

uint8_t BSP_USB_Printf(const char *fmt, ...) {
  static va_list ap;
  uint16_t len = 0;

  va_start(ap, fmt);
  len = vsprintf((char *)usb_tx_buf, fmt, ap);
  va_end(ap);

  CDC_Transmit_FS(usb_tx_buf, len);

  return 0;
}
