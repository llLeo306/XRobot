#include "bsp_wifi_client.h"

#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>

#include "bsp.h"

static char cmd_buff[100];

void bsp_wifi_client_init() {}

int8_t bsp_wifi_connect(const char *name, const char *password) {
  (void)name;
  (void)password;
  (void)snprintf(cmd_buff, sizeof(cmd_buff), "nmcli con delete %s", name);
  (void)system(cmd_buff);
  (void)snprintf(cmd_buff, sizeof(cmd_buff),
                 "nmcli dev wifi connect %s password \"%s\"", name, password);
  int ans = system(cmd_buff);

  if (-1 == ans) {
    return BSP_ERR;
  } else {
    if (WIFEXITED(ans)) {
      if (0 == WEXITSTATUS(ans)) {
        return BSP_OK;
      } else {
        return BSP_ERR;
      }
    } else {
      return BSP_ERR;
    }
  }
}

bool bsp_wifi_connected() {
  int ret = open("/sys/class/net/wlan0/operstate", O_RDONLY);

  char status[3] = "wl\0";
  (void)read(ret, status, 2);
  status[2] = '\0';
  if (0 == strcmp("up", status)) {
    return true;
  } else {
    return false;
  }
}
