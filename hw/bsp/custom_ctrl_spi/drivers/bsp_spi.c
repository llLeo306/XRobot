#include "bsp_spi.h"

#include "main.h"

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;

static bsp_callback_t callback_list[BSP_SPI_NUM][BSP_SPI_CB_NUM];

static bsp_spi_t spi_get(SPI_HandleTypeDef *hspi) {
  if (hspi->Instance == SPI1) {
    // 根据当前片选引脚的状态，返回对应的枚举值
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_RESET) {
      return BSP_SPI1_CS_1;
    } else if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_RESET) {
      return BSP_SPI1_CS_2;
    } else if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_RESET) {
      return BSP_SPI1_CS_3;
    }
  } else if (hspi->Instance == SPI2) {
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_RESET) {
      return BSP_SPI2_CS_1;
    } else if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == GPIO_PIN_RESET) {
      return BSP_SPI2_CS_2;
    } else if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_RESET) {
      return BSP_SPI2_CS_3;
    }
  }
  return BSP_SPI_ERR;
}

static void bsp_spi_callback(bsp_spi_callback_t cb_type,
                             SPI_HandleTypeDef *hspi) {
  bsp_spi_t bsp_spi = spi_get(hspi);
  if (bsp_spi != BSP_SPI_ERR) {
    bsp_callback_t cb = callback_list[bsp_spi][cb_type];

    if (cb.fn) {
      cb.fn(cb.arg);
    }
  }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
  bsp_spi_callback(BSP_SPI_RX_CPLT_CB, hspi);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
  bsp_spi_callback(BSP_SPI_TX_CPLT_CB, hspi);
}

SPI_HandleTypeDef *bsp_spi_get_handle(bsp_spi_t spi) {
  switch (spi) {
    case BSP_SPI1_CS_1:
    case BSP_SPI1_CS_2:
    case BSP_SPI1_CS_3:
      return &hspi1;
    case BSP_SPI2_CS_1:
    case BSP_SPI2_CS_2:
    case BSP_SPI2_CS_3:
      return &hspi2;
    /*
    case BSP_SPI_XXX:
            return &hspiX;
    */
    default:
      return NULL;
  }
}

bsp_status_t bsp_spi_register_callback(bsp_spi_t spi, bsp_spi_callback_t type,
                                       void (*callback)(void *),
                                       void *callback_arg) {
  assert_param(callback);
  assert_param(type != BSP_SPI_CB_NUM);

  callback_list[spi][type].fn = callback;
  callback_list[spi][type].arg = callback_arg;
  return BSP_OK;
}

void bsp_spi_select(bsp_spi_t spi) {
  switch (spi) {
    case BSP_SPI1_CS_1:
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);  // 拉低片选 1
      break;
    case BSP_SPI1_CS_2:
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);  // 拉低片选 2
      break;
    case BSP_SPI1_CS_3:
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);  // 拉低片选 3
      break;
    case BSP_SPI2_CS_1:
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
      break;
    case BSP_SPI2_CS_2:
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
      break;
    case BSP_SPI2_CS_3:
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
      break;
    default:
      break;
  }
}

void bsp_spi_deselect(bsp_spi_t spi) {
  switch (spi) {
    case BSP_SPI1_CS_1:
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);  // 拉高片选 1
      break;
    case BSP_SPI1_CS_2:
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);  // 拉高片选 2
      break;
    case BSP_SPI1_CS_3:
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);  // 拉高片选 3
      break;
    case BSP_SPI2_CS_1:
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
      break;
    case BSP_SPI2_CS_2:
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
      break;
    case BSP_SPI2_CS_3:
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
      break;
    default:
      break;
  }
}

bsp_status_t bsp_spi_transmit(bsp_spi_t spi, uint8_t *data, size_t size,
                              bool block) {
  SPI_HandleTypeDef *hspi = bsp_spi_get_handle(spi);
  if (hspi == NULL) {
    return BSP_ERR;
  }

  bsp_spi_select(spi);  // 拉低片选

  bsp_status_t status;
  if (block) {
    status = HAL_SPI_Transmit(hspi, data, size, 10) != HAL_OK;
  } else {
    status = HAL_SPI_Transmit_DMA(hspi, data, size) != HAL_OK;
  }

  bsp_spi_deselect(spi);  // 拉高片选
  return status;
}

bsp_status_t bsp_spi_receive(bsp_spi_t spi, uint8_t *data, size_t size,
                             bool block) {
  SPI_HandleTypeDef *hspi = bsp_spi_get_handle(spi);
  if (hspi == NULL) {
    return BSP_ERR;
  }

  bsp_spi_select(spi);  // 拉低片选

  bsp_status_t status;
  if (block) {
    status = HAL_SPI_Receive(hspi, data, size, 10) != HAL_OK;
  } else {
    status = HAL_SPI_Receive_DMA(hspi, data, size) != HAL_OK;
  }

  bsp_spi_deselect(spi);  // 拉高片选
  return status;
}

bsp_status_t bsp_spi_transmit_receive(bsp_spi_t spi, uint8_t *txbuff,
                                      uint8_t *rxbuff, size_t size,
                                      bool block) {
  SPI_HandleTypeDef *hspi = bsp_spi_get_handle(spi);
  if (hspi == NULL) {
    return BSP_ERR;
  }

  bsp_spi_select(spi);  // 拉低片选

  bsp_status_t status;
  if (block) {
    status = HAL_SPI_TransmitReceive(hspi, txbuff, rxbuff, size, 10) != HAL_OK;
  } else {
    status = HAL_SPI_TransmitReceive_DMA(hspi, txbuff, rxbuff, size) != HAL_OK;
  }

  bsp_spi_deselect(spi);  // 拉高片选
  return status;
}
