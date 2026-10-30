/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_netxduo.h
  * @author  MCD Application Team
  * @brief   NetXDuo applicative header file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2020-2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APP_NETXDUO_H__
#define __APP_NETXDUO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/* Includes ------------------------------------------------------------------*/
#include "nx_api.h"

/* Private includes ----------------------------------------------------------*/
#include "nx_stm32_eth_driver.h"

/* USER CODE BEGIN Includes */
#include "main.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */
/* The DEFAULT_PAYLOAD_SIZE should match with RxBuffLen configured via MX_ETH_Init */
#ifndef DEFAULT_PAYLOAD_SIZE
#define DEFAULT_PAYLOAD_SIZE      1536
#endif

#ifndef DEFAULT_ARP_CACHE_SIZE
#define DEFAULT_ARP_CACHE_SIZE    1024
#endif

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
UINT MX_NetXDuo_Init(VOID *memory_ptr);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* NetX's internal IP thread runs nx_eth_phy_init() (PHY driver bring-up) and
 * link enable. App threads (PCAN=3, Network/FSM/Sensor=4) cycle on short
 * sleeps and are otherwise almost always ready, which starves priority 10
 * indefinitely under ThreadX's strict priority scheduling, leaving the PHY
 * driver's IO function pointers unregistered (NULL) when app code calls into
 * it. Give the IP thread priority over the app's cyclic threads.
 */
#define NX_APP_INSTANCE_PRIORITY             2

/* USER CODE END PD */

#define NX_APP_DEFAULT_TIMEOUT               (10 * NX_IP_PERIODIC_RATE)

#define NX_APP_PACKET_POOL_SIZE              ((DEFAULT_PAYLOAD_SIZE + sizeof(NX_PACKET)) * 20)

#define NX_APP_THREAD_STACK_SIZE             1024*2

#define Nx_IP_INSTANCE_THREAD_SIZE           1024

#define NX_APP_THREAD_PRIORITY               10

#ifndef NX_APP_INSTANCE_PRIORITY
#define NX_APP_INSTANCE_PRIORITY             NX_APP_THREAD_PRIORITY
#endif

#define NX_APP_DEFAULT_IP_ADDRESS                   IP_ADDRESS(10, 19, 0, 110)

#define NX_APP_DEFAULT_NET_MASK                     IP_ADDRESS(255, 255, 0, 0)

/* USER CODE BEGIN 2 */
UINT NetXDuo_init();
UINT NetXDuo_init_udp_socket(NX_UDP_SOCKET *socket, UINT port);
UINT NetXDuo_receive(NX_UDP_SOCKET *socket, uint8_t *data, UINT *bytes_read);
UINT NetXDuo_send(NX_UDP_SOCKET *socket, const uint8_t *data, UINT size, UINT port);
/* USER CODE END 2 */

#ifdef __cplusplus
}
#endif
#endif /* __APP_NETXDUO_H__ */
