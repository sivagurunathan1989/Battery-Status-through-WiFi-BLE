/**
 *
 * \file
 *
 * \brief WINC1500 MQTT chat example.
 *
 * Copyright (c) 2015 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

/** \mainpage
 * \section intro Introduction
 * This example demonstrates the use of the SAMW25 Xplained Pro board
 * along with BTLC1000 XPlained Pro board to implement an MQTT based chat.
 * It uses the following hardware:
 * - the SAMW25 Xplained Pro.
 * - the BTLC1000 Xplained Pro on EXT1.
 *
 * \section files Main Files
 * - main.c : Initialize the WINC1500, connect to MQTT broker and chat with the other devices.
 * - mqtt.h : Implementation of MQTT 3.1
 *
 * \section usage Usage
 * -# Configure below code in the main.h for AP information to be connected.
 * \code
 *    #define MAIN_WLAN_SSID         "DEMO_AP"
 *    #define MAIN_WLAN_AUTH         M2M_WIFI_SEC_WPA_PSK
 *    #define MAIN_WLAN_PSK          "12345678"
 * \endcode
 * -# Build the program and download it into the board.
 * \section contactinfo Contact Information
 * For further information, visit
 * <A href="http://www.atmel.com">Atmel</A>.\n
 */

#include "asf.h"
#include "main.h"
#include "driver/include/m2m_wifi.h"
#include "iot/mqtt/mqtt.h"
#include "iot/sw_timer.h"
#include "socket/include/socket.h"
#include "battery.h"
#include "at_ble_api.h"
#include "ble_manager.h"
#include "timer_hw.h"

/* === GLOBALS ============================================================ */

#define BATTERY_UPDATE_INTERVAL	(1) //1 second
#define BATTERY_MAX_LEVEL		(100)
#define BATTERY_MIN_LEVEL		(0)

uint8_t db_mem[1024] = {0};
bat_gatt_service_handler_t bas_service_handler;

bool volatile timer_cb_done = false;
bool volatile flag = true;
bool volatile battery_flag = true;
at_ble_handle_t bat_connection_handle;

/* Application instruction phrase. */
#define STRING_EOL    "\r\n"
#define STRING_HEADER "-- WINC1500 Wi-Fi MQTT chat example --"STRING_EOL \
	"-- "BOARD_NAME " --"STRING_EOL	\
	"-- Compiled: "__DATE__ " "__TIME__ " --"STRING_EOL

/** UART module for debug. */
static struct usart_module cdc_uart_module;

/** Instance of Timer module. */
struct sw_timer_module swt_module_inst;

/** User name of chat. */
char mqtt_user[64] = "";

/* Instance of MQTT service. */
static struct mqtt_module mqtt_inst;

/* Receive buffer of the MQTT service. */
static char mqtt_buffer[MAIN_MQTT_BUFFER_SIZE];

/** UART buffer. */
static char uart_buffer[MAIN_CHAT_BUFFER_SIZE];

/** Written size of UART buffer. */
static int uart_buffer_written = 0;

/** A buffer of character from the serial. */
static uint16_t uart_ch_buffer;

/**
* \Timer callback handler called on timer expiry
*/
static void timer_callback_handler(void)
{
	//Timer call back
	timer_cb_done = true;
}

/* Advertisement data set and Advertisement start */
static at_ble_status_t battery_service_advertise(void)
{
	at_ble_status_t status = AT_BLE_FAILURE;
	
	if((status = ble_advertisement_data_set()) != AT_BLE_SUCCESS)
	{
		//printf("advertisement data set failed reason :%d",status);
		return status;
	}
	
	/* Start of advertisement */
	if((status = at_ble_adv_start(AT_BLE_ADV_TYPE_UNDIRECTED, AT_BLE_ADV_GEN_DISCOVERABLE, NULL, AT_BLE_ADV_FP_ANY, APP_BAS_FAST_ADV, APP_BAS_ADV_TIMEOUT, 0)) == AT_BLE_SUCCESS)
	{
		//printf("BLE Started Adv");
		LED_On(LED0);
		return AT_BLE_SUCCESS;
	}
	else
	{
		//printf("BLE Adv start Failed reason :%d",status);
	}
	return status;
}


/* Callback registered for AT_BLE_PAIR_DONE event from stack */
static at_ble_status_t ble_paired_app_event(void *param)
{
	timer_cb_done = false;
	hw_timer_start(BATTERY_UPDATE_INTERVAL);
	ALL_UNUSED(param);
	return AT_BLE_SUCCESS;
}

/* Callback registered for AT_BLE_DISCONNECTED event from stack */
static at_ble_status_t ble_disconnected_app_event(void *param)
{
	timer_cb_done = false;
	flag = true;
	hw_timer_stop();
	battery_service_advertise();
	ALL_UNUSED(param);
	return AT_BLE_SUCCESS;
}

static at_ble_status_t ble_connected_app_event(void *param)
{
	at_ble_connected_t *connected = (at_ble_connected_t *)param;
	bat_connection_handle = connected->handle;
	#if !BLE_PAIR_ENABLE
	ble_paired_app_event(param);
	#else
	ALL_UNUSED(param);
	#endif
	return AT_BLE_SUCCESS;
}

/* Callback registered for AT_BLE_NOTIFICATION_CONFIRMED event from stack */
static at_ble_status_t ble_notification_confirmed_app_event(void *param)
{
	at_ble_cmd_complete_event_t *notification_status = (at_ble_cmd_complete_event_t *)param;
	if(!notification_status->status)
	{
		flag = true;
		//printf("sending notification to the peer success");
	}
	return AT_BLE_SUCCESS;
}

/* Callback registered for AT_BLE_CHARACTERISTIC_CHANGED event from stack */
static at_ble_status_t ble_char_changed_app_event(void *param)
{
	at_ble_characteristic_changed_t *char_handle = (at_ble_characteristic_changed_t *)param;
	return bat_char_changed_event(char_handle->conn_handle,&bas_service_handler, char_handle, &flag);
}



static const ble_event_callback_t battery_app_gap_cb[] = {
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	ble_connected_app_event,
	ble_disconnected_app_event,
	NULL,
	NULL,
	ble_paired_app_event,
	NULL,
	NULL,
	NULL,
	NULL,
	ble_paired_app_event,
	NULL,
	NULL,
	NULL,
	NULL
};

static const ble_event_callback_t battery_app_gatt_server_cb[] = {
	ble_notification_confirmed_app_event,
	NULL,
	ble_char_changed_app_event,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL
};


/**
 * \brief Callback of USART input.
 *
 * \param[in] module USART module structure.
 */
static void uart_callback(const struct usart_module *const module)
{
	/* If input string is bigger than buffer size limit, ignore the excess part. */
	if (uart_buffer_written < MAIN_CHAT_BUFFER_SIZE) {
		uart_buffer[uart_buffer_written++] = uart_ch_buffer & 0xFF;
	}
}

/**
 * \brief Callback to get the Wi-Fi status update.
 *
 * \param[in] msg_type type of Wi-Fi notification. Possible types are:
 *  - [M2M_WIFI_RESP_CURRENT_RSSI](@ref M2M_WIFI_RESP_CURRENT_RSSI)
 *  - [M2M_WIFI_RESP_CON_STATE_CHANGED](@ref M2M_WIFI_RESP_CON_STATE_CHANGED)
 *  - [M2M_WIFI_RESP_CONNTION_STATE](@ref M2M_WIFI_RESP_CONNTION_STATE)
 *  - [M2M_WIFI_RESP_SCAN_DONE](@ref M2M_WIFI_RESP_SCAN_DONE)
 *  - [M2M_WIFI_RESP_SCAN_RESULT](@ref M2M_WIFI_RESP_SCAN_RESULT)
 *  - [M2M_WIFI_REQ_WPS](@ref M2M_WIFI_REQ_WPS)
 *  - [M2M_WIFI_RESP_IP_CONFIGURED](@ref M2M_WIFI_RESP_IP_CONFIGURED)
 *  - [M2M_WIFI_RESP_IP_CONFLICT](@ref M2M_WIFI_RESP_IP_CONFLICT)
 *  - [M2M_WIFI_RESP_P2P](@ref M2M_WIFI_RESP_P2P)
 *  - [M2M_WIFI_RESP_AP](@ref M2M_WIFI_RESP_AP)
 *  - [M2M_WIFI_RESP_CLIENT_INFO](@ref M2M_WIFI_RESP_CLIENT_INFO)
 * \param[in] pvMsg A pointer to a buffer containing the notification parameters
 * (if any). It should be casted to the correct data type corresponding to the
 * notification type. Existing types are:
 *  - tstrM2mWifiStateChanged
 *  - tstrM2MWPSInfo
 *  - tstrM2MP2pResp
 *  - tstrM2MAPResp
 *  - tstrM2mScanDone
 *  - tstrM2mWifiscanResult
 */
static void wifi_callback(uint8 msg_type, void *msg_data)
{
	tstrM2mWifiStateChanged *msg_wifi_state;
	uint8 *msg_ip_addr;

	switch (msg_type) {
	case M2M_WIFI_RESP_CON_STATE_CHANGED:
		msg_wifi_state = (tstrM2mWifiStateChanged *)msg_data;
		if (msg_wifi_state->u8CurrState == M2M_WIFI_CONNECTED) {
			/* If Wi-Fi is connected. */
			//printf("Wi-Fi connected\r\n");
			m2m_wifi_request_dhcp_client();
		} else if (msg_wifi_state->u8CurrState == M2M_WIFI_DISCONNECTED) {
			/* If Wi-Fi is disconnected. */
			//printf("Wi-Fi disconnected\r\n");
			m2m_wifi_connect((char *)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID),
					MAIN_WLAN_AUTH, (char *)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);
			/* Disconnect from MQTT broker. */
			/* Force close the MQTT connection, because cannot send a disconnect message to the broker when network is broken. */
			mqtt_disconnect(&mqtt_inst, 1);
		}

		break;

	case M2M_WIFI_REQ_DHCP_CONF:
		msg_ip_addr = (uint8 *)msg_data;
		//printf("Wi-Fi IP is %u.%u.%u.%u\r\n",msg_ip_addr[0], msg_ip_addr[1], msg_ip_addr[2], msg_ip_addr[3]);
		/* Try to connect to MQTT broker when Wi-Fi was connected. */
		mqtt_connect(&mqtt_inst, main_mqtt_broker);
		break;

	default:
		break;
	}
}

/**
 * \brief Callback to get the Socket event.
 *
 * \param[in] Socket descriptor.
 * \param[in] msg_type type of Socket notification. Possible types are:
 *  - [SOCKET_MSG_CONNECT](@ref SOCKET_MSG_CONNECT)
 *  - [SOCKET_MSG_BIND](@ref SOCKET_MSG_BIND)
 *  - [SOCKET_MSG_LISTEN](@ref SOCKET_MSG_LISTEN)
 *  - [SOCKET_MSG_ACCEPT](@ref SOCKET_MSG_ACCEPT)
 *  - [SOCKET_MSG_RECV](@ref SOCKET_MSG_RECV)
 *  - [SOCKET_MSG_SEND](@ref SOCKET_MSG_SEND)
 *  - [SOCKET_MSG_SENDTO](@ref SOCKET_MSG_SENDTO)
 *  - [SOCKET_MSG_RECVFROM](@ref SOCKET_MSG_RECVFROM)
 * \param[in] msg_data A structure contains notification informations.
 */
static void socket_event_handler(SOCKET sock, uint8_t msg_type, void *msg_data)
{
	mqtt_socket_event_handler(sock, msg_type, msg_data);
}

/**
 * \brief Callback of gethostbyname function.
 *
 * \param[in] doamin_name Domain name.
 * \param[in] server_ip IP of server.
 */
static void socket_resolve_handler(uint8_t *doamin_name, uint32_t server_ip)
{
	mqtt_socket_resolve_handler(doamin_name, server_ip);
}

/**
 * \brief Callback to get the MQTT status update.
 *
 * \param[in] conn_id instance id of connection which is being used.
 * \param[in] type type of MQTT notification. Possible types are:
 *  - [MQTT_CALLBACK_SOCK_CONNECTED](@ref MQTT_CALLBACK_SOCK_CONNECTED)
 *  - [MQTT_CALLBACK_CONNECTED](@ref MQTT_CALLBACK_CONNECTED)
 *  - [MQTT_CALLBACK_PUBLISHED](@ref MQTT_CALLBACK_PUBLISHED)
 *  - [MQTT_CALLBACK_SUBSCRIBED](@ref MQTT_CALLBACK_SUBSCRIBED)
 *  - [MQTT_CALLBACK_UNSUBSCRIBED](@ref MQTT_CALLBACK_UNSUBSCRIBED)
 *  - [MQTT_CALLBACK_DISCONNECTED](@ref MQTT_CALLBACK_DISCONNECTED)
 *  - [MQTT_CALLBACK_RECV_PUBLISH](@ref MQTT_CALLBACK_RECV_PUBLISH)
 * \param[in] data A structure contains notification informations. @ref mqtt_data
 */
static void mqtt_callback(struct mqtt_module *module_inst, int type, union mqtt_data *data)
{
	switch (type) {
	case MQTT_CALLBACK_SOCK_CONNECTED:
	{
		/*
		 * If connecting to broker server is complete successfully, Start sending CONNECT message of MQTT.
		 * Or else retry to connect to broker server.
		 */
		if (data->sock_connected.result >= 0) {
			mqtt_connect_broker(module_inst, 1, NULL, NULL, mqtt_user, NULL, NULL, 0, 0, 0);
		} else {
			//printf("Connect fail to server(%s)! retry it automatically.\r\n", main_mqtt_broker);
			mqtt_connect(module_inst, main_mqtt_broker); /* Retry that. */
		}
	}
	break;

	case MQTT_CALLBACK_CONNECTED:
		if (data->connected.result == MQTT_CONN_RESULT_ACCEPT) {
			LED_On(LED0);
			/* Subscribe chat topic. */
			//mqtt_subscribe(module_inst, MAIN_CHAT_TOPIC "#", 0);
			/* Enable USART receiving callback. */
			//printf("Preparation of the chat has been completed.\r\n");
		} else {
			/* Cannot connect for some reason. */
			//printf("MQTT broker decline your access! error code %d\r\n", data->connected.result);
			LED_Off(LED0);
		}

		break;

	case MQTT_CALLBACK_RECV_PUBLISH:
		/* You received publish message which you had subscribed. */
		if (data->recv_publish.topic != NULL && data->recv_publish.msg != NULL) {
			if (!strncmp(data->recv_publish.topic, MAIN_MQTT_TOPIC, strlen(MAIN_MQTT_TOPIC))) {
				/* Print user name and message */
				for (int i = strlen(MAIN_MQTT_TOPIC); i < data->recv_publish.topic_size; i++) {
					//printf("%c", data->recv_publish.topic[i]);
				}
				//printf(" >> ");
				for (int i = 0; i < data->recv_publish.msg_size; i++) {
					//printf("%c", data->recv_publish.msg[i]);
				}
				//printf("\r\n");
			}
		}

		break;

	case MQTT_CALLBACK_DISCONNECTED:
		/* Stop timer and USART callback. */
		//printf("MQTT disconnected\r\n");
		LED_Off(LED0);
		break;
	}
}

/**
 * \brief Configure UART console.
 */
static void configure_console(void)
{
	struct usart_config usart_conf;

	usart_get_config_defaults(&usart_conf);
	usart_conf.mux_setting = EDBG_CDC_SERCOM_MUX_SETTING;
	usart_conf.pinmux_pad0 = EDBG_CDC_SERCOM_PINMUX_PAD0;
	usart_conf.pinmux_pad1 = EDBG_CDC_SERCOM_PINMUX_PAD1;
	usart_conf.pinmux_pad2 = EDBG_CDC_SERCOM_PINMUX_PAD2;
	usart_conf.pinmux_pad3 = EDBG_CDC_SERCOM_PINMUX_PAD3;
	usart_conf.baudrate    = 115200;

	stdio_serial_init(&cdc_uart_module, EDBG_CDC_MODULE, &usart_conf);
	/* Register USART callback for receiving user input. */
	usart_register_callback(&cdc_uart_module, (usart_callback_t)uart_callback, USART_CALLBACK_BUFFER_RECEIVED);
	usart_enable(&cdc_uart_module);
}

/**
 * \brief Configure Timer module.
 */
static void configure_timer(void)
{
	struct sw_timer_config swt_conf;
	sw_timer_get_config_defaults(&swt_conf);

	sw_timer_init(&swt_module_inst, &swt_conf);
	sw_timer_enable(&swt_module_inst);
}

/**
 * \brief Configure MQTT service.
 */
static void configure_mqtt(void)
{
	struct mqtt_config mqtt_conf;
	int result;

	mqtt_get_config_defaults(&mqtt_conf);
	/* To use the MQTT service, it is necessary to always set the buffer and the timer. */
	mqtt_conf.timer_inst = &swt_module_inst;
	mqtt_conf.recv_buffer = mqtt_buffer;
	mqtt_conf.recv_buffer_size = MAIN_MQTT_BUFFER_SIZE;

	result = mqtt_init(&mqtt_inst, &mqtt_conf);
	if (result < 0) {
		//printf("MQTT initialization failed. Error code is (%d)\r\n", result);
		while (1) {
		}
	}

	result = mqtt_register_callback(&mqtt_inst, mqtt_callback);
	if (result < 0) {
		//printf("MQTT register callback failed. Error code is (%d)\r\n", result);
		while (1) {
		}
	}
}

/**
 * \brief Checking the USART buffer.
 *
 * Finding the new line character(\n or \r\n) in the USART buffer.
 * If buffer was overflowed, Sending the buffer.
 */
static void check_usart_buffer(char *topic)
{
	int i;

	/* Publish the input string when newline was received or input string is bigger than buffer size limit. */
	if (uart_buffer_written >= MAIN_CHAT_BUFFER_SIZE) {
		mqtt_publish(&mqtt_inst, topic, uart_buffer, MAIN_CHAT_BUFFER_SIZE, 0, 0);
		uart_buffer_written = 0;
	} else {
		for (i = 0; i < uart_buffer_written; i++) {
			/* Find newline character ('\n' or '\r\n') and publish the previous string . */
			if (uart_buffer[i] == '\n') {
				mqtt_publish(&mqtt_inst, topic, uart_buffer, (i > 0 && uart_buffer[i - 1] == '\r') ? i - 1 : i, 0, 0);
				/* Move remain data to start of the buffer. */
				if (uart_buffer_written > i + 1) {
					memmove(uart_buffer, uart_buffer + i + 1, uart_buffer_written - i - 1);
					uart_buffer_written = uart_buffer_written - i - 1;
				} else {
					uart_buffer_written = 0;
				}

				break;
			}
		}
	}
}

/**
 * \brief Main application function.
 *
 * Application entry point.
 *
 * \return program return value.
 */
int main(void)
{
	tstrWifiInitParam param;
	int8_t ret, ret_ble;
	char topic[strlen(MAIN_MQTT_TOPIC) + MAIN_CHAT_USER_NAME_SIZE + 1],msg[5];
	uint8_t battery_level = BATTERY_MIN_LEVEL;

	/* Initialize the board. */
	system_init();

	/* Initialize the UART console. */
	//configure_console();

	/* Output example information */
	//printf(STRING_HEADER);

	/* Initialize the Timer. */
	configure_timer();

	/* Initialize the MQTT service. */
	configure_mqtt();

	/* Initialize the hardware timer */
	hw_timer_init();

	/* Register the callback */
	hw_timer_register_callback(timer_callback_handler);

	/* Initialize the BSP. */
	nm_bsp_init();


	/* initialize the ble chip  and Set the device mac address */
	ble_device_init(NULL);

	/* Initialize the battery service */
	bat_init_service(&bas_service_handler, &battery_level);

	/* Define the primary service in the GATT server database */
	if((ret = bat_primary_service_define(&bas_service_handler))!= AT_BLE_SUCCESS)
	{
		//printf("defining battery service failed %d", ret);
	}

	battery_service_advertise();

	/* Register callbacks for gap related events */
	ble_mgr_events_callback_handler(REGISTER_CALL_BACK, BLE_GAP_EVENT_TYPE,	battery_app_gap_cb);

	/* Register callbacks for gatt server related events */
	ble_mgr_events_callback_handler(REGISTER_CALL_BACK,	BLE_GATT_SERVER_EVENT_TYPE,	battery_app_gatt_server_cb);


	/* Setup user name first */
 	sprintf(mqtt_user,"%s", MAIN_MQTT_TOPIC_NAME);
	sprintf(topic, "%s%s", MAIN_MQTT_TOPIC,MAIN_MQTT_TOPIC_NAME);

	/* Initialize Wi-Fi parameters structure. */
	memset((uint8_t *)&param, 0, sizeof(tstrWifiInitParam));

	/* Initialize Wi-Fi driver with data and status callbacks. */
	param.pfAppWifiCb = wifi_callback; /* Set Wi-Fi event callback. */
	ret = m2m_wifi_init(&param);
	if (M2M_SUCCESS != ret) {
		//printf("main: m2m_wifi_init call error!(%d)\r\n", ret);
		while (1) { /* Loop forever. */
		}
	}

	/* Initialize socket interface. */
	socketInit();
	registerSocketCallback(socket_event_handler, socket_resolve_handler);

	/* Connect to router. */
	m2m_wifi_connect((char *)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID),
			MAIN_WLAN_AUTH, (char *)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);

	while (1) {
		/* Handle pending events from network controller. */
		m2m_wifi_handle_events(NULL);
		/* BLE Event Task */
		ble_event_task();
		/* Checks the timer timeout. */
		sw_timer_task(&swt_module_inst);
		/* Checks the USART buffer. */
		if (timer_cb_done)
		{
			timer_cb_done = false;
			/* send the notification and Update the battery level  */
			if(flag == true){
				ret_ble = bat_update_char_value(bat_connection_handle,&bas_service_handler, battery_level, &flag);
				sprintf((char*) msg, "%d",battery_level);
				mqtt_publish(&mqtt_inst, topic, msg, strlen((char *)msg), 0, 0);
				LED_Toggle(LED0);
				if (ret_ble == AT_BLE_SUCCESS) {
					DBG_LOG("Battery Level:%d%%", battery_level);
					} else {
					LED_Off(LED0);
					//printf("main: failed to send status report error!\r\n");
				}
			}
			if(battery_level == BATTERY_MAX_LEVEL)
			{
				battery_flag = false;
			}
			else if(battery_level == BATTERY_MIN_LEVEL)
			{
				battery_flag = true;
			}
			if(battery_flag)
			{
				battery_level= battery_level+1;
			}
			else
			{
				battery_level= battery_level-1;
			}
		}		
	}
}
