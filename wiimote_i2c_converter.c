/*
 * Copyright (C) 2019 jp-96
 *
 * Copyright (C) 2017 BlueKitchen GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 * 4. Any redistribution, use, or modification is done solely for
 *    personal benefit and not for any commercial purpose or for
 *    monetary gain.
 *
 * THIS SOFTWARE IS PROVIDED BY BLUEKITCHEN GMBH AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL MATTHIAS
 * RINGWALD OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Please inquire about commercial licensing options at
 * contact@bluekitchen-gmbh.com
 *
 */

#define __BTSTACK_FILE__ "wiimote_i2c_converter.c"

/*
 * wiimote_i2c_converter.c
 */

#include <inttypes.h>
#include <stdio.h>

#include "btstack_config.h"
#include "btstack.h"

#include "esp_log.h"
#include "driver/i2c.h"

// Wiimote
static const char * wiimote_addr_string = "AA-AA-AA-AA-AA-AA";
static bd_addr_t wiimote_addr;

#define WIIMOTE_DATA_PSM        0x13
#define WIIMOTE_MTU             672

 // Player LEDs(http://wiibrew.org/wiki/Wiimote#Player_LEDs)
#define WIIMOTE_FEEDBACK_EVENT_LED1 0x10
#define WIIMOTE_FEEDBACK_EVENT_LED2 0x20
#define WIIMOTE_FEEDBACK_EVENT_LED3 0x40
#define WIIMOTE_FEEDBACK_EVENT_LED4 0x80

 // Rumble(http://wiibrew.org/wiki/Wiimote#Rumble)
#define WIIMOTE_FEEDBACK_EVENT_RUMBLE 0x01

uint8_t wiimote_data = 0;
uint8_t newCtrl = 0;
uint8_t oldCtrl = 0;
bool nitifyConnect = false;

// L2CAP
static uint16_t           l2cap_wiimote_interrupt_cid = -1;

static btstack_packet_callback_registration_t hci_event_callback_registration;

/* @section Main application configuration
 *
 * @text In the application configuration, L2CAP is initialized
 */

static void hci_packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);

static void wiimote_host_setup(void){

    // register for HCI events
    hci_event_callback_registration.callback = &hci_packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    // Initialize L2CAP
    l2cap_init();

    // Disable stdout buffering
    setbuf(stdout, NULL);
}

// I2C definitions
#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define I2C_MASTER_SCL_IO 26               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 25               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUMBER(1) /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

static esp_err_t i2c_master_init()
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode,
                              I2C_MASTER_RX_BUF_DISABLE,
                              I2C_MASTER_TX_BUF_DISABLE, 0);
}

#define ESP_SLAVE_ADDR 0x28 /*!< ESP32 slave address, you can set any 7bit value */
#define REG_MOUS_CTRL 0x01 /*!< ESP32 slave reg address */
#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
/**
 * @brief Test code to write esp-i2c-slave
 *        Master device write data to slave(both esp32),
 *        the data will be stored in slave buffer.
 *        We can read them out from slave buffer.
 *
 * ___________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write n bytes + ack  | stop |
 * --------|---------------------------|----------------------|------|
 *
 */
static esp_err_t i2c_master_write_slave(i2c_port_t i2c_num, uint8_t reg, uint8_t wdata)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ESP_SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, wdata, ACK_CHECK_EN);

    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static const char *TAG = "wii-i2c-converter";

static void send_i2c_reg_data(uint8_t reg, uint8_t wdata)
{
  int ret = i2c_master_write_slave(I2C_MASTER_NUM, reg, wdata);
  if (ret == ESP_ERR_TIMEOUT) {
      ESP_LOGE(TAG, "I2C Timeout");
  } else if (ret == ESP_OK) {
      ESP_LOGI(TAG, "write result: ESP_OK");
  } else {
      ESP_LOGI(TAG, "write result: UNKNOWN");
  }
}

/*
 * @section HCI Packet Handler
 *
 * @text The hci packet handler responds to various HCI Events.
 */
static void hci_packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    uint8_t   event;
    uint8_t   status;
    uint16_t  l2cap_cid;

    switch (packet_type) {
        case HCI_EVENT_PACKET:
            event = hci_event_packet_get_type(packet);
            switch (event) {
                case BTSTACK_EVENT_STATE:
                    if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING){
                        status = l2cap_create_channel(hci_packet_handler, wiimote_addr, WIIMOTE_DATA_PSM, WIIMOTE_MTU, &l2cap_wiimote_interrupt_cid);
                        if (status){
                            printf("Connecting to Wiimote Interrupt failed: 0x%02x\n", status);
                            break;
                        }
                    }
                    break;

                case L2CAP_EVENT_CHANNEL_OPENED:
                    status = l2cap_event_channel_opened_get_status(packet);
                    if (status){
                        printf("L2CAP Connection failed: 0x%02x\n", status);
                        break;
                    }
                    l2cap_cid  = little_endian_read_16(packet, 13);
                    if (!l2cap_cid) break;
                    if (l2cap_cid == l2cap_wiimote_interrupt_cid){
                        printf("Wiimote Connection established\n");
                        nitifyConnect = true;
                    }
                    break;

                case L2CAP_EVENT_CAN_SEND_NOW:
                    if(channel == l2cap_wiimote_interrupt_cid){
                        if(nitifyConnect) {
                          nitifyConnect = false;
                          wiimote_data |= WIIMOTE_FEEDBACK_EVENT_LED4; //detect connection
                        }
                        uint8_t report[] = { 0xA2, 0x11, wiimote_data };
                        printf("L2CAP_EVENT_CAN_SEND_NOW: 0x%02x\n", wiimote_data);
                        l2cap_send(l2cap_wiimote_interrupt_cid, (uint8_t*) report, sizeof(report));
                    }
                    break;
                default:
                    break;
            }
            break;
        case L2CAP_DATA_PACKET:
            if (channel == l2cap_wiimote_interrupt_cid){

                printf("Wiimote Interrupt: ");
                printf_hexdump(packet, size);
                newCtrl = ((packet[2] & 0x0F) << 4) + (packet[3] & 0x0F) + 0x0C; // crosskey, button(1, 2), validation bit

                if (newCtrl == oldCtrl) {
                  ; //do nothing(protection agaist chattering)
                }
                else {
                  printf("send_i2c_reg_data: 0x%02x\n", newCtrl);
                  send_i2c_reg_data(REG_MOUS_CTRL, newCtrl); //send control information to register 01
                  oldCtrl = newCtrl;
                }

                l2cap_request_can_send_now_event(l2cap_wiimote_interrupt_cid);
            } else {
                printf("Unknown channel.\n");
            }
            break;
        default:
            break;
    }
}

int btstack_main(int argc, const char * argv[]);
int btstack_main(int argc, const char * argv[]){

    (void)argc;
    (void)argv;

    wiimote_host_setup();

    i2c_master_init();

    // parse human readable Bluetooth address
    sscanf_bd_addr(wiimote_addr_string, wiimote_addr);

    // Turn on the device
    hci_power_control(HCI_POWER_ON);

    return 0;
}

/* EXAMPLE_END */
