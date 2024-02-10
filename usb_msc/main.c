
/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "bsp/board.h"
#include "tusb.h"
#include "hardware/spi.h"
#include "fpga_program.h"
#include "fat_util.h"
#include "flash_util.h"
#include "util.h"

#define spi_default PICO_DEFAULT_SPI_INSTANCE
#define FPGA_CONFIG_LED 18

#define STATEA_LED 26
#define STATEB_LED 25

#define GPIO_SW0 29
#define GPIO_SW1 28
#define GPIO_SW2 27

// TODO
#define ERR_LED 26

/* Blink pattern
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */
enum  {
  BLINK_NOT_MOUNTED = 500,
  BLINK_MOUNTED = 500,
  BLINK_SUSPENDED = 500,
  BLINK_TOUCHED_SRAM = 100,
  BLINK_TOUCHED_FPGA = 2500,
};
void led_blinking_task(void);
void dir_fill_req_entries(uint16_t cluster_num, uint16_t parent_cluster);

void set_err_led(int on)
{
  // TODO
}




uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;

int main() {
    const uint LED_PIN = 24; // LED1
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    gpio_init(STATEA_LED);
    gpio_set_dir(STATEA_LED, GPIO_OUT);
    gpio_init(STATEB_LED);
    gpio_set_dir(STATEB_LED, GPIO_OUT);
    gpio_put(25, 0);
    gpio_put(26, 0);

    gpio_init(GPIO_SW0); gpio_set_dir(GPIO_SW0, GPIO_IN); gpio_pull_up(GPIO_SW0);
    gpio_init(GPIO_SW1); gpio_set_dir(GPIO_SW1, GPIO_IN); gpio_pull_up(GPIO_SW1);
    gpio_init(GPIO_SW2); gpio_set_dir(GPIO_SW2, GPIO_IN); gpio_pull_up(GPIO_SW2);

    board_init();
    spi_init(spi1, 500E3); //Min speed seems to be ~100kHz, below that USB gets angry

    gpio_set_function(11, GPIO_FUNC_SPI); // TX pin
    gpio_set_function(10, GPIO_FUNC_SPI); // CLK pin
    // gpio_set_function(PICO_DEFAULT_SPI_TX_PIN, GPIO_FUNC_SPI);

    gpio_init(FPGA_CONFIG_LED); // LED0 // note should be LED3 at somepoint
    gpio_set_dir(FPGA_CONFIG_LED, GPIO_OUT);
    bitstream_init_spi();

    // check boot sector on spi flash, if not right, overwrite
    struct boot_sector flash_boot_sec = {};
    struct fat_filesystem *fs = get_filesystem();
    spi_flash_read(0x00, &flash_boot_sec, sizeof(flash_boot_sec));
    if (memcmp(&flash_boot_sec, &fs->boot_sec, sizeof(struct boot_sector))) {
        uint16_t num_erases = sizeof(struct fat_filesystem) / 4096;
        uint32_t num_writes = sizeof(struct fat_filesystem) / 256;
        
        for (uint16_t i = 0; i < num_erases; i++) {
            spi_flash_sector_erase(4096 * i);
        }

        for (uint32_t i = 0; i < num_writes; i++) {
            spi_flash_page_program(i * 256, (void *)fs);
        }
    }


    tud_init(BOARD_TUD_RHPORT);
    FPGA_DONE_PIN_SETUP();
    FPGA_NPROG_SETUP();

    dir_fill_req_entries(3, 0);
    dir_fill_req_entries(4, 0);

    // check SPI flash?


    while (true) {
        tud_task(); // tinyusb device task
        led_blinking_task();
        if (!FPGA_ISDONE()) {
          gpio_put(FPGA_CONFIG_LED, 0);
        } else {
          gpio_put(FPGA_CONFIG_LED, 1);
        }
    }
}

// Invoked when device is mounted
void tud_mount_cb(void)
{
  blink_interval_ms = BLINK_MOUNTED;
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
  blink_interval_ms = BLINK_NOT_MOUNTED;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
  (void) remote_wakeup_en;
  blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
  blink_interval_ms = BLINK_MOUNTED;
}

//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
void led_blinking_task(void)
{
  static uint32_t start_ms = 0;
  static bool led_state = false;

  // Blink every interval ms
  if ( board_millis() - start_ms < blink_interval_ms) return; // not enough time
  start_ms += blink_interval_ms;

  board_led_write(led_state);
  led_state = 1 - led_state; // toggle
}
