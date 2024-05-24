
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/flash.h"

#include "config.h"
#include "flash_util.h"
#include "uf2.h"
#include "config.h"
#include "error.h"
#include "util.h"

#define PRELOAD_PROGRAM 0x4D524750
#define PRELOAD_DONE 0x444F4E45
#define PRELOAD_XFER_BLOCK_SIZE 512

/* Taken from  http://elm-chan.org/junk/32bit/binclude.html */
#define IMPORT_BIN(sect, file, sym) asm (\
    ".pushsection " #sect "\n"              /* Change section */\
    ".balign 4\n"                           /* Word alignment */\
    ".global " #sym "\n"                    /* Export the object address */\
    #sym ":\n"                              /* Define the object label */\
    ".incbin \"" file "\"\n"                /* Import the file */\
    ".global _sizeof_" #sym "\n"            /* Export the object size */\
    ".set _sizeof_" #sym ", . - " #sym "\n" /* Define the object size */\
    ".balign 4\n"                           /* Word alignment */\
    ".popsection")                          /* Restore section */

#define FIRMWARE_PRELOAD_FLASH_OFFSET (3 * 1024 * 1024)

extern struct config_options CONFIG;

const uint32_t* uf2_firmware_preload_status_block = (const uint32_t *) (XIP_BASE + FIRMWARE_PRELOAD_FLASH_OFFSET);
const struct UF2_Block* uf2_firmware_preload_blocks = (const struct UF2_Block*) (XIP_BASE + FIRMWARE_PRELOAD_FLASH_OFFSET + 512);

IMPORT_BIN(".rodata", "sonata-fpga-image.bit", bitstream_preload_bytes);
extern const uint8_t bitstream_preload_bytes[];
extern const uint8_t _sizeof_bitstream_preload_bytes;
const uint32_t bitstream_preload_bytes_size = (uint32_t)(&_sizeof_bitstream_preload_bytes);

volatile uint32_t __attribute__ ((aligned (FLASH_SECTOR_SIZE)))
  __in_flash("preload_metadata")
  bitstream_preload_status_block[FLASH_SECTOR_SIZE / 4] = {PRELOAD_PROGRAM};

uint8_t preload_rdback_buf[PRELOAD_XFER_BLOCK_SIZE];

extern uint32_t FLASH_BITSTREAM_OFFSET[];

static void write_firmware_preload(const struct UF2_Block* blocks) {
  uint32_t num_blocks = blocks->numBlocks;

  uint32_t firmware_flash_addr = FLASH_BITSTREAM_OFFSET[0];
  uint32_t firmware_block_counter = CONST_64k;

  struct UF2_Block* cur_block = blocks;

  firmware_init_spi(CONFIG.flash_prog_speed);

  for(int i = 0;i < num_blocks; ++i) {
    // Max erase is 64kB, so do an erase everytime we've programmed 64k
    if (firmware_block_counter >= CONST_64k) {
      PRINT_INFO("Erasing @ %lX", firmware_flash_addr);
      while (spi_flash_is_busy());
      if (spi_flash_64k_erase_nonblocking(firmware_flash_addr)) { // todo add different index options to flash
          PRINT_ERR("FW Erase error @ %lX", firmware_flash_addr);
      }
      while (spi_flash_is_busy());
      firmware_block_counter = 0;
    }

    if (is_uf2_block(cur_block)) {
      if (spi_flash_write_buffer(firmware_flash_addr, cur_block, 512)) {
          PRINT_ERR("FW prog err @ %lX", firmware_flash_addr);
      }

      spi_flash_read(firmware_flash_addr, preload_rdback_buf, 512);
      if (memcmp(cur_block, preload_rdback_buf, 512)) {
          PRINT_ERR("Verify error @ %lX", firmware_flash_addr);
      }

      firmware_block_counter += 512;
      firmware_flash_addr += 512;
      cur_block++;
    } else {
      PRINT_ERR("Expected UF2 block @ %lx", cur_block);
      break;
    }
  }

  PRINT_INFO("Firmware preload done");

  release_spi_io();

  flash_range_erase(FIRMWARE_PRELOAD_FLASH_OFFSET, FLASH_SECTOR_SIZE);

  *((uint32_t*)preload_rdback_buf) = PRELOAD_DONE;

  flash_range_program(FIRMWARE_PRELOAD_FLASH_OFFSET, preload_rdback_buf, FLASH_PAGE_SIZE);
}

static void write_bitstream_preload(const uint8_t* bytes, const uint32_t length) {
  uint8_t* cur_bytes = bytes;
  uint32_t bitstream_flash_addr = FLASH_BITSTREAM_OFFSET[0];
  uint32_t bitstream_erase_block = CONST_64k;

  bitstream_init_spi(CONFIG.flash_prog_speed);

  for(int i = 0;i < length; i += PRELOAD_XFER_BLOCK_SIZE) {
    uint32_t remaining_bytes = length - i;
    uint32_t bytes_to_xfer = min(remaining_bytes, PRELOAD_XFER_BLOCK_SIZE);

    if (bitstream_erase_block >= CONST_64k) {
      PRINT_INFO("Erasing @ %lX", bitstream_flash_addr);
      while (spi_flash_is_busy());
      if (spi_flash_64k_erase_nonblocking(bitstream_flash_addr)) { // todo add different index options to flash
          PRINT_ERR("FW Erase error @ %lX", bitstream_flash_addr);
      }
      while (spi_flash_is_busy());

      bitstream_erase_block = 0;
    }

    if (spi_flash_write_buffer(bitstream_flash_addr, cur_bytes, bytes_to_xfer)) {
        PRINT_ERR("FW prog err @ %lX", bitstream_flash_addr);
    }

    spi_flash_read(bitstream_flash_addr, preload_rdback_buf, bytes_to_xfer);
    if (memcmp(cur_bytes, preload_rdback_buf, bytes_to_xfer)) {
        PRINT_ERR("Verify error @ %lX", bitstream_flash_addr);
    }

    bitstream_flash_addr += PRELOAD_XFER_BLOCK_SIZE;
    bitstream_erase_block += PRELOAD_XFER_BLOCK_SIZE;
    cur_bytes += PRELOAD_XFER_BLOCK_SIZE;
  }

  PRINT_INFO("Preload done");

  release_spi_io();

  uint32_t bitstream_preload_status_block_flash = ((uint32_t)&bitstream_preload_status_block[0]) - XIP_BASE;
  PRINT_INFO("Status block addr %lx, block flash index %lx", ((uint32_t)&bitstream_preload_status_block[0]), bitstream_preload_status_block_flash);
  PRINT_INFO("Bitstream preload status pre write %lx", bitstream_preload_status_block[0]);
  flash_range_erase(bitstream_preload_status_block_flash, FLASH_SECTOR_SIZE);

  *((uint32_t*)preload_rdback_buf) = PRELOAD_DONE;

  flash_range_program(bitstream_preload_status_block_flash, preload_rdback_buf, FLASH_PAGE_SIZE);
  PRINT_INFO("Bitstream preload status post write %lx", bitstream_preload_status_block[0]);
}

void check_pico_flash_for_bitstream_preload(void)
{
  if (bitstream_preload_status_block[0] == PRELOAD_PROGRAM) {
    PRINT_INFO("Bitstream preload present, not yet programmed");
    PRINT_INFO("%d bytes to write", bitstream_preload_bytes_size);
    write_bitstream_preload(bitstream_preload_bytes, bitstream_preload_bytes_size);
  } else if (bitstream_preload_status_block[0] == PRELOAD_DONE) {
    PRINT_INFO("Bitstream preload present, programming done");
  } else {
    PRINT_INFO("No bitstream preload present");
  }
}

void check_pico_flash_for_firmware_preload(void)
{
  if (*uf2_firmware_preload_status_block == PRELOAD_PROGRAM) {
    PRINT_INFO("Firmware preload blocks present, not yet programmed");
    if (!is_uf2_block(uf2_firmware_preload_blocks)) {
      PRINT_INFO("First preload UF2 block is invalid, skipping programming");
      return;
    } else {
      PRINT_INFO("%d Firmware UF2 blocks to write", uf2_firmware_preload_blocks->numBlocks);
      write_firmware_preload(uf2_firmware_preload_blocks);
    }
  } else if (*uf2_firmware_preload_status_block == PRELOAD_DONE) {
    PRINT_INFO("Firmware preload blocks present, programming done");
  } else {
    PRINT_INFO("No firmware preload blocks present");
  }
}
