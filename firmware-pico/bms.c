#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/sleep.h"
#include "hardware/pio.h"
#include "hardware/spi.h"
#include "hardware/pll.h"
#include "hardware/xosc.h"
#include "hardware/rosc.h"
#include "hardware/clocks.h"
#include "hardware/sync.h"
#include "bms.pio.h"
#include "hardware/regs/io_bank0.h"
#include "can.h"
#include <math.h>

// Min difference to enable balancing. 131 = 10mV
#define BALANCE_DIFF 131
// Min absolute voltage to enable balancing. 52428 = 4.0V, 53738 = 4.1V, 54525 = 4.16V
#define BALANCE_MIN 52428

// The number of active battery interfaces
#define CHAIN_COUNT 2
#define MAX_MODULES (16 * CHAIN_COUNT)

// Define pins for SPI (to CAN)
#define SPI_PORT  spi0
#define SPI_MISO  16
#define SPI_CS    17
#define SPI_CLK   18
#define SPI_MOSI  19
#define CAN_INT   20 // Interrupt from CAN controller
#define CAN_CLK   21 // 8MHz clock for CAN

// Struct for RS485 transceiver info
struct battery_interface {
  uint16_t serial_out;
  uint16_t serial_master;
  uint16_t serial_enable;
  uint16_t serial_in;
  uint16_t module_count;
  uint16_t sm;
};

// Define pins for RS485 transceivers
struct battery_interface battery_interfaces[CHAIN_COUNT] = {
  {
    .serial_out    = 2,
    .serial_master = 3,
    .serial_enable = 4,
    .serial_in     = 5,
    .module_count  = 0,
    .sm            = 0,
  }, {
    .serial_out    = 10,
    .serial_master = 11,
    .serial_enable = 12,
    .serial_in     = 13,
    .module_count  = 0,
    .sm            = 2,
  }
};

// Buffers for received data
uint8_t rx_data_buffer[128];

// Other global variables
uint8_t error_count = 0;
uint16_t balance_threshold = 0;
uint16_t cell_voltage[MAX_MODULES][16];
uint16_t aux_voltage[MAX_MODULES][8];
uint16_t balance_bitmap[MAX_MODULES];
uint32_t pack_voltage = 0;

// Variables for balancing process and reporting
uint16_t max_voltage;
uint16_t min_voltage;
uint16_t max_temperature;
uint16_t min_temperature;

// Timer
uint8_t rewake;        // Whether the modules need to be woken up on next execution

// PIO and state machine selection
#define SM_SQ 0

void SPI_configure() {
  spi_init(SPI_PORT, 1000000);
  spi_set_format(SPI_PORT, 8, 0,0,SPI_MSB_FIRST);
  gpio_set_function(SPI_MISO, GPIO_FUNC_SPI);
  gpio_set_function(SPI_MOSI, GPIO_FUNC_SPI);
  gpio_set_function(SPI_CLK, GPIO_FUNC_SPI);
  gpio_init(SPI_CS);
  gpio_set_dir(SPI_CS, GPIO_OUT);
  gpio_put(SPI_CS, 1);
}

void CAN_reset() {
  gpio_put(SPI_CS, 0);
  spi_write_blocking(SPI_PORT, (uint8_t[]){CMD_RESET},1);
  gpio_put(SPI_CS, 1);
  busy_wait_us(100);
}

uint8_t CAN_reg_read(uint8_t reg) {
  uint8_t data;
  gpio_put(SPI_CS, 0);
  spi_write_blocking(SPI_PORT, (uint8_t[]){CMD_READ, reg}, 2);
  spi_read_blocking(SPI_PORT, 0, &data, 1);
  gpio_put(SPI_CS, 1);
  return(data);
}

void CAN_reg_write(uint8_t reg, uint8_t val) {
  gpio_put(SPI_CS, 0);
  spi_write_blocking(SPI_PORT, (uint8_t[]){CMD_WRITE, reg, val}, 3);
  gpio_put(SPI_CS, 1);
}

void CAN_reg_modify(uint8_t reg, uint8_t mask, uint8_t val) {
  gpio_put(SPI_CS, 0);
  busy_wait_us(2);
  spi_write_blocking(SPI_PORT, (uint8_t[]){CMD_MODIFY, reg, mask, val}, 4);
  busy_wait_us(2);
  gpio_put(SPI_CS, 1);
}

void CAN_configure(uint16_t id) {
  // Configure speed to 500kbps based on 8MHz Crystal
  // Magic constants from https://github.com/sandeepmistry/arduino-CAN/blob/master/src/MCP2515.cpp
  CAN_reg_write(REG_CNF1, 0x00);
  CAN_reg_write(REG_CNF2, 0x90);
  CAN_reg_write(REG_CNF3, 0x02);

  // Enable Filters
  CAN_reg_write(REG_RXBnCTRL(0), 1<<2); // Enable rollover from BUF0 to BUF1
  CAN_reg_write(REG_RXBnCTRL(1), 0);
  // Set masks for RXB0 and RXB1 the same
  for(int n=0; n<2; n++) {
    uint16_t mask = 0x7ff;
    CAN_reg_write(REG_RXMnSIDH(n), mask >> 3);
    CAN_reg_write(REG_RXMnSIDL(n), mask << 5);
    CAN_reg_write(REG_RXMnEID8(n), 0);
    CAN_reg_write(REG_RXMnEID0(n), 0);
  }
  // Set match ID for all filters the same
  for(int n=0; n<6; n++) {
    CAN_reg_write(REG_RXFnSIDH(n), id >> 3);
    CAN_reg_write(REG_RXFnSIDL(n), id << 5);
    CAN_reg_write(REG_RXFnEID8(n), 0);
    CAN_reg_write(REG_RXFnEID0(n), 0);
  }

  // Enable receive interrupts
  CAN_reg_write(REG_CANINTE, 3);

  // Set normal operation mode
  CAN_reg_write(REG_CANCTRL, MODE_NORMAL);
}

void CAN_transmit(uint16_t id, uint8_t* data, uint8_t length) {
  CAN_reg_write(REG_TXBnSIDH(0), id >> 3); // Set CAN ID
  CAN_reg_write(REG_TXBnSIDL(0), id << 5); // Set CAN ID
  CAN_reg_write(REG_TXBnEID8(0), 0x00);    // Extended ID
  CAN_reg_write(REG_TXBnEID0(0), 0x00);    // Extended ID

  CAN_reg_write(REG_TXBnDLC(0), length);   // Frame length

  for (int i = 0; i < length; i++) {       // Write the frame data
    CAN_reg_write(REG_TXBnD0(0) + i, data[i]);
  }

  CAN_reg_write(REG_TXBnCTRL(0), 0x08);    // Start sending
  busy_wait_us(1000); // Allow up to 1ms to transmit
  CAN_reg_write(REG_TXBnCTRL(0), 0);    // Stop sending
  CAN_reg_modify(REG_CANINTF, FLAG_TXnIF(0), 0x00); // Clear interrupt flag
}

// Calculate message CRC.
uint16_t crc16(uint8_t * message, uint8_t length) {
  uint16_t crc = 0;
  uint16_t j;
  while (length--) {
    crc ^= *message++;
    for (j=0; j<8; j++) {
      crc = (crc >> 1) ^ ((crc & 1) ? 0xa001 : 0);
    }
  }
  return crc;
}

// Deactivate the TX PIO and send a square wave to wake up the device
void wakeup(struct battery_interface * battery_interface) {
  // Enable the line driver
  gpio_put(battery_interface->serial_enable, 0);
  // Disable TX PIO
  pio_sm_set_enabled(pio0, battery_interface->sm, false);
  // Wait for it to be disabled
  busy_wait_ms(1);
  // Loop for 200 x 10us = 2ms
  for(int n=0; n<200; n++) {
    pio_sm_set_pins(pio0, battery_interface->sm, (1 << battery_interface->serial_master) | (1 << battery_interface->serial_out)); // Drive DO high (DE enabled)
    busy_wait_us(4);
    pio_sm_set_pins(pio0, battery_interface->sm, (1 << battery_interface->serial_master) | (0 << battery_interface->serial_out)); // Drive DO low (DE enabled)
    busy_wait_us(4);
  }
  pio_sm_set_pins(pio0, battery_interface->sm, (1 << battery_interface->serial_master) | (1 << battery_interface->serial_out)); // Drive DO high (DE enabled)
  busy_wait_us(4);
  // Disable DE, stop driving bus
  pio_sm_set_pins(pio0, battery_interface->sm, 0);
  // Re-enable TX PIO
  pio_sm_set_enabled(pio0, battery_interface->sm, true);
}

// Send a command string
void send_command(struct battery_interface * battery_interface, uint8_t* command, uint8_t length) {
  // Append framing but to the first byte and send it
  pio_sm_put_blocking(pio0, battery_interface->sm, command[0] | 0x100);
  // Send remaining bytes
  for(int n=1; n<length; n++)
    pio_sm_put_blocking(pio0, battery_interface->sm, command[n]);
  // Calculate and send CRC16
  uint16_t crc = crc16(command, length);
  pio_sm_put_blocking(pio0, battery_interface->sm, crc & 0xFF);
  pio_sm_put_blocking(pio0, battery_interface->sm, crc >> 8);
  busy_wait_us(20); // Always insert a short pause after sending commands
}

// Receive data from PIO into a local buffer, size limit and timeout in microseconds specified
// It's probably unnecessary to do this with an interrupt because we know when we expect to receive data
uint16_t receive_data(struct battery_interface * battery_interface, uint8_t* buffer, uint16_t size, uint32_t timeout) {
  uint16_t rx_data_offset = 0;
  // Return immediately if size is zero
  if(size == 0) return 0;
  // Loop until timeout expires
  for(int n=0; n<timeout; n++) {
    // Check for data in input FIFO
    while(!pio_sm_is_rx_fifo_empty(pio1, battery_interface->sm)) {
      // Receive one byte
      buffer[rx_data_offset++] = pio_sm_get_blocking(pio1, battery_interface->sm);
      // Return full size if we've filled the string
      if(rx_data_offset == size) return size;
    }
    // Sleep 1 microsecond each loop
    busy_wait_us(1);
  }
  // Return partial length received
  return rx_data_offset;
}

// Configure all daisychained packs with sequential addresses
void configure(struct battery_interface * battery_interface) {
  // Fully Enable Differential Interfaces and Select Auto-Addressing Mode
  send_command(battery_interface, (uint8_t[]){0xF2,0x10,0x10,0xE0}, 4);
  // Configure the bq76PL455A-Q1 device to use auto-addressing to select address
  send_command(battery_interface, (uint8_t[]){0xF1,0x0E,0x10}, 3);
  // Configure the bq76PL455A-Q1 device to enter auto-address mode
  send_command(battery_interface, (uint8_t[]){0xF1,0x0C,0x08}, 3);
  // Configure 16 devices with sequential addresses
  for(int n=0; n<16; n++) {
    send_command(battery_interface, (uint8_t[]){0xF1,0x0A,n}, 3);
  }
  for(int n=0; n<16; n++) {
    // Attempt to read back the address from each device
    pio_sm_clear_fifos(pio1, battery_interface->sm);
    send_command(battery_interface, (uint8_t[]){0x81,n,0x0A,0x00}, 4);
    uint16_t received = receive_data(battery_interface, rx_data_buffer, 4, 10000);
    // If we don't receive a response, assume there are no more modules
    // and return the number of modules successfully found.
    if(received != 4) {
      battery_interface->module_count = n;
      return;
    }
  }
  battery_interface->module_count = 16;
}

// Request that all packs simultaneously sample voltage
void sample_all() {
  // 0xFF 0xFF 0xFF - these 24 bits enable sampling of 16 cell voltages and
  //                  8 AUX channels. Some will contain temperature data.
  //                  16x oversampling.
  for(int n=0; n<CHAIN_COUNT; n++)
    send_command(battery_interfaces + n, (uint8_t[]){0xF6,0x02,0x00,0xFF,0xFF,0xFF,0x00,0x04}, 8);
  // Wait for sampling to complete
  busy_wait_ms(10);
}

// Return 1 if all PCB temperature sensors on a module are above 1.0v
uint8_t pcb_below_temp(uint8_t module) {
  if(aux_voltage[module][3] < 13107) return 0;
  if(aux_voltage[module][4] < 13107) return 0;
  if(aux_voltage[module][5] < 13107) return 0;
  if(aux_voltage[module][6] < 13107) return 0;
  return 1;
}

void reconfigure_clocks() {
  // Clock the peripherals, ref clk, and rtc from the 12MHz crystal oscillator
  clock_configure(clk_peri, 0, CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_XOSC_CLKSRC, 12000000, 12000000);
  clock_configure(clk_ref, CLOCKS_CLK_REF_CTRL_SRC_VALUE_XOSC_CLKSRC, 0, 12000000, 12000000);
  clock_configure(clk_rtc, 0, CLOCKS_CLK_RTC_CTRL_AUXSRC_VALUE_XOSC_CLKSRC, 12000000, 46875);
  // Shut down unused clocks, PLLs and oscillators
  clock_stop(clk_usb);
  clock_stop(clk_usb);
  clock_stop(clk_adc);
  pll_deinit(pll_usb);
  rosc_disable();
  // Disable more clocks when sleeping
  clocks_hw->sleep_en0 = 0;
  clocks_hw->sleep_en1 = CLOCKS_WAKE_EN1_CLK_SYS_TIMER_BITS;
}
int main()
{
  // Set system clock to 80MHz, this seems like a reasonable value for the 4MHz data
  set_sys_clock_khz(80000, true);
  reconfigure_clocks();

  // Used for program loading
  int offset;

  // Load and initialize the TX PIO program
  offset = pio_add_program(pio0, &daisychain_tx_program);
  for(int n=0; n<CHAIN_COUNT; n++)
    daisychain_tx_program_init(pio0, battery_interfaces[n].sm, offset, battery_interfaces[n].serial_out, battery_interfaces[n].serial_master);

  // Load and initialize the RX PIO program
  offset = pio_add_program(pio1, &daisychain_rx_program);
  for(int n=0; n<CHAIN_COUNT; n++)
  daisychain_rx_program_init(pio1, battery_interfaces[n].sm, offset, battery_interfaces[n].serial_in, battery_interfaces[n].serial_master);

  // Configure serial enable pins
  for(int chain = 0; chain < CHAIN_COUNT; chain++) {
    gpio_init(battery_interfaces[chain].serial_enable);
    gpio_set_dir(battery_interfaces[chain].serial_enable, GPIO_OUT);
  }

  // Output 8MHz square wave on CAN_CLK pin
  clock_gpio_init(CAN_CLK, CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, 10);

  // Configure SPI to communicate with CAN
  SPI_configure();
  // Set up CAN to receive messages
  CAN_reset();
  CAN_configure(0x4F8);

softreset:
  // Wake modules immediately
  rewake = 1;
  // Main loop.
  while (1) {
    // Start counter for total voltage
    pack_voltage = 0;

    // If we've been sleeping, wake the modules
    if(rewake) {
      rewake = 0;
      // Wake and reset up the modules
      for(int n=0; n<CHAIN_COUNT; n++)
        wakeup(battery_interfaces + n);

      // Give the modules time to reset
      busy_wait_ms(10);

      // Configure the module addresses
      for(int n=0; n<CHAIN_COUNT; n++)
        configure(battery_interfaces + n);
    }

    for(int chain = 0; chain < CHAIN_COUNT; chain++) {
      // Set discharge timeout (1min, all modules)
      send_command(battery_interfaces + chain, (uint8_t[]){ 0xF1,0x13,(2<<4) | (1<<3) }, 3);
      // Disable discharge (all modules)
      send_command(battery_interfaces + chain, (uint8_t[]){ 0xF2,0x14,0,0 }, 4);
      // Set communication timeout (10s) (all modules)
      send_command(battery_interfaces + chain, (uint8_t[]){ 0xF1,0x28,(6<<4) }, 3);
    }
    // Send a broadcast message to all modules in chain to simultaneously sample all cells
    sample_all();

    // Count modules
    uint8_t total_module_count = 0;
    for(int n=0; n<CHAIN_COUNT;n++) total_module_count+= battery_interfaces[n].module_count;

    // Collect voltages and temperature data for all modules
    // We want to complete this loop as fast as possible because balancing must be disabled during measurement
    max_voltage = 0;
    min_voltage = 65535;
    max_temperature = 0;
    min_temperature = 65535;
    for(int module = 0; module < MAX_MODULES; module++) {
      uint8_t chain = module / 16;
      uint8_t submodule = module % 16;
      if(submodule >= battery_interfaces[chain].module_count) continue;

      // Clear the input FIFO just in case
      pio_sm_clear_fifos(pio1, battery_interfaces[chain].sm);
      // Request sampled voltage data from module
      send_command(battery_interfaces + chain, (uint8_t[]){0x81,submodule,0x02,0x20}, 4);
      // Receive response data from PIO FIFO into CPU buffer - 51 bytes of data with 10ms timeout
      // 24 values * 2 bytes + length + 2 byte checksum = 51
      uint16_t received = receive_data(battery_interfaces + chain, rx_data_buffer, 51, 10000);
      // TODO: check RX CRC here
      if(received == 51) {
        for(int cell=0; cell<16; cell++) {
          // nb. Cells are in reverse, cell 16 is reported first
          cell_voltage[module][cell] = rx_data_buffer[(15-cell)*2+1] << 8 | rx_data_buffer[(15-cell)*2+2];
          pack_voltage += cell_voltage[module][cell];
          if(cell_voltage[module][cell] > max_voltage) max_voltage = cell_voltage[module][cell];
          if(cell_voltage[module][cell] < min_voltage) min_voltage = cell_voltage[module][cell];
        }
        for(int aux=0; aux<8; aux++) {
          aux_voltage[module][aux] = rx_data_buffer[(16+aux)*2+1] << 8 | rx_data_buffer[(16+aux)*2+2];
        }
        for(int aux=1; aux<3; aux++) {
          if(aux_voltage[module][aux] > max_temperature) max_temperature = aux_voltage[module][aux];
          if(aux_voltage[module][aux] < min_temperature) min_temperature = aux_voltage[module][aux];
        }
      } else {
        error_count++;
        goto softreset;
      }
    }

    // Work out if balancing is required
    if(max_voltage > BALANCE_MIN) {
      if(max_voltage > min_voltage + BALANCE_DIFF) { // Min cell + 10mV
        // At least one cell is overcharged, lets balance!
        balance_threshold = min_voltage + BALANCE_DIFF; // Min cell + 10mV
        if(balance_threshold < BALANCE_MIN) balance_threshold = BALANCE_MIN; // No less than BALANCE_MIN
      } else {
        // Cells are balanced
        balance_threshold = 0;
      }
    } else {
      // Under balancing threshold
      balance_threshold = 0;
    }

    // Balancing
    for(int module = 0; module < MAX_MODULES; module++) {
      uint8_t chain = module / 16;
      uint8_t submodule = module % 16;
      if(submodule >= battery_interfaces[chain].module_count) continue;
      balance_bitmap[module] = 0;
      uint16_t max_v = 0;
      for(int cell=0; cell<16; cell++)
        if(pcb_below_temp(module)) // Don't balance if PCB is hot
          if(balance_threshold) // Don't balance unless threshold set
            if(cell_voltage[module][cell] > balance_threshold) // Compare cell voltage to threshold
              if(cell_voltage[module][cell] > max_v) { // Only balance the highest voltage cell
                balance_bitmap[module] = (1 << cell); // Only ever balance one cell
                max_v = cell_voltage[module][cell];
              }
      send_command(battery_interfaces + chain, (uint8_t[]){ 0x92,submodule,0x14,balance_bitmap[module] >> 8, balance_bitmap[module] }, 5);
    }

    // Send general status information to CAN
    CAN_transmit(0x4f0, (uint8_t[]){ pack_voltage>>24, pack_voltage>>16, pack_voltage>>8, pack_voltage, balance_threshold >> 8, balance_threshold, error_count, total_module_count }, 8);
    CAN_transmit(0x4f1, (uint8_t[]){ max_voltage >> 8, max_voltage, min_voltage >> 8, min_voltage, max_temperature >> 8, max_temperature, min_temperature >> 8, min_temperature }, 8);

    sleep_ms(2000);
  }
}
