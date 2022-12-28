#include <math.h>
#include <stdio.h>

#include "bms.pio.h"
#include "can.h"
#include "device/usbd.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "hardware/pll.h"
#include "hardware/regs/io_bank0.h"
#include "hardware/regs/usb.h"
#include "hardware/rosc.h"
#include "hardware/spi.h"
#include "hardware/structs/usb.h"
#include "hardware/sync.h"
#include "hardware/xosc.h"
#include "pico/sleep.h"
#include "pico/stdlib.h"

// Number of parallel strings
#define PARALLEL_STRINGS 1
// Number of modules
#define MODULES_1 10
#define MODULES_2 0

// Min absolute voltage to enable balancing.
// 52428 = 4.0V, 53738 = 4.1V, 54525 = 4.16V
#define BALANCE_MIN 52428
// Min difference to enable balancing. 131 = 10mV
#define BALANCE_DIFF 131
// Number of cells per module to bleed simultaneously
#define MAX_BALANCE_CELLS 1

// It should not be necessary for most users to change anything below this
// point.

// The number of battery interfaces on the board
#define CHAIN_COUNT 2
#define MAX_MODULES (16 * CHAIN_COUNT)

// Hardware wakeup pins
#define WAKE1 26
#define WAKE2 27

// Define pins for SPI (to CAN)
#define SPI_PORT spi0
#define SPI_MISO 16
#define SPI_CS 17
#define SPI_CLK 18
#define SPI_MOSI 19
#define CAN_INT 20    // Interrupt from CAN controller
#define CAN_CLK 21    // 8MHz clock for CAN
#define CAN_SLEEP 22  // Shut down CAN transceiver

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
struct battery_interface battery_interfaces[3] = {{
                                                      .serial_out = 2,
                                                      .serial_master = 3,
                                                      .serial_enable = 4,
                                                      .serial_in = 5,
                                                      .module_count = MODULES_1,
                                                      .sm = 0,
                                                  },
                                                  {
                                                      .serial_out = 10,
                                                      .serial_master = 11,
                                                      .serial_enable = 12,
                                                      .serial_in = 13,
                                                      .module_count = MODULES_2,
                                                      .sm = 1,
                                                  }};

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
uint8_t rewake;  // Whether the modules need to be woken up on next execution

// PIO and state machine selection
#define SM_SQ 0

void SPI_configure() {
  spi_init(SPI_PORT, 1000000);
  spi_set_format(SPI_PORT, 8, 0, 0, SPI_MSB_FIRST);
  gpio_set_function(SPI_MISO, GPIO_FUNC_SPI);
  gpio_set_function(SPI_MOSI, GPIO_FUNC_SPI);
  gpio_set_function(SPI_CLK, GPIO_FUNC_SPI);
  gpio_init(SPI_CS);
  gpio_set_dir(SPI_CS, GPIO_OUT);
  gpio_put(SPI_CS, 1);
}

void CAN_reset() {
  gpio_put(SPI_CS, 0);
  spi_write_blocking(SPI_PORT, (uint8_t[]){CMD_RESET}, 1);
  gpio_put(SPI_CS, 1);
  busy_wait_us(100);
}

uint8_t CAN_reg_read(uint8_t reg) {
  uint8_t data;
  gpio_put(SPI_CS, 0);
  spi_write_blocking(SPI_PORT, (uint8_t[]){CMD_READ, reg}, 2);
  spi_read_blocking(SPI_PORT, 0, &data, 1);
  gpio_put(SPI_CS, 1);
  return (data);
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
  // Magic constants from
  // https://github.com/sandeepmistry/arduino-CAN/blob/master/src/MCP2515.cpp
  CAN_reg_write(REG_CNF1, 0x00);
  CAN_reg_write(REG_CNF2, 0x90);
  CAN_reg_write(REG_CNF3, 0x02);

  // Enable Filters
  CAN_reg_write(REG_RXBnCTRL(0), 1 << 2);  // Enable rollover from BUF0 to BUF1
  CAN_reg_write(REG_RXBnCTRL(1), 0);
  // Set masks for RXB0 and RXB1 the same
  for (int n = 0; n < 2; n++) {
    uint16_t mask = 0x7ff;
    CAN_reg_write(REG_RXMnSIDH(n), mask >> 3);
    CAN_reg_write(REG_RXMnSIDL(n), mask << 5);
    CAN_reg_write(REG_RXMnEID8(n), 0);
    CAN_reg_write(REG_RXMnEID0(n), 0);
  }
  // Set match ID for all filters the same
  for (int n = 0; n < 6; n++) {
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
  CAN_reg_write(REG_TXBnSIDH(0), id >> 3);  // Set CAN ID
  CAN_reg_write(REG_TXBnSIDL(0), id << 5);  // Set CAN ID
  CAN_reg_write(REG_TXBnEID8(0), 0x00);     // Extended ID
  CAN_reg_write(REG_TXBnEID0(0), 0x00);     // Extended ID

  CAN_reg_write(REG_TXBnDLC(0), length);  // Frame length

  for (int i = 0; i < length; i++) {  // Write the frame data
    CAN_reg_write(REG_TXBnD0(0) + i, data[i]);
  }

  CAN_reg_write(REG_TXBnCTRL(0), 0x08);              // Start sending
  busy_wait_us(1000);                                // Allow up to 1ms to transmit
  CAN_reg_write(REG_TXBnCTRL(0), 0);                 // Stop sending
  CAN_reg_modify(REG_CANINTF, FLAG_TXnIF(0), 0x00);  // Clear interrupt flag
}

// Calculate message CRC.
uint16_t crc16(uint8_t* message, uint8_t length) {
  uint16_t crc = 0;
  uint16_t j;
  while (length--) {
    crc ^= *message++;
    for (j = 0; j < 8; j++) {
      crc = (crc >> 1) ^ ((crc & 1) ? 0xa001 : 0);
    }
  }
  return crc;
}

// Deactivate the TX PIO and send a square wave to wake up the device
void wakeup(struct battery_interface* battery_interface) {
  // Enable the line driver
  gpio_put(battery_interface->serial_enable, 0);
  // Disable TX PIO
  pio_sm_set_enabled(pio0, battery_interface->sm, false);
  // Wait for it to be disabled
  busy_wait_ms(1);
  // Loop for 100 x 10us
  for (int n = 0; n < 100; n++) {
    pio_sm_set_pins(pio0, battery_interface->sm, (1 << battery_interface->serial_master) | (1 << battery_interface->serial_out));  // Drive DO high (DE enabled)
    busy_wait_us(2);
    pio_sm_set_pins(pio0, battery_interface->sm, (1 << battery_interface->serial_master) | (0 << battery_interface->serial_out));  // Drive DO low (DE enabled)
    busy_wait_us(2);
  }
  pio_sm_set_pins(pio0, battery_interface->sm, (1 << battery_interface->serial_master) | (1 << battery_interface->serial_out));  // Drive DO high (DE enabled)
  busy_wait_us(2);
  // Disable DE, stop driving bus
  pio_sm_set_pins(pio0, battery_interface->sm, 0);
  // Re-enable TX PIO
  pio_sm_set_enabled(pio0, battery_interface->sm, true);
}

// Send a command string
void send_command(struct battery_interface* battery_interface, uint8_t* command, uint8_t length) {
  uint16_t crc = crc16(command, length);
  // Append framing but to the first byte and send it
  pio_sm_put_blocking(pio0, battery_interface->sm, command[0] | 0x100);
  // Send remaining bytes
  for (int n = 1; n < length; n++) pio_sm_put_blocking(pio0, battery_interface->sm, command[n]);
  // Send CRC16
  pio_sm_put_blocking(pio0, battery_interface->sm, crc & 0xFF);
  pio_sm_put_blocking(pio0, battery_interface->sm, crc >> 8);
  busy_wait_us(20);  // Always insert a short pause after sending commands
}

// Receive data from PIO into a local buffer, size limit and timeout in
// microseconds specified It's probably unnecessary to do this with an interrupt
// because we know when we expect to receive data
uint16_t receive_data(struct battery_interface* battery_interface, uint8_t* buffer, uint16_t size, uint32_t timeout) {
  uint16_t rx_data_offset = 0;
  // Return immediately if size is zero
  if (size == 0) return 0;
  // Loop until timeout expires
  for (int n = 0; n < timeout; n++) {
    // Check for data in input FIFO
    while (!pio_sm_is_rx_fifo_empty(pio1, battery_interface->sm)) {
      // Receive one byte
      buffer[rx_data_offset++] = pio_sm_get_blocking(pio1, battery_interface->sm);
      // Return full size if we've filled the string
      if (rx_data_offset == size) return size;
    }
    // Sleep 1 microsecond each loop
    busy_wait_us(1);
  }
  // Return partial length received
  return rx_data_offset;
}

// Configure all daisychained packs with sequential addresses
void configure(struct battery_interface* battery_interface) {
  // Fully Enable Differential Interfaces and Select Auto-Addressing Mode
  send_command(battery_interface, (uint8_t[]){0xF2, 0x10, 0x10, 0xE0}, 4);
  // Configure the bq76PL455A-Q1 device to use auto-addressing to select address
  send_command(battery_interface, (uint8_t[]){0xF1, 0x0E, 0x10}, 3);
  // Configure the bq76PL455A-Q1 device to enter auto-address mode
  send_command(battery_interface, (uint8_t[]){0xF1, 0x0C, 0x08}, 3);
  // Configure 16 devices with sequential addresses
  for (int n = 0; n < 16; n++) {
    send_command(battery_interface, (uint8_t[]){0xF1, 0x0A, n}, 3);
  }
}

// Request that all packs simultaneously sample voltage
void sample_all() {
  // 0xFF 0xFF 0xFF - these 24 bits enable sampling of 16 cell voltages and
  //                  8 AUX channels. Some will contain temperature data.
  //                  16x oversampling.
  for (int n = 0; n < CHAIN_COUNT; n++) send_command(battery_interfaces + n, (uint8_t[]){0xF6, 0x02, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x04}, 8);
  // Wait for sampling to complete
  busy_wait_ms(10);
}

// Put all modules to sleep
void sleep_modules() {
  // Broadcast sleep command to each chain
  for (int n = 0; n < CHAIN_COUNT; n++) send_command(battery_interfaces + n, (uint8_t[]){0xF1, 0x0C, 0x48}, 3);
  // Wait a little for safety
  busy_wait_ms(1);
  // Disable line drivers
  for (int n = 0; n < CHAIN_COUNT; n++) gpio_put(battery_interfaces[n].serial_enable, 1);
  // Ensure modules are woken when needed again
  rewake = 1;
}

// Return 1 if all PCB temperature sensors on a module are above 1.0v
uint8_t pcb_below_temp(uint8_t module) {
  if (aux_voltage[module][3] < 13107) return 0;
  if (aux_voltage[module][4] < 13107) return 0;
  if (aux_voltage[module][5] < 13107) return 0;
  if (aux_voltage[module][6] < 13107) return 0;
  return 1;
}

// Dummy interrupt for hardware wakeup
void gpio_callback() {}

void reconfigure_clocks() {
  // Clock the peripherals, ref clk, and rtc from the 12MHz crystal oscillator
  clock_configure(clk_peri, 0, CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_XOSC_CLKSRC, 12000000, 12000000);
  clock_configure(clk_ref, CLOCKS_CLK_REF_CTRL_SRC_VALUE_XOSC_CLKSRC, 0, 12000000, 12000000);
  clock_configure(clk_rtc, 0, CLOCKS_CLK_RTC_CTRL_AUXSRC_VALUE_XOSC_CLKSRC, 12000000, 46875);
  // Shut down unused clocks, PLLs and oscillators
  clock_stop(clk_adc);
  rosc_disable();
  // Disable more clocks when sleeping
  clocks_hw->sleep_en0 = CLOCKS_SLEEP_EN0_CLK_SYS_PLL_USB_BITS;
  clocks_hw->sleep_en1 = CLOCKS_SLEEP_EN1_CLK_SYS_TIMER_BITS | CLOCKS_SLEEP_EN1_CLK_SYS_XOSC_BITS | CLOCKS_SLEEP_EN1_CLK_USB_USBCTRL_BITS | CLOCKS_SLEEP_EN1_CLK_SYS_USBCTRL_BITS;
}

float voltage(uint16_t adc) {
  float v = adc;
  return (v / 13107.f);
}

float temperature(uint16_t adc) {
  float r = 0.0000000347363427499292f * adc * adc - 0.001025770762903f * adc + 2.68235340614337f;
  float t = log(r) * -30.5280964239816f + 95.6841501312447f;
  return t;
}

void deep_sleep() {
  // Deep sleep until woken by hardware;
  tud_disconnect();  // Disconnect USB
  CAN_reg_write(REG_CANCTRL, MODE_SLEEP);
  gpio_put(CAN_SLEEP, 1);  // Sleep the CAN transceiver
  uint32_t s = save_and_disable_interrupts();
  gpio_set_irq_enabled_with_callback(WAKE1, GPIO_IRQ_LEVEL_HIGH, true, &gpio_callback);
  gpio_set_irq_enabled_with_callback(WAKE2, GPIO_IRQ_LEVEL_HIGH, true, &gpio_callback);
  gpio_set_dormant_irq_enabled(WAKE1, GPIO_IRQ_LEVEL_HIGH, true);
  gpio_set_dormant_irq_enabled(WAKE2, GPIO_IRQ_LEVEL_HIGH, true);
  clocks_hw->sleep_en0 = 0;
  clocks_hw->sleep_en1 = 0;
  xosc_dormant();
  reconfigure_clocks();
  gpio_set_irq_enabled_with_callback(WAKE1, GPIO_IRQ_LEVEL_HIGH, false, &gpio_callback);
  gpio_set_irq_enabled_with_callback(WAKE2, GPIO_IRQ_LEVEL_HIGH, false, &gpio_callback);
  restore_interrupts(s);
  SPI_configure();
  gpio_put(CAN_SLEEP, 0);  // Wake the CAN transceiver
  CAN_reg_write(REG_CANCTRL, MODE_NORMAL);
  tud_connect();
  stdio_usb_init();  // Restore USB
}

int usb_suspended() { return (usb_hw->sie_status & USB_SIE_STATUS_SUSPENDED_BITS); }

int main() {
  // Set system clock to 80MHz, this seems like a reasonable value for the 4MHz
  // data
  set_sys_clock_khz(80000, true);
  stdio_init_all();
  reconfigure_clocks();
  // Used for program loading
  int offset;

  //  Used to keep track of battery voltage data stream
  uint8_t can_string = 0, can_module = 0, can_cell = 0;

  // Load and initialize the TX PIO program
  offset = pio_add_program(pio0, &daisychain_tx_program);
  for (int n = 0; n < CHAIN_COUNT; n++) daisychain_tx_program_init(pio0, battery_interfaces[n].sm, offset, battery_interfaces[n].serial_out, battery_interfaces[n].serial_master);

  // Load and initialize the RX PIO program
  offset = pio_add_program(pio1, &daisychain_rx_program);
  for (int n = 0; n < CHAIN_COUNT; n++) daisychain_rx_program_init(pio1, battery_interfaces[n].sm, offset, battery_interfaces[n].serial_in, battery_interfaces[n].serial_master);

  // Configure serial enable pins
  for (int chain = 0; chain < CHAIN_COUNT; chain++) {
    gpio_init(battery_interfaces[chain].serial_enable);
    gpio_set_dir(battery_interfaces[chain].serial_enable, GPIO_OUT);
  }

  // Configure hardware wakeup pins
  gpio_init(WAKE1);
  gpio_init(WAKE2);
  gpio_set_dir(WAKE1, GPIO_IN);
  gpio_set_dir(WAKE2, GPIO_IN);
  gpio_disable_pulls(WAKE1);
  gpio_disable_pulls(WAKE2);

  // Configure CAN transceiver sleep line
  gpio_init(CAN_SLEEP);
  gpio_set_dir(CAN_SLEEP, GPIO_OUT);
  gpio_put(CAN_SLEEP, 0);  // Logic low to wake transceiver

  // Output 8MHz square wave on CAN_CLK pin
  clock_gpio_init(CAN_CLK, CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, 10);

  // Configure SPI to communicate with CAN
  SPI_configure();
  // Set up CAN to receive messages
  CAN_reset();
  CAN_configure(0x4F8);

  // Wake modules immediately
  rewake = 1;
  // Main loop.
  while (1) {
    // Start counter for total voltage
    pack_voltage = 0;

    // If we've been sleeping, wake the modules
    if (rewake) {
      rewake = 0;
      // Wake and reset up the modules
      for (int n = 0; n < CHAIN_COUNT; n++) wakeup(battery_interfaces + n);

      // Give the modules time to reset
      busy_wait_ms(10);

      // Configure the module addresses
      for (int n = 0; n < CHAIN_COUNT; n++) configure(battery_interfaces + n);
    }

    for (int chain = 0; chain < CHAIN_COUNT; chain++) {
      // Set discharge timeout (1min, all modules)
      send_command(battery_interfaces + chain, (uint8_t[]){0xF1, 0x13, (2 << 4) | (1 << 3)}, 3);
      // Disable discharge (all modules)
      send_command(battery_interfaces + chain, (uint8_t[]){0xF2, 0x14, 0, 0}, 4);
      // Set communication timeout (10s) (all modules)
      send_command(battery_interfaces + chain, (uint8_t[]){0xF1, 0x28, (6 << 4)}, 3);
    }
    // Send a broadcast message to all modules in chain to simultaneously sample
    // all cells
    sample_all();

    // Collect voltages and temperature data for all modules
    // We want to complete this loop as fast as possible because balancing must
    // be disabled during measurement
    max_voltage = 0;
    min_voltage = 65535;
    max_temperature = 65535;
    min_temperature = 0;
    for (int module = 0; module < MAX_MODULES; module++) {
      uint8_t chain = module / 16;
      uint8_t submodule = module % 16;
      if (submodule >= battery_interfaces[chain].module_count) continue;

      // Clear the input FIFO just in case
      pio_sm_clear_fifos(pio1, battery_interfaces[chain].sm);
      // Request sampled voltage data from module
      send_command(battery_interfaces + chain, (uint8_t[]){0x81, submodule, 0x02, 0x20}, 4);
      // Receive response data from PIO FIFO into CPU buffer - 51 bytes of data
      // with 10ms timeout 24 values * 2 bytes + length + 2 byte checksum = 51
      uint16_t received = receive_data(battery_interfaces + chain, rx_data_buffer, 51, 10000);
      // Check RX CRC
      uint16_t rx_crc = crc16(rx_data_buffer, 51);
      if (received == 51 && rx_crc == 0) {
        for (int cell = 0; cell < 16; cell++) {
          // nb. Cells are in reverse, cell 16 is reported first
          cell_voltage[module][cell] = rx_data_buffer[(15 - cell) * 2 + 1] << 8 | rx_data_buffer[(15 - cell) * 2 + 2];
          pack_voltage += cell_voltage[module][cell];
          if (cell_voltage[module][cell] > max_voltage) max_voltage = cell_voltage[module][cell];
          if (cell_voltage[module][cell] < min_voltage) min_voltage = cell_voltage[module][cell];
        }
        for (int aux = 0; aux < 8; aux++) {
          aux_voltage[module][aux] = rx_data_buffer[(16 + aux) * 2 + 1] << 8 | rx_data_buffer[(16 + aux) * 2 + 2];
        }
        for (int aux = 1; aux < 3; aux++) {
          // Higher temperatures mean lower values!
          if (aux_voltage[module][aux] < max_temperature) max_temperature = aux_voltage[module][aux];
          if (aux_voltage[module][aux] > min_temperature) min_temperature = aux_voltage[module][aux];
        }
      } else {
        if (received == 51)
          printf("CRC Error %i %i: %04x\n", chain, submodule, rx_crc);
        else
          printf("RX Error %i %i: %i\n", chain, submodule, received);
        error_count++;
        rewake = 1;
        balance_threshold = 0;
        goto softreset;
      }
    }

    // Work out if balancing is required
    if (max_voltage > BALANCE_MIN) {
      if (max_voltage > min_voltage + BALANCE_DIFF) {  // Min cell + 10mV
        // At least one cell is overcharged, lets balance!
        balance_threshold = min_voltage + BALANCE_DIFF;                        // Min cell + 10mV
        if (balance_threshold < BALANCE_MIN) balance_threshold = BALANCE_MIN;  // No less than BALANCE_MIN
      } else {
        // Cells are balanced
        balance_threshold = 0;
      }
    } else {
      // Under balancing threshold
      balance_threshold = 0;
    }

    // Balancing
    for (int module = 0; module < MAX_MODULES; module++) {
      uint8_t chain = module / 16;
      uint8_t submodule = module % 16;
      if (submodule >= battery_interfaces[chain].module_count) continue;
      balance_bitmap[module] = 0;
      // Only balance if conditions are appropriate
      if (pcb_below_temp(module) && balance_threshold)
        // Add up to MAX_BALANCE_CELLS to balancing bitmap
        for (int n = 0; n < MAX_BALANCE_CELLS; n++) {
          uint16_t max_v = 0;
          int8_t selected_cell = 0;
          // Loop over each cell
          for (int cell = 0; cell < 16; cell++) {
            // Ignore cell if already balancing
            if (balance_bitmap[module] & (1 << cell)) continue;
            // If cell is highest cell so far, provisionally select it
            if (cell_voltage[module][cell] > balance_threshold)
              if (cell_voltage[module][cell] > max_v) {
                selected_cell = cell + 1;
                max_v = cell_voltage[module][cell];
              }
          }
          // Add selected candidate to final bitmap
          if (selected_cell) balance_bitmap[module] |= (1 << (selected_cell - 1));
        }
      send_command(battery_interfaces + chain, (uint8_t[]){0x92, submodule, 0x14, balance_bitmap[module] >> 8, balance_bitmap[module]}, 5);
      if (!usb_suspended()) {
        for (int cell = 0; cell < 16; cell++) {
          float v = cell_voltage[module][cell] / 13107.f;
          printf("Module %i Cell %i Voltage: %.4f\n", module, cell, v);
        }
        printf("Module %i T1: %.2f T2: %.2f\n", module, temperature(aux_voltage[module][1]), temperature(aux_voltage[module][2]));
        printf("Module %i PCB: %.2f %.2f %.2f %.2f\n", module, voltage(aux_voltage[module][3]), voltage(aux_voltage[module][4]), voltage(aux_voltage[module][5]), voltage(aux_voltage[module][6]));
        printf("Module %i Balance: %02x\n", module, balance_bitmap[module]);
      }
    }
    if (!usb_suspended()) {
      float v = balance_threshold / 13107.f;
      printf("Balance Threshold: %.2f\n", v);
    }

    // Send general status information to CAN
    pack_voltage /= PARALLEL_STRINGS;
    uint8_t total_module_count = battery_interfaces[0].module_count + battery_interfaces[1].module_count + battery_interfaces[2].module_count;
    CAN_transmit(0x4f0, (uint8_t[]){pack_voltage >> 24, pack_voltage >> 16, pack_voltage >> 8, pack_voltage, balance_threshold >> 8, balance_threshold, error_count, total_module_count}, 8);
    CAN_transmit(0x4f1, (uint8_t[]){max_voltage >> 8, max_voltage, min_voltage >> 8, min_voltage, max_temperature >> 8, max_temperature, min_temperature >> 8, min_temperature}, 8);
    // Send out individual cell voltages one at a time
    uint16_t v = cell_voltage[can_string * 16 + can_module][can_cell];
    CAN_transmit(0x4f2, (uint8_t[]){can_string, can_module, can_cell, v >> 8, v}, 5);
    can_cell++;
    if (can_cell == 16) {
      can_cell = 0;
      can_module++;
      if (can_module == battery_interfaces[can_string].module_count) {
        can_module = 0;
        if (can_string == 2)
          can_string = 0;
        else if (battery_interfaces[can_string + 1].module_count == 0)
          can_string = 0;
        else
          can_string++;
      }
    }

  softreset:
    // Sleep for a minimum of 500ms second per loop.
    sleep_ms(500);
    // If there's no reason to be awake, go into very low power sleep
    if (!balance_threshold && !gpio_get(WAKE1) && !gpio_get(WAKE2) && usb_suspended()) {
      sleep_modules();
      deep_sleep();
      rewake = 1;
    }
  }
}
