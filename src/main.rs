#![no_std]
#![no_main]

#![feature(stmt_expr_attributes)] //CHECK Maybe refactor so this isn't needed

#[unsafe(link_section = ".boot2")]
#[unsafe(no_mangle)]
#[used]
pub static BOOT2_FIRMWARE: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

use defmt::{info, println};
use embedded_hal::digital::{OutputPin, StatefulOutputPin};
use ringbuffer::{ConstGenericRingBuffer, RingBuffer};
use rp2040_hal::{fugit::RateExtU32, gpio::Pins, uart::{DataBits, StopBits, UartConfig, UartPeripheral}, Clock};
use usb_device::{bus::UsbBusAllocator, device::{StringDescriptors, UsbDeviceBuilder, UsbVidPid}, UsbError};
use usbd_serial::{SerialPort, USB_CLASS_CDC};
use core::result::Result::{Err, Ok};
use panic_probe as _;
use defmt_rtt as _;

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;
pub const BAUD: u32 = 115200;

#[rp2040_hal::entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = rp2040_hal::pac::Peripherals::take().unwrap();
    let core = rp2040_hal::pac::CorePeripherals::take().unwrap();

    info!("point 0");

    // The single-cycle I/O block controls our GPIO pins
    let sio = rp2040_hal::Sio::new(pac.SIO);

    let pins = Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS);

    let mut pin = pins.gpio0.into_push_pull_output();

    pin.set_low().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = rp2040_hal::Watchdog::new(pac.WATCHDOG);

    info!("point 1");

    // Configure the clocks
    let clocks = rp2040_hal::clocks::init_clocks_and_plls(
      XTAL_FREQ_HZ,
      pac.XOSC,
      pac.CLOCKS,
      pac.PLL_SYS,
      pac.PLL_USB,
      &mut pac.RESETS,
      &mut watchdog,
    )
    .unwrap();

    info!("point 2");

    let speed = clocks.system_clock.freq().to_Hz();
    // info!("clock speed: {}Hz", speed);
    let mut delay = cortex_m::delay::Delay::new(core.SYST, speed);
    

    info!("point 5");

    let usb_bus = UsbBusAllocator::new(rp2040_hal::usb::UsbBus::new(
      pac.USBCTRL_REGS,
      pac.USBCTRL_DPRAM,
      clocks.usb_clock,
      true,
      &mut pac.RESETS,
    ));

    info!("Starting loop");

    let mut serial = SerialPort::new(&usb_bus);

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        // .product("Serial port")
        .device_class(USB_CLASS_CDC)
        .build();
    

    const FLASH: bool = false;
    {
      let mut ehat_enable_pin = pins.gpio10.into_push_pull_output();
      let mut ehat_flash_pin = pins.gpio7.into_push_pull_output();      
      let mut ehat_reset_pin = pins.gpio8.into_push_pull_output();      
      let mut ehat_gpio2_pin = pins.gpio9.into_push_pull_output();     
      let mut ehat_vcc_pin = pins.gpio6.into_push_pull_output();     
      let mut ehat_gnd_pin = pins.gpio11.into_push_pull_output();     

      ehat_vcc_pin.set_high().unwrap(); // This is almost certainly insufficient current
      ehat_gnd_pin.set_low().unwrap(); // Ditto
      ehat_gpio2_pin.set_high().unwrap();
      // Set low to flash
      if FLASH {
        ehat_flash_pin.set_low().unwrap();
      } else {
        ehat_flash_pin.set_high().unwrap();
      }
      ehat_enable_pin.set_high().unwrap();

      ehat_reset_pin.set_low().unwrap();
      delay.delay_ms(10);
      ehat_reset_pin.set_high().unwrap();

      // The RX/TX pins, 04 and 05, are handled as part of uart_pins
    }


    let mut gpioTx = pins.gpio4; //RAINY This conflicts with optics - but I guess also in the sense that we only have so many uarts
    let mut gpioRx = pins.gpio5;
        
    let uart_pins = (
      gpioTx.into_function(),
      gpioRx.into_function(),
    );
    let uart = UartPeripheral::new(pac.UART1, uart_pins, &mut pac.RESETS)
      .enable(
        UartConfig::new(BAUD.Hz(), DataBits::Eight, None, StopBits::One),
        clocks.peripheral_clock.freq(),
      ).unwrap();

    const SIZE: usize = 128;
    let mut buf_usb2uart = ConstGenericRingBuffer::<u8, SIZE>::new();
    let mut buf_uart2usb = ConstGenericRingBuffer::<u8, SIZE>::new();

    loop {
      info!("Loop start");
      pin.toggle().unwrap();
      // delay.delay_ms(5);
      if !usb_dev.poll(&mut [&mut serial]) {
        info!("Poll fail");
        // continue; // It seems like poll ACTUALLY only succeeds when there's data to be read.  We may need to WRITE data.
      } else {
        info!("Poll succeed");
      }

      // Copy as many bytes from usb as are available in ring
      let mut temp_usb2uart = [0u8; SIZE];
      let usb_in_count = match serial.read(&mut temp_usb2uart[0..(buf_usb2uart.capacity() - buf_usb2uart.len())]) {
        Ok(count) => {
          info!("1>2 read {}", count);
          // count bytes were read to &buf[..count]
          count
        },
        Err(UsbError::WouldBlock) => {
          info!("1>2 read would block");
          // No data received
          0
        },
        Err(err) => {
          info!("1>2 read error");
          // An error occurred
          0
        },
      };
      // Copy bytes to ring
      for &b in &temp_usb2uart[0..usb_in_count] {
        buf_usb2uart.enqueue(b);
      }
      // Copy ring to uart
      let mut remaining_usb2uart = buf_usb2uart.len();
      buf_usb2uart.copy_to_slice(0, &mut temp_usb2uart[0..remaining_usb2uart]);
      match uart.write_raw(&temp_usb2uart[0..remaining_usb2uart]) {
        Ok(remaining) => {
          let written = remaining_usb2uart - remaining.len();
          info!("1>2 wrote {}", written);
          for _ in 0..written {
            buf_usb2uart.dequeue();
          }
        },
        Err(nb::Error::WouldBlock) => {
          info!("2>1 write would block");
          //CHECK This shouldn't be possible, by the signature; what's up with this
        },
      }


      // Copy as many bytes from uart as are available in ring
      let mut temp_uart2usb = [0u8; SIZE];
      let uart_in_count = match uart.read_raw(&mut temp_uart2usb[0..(buf_uart2usb.capacity() - buf_uart2usb.len())]) {
        Ok(count) => {
          info!("2>1 read {}", count);
          // count bytes were read to &buf[..count]
          count
        },
        Err(nb::Error::WouldBlock) => {
          info!("2>1 read would block");
          // No data received
          0
        },
        Err(err) => {
          info!("2>1 read error");
          // An error occurred
          0
        },
      };
      // Copy bytes to ring
      for &b in &temp_uart2usb[0..uart_in_count] {
        buf_uart2usb.enqueue(b);
      }
      // Copy ring to usb
      let mut remaining_uart2usb = buf_uart2usb.len();
      buf_uart2usb.copy_to_slice(0, &mut temp_uart2usb[0..remaining_uart2usb]);
      match serial.write(&temp_uart2usb[0..remaining_uart2usb]) {
        Ok(written) => {
          info!("2>1 wrote {}", written);
          for _ in 0..written {
            buf_uart2usb.dequeue();
          }
        },
        Err(UsbError::WouldBlock) => {
          if remaining_uart2usb == 0 {
            info!("2>1 nothing to write");
          } else {
            // No data could be written (buffers full)
            info!("2>1 write would block");
          }
        },
        Err(err) => {
          info!("2>1 write error");
          // An error occurred
        },
      }
    }
}
