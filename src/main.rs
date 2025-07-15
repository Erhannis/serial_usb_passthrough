#![no_std]
#![no_main]

#![feature(stmt_expr_attributes)] //CHECK Maybe refactor so this isn't needed

#[unsafe(link_section = ".boot2")]
#[unsafe(no_mangle)]
#[used]
pub static BOOT2_FIRMWARE: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

use defmt::{info, println};
use embedded_hal::digital::{OutputPin, StatefulOutputPin};
use rp2040_hal::{gpio::Pins, Clock};
use usb_device::{bus::UsbBusAllocator, device::{StringDescriptors, UsbDeviceBuilder, UsbVidPid}, UsbError};
use usbd_serial::{SerialPort, USB_CLASS_CDC};
use core::result::Result::{Err, Ok};
use panic_probe as _;
use defmt_rtt as _;

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

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
    
    // let mut gpioTx = pins.gpio4; //RAINY This conflicts with optics - but I guess also in the sense that we only have so many uarts
    // let mut gpioRx = pins.gpio5;
    //
    // let tx_pinid = gpioTx.id();
    //
    // let uart_pins = (
    //   gpioTx.into_function(),
    //   gpioRx.into_function(),
    // );
    // let uart = UartPeripheral::new(peripherals.UART1, uart_pins, &mut peripherals.RESETS)
    //   .enable(
    //     UartConfig::new(BAUD.Hz(), DataBits::Eight, None, StopBits::One),
    //     clocks.peripheral_clock.freq(),
    //   ).unwrap();

    loop {
      info!("Loop start");
      pin.toggle().unwrap();
      delay.delay_ms(10);
      if !usb_dev.poll(&mut [&mut serial]) {
          info!("Poll fail");
          continue;
      }
      info!("Poll succeed");

      let mut buf = [0u8; 64];

      let count = match serial.read(&mut buf[..]) {
          Ok(count) => {
            info!("read {}", count);
            // count bytes were read to &buf[..count]
            count
          },
          Err(UsbError::WouldBlock) => {
            info!("read would block");
            // No data received
            0
          },
          Err(err) => {
            info!("read error");
            // An error occurred
            0
          },
      };

      let buf_out: &[u8] = if count > 0 { &buf[0..count] } else { b"hello\r\n" };
      match serial.write(buf_out) {
          Ok(count) => {
            info!("wrote {}", count);
            // count bytes were written
          },
          Err(UsbError::WouldBlock) => {
            info!("write would block");
            // No data could be written (buffers full)
          },
          Err(err) => {
            info!("write error");
            // An error occurred
          },
      };
    }
}
