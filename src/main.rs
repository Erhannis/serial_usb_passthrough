#![no_std]
#![no_main]

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

  // Set up the watchdog driver - needed by the clock setup code
  let mut watchdog = rp2040_hal::Watchdog::new(pac.WATCHDOG);

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

  // The delay object lets us wait for specified amounts of time (in
  // milliseconds)
  // let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

  // The single-cycle I/O block controls our GPIO pins
  let sio = rp2040_hal::Sio::new(pac.SIO);

  let speed = clocks.system_clock.freq().to_Hz();
  // info!("clock speed: {}Hz", speed);
  let mut delay = cortex_m::delay::Delay::new(core.SYST, speed);
  
  let pins = Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS);
  let mut pin = pins.gpio0.into_push_pull_output();

  loop {
    pin.toggle().unwrap();
  }


  let usb_bus: UsbBusAllocator<rp2040_hal::usb::UsbBus> = UsbBusAllocator::new(rp2040_hal::usb::UsbBus::new(
    pac.USBCTRL_REGS,
    pac.USBCTRL_DPRAM,
    clocks.usb_clock,
    true,
    &mut pac.RESETS,
  ));

  let mut serial = SerialPort::new(&usb_bus);

  let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
      // .product("Serial port")
      .device_class(USB_CLASS_CDC)
      .build();
  
  println!("Starting loop");
  
  loop {
      pin.toggle().unwrap();
      if !usb_dev.poll(&mut [&mut serial]) {
          println!("Fail poll");
          continue;
      }

      let mut buf = [0u8; 64];

      match serial.read(&mut buf[..]) {
          Ok(count) => {
            println!("read {}", count);
            // count bytes were read to &buf[..count]
          },
          Err(UsbError::WouldBlock) => {
            println!("read would block");
            // No data received
          },
          Err(err) => {
            println!("read error");
            // An error occurred
          },
      };

      match serial.write(b"hello\n") {
          Ok(count) => {
            println!("wrote {}", count);
            // count bytes were written
          },
          Err(UsbError::WouldBlock) => {
            println!("write would block");
            // No data could be written (buffers full)
          },
          Err(err) => {
            println!("write error");
            // An error occurred
          },
      };
  }
}
