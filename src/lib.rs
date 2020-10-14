//! Bluetooth HCI for STMicro's BlueNRG-MS Bluetooth controllers.
//!
//! *Note*: This crate does not provide support for the BlueNRG-1 or BlueNRG-2 SoCs.
//!
//! # Design
//!
//! The BlueNRG-MS is an external Bluetooth Radio Controller that communicates with the application
//! processor over HCI
//!
//! This crate defines a public struct, [`BlueNRG`] that owns the chip select and data ready
//! pins, and a receive buffer for the data that comes from the controller. It also defines a
//! private struct, [`ActiveBlueNRG`] that borrows a handle to the writer interface. `ActiveBlueNRG`
//! implements [`bluetooth_hci::Controller`], which provides access to the full Bluetooth HCI.
//!
//! BlueNRG-MS implements parts of version 4.1 of the Bluetooth [specification].
//!
//! The fundamental way to use the [`BlueNRG`] is its [`with_writer`](BlueNRG::with_writer) function,
//! which invokes its closure on at [`ActiveBlueNRG`], so sending HCI commands and reading HCI
//! events can only be done from within that closure.
//!
//! # Vendor-Specific Commands
//!
//! BlueNRG-MS provides several vendor-specific commands that control the behavior of the
//! controller.
//!
//! # Vendor-Specific Events
//!
//! BlueNRG-MS provides several vendor-specific events that provide data related to the
//! controller. Many of these events are forwarded from the link layer, and these are documented
//! with a reference to the appropriate section of the Bluetooth specification.
//!
//! # Example
//!
//! TODO
//!
//! [specification]: https://www.bluetooth.com/specifications/bluetooth-core-specification

#![deny(missing_docs)]

#[macro_use]
extern crate bitflags;
#[macro_use]
extern crate bluetooth_hci as hci;
extern crate byteorder;
extern crate nb;

use core::convert::TryFrom;
use hci::host::HciHeader;
use hci::Controller;

mod cb;
mod command;
pub mod event;
mod opcode;

pub use command::gap;
pub use command::gatt;
pub use command::hal;
pub use command::l2cap;

pub use hci::host::{AdvertisingFilterPolicy, AdvertisingType, OwnAddressType};

/// Enumeration of potential errors that may occur when reading from or writing to the chip.
#[derive(Debug, PartialEq)]
pub enum Error {
    /// error in HCI communications 
    Comms,
}

/// Handle for interfacing with the BlueNRG-MS.
pub struct BlueNRG<'buf> {
    /// Buffer used to hold bytes read from the controller until the application can process them.
    /// Should be at least 257 bytes (to hold a header and maximum BLE payload of 255 bytes).
    rx_buffer: cb::Buffer<'buf, u8>,
}
/// Closure that does the packet writing.
pub type Writer = dyn Fn(&[u8], &[u8]) -> nb::Result<(), Error>;
/// Handle for actively communicating with the controller.
///
/// An `ActiveBlueNRG` should not be created by the application, but is passed to closures given to
/// [`BlueNRG::with_writer`].  `ActiveBlueNRG` implements [`bluetooth_hci::Controller`], so it is used
/// to access the HCI functions for the controller.
pub struct ActiveBlueNRG<'bnrg, 'dbuf, 'wrt> {
    /// Mutably borrow the BlueNRG handle so we can access pin and buffer.
    d: &'bnrg mut BlueNRG<'dbuf>,
    write_pkt: &'wrt (dyn Fn(&[u8], &[u8]) -> nb::Result<(), Error> + 'wrt),
}

/// useful writer for debugging raw packet data
pub fn write_to_stdout(header: &[u8], payload: &[u8]) -> nb::Result<(), Error> {
    println!("hdr:{:x?}\ndata:{:x?}\n", header, payload);
    Ok(())
}

impl<'bnrg, 'dbuf, 'wrt> ActiveBlueNRG<'bnrg, 'dbuf, 'wrt> {
    fn read_available_data(&mut self) -> nb::Result<(), Error> {
        let rx = self
            .d
            .rx_buffer
            .next_mut_slice(self.d.rx_buffer.next_contiguous_slice_len());
        for byte in rx.iter_mut() {
            *byte = 0;
        }

        Ok(())
    }

    fn write_command(&mut self, opcode: opcode::Opcode, params: &[u8]) -> nb::Result<(), Error> {
        const HEADER_LEN: usize = 4;
        let mut header = [0; HEADER_LEN];
        hci::host::uart::CommandHeader::new(opcode, params.len()).copy_into_slice(&mut header);
        self.write(&header, &params)
    }
}

impl<'bnrg, 'dbuf, 'wrt> hci::Controller for ActiveBlueNRG<'bnrg, 'dbuf, 'wrt> {
    type Error = Error;
    type Header = hci::host::uart::CommandHeader;
    type Vendor = BlueNRGTypes;

    fn write(&mut self, header: &[u8], payload: &[u8]) -> nb::Result<(), Self::Error> {
        //(self.write_pkt)(&header, &payload)
        (self.write_pkt)(&header, &payload)
    }

    fn read_into(&mut self, buffer: &mut [u8]) -> nb::Result<(), Self::Error> {
        let result = if buffer.len() > self.d.rx_buffer.size() {
            let r = self.read_available_data();
            r
        } else {
            Ok(())
        };

        if buffer.len() <= self.d.rx_buffer.size() {
            self.d.rx_buffer.take_slice(buffer.len(), buffer);
            Ok(())
        } else if let Err(e) = result {
            Err(e)
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    fn peek(&mut self, n: usize) -> nb::Result<u8, Self::Error> {
        if n < self.d.rx_buffer.size() {
            Ok(self.d.rx_buffer.peek(n))
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

/// Specify vendor-specific extensions for the BlueNRG.
pub struct BlueNRGTypes;
impl hci::Vendor for BlueNRGTypes {
    type Status = event::Status;
    type Event = event::BlueNRGEvent;
}

/// Master trait that encompasses all commands, and communicates over UART.
pub trait UartController<E>:
    crate::gap::Commands<Error = E>
    + crate::gatt::Commands<Error = E>
    + crate::hal::Commands<Error = E>
    + crate::l2cap::Commands<Error = E>
    + bluetooth_hci::host::uart::Hci<E, crate::event::BlueNRGEvent, crate::event::BlueNRGError>
{
}
impl<T, E> UartController<E> for T where
    T: crate::gap::Commands<Error = E>
        + crate::gatt::Commands<Error = E>
        + crate::hal::Commands<Error = E>
        + crate::l2cap::Commands<Error = E>
        + bluetooth_hci::host::uart::Hci<E, crate::event::BlueNRGEvent, crate::event::BlueNRGError>
{
}

impl<'buf> BlueNRG<'buf> {
    /// Returns a new BlueNRG struct with the given RX Buffer
    pub fn new(rx_buffer: &'buf mut [u8]) -> BlueNRG<'buf> {
        BlueNRG {
            rx_buffer: cb::Buffer::new(rx_buffer),
        }
    }

    /// Invokes the given body function with an ActiveBlueNRG that uses this BlueNRG struct and the
    /// provided writer function.
    ///
    /// Returns the result of the invoked body.
    pub fn with_writer<T, F, W>(&mut self, writer: W, body: F) -> T
    where
        F: FnOnce(&mut ActiveBlueNRG) -> T,
        W: Fn(&[u8], &[u8]) -> nb::Result<(), Error>, // Writer type alias
    {
        let mut active = ActiveBlueNRG {
            d: self,
            write_pkt: &writer,
        };
        body(&mut active)
    }

    /// Resets the BlueNRG Controller. Uses the given timer to delay 1 cycle at `freq` Hz after
    /// toggling the reset pin.
    pub fn reset(&mut self) -> Result<(), Error> {
        //self.reset.set_low().map_err(nb::Error::Other)?;
        // TODO:add delay
        //self.reset.set_high().map_err(nb::Error::Other)?;
        // TODO:add delay

        Ok(())
    }

}

/// Vendor-specific interpretation of the local version information from the controller.
#[derive(Clone)]
pub struct Version {
    /// Version of the controller hardware.
    pub hw_version: u8,

    /// Major version of the controller firmware
    pub major: u8,

    /// Minor version of the controller firmware
    pub minor: u8,

    /// Patch version of the controller firmware
    pub patch: u8,
}

/// Extension trait to convert [`hci::event::command::LocalVersionInfo`] into the BlueNRG-specific
/// [`Version`] struct.
pub trait LocalVersionInfoExt {
    /// Converts LocalVersionInfo as returned by the controller into a BlueNRG-specific [`Version`]
    /// struct.
    fn bluenrg_version(&self) -> Version;
}

impl<VS> LocalVersionInfoExt for hci::event::command::LocalVersionInfo<VS> {
    fn bluenrg_version(&self) -> Version {
        Version {
            hw_version: (self.hci_revision >> 8) as u8,
            major: (self.hci_revision & 0xFF) as u8,
            minor: ((self.lmp_subversion >> 4) & 0xF) as u8,
            patch: (self.lmp_subversion & 0xF) as u8,
        }
    }
}

/// Hardware event codes returned by the `HardwareError` HCI event.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum HardwareError {
    /// Error on the SPI bus has been detected, most likely caused by incorrect SPI configuration on
    /// the external micro-controller.
    SpiFraming,

    /// Caused by a slow crystal startup and they are an indication that the HS_STARTUP_TIME in the
    /// device configuration needs to be tuned. After this event is recommended to hardware reset
    /// the device.
    RadioState,

    /// Caused by a slow crystal startup and they are an indication that the HS_STARTUP_TIME in the
    /// device configuration needs to be tuned. After this event is recommended to hardware reset
    /// the device.
    TimerOverrun,
}

/// Error type for `TryFrom<u8>` to `HardwareError`. Includes the invalid byte.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct InvalidHardwareError(pub u8);

impl TryFrom<u8> for HardwareError {
    type Error = InvalidHardwareError;
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(HardwareError::SpiFraming),
            1 => Ok(HardwareError::RadioState),
            2 => Ok(HardwareError::TimerOverrun),
            _ => Err(InvalidHardwareError(value)),
        }
    }
}
