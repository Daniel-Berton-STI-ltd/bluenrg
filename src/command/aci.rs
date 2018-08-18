//! Vendor-specific HCI commands and types needed for those commands.

extern crate bluetooth_hci as hci;
extern crate byteorder;
extern crate embedded_hal as hal;
extern crate nb;

use byteorder::{ByteOrder, LittleEndian};

/// Vendor-specific HCI commands for the [ActiveBlueNRG](::ActiveBlueNRG).
pub trait Commands {
    /// Type of communication errors.
    type Error;

    /// This command writes a value to a low level configure data structure. It is useful to setup
    /// directly some low level parameters for the system in the runtime.
    ///
    /// # Errors
    ///
    /// Only underlying communication errors are reported.
    ///
    /// # Generated events
    ///
    /// The controller will generate a [command
    /// complete](::event::command::ReturnParameters::AciWriteConfigData) event.
    fn write_config_data(&mut self, config: &ConfigData) -> nb::Result<(), Self::Error>;

    /// This command requests the value in the low level configure data structure.
    ///
    /// # Errors
    ///
    /// Only underlying communication errors are reported.
    ///
    /// # Generated events
    ///
    /// The controller will generate a [command
    /// complete](::event::command::ReturnParameters::AciReadConfigData) event.
    fn read_config_data(&mut self, param: ConfigParameter) -> nb::Result<(), Self::Error>;

    /// This command sets the TX power level of the BlueNRG-MS.
    ///
    /// When the system starts up or reboots, the default TX power level will be used, which is the
    /// maximum value of [8 dBm](PowerLevel::Dbm8_0). Once this command is given, the output power
    /// will be changed instantly, regardless if there is Bluetooth communication going on or
    /// not. For example, for debugging purpose, the BlueNRG-MS can be set to advertise all the
    /// time. And use this command to observe the signal strength changing.
    ///
    /// The system will keep the last received TX power level from the command, i.e. the 2nd
    /// command overwrites the previous TX power level. The new TX power level remains until
    /// another Set TX Power command, or the system reboots.
    ///
    /// # Errors
    ///
    /// Only underlying communication errors are reported.
    ///
    /// # Generated events
    ///
    /// The controller will generate a [command
    /// complete](::event::command::ReturnParameters::AciSetTxPowerLevel) event.
    fn set_tx_power_level(&mut self, level: PowerLevel) -> nb::Result<(), Self::Error>;

    /// Puts the device in standby mode.
    ///
    /// Normally the BlueNRG-MS will automatically enter sleep mode to save power. This command
    /// further put the device into the Standby mode instead of the sleep mode. The difference is
    /// that, in sleep mode, the device can still wake up itself with the internal timer. But in
    /// standby mode, this timer is also disabled. So the only possibility to wake up the device is
    /// by the external signals, e.g. a HCI command sent via SPI bus.
    ///
    /// Based on the measurement, the current consumption under sleep mode is ~2 uA. And this value
    /// is ~1.5 uA in standby mode.
    ///
    /// # Errors
    ///
    /// Only underlying communication errors are reported.
    ///
    /// # Generated events
    ///
    /// The controller will generate a [command
    /// complete](::event::command::ReturnParameters::AciDeviceStandby) event.
    ///
    /// The command is only accepted when there is no other Bluetooth activity. Otherwise an error
    /// code [command disallowed](hci::Status::CommandDisallowed) will return.
    fn device_standby(&mut self) -> nb::Result<(), Self::Error>;
}

impl<'spi, 'dbuf, SPI, OutputPin1, OutputPin2, InputPin, E> Commands
    for ::ActiveBlueNRG<'spi, 'dbuf, SPI, OutputPin1, OutputPin2, InputPin>
where
    SPI: hal::blocking::spi::Transfer<u8, Error = E> + hal::blocking::spi::Write<u8, Error = E>,
    OutputPin1: hal::digital::OutputPin,
    OutputPin2: hal::digital::OutputPin,
    InputPin: hal::digital::InputPin,
{
    type Error = E;

    impl_variable_length_params!(
        write_config_data,
        ConfigData,
        ::opcode::ACI_WRITE_CONFIG_DATA
    );

    fn read_config_data(&mut self, param: ConfigParameter) -> nb::Result<(), Self::Error> {
        self.write_command(::opcode::ACI_READ_CONFIG_DATA, &[param as u8])
    }

    fn set_tx_power_level(&mut self, level: PowerLevel) -> nb::Result<(), Self::Error> {
        let mut bytes = [0; 2];
        LittleEndian::write_u16(&mut bytes, level as u16);

        self.write_command(::opcode::ACI_SET_TX_POWER_LEVEL, &bytes)
    }

    fn device_standby(&mut self) -> nb::Result<(), Self::Error> {
        self.write_command(::opcode::ACI_DEVICE_STANDBY, &[])
    }
}

/// Potential errors from parameter validation.
///
/// Before some commands are sent to the controller, the parameters are validated. This type
/// enumerates the potential validation errors. Must be specialized on the types of communication
/// errors.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum Error<E> {
    /// Underlying communication error.
    Comm(E),
}

/*
fn rewrap_error<E>(e: nb::Error<E>) -> nb::Error<Error<E>> {
    match e {
        nb::Error::WouldBlock => nb::Error::WouldBlock,
        nb::Error::Other(c) => nb::Error::Other(Error::Comm(c)),
    }
}
*/

/// Low-level configuration parameters for the controller.
pub struct ConfigData {
    offset: u8,
    length: u8,
    value_buf: [u8; ConfigData::MAX_LENGTH],
}

impl ConfigData {
    /// Maximum length needed to serialize the data.
    pub const MAX_LENGTH: usize = 0x2E;

    /// Serializes the data into the given buffer.
    ///
    /// Returns the number of valid bytes in the buffer.
    ///
    /// # Panics
    ///
    /// The buffer must be large enough to support the serialized data (at least
    /// [`MAX_LENGTH`](ConfigData::MAX_LENGTH) bytes).
    pub fn into_bytes(&self, bytes: &mut [u8]) -> usize {
        bytes[0] = self.offset;
        bytes[1] = self.length;

        let len = self.length as usize;
        bytes[2..2 + len].copy_from_slice(&self.value_buf[..len]);

        2 + len
    }

    /// Builder for [ConfigData].
    ///
    /// The controller allows us to write any _contiguous_ portion of the [ConfigData] structure in
    /// [`write_config_data`](Commands::write_config_data).  The builder associated functions allow
    /// us to start with any field, and the returned builder allows only either chaining the next
    /// field or building the structure to write.
    pub fn public_address(addr: hci::BdAddr) -> ConfigDataDiversifierBuilder {
        let mut data = ConfigData {
            offset: 0,
            length: 6,
            value_buf: [0; Self::MAX_LENGTH],
        };

        data.value_buf[0..6].copy_from_slice(&addr.0);

        ConfigDataDiversifierBuilder { data }
    }

    /// Builder for [ConfigData].
    ///
    /// The controller allows us to write any _contiguous_ portion of the [ConfigData] structure in
    /// [`write_config_data`](Commands::write_config_data).  The builder associated functions allow
    /// us to start with any field, and the returned builder allows only either chaining the next
    /// field or building the structure to write.
    pub fn diversifier(d: u16) -> ConfigDataEncryptionRootBuilder {
        let mut data = ConfigData {
            offset: 6,
            length: 2,
            value_buf: [0; Self::MAX_LENGTH],
        };
        LittleEndian::write_u16(&mut data.value_buf[0..2], d);

        ConfigDataEncryptionRootBuilder { data }
    }

    /// Builder for [ConfigData].
    ///
    /// The controller allows us to write any _contiguous_ portion of the [ConfigData] structure in
    /// [`write_config_data`](Commands::write_config_data).  The builder associated functions allow
    /// us to start with any field, and the returned builder allows only either chaining the next
    /// field or building the structure to write.
    pub fn encryption_root(key: hci::host::EncryptionKey) -> ConfigDataIdentityRootBuilder {
        let mut data = ConfigData {
            offset: 8,
            length: 16,
            value_buf: [0; Self::MAX_LENGTH],
        };
        data.value_buf[0..16].copy_from_slice(&key.0);

        ConfigDataIdentityRootBuilder { data }
    }

    /// Builder for [ConfigData].
    ///
    /// The controller allows us to write any _contiguous_ portion of the [ConfigData] structure in
    /// [`write_config_data`](Commands::write_config_data).  The builder associated functions allow
    /// us to start with any field, and the returned builder allows only either chaining the next
    /// field or building the structure to write.
    pub fn identity_root(key: hci::host::EncryptionKey) -> ConfigDataLinkLayerOnlyBuilder {
        let mut data = ConfigData {
            offset: 24,
            length: 16,
            value_buf: [0; Self::MAX_LENGTH],
        };
        data.value_buf[0..16].copy_from_slice(&key.0);
        ConfigDataLinkLayerOnlyBuilder { data }
    }

    /// Builder for [ConfigData].
    ///
    /// The controller allows us to write any _contiguous_ portion of the [ConfigData] structure in
    /// [`write_config_data`](Commands::write_config_data).  The builder associated functions allow
    /// us to start with any field, and the returned builder allows only either chaining the next
    /// field or building the structure to write.
    pub fn link_layer_only(ll_only: bool) -> ConfigDataRoleBuilder {
        let mut data = ConfigData {
            offset: 40,
            length: 1,
            value_buf: [0; Self::MAX_LENGTH],
        };
        data.value_buf[0] = ll_only as u8;
        ConfigDataRoleBuilder { data }
    }

    /// Builder for [ConfigData].
    ///
    /// The controller allows us to write any _contiguous_ portion of the [ConfigData] structure in
    /// [`write_config_data`](Commands::write_config_data).  The builder associated functions allow
    /// us to start with any field, and the returned builder allows only either chaining the next
    /// field or building the structure to write.
    pub fn role(role: Role) -> ConfigDataCompleteBuilder {
        let mut data = ConfigData {
            offset: 41,
            length: 1,
            value_buf: [0; Self::MAX_LENGTH],
        };
        data.value_buf[0] = role as u8;
        ConfigDataCompleteBuilder { data }
    }
}

/// Builder for [ConfigData].
pub struct ConfigDataDiversifierBuilder {
    data: ConfigData,
}

impl ConfigDataDiversifierBuilder {
    /// Specify the diversifier and continue building.
    pub fn diversifier(mut self, d: u16) -> ConfigDataEncryptionRootBuilder {
        let len = self.data.length as usize;
        LittleEndian::write_u16(&mut self.data.value_buf[len..2 + len], d);
        self.data.length += 2;

        ConfigDataEncryptionRootBuilder { data: self.data }
    }

    /// Build the [ConfigData] as-is. It includes only the public address.
    pub fn build(self) -> ConfigData {
        self.data
    }
}

/// Builder for [ConfigData].
pub struct ConfigDataEncryptionRootBuilder {
    data: ConfigData,
}

impl ConfigDataEncryptionRootBuilder {
    /// Specify the encryption root and continue building.
    pub fn encryption_root(
        mut self,
        key: hci::host::EncryptionKey,
    ) -> ConfigDataIdentityRootBuilder {
        let len = self.data.length as usize;
        self.data.value_buf[len..16 + len].copy_from_slice(&key.0);
        self.data.length += 16;

        ConfigDataIdentityRootBuilder { data: self.data }
    }

    /// Build the [ConfigData] as-is. It includes the diversifier, and may include fields before it,
    /// but does not include any fields after it (including the encryption root).
    pub fn build(self) -> ConfigData {
        self.data
    }
}

/// Builder for [ConfigData].
pub struct ConfigDataIdentityRootBuilder {
    data: ConfigData,
}

impl ConfigDataIdentityRootBuilder {
    /// Specify the identity root and continue building.
    pub fn identity_root(
        mut self,
        key: hci::host::EncryptionKey,
    ) -> ConfigDataLinkLayerOnlyBuilder {
        let len = self.data.length as usize;
        self.data.value_buf[len..16 + len].copy_from_slice(&key.0);
        self.data.length += 16;

        ConfigDataLinkLayerOnlyBuilder { data: self.data }
    }

    /// Build the [ConfigData] as-is. It includes the encryption root, and may include fields before
    /// it, but does not include any fields after it (including the identity root).
    pub fn build(self) -> ConfigData {
        self.data
    }
}

/// Builder for [ConfigData].
pub struct ConfigDataLinkLayerOnlyBuilder {
    data: ConfigData,
}

impl ConfigDataLinkLayerOnlyBuilder {
    /// Specify whether to use the link layer only and continue building.
    pub fn link_layer_only(mut self, ll_only: bool) -> ConfigDataRoleBuilder {
        self.data.value_buf[self.data.length as usize] = ll_only as u8;
        self.data.length += 1;
        ConfigDataRoleBuilder { data: self.data }
    }

    /// Build the [ConfigData] as-is. It includes the identity root, and may include fields before
    /// it, but does not include any fields after it (including the link layer only flag).
    pub fn build(self) -> ConfigData {
        self.data
    }
}

/// Builder for [ConfigData].
pub struct ConfigDataRoleBuilder {
    data: ConfigData,
}

impl ConfigDataRoleBuilder {
    /// Specify the device role and continue building.
    pub fn role(mut self, role: Role) -> ConfigDataCompleteBuilder {
        self.data.value_buf[self.data.length as usize] = role as u8;
        self.data.length += 1;
        ConfigDataCompleteBuilder { data: self.data }
    }

    /// Build the [ConfigData] as-is. It includes the link layer only flag, and may include fields
    /// before it, but does not include any fields after it (including the role).
    pub fn build(self) -> ConfigData {
        self.data
    }
}

/// Builder for [ConfigData].
pub struct ConfigDataCompleteBuilder {
    data: ConfigData,
}

impl ConfigDataCompleteBuilder {
    /// Build the [ConfigData] as-is. It includes the role field, and may include fields before it.
    pub fn build(self) -> ConfigData {
        self.data
    }
}

/// Roles that the server can adopt.
#[repr(u8)]
pub enum Role {
    /// Peripheral and primary device.
    /// - Only one connection.
    /// - 6 KB of RAM retention.
    Peripheral6Kb = 1,

    /// Peripheral and primary device.
    /// - Only one connection.
    /// - 12 KB of RAM retention.
    Peripheral12Kb = 2,

    /// Primary device and peripheral
    /// - Up to 8 connections
    /// - 12 KB of RAM retention
    Primary12Kb = 3,

    /// Primary device and peripheral.
    /// - Simultaneous advertising and scanning
    /// - Up to 4 connections
    /// - This mode is available starting from BlueNRG-MS FW stack version 7.1.b
    SimultaneousAdvertisingScanning = 4,
}

/// Configuration parameters that are readable by the
/// [`read_config_data`](Commands::read_config_data) command.
#[repr(u8)]
pub enum ConfigParameter {
    /// Bluetooth public address.
    PublicAddress = 0,

    /// Diversifier used to derive CSRK (connection signature resolving key).
    Diversifier = 6,

    /// Encryption root key used to derive the LTK (long-term key) and CSRK (connection signature
    /// resolving key).
    EncryptionRoot = 8,

    /// Identity root key used to derive the LTK (long-term key) and CSRK (connection signature
    /// resolving key).
    IdentityRoot = 24,

    /// Switch on/off Link Layer only mode.
    LinkLayerOnly = 40,

    /// BlueNRG-MS roles and mode configuration.
    Role = 41,
}

/// Transmitter power levels available for the system.
///
/// The controller uses two parameters to determine the actual power level: enable high power, and
/// PA level. This enum combines the two parameters. The high byte is the PA level; the low byte is
/// the enable high power flag.
#[repr(u16)]
pub enum PowerLevel {
    /// PA level 0, low power.
    DbmNeg18 = 0x000,
    /// PA level 0, high power.
    DbmNeg15 = 0x001,
    /// PA level 1, low power.
    DbmNeg14_7 = 0x100,
    /// PA level 1, high power.
    DbmNeg11_7 = 0x101,
    /// PA level 2, low power.
    DbmNeg11_4 = 0x200,
    /// PA level 2, high power.
    DbmNeg8_4 = 0x201,
    /// PA level 3, low power.
    DbmNeg8_1 = 0x300,
    /// PA level 3, high power.
    DbmNeg5_1 = 0x301,
    /// PA level 4, low power.
    DbmNeg4_9 = 0x400,
    /// PA level 4, high power.
    DbmNeg2_1 = 0x401,
    /// PA level 5, low power.
    DbmNeg1_6 = 0x500,
    /// PA level 5, high power.
    Dbm1_4 = 0x501,
    /// PA level 6, low power.
    Dbm1_7 = 0x600,
    /// PA level 6, high power.
    Dbm4_7 = 0x601,
    /// PA level 7, low power.
    Dbm5_0 = 0x700,
    /// PA level 7, high power.
    Dbm8_0 = 0x701,
}
