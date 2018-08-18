//! Return parameters for vendor-specific commands.
//!
//! This module defines the parameters returned in the Command Complete event for vendor-specific
//! commands.  These commands are defined for the BlueNRG controller, but are not standard HCI
//! commands.

extern crate bluetooth_hci as hci;

use byteorder::{ByteOrder, LittleEndian};
use core::convert::{TryFrom, TryInto};
use core::fmt::{Debug, Formatter, Result as FmtResult};

/// Vendor-specific commands that may generate the [Command
/// Complete](hci::event::command::ReturnParameters::Vendor) event. If the commands have defined
/// return parameters, they are included in the enum.
#[derive(Clone, Debug)]
pub enum ReturnParameters {
    /// Status returned by the [ACI Write Config Data](::aci::Commands::write_config_data) command.
    AciWriteConfigData(hci::Status),

    /// Parameters returned by the [ACI Read Config Data](::aci::Commands::read_config_data)
    /// command.
    AciReadConfigData(AciConfigData),

    /// Status returned by the [ACI Set Tx Power Level](::aci::Commands::set_tx_power_level)
    /// command.
    AciSetTxPowerLevel(hci::Status),

    /// Status returned by the [GAP Set Non-Discoverable](::gap::Commands::set_nondiscoverable)
    /// command.
    GapSetNonDiscoverable(hci::Status),

    /// Status returned by the [GAP Set Discoverable](::gap::Commands::set_discoverable)
    /// command.
    GapSetDiscoverable(hci::Status),

    /// Status returned by the [GAP Set Direct
    /// Connectable](::gap::Commands::set_direct_connectable) command.
    GapSetDirectConnectable(hci::Status),

    /// Status returned by the [GAP Set IO Capability](::gap::Commands::set_io_capability)
    /// command.
    GapSetIoCapability(hci::Status),

    /// Status returned by the [GAP Set Authentication
    /// Requirement](::gap::Commands::set_authentication_requirement) command.
    GapSetAuthenticationRequirement(hci::Status),

    /// Status returned by the [GAP Set Authorization
    /// Requirement](::gap::Commands::set_authorization_requirement) command.
    GapSetAuthorizationRequirement(hci::Status),

    /// Status returned by the [GAP Pass Key Response](::gap::Commands::pass_key_response)
    /// command.
    GapPassKeyResponse(hci::Status),

    /// Status returned by the [GAP Authorization
    /// Response](::gap::Commands::authorization_response) command.
    GapAuthorizationResponse(hci::Status),

    /// Parameters returned by the [GAP Init](::gap::Commands::init) command.
    GapInit(GapInit),

    /// Parameters returned by the [GAP Set
    /// Non-Connectable](::gap::Commands::set_nonconnectable) command.
    GapSetNonConnectable(hci::Status),

    /// Parameters returned by the [GAP Set
    /// Undirected Connectable](::gap::Commands::set_undirected_connectable) command.
    GapSetUndirectedConnectable(hci::Status),

    /// Parameters returned by the [GAP Update Advertising
    /// Data](::gap::Commands::update_advertising_data) command.
    GapUpdateAdvertisingData(hci::Status),

    /// Parameters returned by the [GAP Delete AD Type](::gap::Commands::delete_ad_type)
    /// command.
    GapDeleteAdType(hci::Status),

    /// Parameters returned by the [GAP Get Security Level](::gap::Commands::get_security_level)
    /// command.
    GapGetSecurityLevel(GapSecurityLevel),

    /// Parameters returned by the [GAP Set Event Mask](::gap::Commands::set_event_mask)
    /// command.
    GapSetEventMask(hci::Status),

    /// Parameters returned by the [GAP Configure
    /// White List](::gap::Commands::configure_white_list) command.
    GapConfigureWhiteList(hci::Status),

    /// Parameters returned by the [GAP Clear Security
    /// Database](::gap::Commands::clear_security_database) command.
    GapClearSecurityDatabase(hci::Status),

    /// Parameters returned by the [GAP Allow Rebond](::gap::Commands::allow_rebond) command.
    GapAllowRebond(hci::Status),

    /// Parameters returned by the [GAP Terminate
    /// Procedure](::gap::Commands::terminate_procedure) command.
    GapTerminateProcedure(hci::Status),

    #[cfg(not(feature = "ms"))]
    /// Parameters returned by the [GAP Resolve Private
    /// Address](::gap::Commands::resolve_private_address) command.
    GapResolvePrivateAddress(hci::Status),

    #[cfg(feature = "ms")]
    /// Parameters returned by the [GAP Resolve Private
    /// Address](::gap::Commands::resolve_private_address) command.
    GapResolvePrivateAddress(GapResolvePrivateAddress),

    /// Parameters returned by the [GAP Get Bonded Devices](::gap::Commands::get_bonded_devices)
    /// command.
    GapGetBondedDevices(GapBondedDevices),

    #[cfg(feature = "ms")]
    /// Parameters returned by the [GAP Set Broadcast Mode](::gap::Commands::set_broadcast_mode)
    /// command.
    GapSetBroadcastMode(hci::Status),

    #[cfg(feature = "ms")]
    /// Parameters returned by the [GAP Start Observation
    /// Procedure](::gap::Commands::start_observation_procedure) command.
    GapStartObservationProcedure(hci::Status),

    /// Parameters returned by the [GAP Is Device Bonded](::gap::Commands::is_device_bonded)
    /// command.
    GapIsDeviceBonded(hci::Status),

    /// Parameters returned by the [GATT Init](::gatt::Commands::init) command.
    GattInit(hci::Status),

    /// Parameters returned by the [GATT Add Service](::gatt::Commands::add_service) command.
    GattAddService(GattService),

    /// Parameters returned by the [GATT Include Service](::gatt::Commands::include_service)
    /// command.
    GattIncludeService(GattService),

    /// Parameters returned by the [GATT Add Characteristic](::gatt::Commands::add_characteristic)
    /// command.
    GattAddCharacteristic(GattCharacteristic),

    /// Parameters returned by the [GATT Add Characteristic
    /// Descriptor](::gatt::Commands::add_characteristic_descriptor) command.
    GattAddCharacteristicDescriptor(GattCharacteristicDescriptor),

    /// Parameters returned by the [GATT Update Characteristic
    /// Value](::gatt::Commands::update_characteristic_value) command.
    GattUpdateCharacteristicValue(hci::Status),

    /// Parameters returned by the [GATT Delete
    /// Characteristic](::gatt::Commands::delete_characteristic) command.
    GattDeleteCharacteristic(hci::Status),

    /// Parameters returned by the [GATT Delete Service](::gatt::Commands::delete_service) command.
    GattDeleteService(hci::Status),

    /// Parameters returned by the [GATT Delete Included
    /// Service](::gatt::Commands::delete_included_service) command.
    GattDeleteIncludedService(hci::Status),

    /// Parameters returned by the [GATT Set Event Mask](::gatt::Commands::set_event_mask) command.
    GattSetEventMask(hci::Status),

    /// Parameters returned by the [GATT Write Without
    /// Response](::gatt::Commands::write_without_response) command.
    GattWriteWithoutResponse(hci::Status),

    /// Parameters returned by the [GATT Signed Write Without
    /// Response](::gatt::Commands::signed_write_without_response) command.
    GattSignedWriteWithoutResponse(hci::Status),

    /// Parameters returned by the [GATT Confirm Indication](::gatt::Commands::confirm_indication)
    /// command.
    GattConfirmIndication(hci::Status),

    /// Parameters returned by the [GATT Write Response](::gatt::Commands::write_response) command.
    GattWriteResponse(hci::Status),

    /// Parameters returned by the [GATT Allow Read](::gatt::Commands::allow_read) command.
    GattAllowRead(hci::Status),

    /// Parameters returned by the [GATT Set Security
    /// Permission](::gatt::Commands::set_security_permission) command.
    GattSetSecurityPermission(hci::Status),

    /// Parameters returned by the [GATT Set Descriptor
    /// Value](::gatt::Commands::set_descriptor_value) command.
    GattSetDescriptorValue(hci::Status),

    /// Parameters returned by the [GATT Read Handle Value](::gatt::Commands::read_handle_value)
    /// command.
    GattReadHandleValue(GattHandleValue),

    /// Parameters returned by the [GATT Read Handle
    /// Value](::gatt::Commands::read_handle_value_offset) command.
    #[cfg(feature = "ms")]
    GattReadHandleValueOffset(GattHandleValue),

    /// Parameters returned by the [GATT Update Long Characteristic
    /// Value](::gatt::Commands::update_long_characteristic_value) command.
    #[cfg(feature = "ms")]
    GattUpdateLongCharacteristicValue(hci::Status),

    /// Status returned by the [L2CAP Connection Parameter Update
    /// Response](::l2cap::Commands::connection_parameter_update_response) command.
    L2CapConnectionParameterUpdateResponse(hci::Status),
}

impl hci::event::VendorReturnParameters for ReturnParameters {
    type Error = super::BlueNRGError;

    fn new(bytes: &[u8]) -> Result<Self, hci::event::Error<Self::Error>> {
        check_len_at_least(bytes, 3)?;

        match hci::Opcode(LittleEndian::read_u16(&bytes[1..])) {
            ::opcode::ACI_WRITE_CONFIG_DATA => Ok(ReturnParameters::AciWriteConfigData(to_status(
                &bytes[3..],
            )?)),
            ::opcode::ACI_READ_CONFIG_DATA => Ok(ReturnParameters::AciReadConfigData(
                to_aci_config_data(&bytes[3..])?,
            )),
            ::opcode::ACI_SET_TX_POWER_LEVEL => Ok(ReturnParameters::AciSetTxPowerLevel(
                to_status(&bytes[3..])?,
            )),
            ::opcode::GAP_SET_NONDISCOVERABLE => Ok(ReturnParameters::GapSetNonDiscoverable(
                to_status(&bytes[3..])?,
            )),
            ::opcode::GAP_SET_DISCOVERABLE => Ok(ReturnParameters::GapSetDiscoverable(to_status(
                &bytes[3..],
            )?)),
            ::opcode::GAP_SET_DIRECT_CONNECTABLE => Ok(ReturnParameters::GapSetDirectConnectable(
                to_status(&bytes[3..])?,
            )),
            ::opcode::GAP_SET_IO_CAPABILITY => Ok(ReturnParameters::GapSetIoCapability(to_status(
                &bytes[3..],
            )?)),
            ::opcode::GAP_SET_AUTHENTICATION_REQUIREMENT => Ok(
                ReturnParameters::GapSetAuthenticationRequirement(to_status(&bytes[3..])?),
            ),
            ::opcode::GAP_SET_AUTHORIZATION_REQUIREMENT => Ok(
                ReturnParameters::GapSetAuthorizationRequirement(to_status(&bytes[3..])?),
            ),
            ::opcode::GAP_PASS_KEY_RESPONSE => Ok(ReturnParameters::GapPassKeyResponse(to_status(
                &bytes[3..],
            )?)),
            ::opcode::GAP_AUTHORIZATION_RESPONSE => Ok(ReturnParameters::GapAuthorizationResponse(
                to_status(&bytes[3..])?,
            )),
            ::opcode::GAP_INIT => Ok(ReturnParameters::GapInit(to_gap_init(&bytes[3..])?)),
            ::opcode::GAP_SET_NONCONNECTABLE => Ok(ReturnParameters::GapSetNonConnectable(
                to_status(&bytes[3..])?,
            )),
            ::opcode::GAP_SET_UNDIRECTED_CONNECTABLE => Ok(
                ReturnParameters::GapSetUndirectedConnectable(to_status(&bytes[3..])?),
            ),
            ::opcode::GAP_UPDATE_ADVERTISING_DATA => Ok(
                ReturnParameters::GapUpdateAdvertisingData(to_status(&bytes[3..])?),
            ),
            ::opcode::GAP_DELETE_AD_TYPE => {
                Ok(ReturnParameters::GapDeleteAdType(to_status(&bytes[3..])?))
            }
            ::opcode::GAP_GET_SECURITY_LEVEL => Ok(ReturnParameters::GapGetSecurityLevel(
                to_gap_security_level(&bytes[3..])?,
            )),
            ::opcode::GAP_SET_EVENT_MASK => {
                Ok(ReturnParameters::GapSetEventMask(to_status(&bytes[3..])?))
            }
            ::opcode::GAP_CONFIGURE_WHITE_LIST => Ok(ReturnParameters::GapConfigureWhiteList(
                to_status(&bytes[3..])?,
            )),
            ::opcode::GAP_CLEAR_SECURITY_DATABASE => Ok(
                ReturnParameters::GapClearSecurityDatabase(to_status(&bytes[3..])?),
            ),
            ::opcode::GAP_ALLOW_REBOND => {
                Ok(ReturnParameters::GapAllowRebond(to_status(&bytes[3..])?))
            }
            ::opcode::GAP_TERMINATE_PROCEDURE => Ok(ReturnParameters::GapTerminateProcedure(
                to_status(&bytes[3..])?,
            )),
            ::opcode::GAP_RESOLVE_PRIVATE_ADDRESS => {
                #[cfg(not(feature = "ms"))]
                {
                    Ok(ReturnParameters::GapResolvePrivateAddress(to_status(
                        &bytes[3..],
                    )?))
                }

                #[cfg(feature = "ms")]
                {
                    Ok(ReturnParameters::GapResolvePrivateAddress(
                        to_gap_resolve_private_address(&bytes[3..])?,
                    ))
                }
            }
            ::opcode::GAP_GET_BONDED_DEVICES => Ok(ReturnParameters::GapGetBondedDevices(
                to_gap_bonded_devices(&bytes[3..])?,
            )),
            ::opcode::GAP_SET_BROADCAST_MODE => {
                #[cfg(feature = "ms")]
                {
                    Ok(ReturnParameters::GapSetBroadcastMode(to_status(
                        &bytes[3..],
                    )?))
                }

                #[cfg(not(feature = "ms"))]
                {
                    Err(hci::event::Error::UnknownOpcode(
                        ::opcode::GAP_SET_BROADCAST_MODE,
                    ))
                }
            }
            ::opcode::GAP_START_OBSERVATION_PROCEDURE => {
                #[cfg(feature = "ms")]
                {
                    Ok(ReturnParameters::GapStartObservationProcedure(to_status(
                        &bytes[3..],
                    )?))
                }

                #[cfg(not(feature = "ms"))]
                {
                    Err(hci::event::Error::UnknownOpcode(
                        ::opcode::GAP_START_OBSERVATION_PROCEDURE,
                    ))
                }
            }
            ::opcode::GAP_IS_DEVICE_BONDED => {
                Ok(ReturnParameters::GapIsDeviceBonded(to_status(&bytes[3..])?))
            }
            ::opcode::GATT_INIT => Ok(ReturnParameters::GattInit(to_status(&bytes[3..])?)),
            ::opcode::GATT_ADD_SERVICE => Ok(ReturnParameters::GattAddService(to_gatt_service(
                &bytes[3..],
            )?)),
            ::opcode::GATT_INCLUDE_SERVICE => Ok(ReturnParameters::GattIncludeService(
                to_gatt_service(&bytes[3..])?,
            )),
            ::opcode::GATT_ADD_CHARACTERISTIC => Ok(ReturnParameters::GattAddCharacteristic(
                to_gatt_characteristic(&bytes[3..])?,
            )),
            ::opcode::GATT_ADD_CHARACTERISTIC_DESCRIPTOR => {
                Ok(ReturnParameters::GattAddCharacteristicDescriptor(
                    to_gatt_characteristic_descriptor(&bytes[3..])?,
                ))
            }
            ::opcode::GATT_UPDATE_CHARACTERISTIC_VALUE => Ok(
                ReturnParameters::GattUpdateCharacteristicValue(to_status(&bytes[3..])?),
            ),
            ::opcode::GATT_DELETE_CHARACTERISTIC => Ok(ReturnParameters::GattDeleteCharacteristic(
                to_status(&bytes[3..])?,
            )),
            ::opcode::GATT_DELETE_SERVICE => {
                Ok(ReturnParameters::GattDeleteService(to_status(&bytes[3..])?))
            }
            ::opcode::GATT_DELETE_INCLUDED_SERVICE => Ok(
                ReturnParameters::GattDeleteIncludedService(to_status(&bytes[3..])?),
            ),
            ::opcode::GATT_SET_EVENT_MASK => {
                Ok(ReturnParameters::GattSetEventMask(to_status(&bytes[3..])?))
            }
            ::opcode::GATT_WRITE_WITHOUT_RESPONSE => Ok(
                ReturnParameters::GattWriteWithoutResponse(to_status(&bytes[3..])?),
            ),
            ::opcode::GATT_SIGNED_WRITE_WITHOUT_RESPONSE => Ok(
                ReturnParameters::GattSignedWriteWithoutResponse(to_status(&bytes[3..])?),
            ),
            ::opcode::GATT_CONFIRM_INDICATION => Ok(ReturnParameters::GattConfirmIndication(
                to_status(&bytes[3..])?,
            )),
            ::opcode::GATT_WRITE_RESPONSE => {
                Ok(ReturnParameters::GattWriteResponse(to_status(&bytes[3..])?))
            }
            ::opcode::GATT_ALLOW_READ => {
                Ok(ReturnParameters::GattAllowRead(to_status(&bytes[3..])?))
            }
            ::opcode::GATT_SET_SECURITY_PERMISSION => Ok(
                ReturnParameters::GattSetSecurityPermission(to_status(&bytes[3..])?),
            ),
            ::opcode::GATT_SET_DESCRIPTOR_VALUE => Ok(ReturnParameters::GattSetDescriptorValue(
                to_status(&bytes[3..])?,
            )),
            ::opcode::GATT_READ_HANDLE_VALUE => Ok(ReturnParameters::GattReadHandleValue(
                to_gatt_handle_value(&bytes[3..])?,
            )),
            ::opcode::GATT_READ_HANDLE_VALUE_OFFSET => {
                #[cfg(feature = "ms")]
                {
                    Ok(ReturnParameters::GattReadHandleValueOffset(
                        to_gatt_handle_value(&bytes[3..])?,
                    ))
                }

                #[cfg(not(feature = "ms"))]
                {
                    Err(hci::event::Error::UnknownOpcode(
                        ::opcode::GATT_READ_HANDLE_VALUE_OFFSET,
                    ))
                }
            }
            ::opcode::GATT_UPDATE_LONG_CHARACTERISTIC_VALUE => {
                #[cfg(feature = "ms")]
                {
                    Ok(ReturnParameters::GattUpdateLongCharacteristicValue(
                        to_status(&bytes[3..])?,
                    ))
                }

                #[cfg(not(feature = "ms"))]
                {
                    Err(hci::event::Error::UnknownOpcode(
                        ::opcode::GATT_UPDATE_LONG_CHARACTERISTIC_VALUE,
                    ))
                }
            }
            ::opcode::L2CAP_CONN_PARAM_UPDATE_RESP => Ok(
                ReturnParameters::L2CapConnectionParameterUpdateResponse(to_status(&bytes[3..])?),
            ),
            other => Err(hci::event::Error::UnknownOpcode(other)),
        }
    }
}

fn check_len_at_least(
    buffer: &[u8],
    len: usize,
) -> Result<(), hci::event::Error<super::BlueNRGError>> {
    if buffer.len() < len {
        Err(hci::event::Error::BadLength(buffer.len(), len))
    } else {
        Ok(())
    }
}

fn to_status(bytes: &[u8]) -> Result<hci::Status, hci::event::Error<super::BlueNRGError>> {
    require_len_at_least!(bytes, 1);
    bytes[0].try_into().map_err(hci::event::rewrap_bad_status)
}

/// Parameters returned by the [ACI Read Config Data](::aci::Commands::read_config_data) command.
#[derive(Clone, Debug)]
pub struct AciConfigData {
    /// Did the command fail, and if so, how?
    pub status: hci::Status,

    /// Requested value.
    ///
    /// The value is requested by offset, and distinguished upon return by length only. This means
    /// that this event cannot distinguish between the 16-byte encryption keys
    /// ([EncryptionRoot](::aci::ConfigParameter::EncryptionRoot) and
    /// [IdentityRoot](::aci::ConfigParameter::IdentityRoot)) or between the single-byte values
    /// ([LinkLayerOnly](::aci::ConfigParameter::LinkLayerOnly) or
    /// [Role](::aci::ConfigParameter::Role)).
    pub value: AciConfigParameter,
}

/// Potential values that can be fetched by [ACI Read Config
/// Data](::aci::Commands::read_config_data).
#[derive(Clone, Debug, PartialEq)]
pub enum AciConfigParameter {
    /// Bluetooth public address. Corresponds to
    /// [PublicAddress](::aci::ConfigParameter::PublicAddress).
    PublicAddress(hci::BdAddr),

    /// Diversifier used to derive CSRK (connection signature resolving key).  Corresponds to
    /// [Diversifier](::aci::ConfigParameter::Diversifier).
    Diversifier(u16),

    /// A requested encryption key. Corresponds to either
    /// [EncryptionRoot](::aci::ConfigParameter::EncryptionRoot) or
    /// [IdentityRoot](::aci::ConfigParameter::IdentityRoot).
    EncryptionKey(hci::host::EncryptionKey),

    /// A single-byte value. Corresponds to either
    /// [LinkLayerOnly](::aci::ConfigParameter::LinkLayerOnly) or
    /// [Role](::aci::ConfigParameter::Role).
    Byte(u8),
}

fn to_aci_config_data(
    bytes: &[u8],
) -> Result<AciConfigData, hci::event::Error<super::BlueNRGError>> {
    require_len_at_least!(bytes, 2);
    Ok(AciConfigData {
        status: to_status(bytes)?,
        value: to_aci_config_parameter(&bytes[1..])?,
    })
}

fn to_aci_config_parameter(
    bytes: &[u8],
) -> Result<AciConfigParameter, hci::event::Error<super::BlueNRGError>> {
    match bytes.len() {
        6 => {
            let mut buf = [0; 6];
            buf.copy_from_slice(bytes);

            Ok(AciConfigParameter::PublicAddress(hci::BdAddr(buf)))
        }
        2 => Ok(AciConfigParameter::Diversifier(LittleEndian::read_u16(
            &bytes,
        ))),
        16 => {
            let mut buf = [0; 16];
            buf.copy_from_slice(bytes);

            Ok(AciConfigParameter::EncryptionKey(hci::host::EncryptionKey(
                buf,
            )))
        }
        1 => Ok(AciConfigParameter::Byte(bytes[0])),
        other => Err(hci::event::Error::Vendor(
            super::BlueNRGError::BadConfigParameterLength(other),
        )),
    }
}

/// Parameters returned by the [GAP Init](::gap::Commands::init) command.
#[derive(Copy, Clone, Debug)]
pub struct GapInit {
    /// Did the command fail, and if so, how?
    ///
    /// Should be one of:
    /// - [Success](hci::Status::Success)
    /// - [InvalidParameters](hci::Status::InvalidParameters)
    pub status: hci::Status,

    /// Handle for the GAP service
    pub service_handle: ::gatt::ServiceHandle,

    /// Handle for the device name characteristic added to the GAP service.
    pub dev_name_handle: ::gatt::CharacteristicHandle,

    /// Handle for the appearance characteristic added to the GAP service.
    pub appearance_handle: ::gatt::CharacteristicHandle,
}

fn to_gap_init(bytes: &[u8]) -> Result<GapInit, hci::event::Error<super::BlueNRGError>> {
    require_len!(bytes, 7);

    Ok(GapInit {
        status: to_status(bytes)?,
        service_handle: ::gatt::ServiceHandle(LittleEndian::read_u16(&bytes[1..])),
        dev_name_handle: ::gatt::CharacteristicHandle(LittleEndian::read_u16(&bytes[3..])),
        appearance_handle: ::gatt::CharacteristicHandle(LittleEndian::read_u16(&bytes[5..])),
    })
}

/// Parameters returned by the [GAP Get Security Level](::gap::Commands::get_security_level)
/// command.
#[derive(Copy, Clone, Debug)]
pub struct GapSecurityLevel {
    /// Did the command fail, and if so, how?
    pub status: hci::Status,

    /// Is MITM (man-in-the-middle) protection required?
    pub mitm_protection_required: bool,

    /// Is bonding required?
    pub bonding_required: bool,

    /// Is out-of-band data present?
    pub out_of_band_data_present: bool,

    /// Is a pass key required, and if so, how is it generated?
    pub pass_key_required: PassKeyRequirement,
}

/// Options for pass key generation.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum PassKeyRequirement {
    /// A pass key is not required.
    NotRequired,
    /// A fixed pin is present which is being used.
    FixedPin,
    /// Pass key required for pairing. An event will be generated when required.
    Generated,
}

impl TryFrom<u8> for PassKeyRequirement {
    type Error = super::BlueNRGError;

    fn try_from(value: u8) -> Result<PassKeyRequirement, Self::Error> {
        match value {
            0x00 => Ok(PassKeyRequirement::NotRequired),
            0x01 => Ok(PassKeyRequirement::FixedPin),
            0x02 => Ok(PassKeyRequirement::Generated),
            _ => Err(super::BlueNRGError::BadPassKeyRequirement(value)),
        }
    }
}

fn to_boolean(value: u8) -> Result<bool, super::BlueNRGError> {
    match value {
        0 => Ok(false),
        1 => Ok(true),
        _ => Err(super::BlueNRGError::BadBooleanValue(value)),
    }
}

fn to_gap_security_level(
    bytes: &[u8],
) -> Result<GapSecurityLevel, hci::event::Error<super::BlueNRGError>> {
    require_len!(bytes, 5);

    Ok(GapSecurityLevel {
        status: to_status(&bytes[0..])?,
        mitm_protection_required: to_boolean(bytes[1]).map_err(hci::event::Error::Vendor)?,
        bonding_required: to_boolean(bytes[2]).map_err(hci::event::Error::Vendor)?,
        out_of_band_data_present: to_boolean(bytes[3]).map_err(hci::event::Error::Vendor)?,
        pass_key_required: bytes[4].try_into().map_err(hci::event::Error::Vendor)?,
    })
}

#[cfg(feature = "ms")]
/// Parameters returned by the [GAP Resolve Private
/// Address](::gap::Commands::resolve_private_address) command.
#[derive(Copy, Clone, Debug)]
pub struct GapResolvePrivateAddress {
    /// Did the command fail, and if so, how?
    pub status: hci::Status,

    /// If the address was successfully resolved, the peer address is returned.  This value is
    /// `None` if the address could not be resolved.
    pub bd_addr: Option<hci::BdAddr>,
}

#[cfg(feature = "ms")]
fn to_gap_resolve_private_address(
    bytes: &[u8],
) -> Result<GapResolvePrivateAddress, hci::event::Error<super::BlueNRGError>> {
    let status = to_status(&bytes)?;
    if status == hci::Status::Success {
        require_len!(bytes, 7);

        let mut addr = [0; 6];
        addr.copy_from_slice(&bytes[1..7]);

        Ok(GapResolvePrivateAddress {
            status: status,
            bd_addr: Some(hci::BdAddr(addr)),
        })
    } else {
        Ok(GapResolvePrivateAddress {
            status: status,
            bd_addr: None,
        })
    }
}

/// Parameters returned by the [GAP Get Bonded Devices](::gap::Commands::get_bonded_devices)
/// command.
#[derive(Copy, Clone)]
pub struct GapBondedDevices {
    /// Did the command fail, and if so, how?
    pub status: hci::Status,

    // Number of peer addresses in the event, and a buffer that can hold all of the addresses.
    address_count: usize,
    address_buffer: [hci::BdAddrType; MAX_ADDRESSES],
}

// Max packet size (255 bytes) less non-address data (4 bytes) divided by peer address size (7):
const MAX_ADDRESSES: usize = 35;

impl GapBondedDevices {
    /// Return an iterator over the bonded device addresses.
    pub fn bonded_addresses(&self) -> &[hci::BdAddrType] {
        &self.address_buffer[..self.address_count]
    }
}

impl Debug for GapBondedDevices {
    fn fmt(&self, f: &mut Formatter) -> FmtResult {
        write!(f, "{{")?;
        for addr in self.bonded_addresses().iter() {
            write!(f, "{:?}, ", addr)?;
        }
        write!(f, "}}")
    }
}

fn to_gap_bonded_devices(
    bytes: &[u8],
) -> Result<GapBondedDevices, hci::event::Error<super::BlueNRGError>> {
    let status = to_status(&bytes)?;
    match status {
        hci::Status::Success => {
            const HEADER_LEN: usize = 2;
            const ADDR_LEN: usize = 7;

            require_len_at_least!(bytes, HEADER_LEN);
            let address_count = bytes[1] as usize;
            if bytes.len() != HEADER_LEN + ADDR_LEN * address_count {
                return Err(hci::event::Error::Vendor(
                    super::BlueNRGError::PartialBondedDeviceAddress,
                ));
            }

            let mut address_buffer = [hci::BdAddrType::Public(hci::BdAddr([0; 6])); MAX_ADDRESSES];
            for i in 0..address_count {
                let index = HEADER_LEN + i * ADDR_LEN;
                let mut addr = [0; 6];
                addr.copy_from_slice(&bytes[(1 + index)..(7 + index)]);
                address_buffer[i] =
                    hci::to_bd_addr_type(bytes[index], hci::BdAddr(addr)).map_err(|e| {
                        hci::event::Error::Vendor(super::BlueNRGError::BadBdAddrType(e.0))
                    })?;
            }

            Ok(GapBondedDevices {
                status,
                address_count,
                address_buffer,
            })
        }
        _ => Ok(GapBondedDevices {
            status,
            address_count: 0,
            address_buffer: [hci::BdAddrType::Public(hci::BdAddr([0; 6])); MAX_ADDRESSES],
        }),
    }
}

/// Parameters returned by the [GATT Add Service](::gatt::Commands::add_service) and [GATT Include
/// Service](::gatt::Commands::include_service) commands.
#[derive(Copy, Clone, Debug)]
pub struct GattService {
    /// Did the command fail, and if so, how?
    pub status: hci::Status,

    /// Handle of the Service
    ///
    /// When this service is added to the server, a handle is allocated by the server to this
    /// service. Also server allocates a range of handles for this service from `service_handle` to
    /// `service_handle +
    /// [max_attribute_records](::gatt::ServiceParameters::max_attribute_records)`.
    pub service_handle: ::gatt::ServiceHandle,
}

fn to_gatt_service(bytes: &[u8]) -> Result<GattService, hci::event::Error<super::BlueNRGError>> {
    require_len!(bytes, 3);

    Ok(GattService {
        status: to_status(&bytes)?,
        service_handle: ::gatt::ServiceHandle(LittleEndian::read_u16(&bytes[1..3])),
    })
}

/// Parameters returned by the [GATT Add Characteristic](::gatt::Commands::add_characteristic)
/// command.
#[derive(Copy, Clone, Debug)]
pub struct GattCharacteristic {
    /// Did the command fail, and if so, how?
    pub status: hci::Status,

    /// Handle of the characteristic.
    pub characteristic_handle: ::gatt::CharacteristicHandle,
}

fn to_gatt_characteristic(
    bytes: &[u8],
) -> Result<GattCharacteristic, hci::event::Error<super::BlueNRGError>> {
    require_len!(bytes, 3);

    Ok(GattCharacteristic {
        status: to_status(&bytes)?,
        characteristic_handle: ::gatt::CharacteristicHandle(LittleEndian::read_u16(&bytes[1..3])),
    })
}

/// Parameters returned by the [GATT Add Characteristic
/// Descriptor](::gatt::Commands::add_characteristic_descriptor) command.
#[derive(Copy, Clone, Debug)]
pub struct GattCharacteristicDescriptor {
    /// Did the command fail, and if so, how?
    pub status: hci::Status,

    /// Handle of the characteristic.
    pub descriptor_handle: ::gatt::DescriptorHandle,
}

fn to_gatt_characteristic_descriptor(
    bytes: &[u8],
) -> Result<GattCharacteristicDescriptor, hci::event::Error<super::BlueNRGError>> {
    require_len!(bytes, 3);

    Ok(GattCharacteristicDescriptor {
        status: to_status(&bytes)?,
        descriptor_handle: ::gatt::DescriptorHandle(LittleEndian::read_u16(&bytes[1..3])),
    })
}

/// Parameters returned by the [GATT Read Handle Value](::gatt::Commands::read_handle_value)
/// command.
#[derive(Copy, Clone)]
pub struct GattHandleValue {
    /// Did the command fail, and if so, how?
    pub status: hci::Status,

    value_buf: [u8; GattHandleValue::MAX_VALUE_BUF],
    value_len: usize,
}

impl Debug for GattHandleValue {
    fn fmt(&self, f: &mut Formatter) -> FmtResult {
        write!(f, "{{")?;
        write!(f, "status: {:?}; value: {{", self.status);
        for addr in self.value().iter() {
            write!(f, "{:?}, ", addr)?;
        }
        write!(f, "}}}}")
    }
}

impl GattHandleValue {
    // Maximum length of the handle value. The spec says the length can be 2 bytes (up to 65535),
    // but the communication layer is limited to 255 bytes in a packet. There are 6 bytes reserved
    // for data other than the value, so the maximum length of the value buffer is 249 bytes.
    const MAX_VALUE_BUF: usize = 249;

    /// Return the handle value. Only valid bytes are returned.
    pub fn value(&self) -> &[u8] {
        &self.value_buf[..self.value_len]
    }
}

fn to_gatt_handle_value(
    bytes: &[u8],
) -> Result<GattHandleValue, hci::event::Error<super::BlueNRGError>> {
    require_len_at_least!(bytes, 3);

    let status = to_status(bytes)?;
    let value_len = LittleEndian::read_u16(&bytes[1..3]) as usize;
    require_len!(bytes, 3 + value_len);

    let mut handle_value = GattHandleValue {
        status,
        value_buf: [0; GattHandleValue::MAX_VALUE_BUF],
        value_len,
    };
    handle_value.value_buf[..value_len].copy_from_slice(&bytes[3..]);

    Ok(handle_value)
}
