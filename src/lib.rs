

use tokio_modbus::prelude::*;

/// # Modbus node address assignments
///
/// | Address | Description |
/// |-------|----------|
/// | 0x01  | Reserved for Modbus default node ID |
/// | 0x1A  | Current-Voltage 2 channel ADC   |
/// | 0x1F  | Current-Voltage 2 channel ADC   |
/// | 0x2A  | Precision current source (0-1000 mA)   |
/// | 0x2F  | Precision current source (0-100 mA)   |
/// | 0x3F  | Dual Type-K Thermocouple Reader |
/// | 0x4A  | 4-20mA current loop source |
/// | 0x4F  | Pyrometer simulator (4-20mA source) |
/// 
///

/// Modbus node IDs
pub const NODEID_DEFAULT: u8 = 0x01; // The Modbus node ID that most devices default to
pub const NODEID_N4VIA02_IV_ADC: u8 = 0x1A; 
pub const NODEID_IV_ADC: u8 = 0x1F;
pub const NODEID_PREC_CURR_SRC: u8 = 0x2F;
pub const NODEID_DUAL_TK: u8 = 0x3F;
pub const NODEID_N4IOA01_CURR_GEN: u8 = 0x4A; // 4-20 mA current loop source (signal generator)
pub const NODEID_PYRO_CURR_GEN: u8 = 0x4F; // TODO switching to new current loop signal generator
pub const NODEID_QUAD_RELAY: u8 = 0x5F; // TODO not yet programmed into relay board
pub const NODEID_MAX: u8 = 0x7F;

/// Register addresses
pub const REG_NODEID_TK:u16 = 0x20; // dual Type-K thermocouple reader
pub const REG_NODEID_IV:u16 = 0xFD; // 0-10 Volt, 0-5 Amp IV ADC
pub const REG_CFG_N4VIA02: u16 = 0xFA; // configuration params
pub const REG_NODEID_N4VIA02: u16 = 0xFD; // 0-1 Amp ADC
pub const REG_NODEID_PREC_CURR:u16 = 0x00; // Precision current source YK-PVCCS0100/YK-PVCC1000
pub const REG_SAVE_CFG_PREC_CURR:u16 = 0x02; // Cause YK-PVCCS to persist its parameters.
pub const REG_IV_ADC_2CH_VALS: u16 = 0x00; // Where 2CH IV ADC stores read values
pub const REG_N4VIA02_CURR_VALS: u16 = 0x00; // Where N4VIA02 stores two current values
pub const REG_N4VIA02_VOLT_VALS: u16 = 0x20; // Where N4VIA02 stores two voltage values
pub const REG_NODEID_N4IOA01: u16 = 0x0E; 
pub const REG_NODEID_PYRO_CURR_GEN:u16 = 0x04; // TODO wrong! node ID may not be settable for Taidacent-B0B7HLZ6B4 
pub const REG_N4IOA01_CURR_VAL: u16 = 0x00;

/// Combine two u16 registers into an i32
pub fn registers_to_i32(registers: &[u16], offset: usize) -> i32 {
    let high = registers[offset] as i32;
    let low = registers[offset + 1] as i32;
    let combined = (high << 16) | low;
    combined
}

pub async fn ping_one_modbus_node_id(ctx: &mut tokio_modbus::client::Context, node_id: u8,  reg_node_id: u16) 
    -> Result<(), Box<dyn std::error::Error>>
{
    println!("Read existing node ID from node {node_id:X?}, reg 0x{node_id:X?} ... ");
    ctx.set_slave(Slave(node_id));
    let read_rsp: Vec<u16> = ctx.read_holding_registers(reg_node_id, 1).await??;
    println!("> read_rsp: {:?}", read_rsp);

    let existing_node_id = read_rsp[0] as u8;
    if existing_node_id != node_id {
        println!("Node ID {node_id:X?} reports node ID of {existing_node_id:X?}");
        panic!("Couldn't verify the old node ID");
    }

    Ok(())
}