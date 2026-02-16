

use tokio_modbus::prelude::*;
use tokio::time::sleep;
use std::time::Duration;

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
pub const NODEID_BROADCAST_0: u8 = 0x00;
pub const NODEID_DEFAULT: u8 = 0x01; // The Modbus node ID that most devices default to
pub const NODEID_N4VIA02_IV_ADC: u8 = 0x1A; 
pub const NODEID_N4AIA04_IV_ADC: u8 = 0x1E;
pub const NODEID_YKDAQ1402_IV_ADC: u8 = 0x1F;
pub const NODEID_YKPVCCS010_CURR_SRC: u8 = 0x2F;
pub const NODEID_YKKTC1202_DUAL_TK: u8 = 0x3F;
pub const NODEID_N4IOA01_CURR_GEN: u8 = 0x4A; // 4-20 mA current loop source (signal generator)
pub const NODEID_PYRO_CURR_GEN: u8 = 0x4F; // TODO switching to new current loop signal generator
pub const NODEID_QUAD_RELAY: u8 = 0x5F; // TODO not yet programmed into relay board
pub const NODEID_MAX: u8 = 0x7F;

/// Register addresses
pub const REG_NODEID_YKKTC1202_DUAL_TK:u16 = 0x20; // dual Type-K thermocouple reader
pub const REG_NODEID_YKDAQ1402_IV_ADC:u16 = 64; // ELECDEMO YK-DAQ1402 0-10 Volt, 0-5 Amp IV ADC
pub const REG_CFG_N4VIA02: u16 = 0xFA; // configuration params
pub const REG_NODEID_N4VIA02: u16 = 0xFD; // 0-1 Amp ADC
pub const REG_NODEID_N4AIA04: u16 = 0x0E; // 4-20 mA ADC
pub const REG_NODEID_YKPVCCS010_CURR_SRC:u16 = 0x00; // Precision current source YK-PVCCS0100/YK-PVCC1000
pub const REG_SAVE_CFG_YKPVCCS010_CURR_SRC:u16 = 0x02; // Cause YK-PVCCS to persist its parameters.
pub const REG_IV_ADC_2CH_VALS: u16 = 0x00; // Where 2CH IV ADC stores read values
pub const REG_N4VIA02_CURR_VALS: u16 = 0x00; // Where N4VIA02 stores two current values
pub const REG_N4VIA02_VOLT_VALS: u16 = 0x20; // Where N4VIA02 stores two voltage values
pub const REG_NODEID_N4IOA01: u16 = 0x0E; 
pub const REG_NODEID_PYRO_CURR_GEN:u16 = 0x04; // TODO wrong! node ID may not be settable for Taidacent-B0B7HLZ6B4 
pub const REG_N4IOA01_CURR_VAL: u16 = 0x00;
pub const REG_YKKTC1202_TEMP_VALS: u16 = 0x00; // The dual RTK's temperature values
pub const REG_YKKTC1202_VALIDITY: u16 = 0x10; // The dual RTK's thermocouple connection state
pub const REG_N4AIA04_CH1_CURR: u16 = 0x0002;


// /// Combine two u16 registers into amn f32
// fn registers_to_f32(registers: &[u16], offset: usize) -> f32 {
//     let high = registers[offset] as u32;
//     let low = registers[offset + 1] as u32;
//     let combined = (high << 16) | low;
    
//     // Convert u32 to f32
//     f32::from_bits(combined)
// }

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
    println!("Read existing node ID from node {node_id:X?}, reg 0x{reg_node_id:X?} ... ");
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

/**
 * Read the voltage and current at active electrode pair.
 * 
 */
pub async fn read_ykdaq1402_iv_adc(ctx: &mut tokio_modbus::client::Context)
-> Result<(f32, f32), Box<dyn std::error::Error>> 
{
    ctx.set_slave(Slave(NODEID_YKDAQ1402_IV_ADC));
    // let read_rsp: Vec<u16> = ctx.read_holding_registers(REG_NODEID_YKDAQ1402_IV_ADC, 3).await??;
    // println!(" YKDAQ1402 CFG ({REG_NODEID_YKDAQ1402_IV_ADC:?})[3]: {read_rsp:?}");
    let iv_adc_vals: Vec<u16> = ctx.read_holding_registers(REG_IV_ADC_2CH_VALS, 4).await??;
    // println!(" YKDAQ1402 VALS ({REG_IV_ADC_2CH_VALS:?})[4]: {iv_adc_vals:?}");
    let ch1_value = registers_to_i32(&iv_adc_vals, 0);
    let verified_volts = (ch1_value as f32) / 10000.0; // resolution is 0.1 mV for 10V range
    let ch2_value = registers_to_i32(&iv_adc_vals, 2);
    let verified_milliamps = (ch2_value as f32)/ 10.0; // resolution is 0.1 mA for 5A range

    // println!(" YKDAQ1402 ch1_value: {ch1_value:?} = {verified_volts:?} V");
    // println!(" YKDAQ1402 ch2_value: {ch2_value:?} = {verified_milliamps:?} mA");
    Ok((verified_volts, verified_milliamps))
}

/**
 * Set the output drive current of the YK-PVCCS0100 precision current source 
 * @return The reading of current
 */
pub async fn set_ykpvccs0100_current_drive(ctx: &mut tokio_modbus::client::Context, milliamps: f32) 
-> Result<f32, Box<dyn std::error::Error>> 
{
    const REG_ADDR_DRIVE_MILLIAMPS: u16  = 0x10;
    const REG_ADDR_MONITOR_MILLIAMPS: u16  = 0x11;

    // println!("set_precision_current_drive: {milliamps:?} mA");

    ctx.set_slave(Slave(NODEID_YKPVCCS010_CURR_SRC)); 
    // println!("Reading existing drive current value");
    // let read_rsp: Vec<u16> = ctx.read_holding_registers(REG_ADDR_DRIVE_MILLIAMPS, 1).await??;
    // println!("existing current read_rsp: {read_rsp:?}");
    // let original_ma_val = read_rsp[0];

    let out_ma_setting: u16 = (milliamps/0.1).round() as u16;
    // println!("writing val of : {}", out_ma_setting);
    ctx.write_single_register(REG_ADDR_DRIVE_MILLIAMPS, out_ma_setting).await??;
    // let read_rsp: Vec<u16> = ctx.read_holding_registers(REG_ADDR_DRIVE_MILLIAMPS, 1).await??;
    // let new_ma_val = read_rsp[0];
    // println!(" prec old: {original_ma_val:?} new: {new_ma_val:?}");

    sleep(Duration::from_millis(125)).await; // TODO sufficient settling time?
    let read_rsp: Vec<u16> = ctx.read_holding_registers(REG_ADDR_MONITOR_MILLIAMPS, 1).await??;
    let actual_ma = (read_rsp[0] as f32)/10.0;// precision is 0.1 mA
    // println!(" prec old: {original_ma_val:?} new: {new_ma_val:?} measured: {actual_ma:?}");

    // // println!(" prec measured: {actual_ma:?} mA");

    Ok(actual_ma)
}

pub async fn read_n4_via02_multimeter(ctx: &mut tokio_modbus::client::Context)
-> Result<(), Box<dyn std::error::Error>> 
{
    // println!("Check N4VIA02_IV... ");
    ctx.set_slave(Slave(NODEID_N4VIA02_IV_ADC));
    // let mut ctx_iv_adc: client::Context = rtu::attach_slave(SerialStream::open(&builder).unwrap(), Slave(NODEID_IV_ADC));
    // let read_rsp: Vec<u16> = ctx.read_holding_registers(REG_CFG_N4VIA02, 6).await??;
    // println!(" N4VIA02 CFG ({REG_NODEID_N4VIA02:?})[6]: {read_rsp:?}");
    // let read_rsp: Vec<u16> = ctx.read_holding_registers(REG_NODEID_N4VIA02, 1).await??;
    // println!(" N4VIA02 NODE ID ({REG_NODEID_N4VIA02:?})[1]: {read_rsp:?}");
    let milliamp_vals: Vec<u16> = ctx.read_holding_registers(REG_N4VIA02_CURR_VALS, 2).await??;
    println!(" N4VIA02 mA VALS ({REG_N4VIA02_CURR_VALS:?})[2]: {milliamp_vals:?}");
    let voltage_vals: Vec<u16> = ctx.read_holding_registers(REG_N4VIA02_VOLT_VALS, 2).await??;
    println!(" N4VIA02 V VALS ({REG_N4VIA02_VOLT_VALS:?})[2]: {voltage_vals:?}");
    Ok(())
}

pub async fn read_n4aia04_420_iv_adc(ctx: &mut tokio_modbus::client::Context)
-> Result<(f32, f32), Box<dyn std::error::Error>> 
{
    // println!("Check N4VIA02_IV... ");
    ctx.set_slave(Slave(NODEID_N4AIA04_IV_ADC));
    // let mut ctx_iv_adc: client::Context = rtu::attach_slave(SerialStream::open(&builder).unwrap(), Slave(NODEID_IV_ADC));
    // let read_rsp: Vec<u16> = ctx.read_holding_registers(REG_CFG_N4VIA02, 6).await??;
    // println!(" N4VIA02 CFG ({REG_NODEID_N4VIA02:?})[6]: {read_rsp:?}");
    // let read_rsp: Vec<u16> = ctx.read_holding_registers(REG_NODEID_N4VIA02, 1).await??;
    // println!(" N4VIA02 NODE ID ({REG_NODEID_N4VIA02:?})[1]: {read_rsp:?}");
    let milliamp_vals: Vec<u16> = ctx.read_holding_registers(REG_N4AIA04_CH1_CURR, 2).await??;
    // println!(" N4AIA04  mA VALS ({REG_N4VIA02_CURR_VALS:?})[2]: {milliamp_vals:?}");
    let ch1_milliamps = (milliamp_vals[0] as f32)/10.0;
    //TODO extract mA and voltage?
    // let voltage_vals: Vec<u16> = ctx.read_holding_registers(REG_N4VIA02_VOLT_VALS, 2).await??;
    // println!(" N4VIA02 V VALS ({REG_N4VIA02_VOLT_VALS:?})[2]: {voltage_vals:?}");
    Ok((0f32, ch1_milliamps))
}


/**
 * Set the pyro simulator current loop controller (4-20 mA source) current value
 */
pub async fn set_n4ioa01_0420_current_loop_drive(ctx: &mut tokio_modbus::client::Context,  milliamps: f32) 
-> Result<(), Box<dyn std::error::Error>> 
{
    ctx.set_slave(Slave(NODEID_N4IOA01_CURR_GEN)); 

    // println!("Reading current loop value");
    // let old_set: Vec<u16> = ctx.read_holding_registers(REG_N4IOA01_CURR_VAL, 1).await??;
    // // println!("read_rsp: {read_rsp:?}");
    // sleep(Duration::from_millis(250)).await;

    let out_ma_setting: u16 = (milliamps * 100.0).round() as u16;
    // println!("writing val of : {milliamps:?} mA  -> {out_ma_setting:?}");
    ctx.write_single_register(REG_N4IOA01_CURR_VAL, out_ma_setting).await??;

    // sleep(Duration::from_millis(500)).await;
    // println!("Reading new current loop value");
    // let new_set: Vec<u16> = ctx.read_holding_registers(REG_N4IOA01_CURR_VAL, 1).await??;
    // println!("420 src values: {read_rsp:?}");
    // println!(" 420src old: {old_set:?} new: {out_ma_setting:?} ");
    Ok(())
}
/**
 * Read the dual thermocouple reader
 */
pub async fn read_ykktc1202_dual_tk_temps(ctx: &mut tokio_modbus::client::Context)
-> Result<(Option<f32>, Option<f32>), Box<dyn std::error::Error>> 
{
    ctx.set_slave(Slave(NODEID_YKKTC1202_DUAL_TK));
    // move RTK check into a separate function
    // let cfg_rsp: Vec<u16> = ctx.read_holding_registers(0x20, 3).await??;
    // println!(" 0x20 cfg_rsp: {:?}", cfg_rsp);

    let tk_valid_resp: Vec<u16> = ctx.read_holding_registers(REG_YKKTC1202_VALIDITY, 2).await??;
    // println!(" REG_TK_VALIDITY: {:?}", tk_valid_resp);
    let mut ch1_tk_conn: bool = tk_valid_resp[0] == 0; // 0: The thermocouple is connected, 1: The thermocouple is not connected
    let mut ch2_tk_conn: bool = tk_valid_resp[1] == 0;

    // TODO use reg address consts
    // example of reading all the dual TK registers:
    let tk_resp: Vec<u16> = ctx.read_holding_registers(REG_YKKTC1202_TEMP_VALS, 2).await??;
    // println!(" REG_TK_TEMP_VALS: {:?}", tk_resp);
    let ch1_tk_val: f32 = (tk_resp[0] as f32) / 10.0; // resolution is 0.1 °C
    let ch2_tk_val: f32 = (tk_resp[1] as f32) / 10.0; // resolution is 0.1 °C

    // the TK reader will sometimes report a thermocouple is disconnected when it's not
    if !ch1_tk_conn && ch1_tk_val < 1000.0 { //1000 C
        ch1_tk_conn = true;
    }
    if !ch2_tk_conn && ch2_tk_val < 1000.0 { //1000 C
        ch2_tk_conn = true;
    }

    let ch1_tk_opt = if ch1_tk_conn { Some(ch1_tk_val) } else { None };
    let ch2_tk_opt = if ch2_tk_conn { Some(ch2_tk_val) } else { None };
    Ok((ch1_tk_opt, ch2_tk_opt))
}
