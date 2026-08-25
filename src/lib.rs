

use tokio_modbus::prelude::*;
use tokio::time::sleep;
use std::{time::Duration};

pub mod smc05;

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
/// | 0x5F  | Octo relay control |
/// | 0x6A  | Dipper stepper motor driver |
/// 
///

/// Modbus node IDs
pub const NODEID_BROADCAST_0: u8 = 0x00;
pub const NODEID_DEFAULT: u8 = 0x01; // The Modbus node ID that most devices default to
pub const NODEID_N4VIA02_IV_ADC: u8 = 0x1A; 
pub const NODEID_WA8TAI_IV_ADC: u8 = 0x1B; // Waveshare WA8TAI 8CH IV ADC
pub const NODEID_WDCU3003_IV_ADC: u8 = 0x13; // Set via the front panel, to decimal 19 == 0x13
pub const NODEID_N4AIA04_IV_ADC: u8 = 0x1E;
pub const NODEID_YKDAQ1402_IV_ADC: u8 = 0x1F;
pub const NODEID_YKPVCCS010_CURR_SRC: u8 = 0x2F;
pub const NODEID_YKKTC1202_DUAL_TK: u8 = 0x3F;
pub const NODEID_N4IOA01_CURR_GEN: u8 = 0x4A; // 4-20 mA current loop source (signal generator)
pub const NODEID_WA26419_8CH_DAC: u8 = 0x4F; // TODO Waveshare 8CH analog output (0-20 mA)
pub const NODEID_R4DVI04_QRELAY_ADC: u8 = 0x5A; // Eletechsup quad relay plus ADC
pub const NODEID_WAV_OCTO_RELAY: u8 = 0x5F; // Waveshare 8-relay board v3, SKU 17658
pub const NODEID_SMC05_STEP_DRIVER: u8 = 0x6A; // SMC05 stepper motor controler
pub const NODEID_MAX: u8 = 0x7F;

/// Register addresses
pub const REG_NODEID_YKKTC1202_DUAL_TK:u16 = 0x20; // dual Type-K thermocouple reader
pub const REG_NODEID_YKDAQ1402_IV_ADC:u16 = 64; // ELECDEMO YK-DAQ1402 0-10 Volt, 0-5 Amp IV ADC
pub const REG_CFG_N4VIA02: u16 = 0xFA; // configuration params
pub const REG_NODEID_N4VIA02: u16 = 0xFD; // 0-1 Amp ADC
pub const REG_NODEID_N4AIA04: u16 = 0x0E; // 4-20 mA ADC
pub const REG_NODEID_YKPVCCS010_CURR_SRC:u16 = 0x00; // Precision current source YK-PVCCS0100/YK-PVCC1000
pub const REG_SAVE_CFG_YKPVCCS010_CURR_SRC:u16 = 0x02; // Cause YK-PVCCS to persist its parameters.
pub const REG_YKPVCCS_BAUD: u16 = 0x0001; // get/set baud for YK-PVCCS models, 0-6 ...19200, 38400, 57600, 115200 max
pub const REG_IV_ADC_2CH_VALS: u16 = 0x00; // Where 2CH IV ADC stores read values
pub const REG_N4VIA02_CURR_VALS: u16 = 0x0000; // Where N4VIA02 stores two current values
pub const REG_N4VIA02_VOLT_VALS: u16 = 0x0020; // Where N4VIA02 stores two voltage values
pub const REG_NODEID_N4IOA01: u16 = 0x0E; 
pub const REG_NODEID_WAVESHARE_V2:u16 = 0x4000; // TODO ensure this value is set on all Waveshare devices
pub const REG_N4IOA01_CURR_VAL: u16 = 0x00;
pub const REG_YKKTC1202_TEMP_VALS: u16 = 0x00; // The dual RTK's temperature values
pub const REG_YKKTC1202_VALIDITY: u16 = 0x10; // The dual RTK's thermocouple connection state
pub const REG_YKKTC1202_BAUD: u16 = 0x0021; // get/set baud for YKKTC1202, 0-6,  ...19200, 38400, 57600, 115200 max
pub const REG_N4AIA04_CH1_CURR: u16 = 0x0002;
pub const REG_NODEID_R4DVI04: u16 = 0x00FD;
pub const REG_WA8TAI_BAUD: u16 = 0x2000; // get/set baud for WA8TAI, 0-7, ...19200, 38400, 57600, 115200, 128k, 256k max
pub const REG_R4DVI04_BAUD: u16 = 0x00FE; // get/set baud for RDVI04, 0-7, ...19200, 38400, 57600, 115200 max

pub const REG_YKPVCCS_DRIVE_MILLIAMPS: u16  = 0x10; // get/set drive mA for YK-PVCCS
pub const REG_YKPVCCS_MONITOR_MILLIAMPS: u16  = 0x11; // read YK-PVCCS ammeter


/// minimum current stabilization time supported by the current source
pub const CURRENT_SOURCE_STABILIZATION_MS: u64 = 25;
/// time we allow the current to settle, after driving, before measuring
pub const CURRENT_SOURCE_WAIT_TIME: Duration = Duration::from_millis(CURRENT_SOURCE_STABILIZATION_MS);

/// The minimum increment for drive current, as specified in the current source docs
pub const MIN_DRIVE_CURRENT_INCR_MA: f32 = 1.0;

/// Arbitrary value for "infinite" resistance (open circuit) between electrodes
pub const INF_INTER_ELECTRODE_OHMS: f32 = 666.;

/// We only recognize current values reported by the current source above this threshold
pub const REPORTED_CURRENT_THRESHOLD_MA: f32 = MIN_DRIVE_CURRENT_INCR_MA;

/// Combine two u16 registers into an i32
pub fn registers_to_i32(registers: &[u16], offset: usize) -> i32 {
    let high = registers[offset] as i32;
    let low = registers[offset + 1] as i32;
    let combined = (high << 16) | low;
    combined
}

/// Ensure that we can connect with the given Modbus node ID.
pub async fn ping_one_modbus_node_id(ctx: &mut tokio_modbus::client::Context, node_id: u8,  reg_node_id: u16) 
    -> Result<(), Box<dyn std::error::Error>>
{
    println!("Read node ID from node {node_id} ({node_id:X?}), reg 0x{reg_node_id:X?} ... ");
    ctx.set_slave(Slave(node_id));
    sleep(Duration::from_millis(50)).await;

    let read_rsp: Vec<u16> = ctx.read_holding_registers(reg_node_id, 1).await??;
    // println!("> read_rsp: {:?}", read_rsp);

    let existing_node_id = read_rsp[0] as u8;
    if existing_node_id != node_id {
        println!("Node ID {node_id} ({node_id:X?}) reports node ID of {existing_node_id} ({existing_node_id:X?})");
        panic!("Couldn't verify the old node ID");
    }
    else {
        println!("Node ID {node_id} ({node_id:X?}) verified");
    }

    Ok(())
}

///
/// Verify that a Modbus node is accessible by reading from register(s)
/// 
pub async fn ping_one_modbus_node_register(ctx: &mut tokio_modbus::client::Context, node_id: u8, register: u16, count: u16) 
    -> Result<(), Box<dyn std::error::Error>>
{
    ctx.set_slave(Slave(node_id));
    let _read_resp: Vec<u16> = ctx.read_holding_registers(register, count).await??;
    Ok(())
}

///
/// Read the voltage and current at active electrode pair.
///
pub async fn read_ykdaq1402_iv_adc(ctx: &mut tokio_modbus::client::Context)
-> Result<(f32, f32), Box<dyn std::error::Error>> 
{
    ctx.set_slave(Slave(NODEID_YKDAQ1402_IV_ADC));
    // let read_rsp: Vec<u16> = ctx.read_holding_registers(REG_NODEID_YKDAQ1402_IV_ADC, 3).await??;
    // println!(" YKDAQ1402 CFG ({REG_NODEID_YKDAQ1402_IV_ADC:?})[3]: {read_rsp:?}");
    let iv_adc_vals: Vec<u16> = ctx.read_holding_registers(REG_IV_ADC_2CH_VALS, 4).await??;
    println!(" YKDAQ1402 VALS ({REG_IV_ADC_2CH_VALS:?})[4]: {iv_adc_vals:?}");
    let ch1_value = registers_to_i32(&iv_adc_vals, 0);
    let verified_volts = (ch1_value as f32) / 10000.0; // resolution is 0.1 mV for 10V range
    let ch2_value = registers_to_i32(&iv_adc_vals, 2);
    let verified_milliamps: f32 = (ch2_value as f32)/ 10.0; // resolution is 0.1 mA for 5A range

    println!(" YKDAQ1402 ch1_value: {ch1_value:?} = {verified_volts:?} V");
    println!(" YKDAQ1402 ch2_value: {ch2_value:?} = {verified_milliamps:?} mA");
    Ok((verified_volts, verified_milliamps))
}


///
/// Read the voltage and current at active electrode pair.
///
pub async fn read_wdcu3003_iv_adc(ctx: &mut tokio_modbus::client::Context)
-> Result<(f32, f32), Box<dyn std::error::Error>> 
{
    ctx.set_slave(Slave(NODEID_WDCU3003_IV_ADC));
    let iv_adc_vals: Vec<u16> = ctx.read_holding_registers(0, 4).await??;
    // println!("WDCU3003 vals: {:?}",iv_adc_vals);
    let raw_potential_val = iv_adc_vals[0] as f32;
    let high_range = iv_adc_vals[1] != 0;
    let raw_current_val: f32 = iv_adc_vals[2] as f32;
    let milliwatts_val = iv_adc_vals[3] as f32;

    let volt_val = raw_potential_val / 1000.;
    let milliamps_val = 
        if high_range { milliwatts_val / volt_val }
        else {raw_current_val / 1000. };

    Ok((volt_val, milliamps_val))
}

/**
 * Set the output drive current of the YK-PVCCS0100 precision current source 
 * @return The reading of current
 */
pub async fn set_ykpvccs0100_current_drive(ctx: &mut tokio_modbus::client::Context, milliamps: f32) 
-> Result<(), Box<dyn std::error::Error>> 
{
    ctx.set_slave(Slave(NODEID_YKPVCCS010_CURR_SRC)); 

    // precision is 0.1 mA
    let out_ma_setting: u16 = (10.0 * milliamps).round() as u16;
    // println!("writing val of : {}", out_ma_setting);
    ctx.write_single_register(REG_YKPVCCS_DRIVE_MILLIAMPS, out_ma_setting).await??;

    Ok(())
}    

pub async fn set_ykpvccs1000_current_drive(ctx: &mut tokio_modbus::client::Context, milliamps: f32) 
-> Result<(), Box<dyn std::error::Error>> 
{
    ctx.set_slave(Slave(NODEID_YKPVCCS010_CURR_SRC)); 

    // precision is 1 mA, range is 0...1000
    let out_ma_setting: u16 = ((milliamps).round()) as u16;
    // println!("writing val of : {}", out_ma_setting);
    ctx.write_single_register(REG_YKPVCCS_DRIVE_MILLIAMPS, out_ma_setting).await??;

    Ok(())
} 

pub async fn read_ykpvccs0100_current_drive(ctx: &mut tokio_modbus::client::Context)
-> Result<f32, Box<dyn std::error::Error>> 
{
    let read_rsp: Vec<u16> = ctx.read_holding_registers(REG_YKPVCCS_MONITOR_MILLIAMPS, 1).await??;
    let actual_ma = (read_rsp[0] as f32)/10.0;// precision is 0.1 mA
    Ok(actual_ma)
}

pub async fn read_ykpvccs1000_current_drive(ctx: &mut tokio_modbus::client::Context)
-> Result<f32, Box<dyn std::error::Error>> 
{
    let read_rsp: Vec<u16> = ctx.read_holding_registers(REG_YKPVCCS_MONITOR_MILLIAMPS, 1).await??;
    let actual_ma = read_rsp[0] as f32;// precision is 1 mA
    Ok(actual_ma)
}


pub async fn read_n4via02_multimeter(ctx: &mut tokio_modbus::client::Context)
-> Result<((f32, f32), (f64, f64)), Box<dyn std::error::Error>> 
{
    const CURRENT_CONVERSION_FACTOR: f64 = 0.5;
    // println!("Check N4VIA02_IV... ");
    ctx.set_slave(Slave(NODEID_N4VIA02_IV_ADC));
    // let mut ctx_iv_adc: client::Context = rtu::attach_slave(SerialStream::open(&builder).unwrap(), Slave(NODEID_IV_ADC));
    // let read_rsp: Vec<u16> = ctx.read_holding_registers(REG_CFG_N4VIA02, 6).await??;
    // println!(" N4VIA02 CFG ({REG_NODEID_N4VIA02:?})[6]: {read_rsp:?}");
    // let read_rsp: Vec<u16> = ctx.read_holding_registers(REG_NODEID_N4VIA02, 1).await??;
    // println!(" N4VIA02 NODE ID ({REG_NODEID_N4VIA02:?})[1]: {read_rsp:?}");
    let milliamp_vals: Vec<u16> = ctx.read_holding_registers(REG_N4VIA02_CURR_VALS, 2).await??;
    println!(" N4VIA02 mA VALS ({REG_N4VIA02_CURR_VALS:?})[2]: {milliamp_vals:?}");
    let ch0_ma = (milliamp_vals[0] as f64) * CURRENT_CONVERSION_FACTOR;
    let ch1_ma = (milliamp_vals[1] as f64) * CURRENT_CONVERSION_FACTOR;
    // let voltage_vals: Vec<u16> = ctx.read_holding_registers(REG_N4VIA02_VOLT_VALS, 2).await??;
    // println!(" N4VIA02 V VALS ({REG_N4VIA02_VOLT_VALS:?})[2]: {voltage_vals:?}");

    Ok(((0.,0.), (ch0_ma, ch1_ma)))
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
    // let ch2_milliamps = (milliamp_vals[1] as f32)/10.0;
    //TODO extract mA and voltage?
    // let voltage_vals: Vec<u16> = ctx.read_holding_registers(REG_N4VIA02_VOLT_VALS, 2).await??;
    // println!(" N4VIA02 V VALS ({REG_N4VIA02_VOLT_VALS:?})[2]: {voltage_vals:?}");
    Ok((0f32, ch1_milliamps))
}



/**
 * Read Waveshare WA8TAI 8CH analog IV ADC.
 * Returns a value that is either milliamps or volts, depending on how the channel was configured
 */
pub async fn read_wa8tai_one_channel(ctx: &mut tokio_modbus::client::Context, channel: u8)
-> Result<f32, Box<dyn std::error::Error>> 
{
    let chan_offset = (channel as u16) - 1; 
    ctx.set_slave(Slave(NODEID_WA8TAI_IV_ADC)); 
    let resp: Vec<u16> = ctx.read_input_registers(chan_offset, 1).await??; // read just one channel
    // println!("AIN resp: {resp:?}");
    let val = resp[0];
    // println!("AIN ch {channel:?} val: {val:?}");
    //output range 4000~20000, unit uA;
    let converted_val = (val as f32) / 1E3; // either milliamps or volts
    Ok(converted_val)
}

/**
 * Read Waveshare WA8TAI 8CH analog IV ADC.
 * Returns a value that is either milliamps or volts, depending on how the channel was configured
 */
pub async fn read_wa8tai_volts_milliamps(ctx: &mut tokio_modbus::client::Context)
-> Result<(f32, f32), Box<dyn std::error::Error>> 
{
    ctx.set_slave(Slave(NODEID_WA8TAI_IV_ADC)); 
    let resp: Vec<u16> = ctx.read_input_registers(0x0000, 2).await??; //read all 8 at once
    // println!("AIN resp: {resp:?}");
    let volts = (resp[0] as f32) / 1E3; // original value is millivolts
    let milliamps =  (resp[1] as f32) / 1E3; //output range 4000~20000, unit uA;

    Ok((volts, milliamps))
}



/**
 * Set the pyro simulator current loop controller (4-20 mA source) current value
 * Note: channel is 1-8
 */
pub async fn set_wa26419_0420_current_loop_drive(ctx: &mut tokio_modbus::client::Context, channel: u8, milliamps: f32) 
-> Result<(), Box<dyn std::error::Error>> 
{
    let chan_address: u16 = 0x0000 + (channel - 1) as u16;
    ctx.set_slave(Slave(NODEID_WA26419_8CH_DAC)); 

    // this module accepts settings in microamps (mA * 1000)
    let desired_microamps = (milliamps * 1000.0).round() as u16;
    // println!("setting microamps: {desired_microamps:?}");
    ctx.write_single_register(chan_address, desired_microamps).await??;

    let resp = ctx.read_holding_registers(chan_address, 1).await??;
    let verified_microamps = resp[0];
    if verified_microamps != desired_microamps {
        println!("desired {desired_microamps:?} verfied {verified_microamps:?}");
    }

    Ok(())
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


/**
 * Read the dual thermocouple reader
 */
pub async fn read_ykktc1202_one_tk_temp(ctx: &mut tokio_modbus::client::Context)
-> Result<Option<f32>, Box<dyn std::error::Error>> 
{
    ctx.set_slave(Slave(NODEID_YKKTC1202_DUAL_TK));
    // move RTK check into a separate function
    // let cfg_rsp: Vec<u16> = ctx.read_holding_registers(0x20, 3).await??;
    // println!(" 0x20 cfg_rsp: {:?}", cfg_rsp);

    let tk_valid_resp: Vec<u16> = ctx.read_holding_registers(REG_YKKTC1202_VALIDITY, 1).await??;
    // println!(" REG_TK_VALIDITY: {:?}", tk_valid_resp);
    let mut ch1_tk_conn: bool = tk_valid_resp[0] == 0; // 0: The thermocouple is connected, 1: The thermocouple is not connected

    // TODO use reg address consts
    // example of reading all the dual TK registers:
    let tk_resp: Vec<u16> = ctx.read_holding_registers(REG_YKKTC1202_TEMP_VALS, 1).await??;
    // println!(" REG_TK_TEMP_VALS: {:?}", tk_resp);
    let ch1_tk_val: f32 = (tk_resp[0] as f32) / 10.0; // resolution is 0.1 °C

    // the TK reader will sometimes report a thermocouple is disconnected when it's not
    if !ch1_tk_conn && ch1_tk_val < 1000.0 { //1000 C
        ch1_tk_conn = true;
    }

    let ch1_tk_opt = if ch1_tk_conn { Some(ch1_tk_val) } else { None };
    Ok(ch1_tk_opt)
}

pub async fn toggle_r4dvi04_relay(ctx: &mut tokio_modbus::client::Context, channel: u8, active: bool)
-> Result<(), Box<dyn std::error::Error>> 
{
    let relay_coil_address: u16 = (channel -1) as u16;
    ctx.set_slave(Slave(NODEID_R4DVI04_QRELAY_ADC));
    ctx.write_single_coil(relay_coil_address, active).await??;
    Ok(())
}

pub async fn toggle_wav_octo_relay(ctx: &mut tokio_modbus::client::Context, channel: u8, active: bool)
-> Result<(), Box<dyn std::error::Error>> 
{
    ctx.set_slave(Slave(NODEID_WAV_OCTO_RELAY));
    let relay_coil_address: u16 = (channel -1) as u16;
    // println!("set relay channel {}  (idx {}) to {}", channel, relay_coil_address, active);
    ctx.write_single_coil(relay_coil_address, active).await??;
    Ok(())
}

pub async fn write_wav_octo_relays(ctx: &mut tokio_modbus::client::Context, channel_vals: &[bool])
-> Result<(), Box<dyn std::error::Error>> 
{
    ctx.set_slave(Slave(NODEID_WAV_OCTO_RELAY));
    ctx.write_multiple_coils(0x0000, &channel_vals).await??;
    Ok(())
}


/// Read the dual thermocouple ADC
pub async fn read_dual_tk_temps(ctx: &mut tokio_modbus::client::Context)
-> Result<(Option<f32>, Option<f32>), Box<dyn std::error::Error>> 
{
    read_ykktc1202_dual_tk_temps(ctx).await
}


pub async fn read_single_tk_temp(ctx: &mut tokio_modbus::client::Context)
-> Result<Option<f32>, Box<dyn std::error::Error>> 
{
    read_ykktc1202_one_tk_temp(ctx).await
}

 /// Set the output drive current of the test electrodes 
pub async fn set_electrode_current_drive(ctx: &mut tokio_modbus::client::Context, milliamps: f32) -> Result<(), Box<dyn std::error::Error>> 
{
    // set_ykpvccs0100_current_drive(ctx, milliamps).await
    set_ykpvccs1000_current_drive(ctx, milliamps).await
}

/// Read the reported current from the current source
pub async fn read_electrode_current_drive(ctx: &mut tokio_modbus::client::Context) -> Result<f32, Box<dyn std::error::Error>> 
{
    //read_ykpvccs0100_current_drive(ctx).await
    read_ykpvccs1000_current_drive(ctx).await
}

// pub async fn read_stable_electrode_iv(ctx: &mut tokio_modbus::client::Context) 
// -> Result<(f32, f32, f32), Box<dyn std::error::Error>> 
// {
//     const NUM_IV_READ_STEPS: i32 = 3;
//     const AVG_IV_FACTOR: f32 = NUM_IV_READ_STEPS as f32;

//     let mut total_volts = 0.;
//     let mut total_milliamps = 0.;


//     for _i in 0..NUM_IV_READ_STEPS {
//         let (step_volts, step_milliamps) = read_wdcu3003_iv_adc(ctx).await?;
//         total_volts += step_volts;
//         total_milliamps += step_milliamps;
//         sleep(CURRENT_SOURCE_STABILIZATION_MS).await;
//     }
//     Ok
// }

/// Set drive current on electrode and measure output
/// # Returns
/// (measured_volts, measured_milliamps, measured_ohms)
pub async fn drive_current_and_measure(ctx: &mut tokio_modbus::client::Context,
    target_drive_ma: f32, 
) 
-> Result<(f32, f32, f32), Box<dyn std::error::Error>> 
{
    // Drive output current pulse based on prior settings, and measure result
    set_electrode_current_drive(ctx, target_drive_ma).await?;
    sleep(CURRENT_SOURCE_WAIT_TIME).await;
    let reported_drive_ma = read_electrode_current_drive(ctx).await?;

    // Measure the average resulting induced current and potential across the electrodes
    let mut total_volts = 0.;
    let mut total_milliamps = 0.;
    const NUM_IV_READ_STEPS: i32 = 3;
    const AVG_IV_FACTOR: f32 = NUM_IV_READ_STEPS as f32;

    for _i in 0..NUM_IV_READ_STEPS {
        let (step_volts, step_milliamps) = read_wdcu3003_iv_adc(ctx).await?;
        total_volts += step_volts;
        total_milliamps += step_milliamps;
        sleep(CURRENT_SOURCE_WAIT_TIME).await;
    }
    // average multiple potential samples
    let measured_volts = total_volts / AVG_IV_FACTOR;
    // average multiple current samples
    let mut measured_milliamps: f32 = 
        if target_drive_ma > 0.  && reported_drive_ma > REPORTED_CURRENT_THRESHOLD_MA  
        {  total_milliamps / AVG_IV_FACTOR }  
        else { 0. };

    // sanity check that measured current is close to (current source reported) drive current
    if reported_drive_ma > 2.*MIN_DRIVE_CURRENT_INCR_MA {
        let mr_current_gap_frac = (reported_drive_ma - measured_milliamps)/reported_drive_ma;
        if mr_current_gap_frac > 0.05 {
            println!("mr_current_gap : {:.3}", mr_current_gap_frac);
            measured_milliamps = reported_drive_ma;
        }
    }

    let measured_ohms = 
        if measured_milliamps > 0. {
            // this also covers the case where volts = 0.0, i.e. zero resistance.
            (1000. * measured_volts) / measured_milliamps 
        }
        else {
            INF_INTER_ELECTRODE_OHMS // arbitrary value based on previous experiments
        };

    return Ok((measured_volts, measured_milliamps, measured_ohms))
}