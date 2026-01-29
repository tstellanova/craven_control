#![allow(unused)]

use std::process::exit;
use std::time::Duration;
use tokio::time::sleep;
// use tokio_modbus::client::Reader;
// use tokio_modbus::Result;
// use tokio_modbus::client::Client;

use tokio_modbus::prelude::*;

use tokio_modbus::client::{Client, Reader, Writer};
// use byteorder::{BigEndian, ByteOrder};

    
use craven_control::*;

/**
 * Read the dual thermocouple reader
 */
async fn read_dual_tk_temps(ctx: &mut tokio_modbus::client::Context)
-> Result<(), Box<dyn std::error::Error>> 
{
    ctx.set_slave(Slave(NODEID_DUAL_TK));
    // move RTK check into a separate function
    // let cfg_rsp: Vec<u16> = ctx.read_holding_registers(0x20, 3).await??;
    // println!(" 0x20 cfg_rsp: {:?}", cfg_rsp);

    let tk_valid_resp: Vec<u16> = ctx.read_holding_registers(REG_TK_VALIDITY, 2).await??;
    // println!("REG_TK_TEMP_VALS tk_valid_resp: {:?}", tk_valid_resp);
    let ch1_tk_conn: bool = tk_valid_resp[0] == 0; // 0: The thermocouple is connected, 1: The thermocouple is not connected
    let ch2_tk_conn: bool = tk_valid_resp[1] == 0;

    // TODO use reg address consts
    // example of reading all the dual TK registers:
    let tk_resp: Vec<u16> = ctx.read_holding_registers(REG_TK_TEMP_VALS, 2).await??;
    // println!(" REG_TK_TEMP_VALS tk_resp: {:?}", tk_resp);
    let ch1_tk_val: f32 = (tk_resp[0] as f32) / 10.0; // resolution is 0.1 °C
    let ch2_tk_val: f32 = (tk_resp[1] as f32) / 10.0; // resolution is 0.1 °C

    if ch1_tk_conn {
        println!(" TK1 valid: {ch1_tk_val:?}°C");
    }
    if ch2_tk_conn {
        println!(" TK2 valid: {ch2_tk_val:?}°C");
    }
    Ok(())
}

async fn read_multimeter(ctx: &mut tokio_modbus::client::Context)
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

async fn read_420_iv_adc(ctx: &mut tokio_modbus::client::Context)
-> Result<(), Box<dyn std::error::Error>> 
{
    // println!("Check N4VIA02_IV... ");
    ctx.set_slave(Slave(NODEID_N4AIA04_IV_ADC));
    // let mut ctx_iv_adc: client::Context = rtu::attach_slave(SerialStream::open(&builder).unwrap(), Slave(NODEID_IV_ADC));
    // let read_rsp: Vec<u16> = ctx.read_holding_registers(REG_CFG_N4VIA02, 6).await??;
    // println!(" N4VIA02 CFG ({REG_NODEID_N4VIA02:?})[6]: {read_rsp:?}");
    // let read_rsp: Vec<u16> = ctx.read_holding_registers(REG_NODEID_N4VIA02, 1).await??;
    // println!(" N4VIA02 NODE ID ({REG_NODEID_N4VIA02:?})[1]: {read_rsp:?}");
    let milliamp_vals: Vec<u16> = ctx.read_holding_registers(REG_N4AIA04_CH1_CURR, 2).await??;
    println!(" N4AIA04  mA VALS ({REG_N4VIA02_CURR_VALS:?})[2]: {milliamp_vals:?}");
    // let voltage_vals: Vec<u16> = ctx.read_holding_registers(REG_N4VIA02_VOLT_VALS, 2).await??;
    // println!(" N4VIA02 V VALS ({REG_N4VIA02_VOLT_VALS:?})[2]: {voltage_vals:?}");
    Ok(())
}

/**
 * Read the voltage and current at active electrode pair.
 * 
 */
async fn read_electrode_pair_iv(ctx: &mut tokio_modbus::client::Context)
-> Result<(), Box<dyn std::error::Error>> 
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

    println!(" YKDAQ1402 ch1_value: {ch1_value:?} = {verified_volts:?} V");
    println!(" YKDAQ1402 ch2_value: {ch2_value:?} = {verified_milliamps:?} mA");
    Ok(())
}


/**
 * Set the pyro simulator current loop controller (4-20 mA source) current value
 */
async fn set_current_loop_drive(ctx: &mut tokio_modbus::client::Context,  milliamps: f32) 
-> Result<(), Box<dyn std::error::Error>> 
{
    ctx.set_slave(Slave(NODEID_N4IOA01_CURR_GEN)); 

    // println!("Reading current loop value");
    let old_set: Vec<u16> = ctx.read_holding_registers(REG_N4IOA01_CURR_VAL, 1).await??;
    // println!("read_rsp: {read_rsp:?}");
    sleep(Duration::from_millis(250)).await;

    let out_ma_setting: u16 = (milliamps/0.01).round() as u16;
    // println!("writing val of : {milliamps:?} mA  -> {out_ma_setting:?}");
    ctx.write_single_register(REG_N4IOA01_CURR_VAL, out_ma_setting).await??;

    sleep(Duration::from_millis(500)).await;
    // println!("Reading new current loop value");
    let new_set: Vec<u16> = ctx.read_holding_registers(REG_N4IOA01_CURR_VAL, 1).await??;
    // println!("420 src values: {read_rsp:?}");
    println!(" 420src old: {old_set:?} new: {out_ma_setting:?} set: {new_set:?}");
    Ok(())
}

/**
 * Set the output drive current of the precision current source 
 */
async fn set_precision_current_drive(ctx: &mut tokio_modbus::client::Context, milliamps: f32) -> Result<(), Box<dyn std::error::Error>> 
{
    const REG_ADDR_DRIVE_MILLIAMPS: u16  = 0x10;
    const REG_ADDR_MONITOR_MILLIAMPS: u16  = 0x11;

    // println!("set_precision_current_drive: {milliamps:?} mA");

    ctx.set_slave(Slave(NODEID_PREC_CURR_SRC)); 
    // println!("Reading existing drive current value");
    let read_rsp: Vec<u16> = ctx.read_holding_registers(REG_ADDR_DRIVE_MILLIAMPS, 1).await??;
    // println!("existing current read_rsp: {read_rsp:?}");
    let original_ma_val = read_rsp[0];

    let out_ma_setting: u16 = (milliamps/0.1).round() as u16;
    // println!("writing val of : {}", out_ma_setting);
    ctx.write_single_register(REG_ADDR_DRIVE_MILLIAMPS, out_ma_setting).await??;
    let read_rsp: Vec<u16> = ctx.read_holding_registers(REG_ADDR_DRIVE_MILLIAMPS, 1).await??;
    let new_ma_val = read_rsp[0];
    // println!(" prec old: {original_ma_val:?} new: {new_ma_val:?}");

    sleep(Duration::from_millis(250)).await; // TODO sufficient settling time?

    let read_rsp: Vec<u16> = ctx.read_holding_registers(REG_ADDR_MONITOR_MILLIAMPS, 1).await??;
    let actual_ma = (read_rsp[0] as f32)/10.0;// precision is 0.1 mA
    println!(" prec old: {original_ma_val:?} new: {new_ma_val:?} measured: {actual_ma:?}");

    // println!(" prec measured: {actual_ma:?} mA");

    Ok(())
}



/**
 * Verify that all the modules we expect to be connected to the RS-485 Modbus are,
 * in fact, connected.
 */
async fn enumerate_required_modules(ctx: &mut tokio_modbus::client::Context) -> Result<(), Box<dyn std::error::Error>> 
{
    ping_one_modbus_node_id(ctx, NODEID_DUAL_TK, REG_NODEID_TK).await?;
    ping_one_modbus_node_id(ctx,NODEID_N4IOA01_CURR_GEN, REG_NODEID_N4IOA01).await?;
    // ping_one_modbus_node_id(ctx,NODEID_N4VIA02_IV_ADC, REG_NODEID_N4VIA02).await?;
    ping_one_modbus_node_id(ctx,NODEID_PREC_CURR_SRC, REG_NODEID_PREC_CURR).await?;
    ping_one_modbus_node_id(ctx, NODEID_YKDAQ1402_IV_ADC, REG_NODEID_YKDAQ1402_IV_ADC).await?;
    ping_one_modbus_node_id(ctx,NODEID_N4AIA04_IV_ADC, REG_NODEID_N4AIA04).await?;

    
    Ok(())
}

#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    use tokio_serial::SerialStream;

    use tokio_modbus::prelude::*;

    // actual usb device ID of one USB-RS485 adapter
    let tty_path = "/dev/cu.usbserial-BG02SI88";
    let baud_rate = 9600;

    // Via TCP server bridge (a Wifi bridge on our local network)
    let socket_addr: std::net::SocketAddr = "10.0.1.151:502".parse()?;

    println!("Connecting to: '{socket_addr:?}'");
    let mut ctx: client::Context = tcp::connect(socket_addr).await?;
    enumerate_required_modules(&mut ctx).await?;

    let mut cur_target_milliamps = 4.0f32;
    loop { 
        set_current_loop_drive(&mut ctx, cur_target_milliamps).await?;
        sleep(Duration::from_millis(250));
        // read_multimeter(&mut ctx).await?;
        read_420_iv_adc(&mut ctx).await?;
        
        sleep(Duration::from_millis(500)).await;
        read_dual_tk_temps(&mut ctx).await?;
        
        sleep(Duration::from_millis(500)).await;
        read_electrode_pair_iv(&mut ctx).await?;

        sleep(Duration::from_millis(100)).await;
        set_precision_current_drive(&mut ctx, cur_target_milliamps).await?;
        
        cur_target_milliamps += 1.0;
        if cur_target_milliamps > 20.0 { cur_target_milliamps = 0.0; }

        // Let the bus & current settle before connecting to a different node
        sleep(Duration::from_millis(1000)).await;
        
        // TODO handle events that would lead to shutting down eg current source
    }


    ctx.disconnect().await?;

    Ok(())
}



