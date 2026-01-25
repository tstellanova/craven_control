#![allow(unused)]

use std::time::Duration;
use tokio::time::sleep;
// use tokio_modbus::client::Reader;
// use tokio_modbus::Result;
// use tokio_modbus::client::Client;

use tokio_modbus::prelude::*;

use tokio_modbus::client::{Client, Reader, Writer};
// use byteorder::{BigEndian, ByteOrder};

    
use craven_control::*;



// /// Combine two u16 registers into am f32
// fn registers_to_f32(registers: &[u16], offset: usize) -> f32 {
//     let high = registers[offset] as u32;
//     let low = registers[offset + 1] as u32;
//     let combined = (high << 16) | low;
    
//     // Convert u32 to f32
//     f32::from_bits(combined)
// }

/**
 * Set the pyro simulator current loop controller (4-20 mA) current value
 * We're currently using a Taidacent-B0B7HLZ6B4 4-20 mA signal generator
 */
async fn set_pyro_current_loop_drive(ctx: &mut tokio_modbus::client::Context, milliamps: f32) -> Result<(), Box<dyn std::error::Error>> 
{
    const REG_ADDR_DRIVE_MILLIAMPS: u16  = 0x01;

    println!("Reading current loop value");
    let read_rsp: Vec<u16> = ctx.read_holding_registers(0x00, 20).await??;
    println!("read_rsp: {read_rsp:?}");

    let out_ma_setting: u16 = (milliamps/0.1).round() as u16;
    println!("writing val of : {milliamps:?} mA  -> {out_ma_setting:?}");
    ctx.write_single_register(REG_ADDR_DRIVE_MILLIAMPS, out_ma_setting).await??;
    let read_rsp: Vec<u16> = ctx.read_holding_registers(REG_ADDR_DRIVE_MILLIAMPS, 1).await??;
    // println!("new val read_rsp: {read_rsp:?}");

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

    // println!("Reading existing drive current value");
    let read_rsp: Vec<u16> = ctx.read_holding_registers(REG_ADDR_DRIVE_MILLIAMPS, 1).await??;
    // println!("existing current read_rsp: {read_rsp:?}");
    let original_ma_val = read_rsp[0];

    let out_ma_setting: u16 = (milliamps/0.1).round() as u16;
    // println!("writing val of : {}", out_ma_setting);
    ctx.write_single_register(REG_ADDR_DRIVE_MILLIAMPS, out_ma_setting).await??;
    let read_rsp: Vec<u16> = ctx.read_holding_registers(REG_ADDR_DRIVE_MILLIAMPS, 1).await??;
    let new_ma_val = read_rsp[0];
    println!(" prec old: {original_ma_val:?} new: {new_ma_val:?}");

    sleep(Duration::from_millis(250)).await; // TODO sufficient settling time?

    let read_rsp: Vec<u16> = ctx.read_holding_registers(REG_ADDR_MONITOR_MILLIAMPS, 1).await??;
    let actual_ma = (read_rsp[0] as f32)/10.0;// precision is 0.1 mA
    println!(" prec measured: {actual_ma:?} mA");

    Ok(())
}

async fn set_one_modbus_node_id(tty_path: &str, baud_rate: u32,  reg_node_id: u16, old_node_id: u8, new_node_id: u8) 
    -> Result<(), Box<dyn std::error::Error>>
{
    let builder = tokio_serial::new(tty_path, baud_rate);
    let first_node = tokio_modbus::Slave(old_node_id);
    let port = tokio_serial::SerialStream::open(&builder).unwrap();
    let mut ctx = tokio_modbus::prelude::rtu::attach_slave(port, first_node);

    println!("Read existing node ID from node {old_node_id:?} first... ");
    let read_rsp: Vec<u16> = ctx.read_holding_registers(reg_node_id, 1).await??;
    println!("> read_rsp: {:?}", read_rsp);

    let existing_node_id = read_rsp[0] as u8;
    if existing_node_id != old_node_id {
        if existing_node_id != new_node_id {
            println!("Node ID {old_node_id:?} reports node ID of {existing_node_id:?}");
            panic!("Couldn't verify the old node ID");
        }
    }
 
    if existing_node_id == old_node_id {
        println!("writing new node ID {new_node_id:?} to reg {reg_node_id:?}");
        let w_resp = ctx.write_single_register(reg_node_id, new_node_id.into()).await?;
        if w_resp.is_err() {
            eprintln!("> w_resp: {:?}", w_resp);
        }
    }

    println!("Disconnecting from old node id: {old_node_id:?}");
    ctx.disconnect().await?;
    
    //Wait for device to reset, if necessary
    sleep(Duration::from_millis(1000)).await;

    println!("Connecting to new node id: {new_node_id:?}");
    let second_node= tokio_modbus::Slave(new_node_id);
    let port = tokio_serial::SerialStream::open(&builder).unwrap();
    let mut ctx = tokio_modbus::prelude::rtu::attach_slave(port, second_node);
    let read_rsp: Vec<u16> = ctx.read_holding_registers(reg_node_id, 1).await??;
    println!("> read_rsp: {:?}", read_rsp);
    let latest_node_id = read_rsp[0] as u8;
    if latest_node_id != new_node_id {
        eprintln!("latest_node_id {latest_node_id:?} != {new_node_id:?}");
    }

    if new_node_id == NODEID_PREC_CURR_SRC {
        // flush the configuration parameter change to the node's persistent storage 
        println!("persisting node configuration");
        let flush_resp = ctx.write_single_register(REG_SAVE_CFG_PREC_CURR, 1).await?;
        if flush_resp.is_err() {
            eprintln!("> flush_resp: {:?}", flush_resp);
        }
    }

    println!("Disconnecting from new node id: {new_node_id:?}");
    ctx.disconnect().await?;

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


    // Examples of setting the Modbus node ID for various devices -- need only be done once
    // set_one_modbus_node_id(tty_path, baud_rate, REG_NODEID_IV, NODEID_DEFAULT, NODEID_IV_ADC).await?;
    // set_one_modbus_node_id(tty_path, baud_rate, REG_NODEID_PREC_CURR, NODEID_DEFAULT, NODEID_PREC_CURR_SRC).await?;
    // set_one_modbus_node_id(tty_path, baud_rate, REG_NODEID_TK, NODEID_DEFAULT, NODEID_DUAL_TK).await?;
    // set_one_modbus_node_id(tty_path, baud_rate, REG_NODEID_PYRO_CURR_GEN, NODEID_DEFAULT, NODEID_PYRO_CURR_GEN).await?;

    let builder: tokio_serial::SerialPortBuilder = tokio_serial::new(tty_path, baud_rate);
    let mut ctx: client::Context = rtu::attach(SerialStream::open(&builder).unwrap());

    // println!("Connecting to: '{socket_addr:?}'");
    // let mut ctx: client::Context = tcp::connect(socket_addr).await?;
    
    let mut cur_target_milliamps = 1.0f32;
    loop { 
        println!("Check IV ADC... ");
        ctx.set_slave(Slave(NODEID_N4VIA02_IV_ADC)); //NODEID_IV_ADC));
        // let mut ctx_iv_adc: client::Context = rtu::attach_slave(SerialStream::open(&builder).unwrap(), Slave(NODEID_IV_ADC));
        let read_rsp: Vec<u16> = ctx.read_holding_registers(REG_CFG_N4VIA02, 6).await??;
        println!(" N4VIA02 CFG ({REG_NODEID_N4VIA02:?})[6]: {read_rsp:?}");
        let read_rsp: Vec<u16> = ctx.read_holding_registers(REG_NODEID_N4VIA02, 1).await??;
        println!(" N4VIA02 NODE ID ({REG_NODEID_N4VIA02:?})[1]: {read_rsp:?}");
        let milliamp_vals: Vec<u16> = ctx.read_holding_registers(REG_N4VIA02_CURR_VALS, 2).await??;
        println!(" N4VIA02 mA VALS ({REG_N4VIA02_CURR_VALS:?})[2]: {milliamp_vals:?}");
        let voltage_vals: Vec<u16> = ctx.read_holding_registers(REG_N4VIA02_VOLT_VALS, 2).await??;
        println!(" N4VIA02 V VALS ({REG_N4VIA02_VOLT_VALS:?})[2]: {voltage_vals:?}");
        
        let scaled_ch0_volts = (voltage_vals[0] as f32)/100.0;
        println!(" scaled_ch0_volts: {scaled_ch0_volts:?}");

        let ch1_milliamps = milliamp_vals[1];
        println!(" ch1_milliamps: {ch1_milliamps:?}");

        // let ch1_value = registers_to_i32(&iv_adc_vals, 0);
        // let verified_volts = (ch1_value as f32) / 10000.0; // resolution is 0.1 mV for 10V range
        // let ch2_value = registers_to_i32(&iv_adc_vals, 2);
        // let verified_milliamps = (ch2_value as f32)/ 10.0; // resolution is 0.1 mA for 5A range
        // println!(" IV ADC ch1_value: {ch1_value:?} = {verified_volts:?} V");
        // println!(" IV ADC ch2_value: {ch2_value:?} = {verified_milliamps:?} mA");

        // Let the bus settle before connecting to a different node
        sleep(Duration::from_millis(1000)).await;

        /* 
        println!("Check Dual TK... ");
        ctx.set_slave(Slave(NODEID_DUAL_TK));
        // let read_rsp: Vec<u16> = ctx.read_holding_registers(REG_NODEID_TK, 1).await??;
        // println!(" Dual TK node ID: {:?}", read_rsp);

        let cfg_rsp: Vec<u16> = ctx.read_holding_registers(0x20, 3).await??;
        println!(" 0x20 cfg_rsp: {:?}", cfg_rsp);

        let ch_valid_rsp: Vec<u16> = ctx.read_holding_registers(0x10, 2).await??;
        // println!(" 0x10 ch_valid_rsp: {:?}", ch_valid_rsp);
        let ch1_tk_conn: bool = ch_valid_rsp[0] == 0; // 0: The thermocouple is connected, 1: The thermocouple is not connected
        let ch2_tk_conn: bool = ch_valid_rsp[1] == 0;

        // TODO use reg address consts
        // example of reading all the dual TK registers:
        let tk_resp: Vec<u16> = ctx.read_holding_registers(0x00, 2).await??;
        // println!(" 0x00 temp_rsp: {:?}", tk_resp);
        let ch1_tk_val: f32 = (tk_resp[0] as f32) / 10.0; // resolution is 0.1 °C
        let ch2_tk_val: f32 = (tk_resp[1] as f32) / 10.0; // resolution is 0.1 °C

        if ch1_tk_conn {
            println!(" TK1 valid: {ch1_tk_val:?}°C");
        }
        if ch2_tk_conn {
            println!(" TK2 valid: {ch2_tk_val:?}°C");
        }
        
        // Let the bus settle before connecting to a different node
        sleep(Duration::from_millis(100)).await;

        println!("Check Prec Curr ... ");
        ctx.set_slave(Slave(NODEID_PREC_CURR_SRC)); 

        set_precision_current_drive(&mut ctx, cur_target_milliamps).await?;
        cur_target_milliamps += 1.0;
        if cur_target_milliamps > 15.0 { cur_target_milliamps = 0.1; }

        // Let the bus & current settle before connecting to a different node
        sleep(Duration::from_millis(250)).await;
        */

        // TODO handle events that would lead to shutting down eg current source
    }


    ctx.disconnect().await?;

    Ok(())
}



