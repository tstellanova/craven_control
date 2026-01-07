#![allow(unused)]

use std::time::Duration;
use tokio::time::sleep;
// use tokio_modbus::client::Reader;
// use tokio_modbus::Result;
// use tokio_modbus::client::Client;

use tokio_modbus::client::{Client, Reader, Writer};

    
/// # Modbus node address assignments
///
/// | Address | Description |
/// |------|----------|
/// | 0x1F  | Current-Voltage 2 channel ADC   |
/// | 0x2F  | Precision current source (0-1000 mA)   |
/// | 0x3F  | Dual Type-K Thermocouple Reader |
///
/// 
/// 

const NODEID_DEFAULT: u8 = 0x01; // The Modbus node ID that most devices default to
const NODEID_IV_ADC: u8 = 0x1F;
const NODEID_PREC_CURR_SRC: u8 =  0x2F;
const NODEID_DUAL_TK: u8 = 0x3F;
const NODEID_PYRO_CURR_GEN: u8 = 0x4F; 
const NODEID_QUAD_RELAY: u8 = 0x5F;
const NODEID_MAX: u8 = 0x7F;

/// Register addresses
const REG_NODEID_IV_START: u16 = 0x00;
const REG_NODEID_TK:u16 = 0x20; // TK
const REG_NODEID_IV:u16 = 0x40; // IV
const REG_NODEID_PREC_CURR:u16 = 0x00; // Precision current source YK-PVCCS0100/YK-PVCC1000
const REG_NODEID_PYRO_CURR_GEN:u16 = 0x04; // Taidacent-B0B7HLZ6B4 node ID register

// --- TODO example of overwriting the dual TK address:
    // let first_node: Slave = Slave(0x01);
    // let port: SerialStream = SerialStream::open(&builder).unwrap();
    // let mut ctx = rtu::attach_slave(port, first_node);

    // const DEVICE_ADDRESS_REG:u16 = 0x20;
    // println!("Connecting first... ");
    // let read_rsp: Vec<u16> = ctx.read_holding_registers(DEVICE_ADDRESS_REG, 3).await??;
    // println!(" read_rsp: {:?}", read_rsp);

    // const NEW_DEVICE_ADDR:u8 = 0x3F;
    // let w_resp = ctx.write_single_register(DEVICE_ADDRESS_REG, NEW_DEVICE_ADDR.into()).await?;
    // println!(" w_resp: {:?}", w_resp);

    // println!("Disconnecting first");
    // ctx.disconnect().await?;


    // println!("Connecting second... ");
    // let second_node: Slave = Slave(NEW_DEVICE_ADDR);
    // let port = SerialStream::open(&builder).unwrap();
    // let mut ctx = rtu::attach_slave(port, second_node);
    // let read_rsp: Vec<u16> = ctx.read_holding_registers(DEVICE_ADDRESS_REG, 3).await??;
    // println!(" read_rsp: {:?}", read_rsp);

// --- TODO example of overwriting the IV ADC address:
    // const DEVICE_ADDRESS_REG:u16 = 0x40; // IV
    // println!("Connecting first... ");
    // let read_rsp: Vec<u16> = ctx.read_holding_registers(DEVICE_ADDRESS_REG, 3).await??;
    // println!(" read_rsp: {:?}", read_rsp);
    // const NEW_DEVICE_ADDR:u8 = NODEID_ADDR_IV_ADC;
    // let w_resp = ctx.write_single_register(DEVICE_ADDRESS_REG, NEW_DEVICE_ADDR.into()).await?;
    // println!(" w_resp: {:?}", w_resp);

    // println!("Disconnecting first");
    // ctx.disconnect().await?;

    // println!("Connecting second... ");
    // let second_node: Slave = Slave(NEW_DEVICE_ADDR);
    // let port = SerialStream::open(&builder).unwrap();
    // let mut ctx = rtu::attach_slave(port, second_node);
    // let read_rsp: Vec<u16> = ctx.read_holding_registers(DEVICE_ADDRESS_REG, 3).await??;
    // println!(" read_rsp: {:?}", read_rsp);

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
    let w_rsp = ctx.write_single_register(REG_ADDR_DRIVE_MILLIAMPS, out_ma_setting).await?;
    println!("w_rsp: {w_rsp:?}");
    let read_rsp: Vec<u16> = ctx.read_holding_registers(REG_ADDR_DRIVE_MILLIAMPS, 1).await??;
    println!("new val read_rsp: {read_rsp:?}");

    Ok(())
}

/**
 * Set the output drive current of the precision current source
 */
async fn set_precision_current_drive(ctx: &mut tokio_modbus::client::Context, milliamps: f32) -> Result<(), Box<dyn std::error::Error>> 
{
    const REG_ADDR_DRIVE_MILLIAMPS: u16  = 0x10;
    const REG_ADDR_MONITOR_MILLIAMPS: u16  = 0x11;

    println!("set_precision_current_drive: {milliamps:?} mA");

    // println!("Reading existing drive current value");
    let read_rsp: Vec<u16> = ctx.read_holding_registers(REG_ADDR_DRIVE_MILLIAMPS, 1).await??;
    println!("existing current read_rsp: {read_rsp:?}");
    // let original_ma_val = read_rsp[0];

    let out_ma_setting: u16 = (milliamps/0.1).round() as u16;
    println!("writing val of : {}", out_ma_setting);
    let w_rsp = ctx.write_single_register(REG_ADDR_DRIVE_MILLIAMPS, out_ma_setting).await?;
    println!("w_rsp: {w_rsp:?}");
    let read_rsp: Vec<u16> = ctx.read_holding_registers(REG_ADDR_DRIVE_MILLIAMPS, 1).await??;
    println!("new val read_rsp: {read_rsp:?}");

    sleep(Duration::from_millis(500)).await;

    let read_rsp: Vec<u16> = ctx.read_holding_registers(REG_ADDR_MONITOR_MILLIAMPS, 1).await??;
    println!("actual current mA: {read_rsp:?}");

    Ok(())
}

async fn set_one_modbus_node_id(tty_path: &str, baud_rate: u32,  reg_node_id: u16, old_node_id: u8, new_node_id: u8) 
    -> Result<(), Box<dyn std::error::Error>>
{
    let builder = tokio_serial::new(tty_path, baud_rate);
    let first_node = tokio_modbus::Slave(old_node_id);
    let port = tokio_serial::SerialStream::open(&builder).unwrap();
    let mut ctx = tokio_modbus::prelude::rtu::attach_slave(port, first_node);

    println!("Reading node ID from node {old_node_id:?} first... ");
    let read_rsp: Vec<u16> = ctx.read_holding_registers(reg_node_id, 1).await??;
    println!("> read_rsp: {:?}", read_rsp);

    let existing_node_id = read_rsp[4] as u8;
    if existing_node_id != old_node_id {
        if existing_node_id != new_node_id {
            // println!("Node ID {old_node_id:?} reports node ID of {existing_node_id:?}");
            panic!("Couldn't verify the old node ID");
        }
    }
 
    if existing_node_id == old_node_id {
        println!("writing {new_node_id:?} to reg {reg_node_id:?}");
        let w_resp = ctx.write_single_register(reg_node_id, new_node_id.into()).await?;
        println!("> w_resp: {:?}", w_resp);
    }

    println!("Disconnecting from old node id: {old_node_id:?}");
    ctx.disconnect().await?;
    

    sleep(Duration::from_millis(1000)).await;

    println!("Connecting to new node id: {new_node_id:?}");
    let second_node= tokio_modbus::Slave(new_node_id);
    let port = tokio_serial::SerialStream::open(&builder).unwrap();
    let mut ctx = tokio_modbus::prelude::rtu::attach_slave(port, second_node);
    let read_rsp: Vec<u16> = ctx.read_holding_registers(reg_node_id, 1).await??;
    println!("> read_rsp: {:?}", read_rsp);
        
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

    // example of setting the Modbus node ID for a particular device
    // set_one_modbus_node_id(tty_path, baud_rate, REG_NODEID_PREC_CURR, NODEID_DEFAULT, NODEID_PREC_CURR_SRC).await?;
    // set_one_modbus_node_id(tty_path, baud_rate, REG_NODEID_PYRO_CURR_GEN, NODEID_DEFAULT, NODEID_PYRO_CURR_GEN).await?;

    let builder = tokio_serial::new(tty_path, baud_rate);
    let first_node: Slave = Slave(NODEID_PREC_CURR_SRC);//NODEID_PYRO_CURR_GEN); //NODEID_PREC_CURR_SRC); //TODO: NODEID_ADDR_PREC_CURR_SRC); // NODEID_DEFAULT); // or eg NODEID_DUAL_TK
    let port: SerialStream = SerialStream::open(&builder).unwrap();
    let mut ctx: client::Context = rtu::attach_slave(port, first_node);

    // set_pyro_current_loop_drive(&mut ctx, 4.3).await?;

    set_precision_current_drive(&mut ctx, 1.0).await?;
    set_precision_current_drive(&mut ctx, 20.0).await?;
    set_precision_current_drive(&mut ctx, 1.0).await?;

    // println!("Connecting first... ");
    // let read_rsp: Vec<u16> = ctx.read_holding_registers(DEVICE_ADDRESS_REG, 3).await??;
    // println!(" read_rsp: {:?}", read_rsp);
    // const NEW_DEVICE_ADDR:u8 = NODEID_ADDR_IV_ADC;
    // let w_resp = ctx.write_single_register(DEVICE_ADDRESS_REG, NEW_DEVICE_ADDR.into()).await?;
    // println!(" w_resp: {:?}", w_resp);

    // println!("Disconnecting first");
    // ctx.disconnect().await?;

    // println!("Connecting second... ");
    // let second_node: Slave = Slave(NEW_DEVICE_ADDR);
    // let port = SerialStream::open(&builder).unwrap();
    // let mut ctx = rtu::attach_slave(port, second_node);
    // let read_rsp: Vec<u16> = ctx.read_holding_registers(DEVICE_ADDRESS_REG, 3).await??;
    // println!(" read_rsp: {:?}", read_rsp);

    // ----- TODO example of reading all the IV ADC registers
    // let read_rsp: Vec<u16> = ctx.read_holding_registers(REG_ADDR_IV_START, 4).await??;
    // println!(" read_rsp: {:?}", read_rsp);

    // // ----- TODO example of reading all the dual TK registers:
    // let read_rsp: Vec<u16> = ctx.read_holding_registers(0x00, 2).await??;
    // println!(" read_rsp: {:?}", read_rsp);

    // // let read_state = client.read_03(1, 0x10, 2).await;
    // let read_rsp: Vec<u16> = ctx.read_holding_registers(0x10, 2).await??;
    // println!(" read_rsp: {:?}", read_rsp);

    // // let read_cfg = client.read_03(1, 0x20, 3).await;
    // let read_rsp: Vec<u16> = ctx.read_holding_registers(0x20, 3).await??;
    // println!(" read_rsp: {:?}", read_rsp);
    
    // ----- TODO currently we're reading and writing the value of a 4-20 mA current loop controller
    // let milliamp_reg_addr: u16 = 1;

    // println!("Reading a sensor value");
    // let read_rsp: Vec<u16> = ctx.read_holding_registers(milliamp_reg_addr, 1).await??;
    // println!("read_rsp: {read_rsp:?}");
    // let original_ma_val = read_rsp[0];

    // let out_ma_setting = 80;
    // println!("writing val of : {}", out_ma_setting);
    // let w_rsp = ctx.write_single_register(milliamp_reg_addr, out_ma_setting).await?;
    // println!("w_rsp: {w_rsp:?}");
    // let read_rsp: Vec<u16> = ctx.read_holding_registers(milliamp_reg_addr, 1).await??;
    // println!("new val read_rsp: {read_rsp:?}");

    // sleep(Duration::from_millis(500)).await;

    // let out_ma_setting = original_ma_val;
    // println!("Resetting val of : {}", out_ma_setting);
    // let w_rsp = ctx.write_single_register(milliamp_reg_addr, out_ma_setting).await?;
    // println!("w_rsp: {w_rsp:?}");
    // let read_rsp = ctx.read_holding_registers(milliamp_reg_addr, 1).await??;
    // println!("read_rsp: {read_rsp:?}");
    // ----- TODO


    // println!("Disconnecting");
    // ctx.disconnect().await?;

    Ok(())
}



