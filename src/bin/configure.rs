#![allow(unused)]

use std::time::Duration;
use tokio::time::sleep;

use tokio_modbus::prelude::*;

use tokio_modbus::client::{Client, Reader, Writer};
use tokio_serial::SerialStream;
// use byteorder::{BigEndian, ByteOrder};

    
/// # Modbus node address assignments
///
/// | Address | Description |
/// |-------|----------|
/// | 0x01  | Reserved for Modbus default node ID |
/// | 0x1F  | Current-Voltage 2 channel ADC   |
/// | 0x2F  | Precision current source (0-1000 mA)   |
/// | 0x3F  | Dual Type-K Thermocouple Reader |
///
/// 
/// 

/// Modbus node IDs
const NODEID_DEFAULT: u8 = 0x01; // The Modbus node ID that most devices default to
const NODEID_N4AIA04_ADC: u8 = 0x1E;
const NODEID_IV_ADC: u8 = 0x1F;
const NODEID_PREC_CURR_SRC: u8 = 0x2F;
const NODEID_DUAL_TK: u8 = 0x3F;
const NODEID_PYRO_SIM: u8 = 0x4F; // TODO switching to new current loop signal generator
const NODEID_QUAD_RELAY: u8 = 0x5F; // TODO not yet programmed into relay board
const NODEID_MAX: u8 = 0x7F;

/// Register addresses
const REG_NODEID_TK:u16 = 0x20; // dual Type-K thermocouple reader
const REG_NODEID_IV:u16 = 0x40; // 0-10 Volt, 0-5 Amp IV ADC
const REG_NODEID_N4AIA04:u16 = 0x0E; // N4AIA04 ADC with 0-20 mA and 5/10 Volt inputs
const REG_NODEID_PREC_CURR:u16 = 0x00; // Precision current source YK-PVCCS0100/YK-PVCC1000
const REG_NODEID_PYRO_SIM:u16 = 0x04; // TODO wrong? node ID may not be settable for Taidacent-B0B7HLZ6B4 

const REG_SAVE_CFG_PREC_CURR:u16 = 0x02; // Cause YK-PVCCS to persist its parameters.
const REG_IV_ADC_2CH_VALS: u16 = 0x00; // Where 2CH IV ADC stores read values


/**
 *  Combine two u16 registers into an i32
 * 
 */ 
fn registers_to_i32(registers: &[u16], offset: usize) -> i32 {
    let high = registers[offset] as i32;
    let low = registers[offset + 1] as i32;
    let combined = (high << 16) | low;
    combined
}



async fn set_one_modbus_node_id(tty_path: &str, baud_rate: u32,  reg_node_id: u16, old_node_id: u8, new_node_id: u8) 
    -> Result<(), Box<dyn std::error::Error>>
{
    let builder = tokio_serial::new(tty_path, baud_rate);
    let mut ctx = rtu::attach_slave(SerialStream::open(&builder).unwrap(), Slave(old_node_id));

    // read a block
    println!("Read a block from 0x00 register  ");
    let block_resp: Vec<u16> = ctx.read_holding_registers(0x00, 16).await??;
    println!("> block_resp 0x00: {:?}", block_resp);

    println!("Read existing node ID from node {old_node_id:?} at reg 0x{reg_node_id:X?} ... ");
    let read_rsp: Vec<u16> = ctx.read_holding_registers(reg_node_id, 1).await??;
    println!("> read_rsp: {:?}", read_rsp);

    let existing_node_id = read_rsp[0] as u8;
    if existing_node_id != old_node_id {
        if existing_node_id != new_node_id {
            println!("Node ID {old_node_id:?} reports node ID of {existing_node_id:?}");
            //panic!("Couldn't verify the old node ID");
        }
        else {
            println!("")
        }
    }
 
    // if existing_node_id == old_node_id {
        println!("writing new node ID {new_node_id:?} to reg {reg_node_id:?}");
        let w_resp = ctx.write_single_register(reg_node_id, new_node_id.into()).await?;
        if w_resp.is_err() {
            eprintln!("> w_resp: {:?}", w_resp);
        }
        let read_rsp: Vec<u16> = ctx.read_holding_registers(0x00, 16).await??;
        println!("> after read_rsp: {:?}", read_rsp);
    // }

    println!("Disconnecting from old node id: {old_node_id:?}");
    ctx.disconnect().await?;
    
    //Wait for device to reset, if necessary
    sleep(Duration::from_millis(3000)).await;

    println!("Connecting to new node id: {new_node_id:?}");
    let mut ctx = rtu::attach_slave(SerialStream::open(&builder).unwrap(), Slave(new_node_id));
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
    else {
        let read_rsp: Vec<u16> = ctx.read_holding_registers(0x00, 16).await??;
        println!("> read_rsp: {:?}", read_rsp);
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


    // Examples of setting the Modbus node ID for various devices -- need only be done once
    // set_one_modbus_node_id(tty_path, baud_rate, REG_NODEID_IV, NODEID_DEFAULT, NODEID_IV_ADC).await?;
    // set_one_modbus_node_id(tty_path, baud_rate, REG_NODEID_PREC_CURR, NODEID_DEFAULT, NODEID_PREC_CURR_SRC).await?;
    // set_one_modbus_node_id(tty_path, baud_rate, REG_NODEID_TK, NODEID_DEFAULT, NODEID_DUAL_TK).await?;
    set_one_modbus_node_id(tty_path, baud_rate, REG_NODEID_PYRO_SIM, NODEID_DEFAULT, NODEID_PYRO_SIM).await?;
    // set_one_modbus_node_id(tty_path, baud_rate, REG_NODEID_N4AIA04, NODEID_DEFAULT, NODEID_N4AIA04_ADC).await?;



    Ok(())
}



