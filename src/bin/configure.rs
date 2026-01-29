#![allow(unused)]

use std::time::Duration;
use tokio::time::sleep;

use tokio_modbus::prelude::*;

use tokio_modbus::client::{Client, Reader, Writer};
use tokio_serial::SerialStream;

    
use craven_control::*;






async fn set_one_modbus_node_id(tty_path: &str, baud_rate: u32,  reg_node_id: u16, old_node_id: u8, new_node_id: u8) 
    -> Result<(), Box<dyn std::error::Error>>
{
    let builder = tokio_serial::new(tty_path, baud_rate);
    let mut ctx = rtu::attach_slave(SerialStream::open(&builder).unwrap(), Slave(old_node_id));

    // read a block
    // println!("Read a block from 0x00 register  ");
    // let block_resp: Vec<u16> = ctx.read_holding_registers(0x00, 8).await??;
    // println!("> block_resp 0x00: {:?}", block_resp);

    println!("Read existing node ID from node {old_node_id:X?}, reg 0x{reg_node_id:X?} ... ");
    let read_rsp: Vec<u16> = ctx.read_holding_registers(reg_node_id, 1).await??;
    println!("> read_rsp: {:?}", read_rsp);

    let existing_node_id = read_rsp[0] as u8;
    if existing_node_id != old_node_id {
        if existing_node_id != new_node_id {
            println!("Node ID {old_node_id:X?} reports node ID of {existing_node_id:X?}");
            //TODO ? ctx.set_slave(Slave(existing_node_id));
            //panic!("Couldn't verify the old node ID");
        }
        else {
            println!("existing_node_id == new_node_id... done");
        }
    }
 
    if existing_node_id != new_node_id {
        println!("writing new node ID {new_node_id:X?} to reg {reg_node_id:X?}");
        if new_node_id != NODEID_QUAD_RELAY {
            let w_resp = ctx.write_single_register(reg_node_id, new_node_id.into()).await?;
            if w_resp.is_err() {
                eprintln!("> w_resp: {:?}", w_resp);
            }
        } 
        else {
            let w_resp = ctx.write_multiple_registers(reg_node_id, &[new_node_id as u16]).await?;
            if w_resp.is_err() {
                eprintln!("> w_resp: {:?}", w_resp);
            }
        }
    }

    // println!("Disconnecting from old node id: {old_node_id:?}");
    // ctx.disconnect().await?;
    
    //Wait for device to reset, if necessary
    sleep(Duration::from_millis(1000)).await;

    println!("Connecting to new node id: {new_node_id:?}");
    // let mut ctx = rtu::attach_slave(SerialStream::open(&builder).unwrap(), Slave(new_node_id));
    ctx.set_slave(Slave(new_node_id));
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
    // else {
    //     let read_rsp: Vec<u16> = ctx.read_holding_registers(0x00, 16).await??;
    //     println!("> read_rsp: {:?}", read_rsp);
    // }

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
    set_one_modbus_node_id(tty_path, baud_rate, REG_NODEID_YKDAQ1402_IV_ADC, NODEID_DEFAULT, NODEID_YKDAQ1402_IV_ADC).await?;
    // set_one_modbus_node_id(tty_path, baud_rate, REG_NODEID_PREC_CURR, NODEID_DEFAULT, NODEID_PREC_CURR_SRC).await?;
    // set_one_modbus_node_id(tty_path, baud_rate, REG_NODEID_TK, NODEID_DEFAULT, NODEID_DUAL_TK).await?;
    // set_one_modbus_node_id(tty_path, baud_rate, REG_NODEID_PYRO_SIM, NODEID_DEFAULT, NODEID_PYRO_SIM).await?;
    // set_one_modbus_node_id(tty_path, baud_rate, REG_NODEID_N4AIA04, NODEID_DEFAULT, NODEID_N4AIA04_ADC).await?;
    // set_one_modbus_node_id(tty_path, baud_rate, REG_NODEID_QUAD_RELAY, 0x00, NODEID_QUAD_RELAY).await?;
    // set_one_modbus_node_id(tty_path, baud_rate, REG_NODEID_QUAD_RELAY, 0x00, NODEID_QUAD_RELAY).await?;
    // set_one_modbus_node_id(tty_path, baud_rate, REG_NODEID_N4VIA02, NODEID_DEFAULT, NODEID_N4VIA02_IV_ADC).await?;
    // set_one_modbus_node_id(tty_path, baud_rate, REG_NODEID_N4IOA01, NODEID_DEFAULT, NODEID_N4IOA01_CURR_GEN).await?;

    // the 3--4th byte of the transmitted frame represents the relay address,the relay 1-relay 8 address are respectively:
    // relay 0x0000,0x0001,0x0002,0x0003,0x0004,0x0005,0x0006,0x0007.
    // eg Turn on the relay no.2 (manual mode) Send: FF 05 00 01 FF 00 C8 24


    Ok(())
}



