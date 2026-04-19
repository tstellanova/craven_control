#![allow(unused)]

use std::time::Duration;
use tokio::time::sleep;

use tokio_modbus::prelude::*;

use tokio_modbus::client::{Client, Reader, Writer};
use tokio_serial::SerialStream;

    
use craven_control::*;




async fn set_one_modbus_node_baud(tty_path: &str, node_id: u8, reg_baud_set: u16, old_baud: u32, new_baud: u32, reg_val: u16) 
    -> Result<(), Box<dyn std::error::Error>>
{
    let builder = tokio_serial::new(tty_path, old_baud);
    let mut ctx = rtu::attach_slave(SerialStream::open(&builder).unwrap(), Slave(node_id));

    println!("Read from node {node_id:X?}, reg 0x{reg_baud_set:X?} at baud {old_baud:?} ... ");
    if node_id == NODEID_WA8TAI_IV_ADC {
        let read_rsp: Vec<u16> = ctx.read_input_registers(reg_baud_set, 1).await??;
        println!("> read_rsp: {:?}", read_rsp);
        let existing_val = read_rsp[0] ;
        println!("Node {node_id:X?} existing val {existing_val}");
    }
    else {
        let read_rsp: Vec<u16> = ctx.read_holding_registers(reg_baud_set, 1).await??;
        println!("> read_rsp: {:?}", read_rsp);
        let existing_val = read_rsp[0] ;
        println!("Node {node_id:X?} existing val {existing_val}");
    }

    sleep(Duration::from_millis(500)).await;
    println!("Writing new val {reg_val} to reg {reg_baud_set:X?}");
    let w_resp = ctx.write_single_register(reg_baud_set, reg_val.into()).await?;
    if w_resp.is_err() {
        eprintln!("> w_resp: {:?}", w_resp);
    }

    println!("Disconnecting from node id: {node_id:?}");
    ctx.disconnect().await?;
    
    //Wait for device to reset, if necessary
    sleep(Duration::from_millis(5000)).await;

    println!("Reconnecting to node id: {node_id:?} at {new_baud:?} baud");
    let builder = tokio_serial::new(tty_path, new_baud);
    let mut ctx = rtu::attach_slave(SerialStream::open(&builder).unwrap(), Slave(node_id));

    sleep(Duration::from_millis(500)).await;
    let read_rsp: Vec<u16> = ctx.read_holding_registers(reg_baud_set, 1).await??;
    let latest_reg_val = read_rsp[0] ;
    println!("> latest_reg_val: {latest_reg_val:?}");

    if latest_reg_val != reg_val {
        eprintln!("latest_reg_val {latest_reg_val:?} != {reg_val:?}");
    }

    if node_id == NODEID_YKPVCCS010_CURR_SRC {
        // flush the configuration parameter change to the node's persistent storage 
        println!("persisting YKPVCCS node configuration");
        let flush_resp = ctx.write_single_register(REG_SAVE_CFG_YKPVCCS010_CURR_SRC, 1).await?;
        if flush_resp.is_err() {
            eprintln!("> flush_resp: {:?}", flush_resp);
        }
    }

    
    println!("Disconnecting from node id: {node_id:?}");
    ctx.disconnect().await?;

    Ok(())
}

async fn enumerate_required_modules(ctx: &mut tokio_modbus::client::Context) -> Result<(), Box<dyn std::error::Error>> 
{
    // measures dual type K thermocouple signal
    ping_one_modbus_node_id(ctx, NODEID_YKKTC1202_DUAL_TK, REG_NODEID_YKKTC1202_DUAL_TK).await?;
 
    // // supplies current to the electrode probe
    ping_one_modbus_node_id(ctx,NODEID_YKPVCCS010_CURR_SRC, REG_NODEID_YKPVCCS010_CURR_SRC).await?;

    // controls furnace on/off
    ping_one_modbus_node_id(ctx, NODEID_R4DVI04_QRELAY_ADC, REG_NODEID_R4DVI04).await?;

    // measures voltage and current across the electrodes
    ping_one_modbus_node_id(ctx,NODEID_WA8TAI_IV_ADC, REG_NODEID_WAVESHARE_V2).await?;

    Ok(())
}

#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    use tokio_serial::SerialStream;

    use tokio_modbus::prelude::*;

    // actual usb device ID of one USB-RS485 adapter
    let tty_path = "/dev/cu.usbserial-BG02SI88";

    // Initial baud rate
    let initial_baud_rate = 9600;
    let final_baud_rate = 115200;

    // Examples of changing the Modbus node baud rate for various devices 
    set_one_modbus_node_baud(tty_path, NODEID_YKKTC1202_DUAL_TK, REG_YKKTC1202_BAUD, initial_baud_rate, final_baud_rate, 6).await?;
    set_one_modbus_node_baud(tty_path, NODEID_WA8TAI_IV_ADC, REG_WA8TAI_BAUD, initial_baud_rate, final_baud_rate, 5).await?;
    set_one_modbus_node_baud(tty_path, NODEID_YKPVCCS010_CURR_SRC, REG_YKPVCCS_BAUD, initial_baud_rate, final_baud_rate, 6).await?;
    set_one_modbus_node_baud(tty_path, NODEID_R4DVI04_QRELAY_ADC, REG_R4DVI04_BAUD, initial_baud_rate, final_baud_rate, 7).await?;

    let builder = tokio_serial::new(tty_path, final_baud_rate);
    let mut ctx = rtu::attach_slave(SerialStream::open(&builder).unwrap(), Slave(254));
    enumerate_required_modules(&mut ctx).await?;

    Ok(())
}



