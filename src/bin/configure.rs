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

    println!("Read existing node ID from node {old_node_id:X?}, reg 0x{reg_node_id:X?} ... ");
    let read_rsp: Vec<u16> = ctx.read_holding_registers(reg_node_id, 1).await??;
    println!("> read_rsp: {:?}", read_rsp);

    let existing_node_id = read_rsp[0] as u8;
    if existing_node_id != old_node_id {
        if existing_node_id != new_node_id {
            println!("Node ID {old_node_id:X?} reports node ID of {existing_node_id:X?}");
            //TODO ? 
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
    
    //Wait for device to reset, if necessary
    sleep(Duration::from_millis(1000)).await;

    println!("Connecting to new node id: {new_node_id:X?}");
    // let mut ctx = rtu::attach_slave(SerialStream::open(&builder).unwrap(), Slave(new_node_id));
    ctx.set_slave(Slave(new_node_id));
    let read_rsp: Vec<u16> = ctx.read_holding_registers(reg_node_id, 1).await??;
    println!("> read_rsp: {:?}", read_rsp);
    let latest_node_id = read_rsp[0] as u8;
    if latest_node_id != new_node_id {
        eprintln!("latest_node_id {latest_node_id:X?} != {new_node_id:X?}");
    }

    if new_node_id == NODEID_YKPVCCS010_CURR_SRC {
        // flush the configuration parameter change to the node's persistent storage 
        println!("persisting node configuration");
        let flush_resp = ctx.write_single_register(REG_SAVE_CFG_YKPVCCS010_CURR_SRC, 1).await?;
        if flush_resp.is_err() {
            eprintln!("> flush_resp: {:?}", flush_resp);
        }
    }

    println!("Disconnecting from new node id: {new_node_id:X?}");
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
    // set_one_modbus_node_id(tty_path, baud_rate, REG_NODEID_YKDAQ1402_IV_ADC, NODEID_DEFAULT, NODEID_YKDAQ1402_IV_ADC).await?;
    // set_one_modbus_node_id(tty_path, baud_rate, REG_NODEID_PREC_CURR, NODEID_DEFAULT, NODEID_PREC_CURR_SRC).await?;
    // set_one_modbus_node_id(tty_path, baud_rate, REG_NODEID_TK, NODEID_DEFAULT, NODEID_DUAL_TK).await?;
    // set_one_modbus_node_id(tty_path, baud_rate, REG_NODEID_PYRO_SIM, NODEID_DEFAULT, NODEID_PYRO_SIM).await?;
    // set_one_modbus_node_id(tty_path, baud_rate, REG_NODEID_N4AIA04, 0x1E, NODEID_N4AIA04_IV_ADC).await?;
    // set_one_modbus_node_id(tty_path, baud_rate, REG_NODEID_QUAD_RELAY, 0x00, NODEID_QUAD_RELAY).await?;
    // set_one_modbus_node_id(tty_path, baud_rate, REG_NODEID_QUAD_RELAY, 0x00, NODEID_QUAD_RELAY).await?;
    // set_one_modbus_node_id(tty_path, baud_rate, REG_NODEID_N4VIA02, NODEID_DEFAULT, NODEID_N4VIA02_IV_ADC).await?;
    // set_one_modbus_node_id(tty_path, baud_rate, REG_NODEID_N4IOA01, NODEID_DEFAULT, NODEID_N4IOA01_CURR_GEN).await?;
    // set_one_modbus_node_id(tty_path, baud_rate, REG_NODEID_WAVESHARE_V2, NODEID_DEFAULT, NODEID_WA8TAI_IV_ADC).await?;
    // set_one_modbus_node_id(tty_path, baud_rate, REG_NODEID_WAVESHARE_V2, NODEID_DEFAULT, NODEID_WA26419_8CH_DAC).await?;
    // set_one_modbus_node_id(tty_path, baud_rate, REG_NODEID_R4DVI04, NODEID_DEFAULT, NODEID_R4DVI04_QRELAY_ADC).await?;
    set_one_modbus_node_id(tty_path, baud_rate, REG_NODEID_WDCU3003M, NODEID_BROADCAST_0, NODEID_WDCU3003M_IV_ADC).await?;


    Ok(())
}


/** 
 * Configure DAC
 */
async fn configure_wa26419(ctx: &mut tokio_modbus::client::Context)
-> Result<(), Box<dyn std::error::Error>>  {
    ctx.set_slave(Slave(NODEID_WA26419_8CH_DAC)); 

    Ok(())
}

/**
 * Configure ADC with odd channels Voltage, even channels Amps
 */
async fn configure_wa8tai_mixed_adc_modes(ctx: &mut tokio_modbus::client::Context)
-> Result<(), Box<dyn std::error::Error>>  {
    let volt_mode_10v: u16 = 0x0000; // Range 0~10V, output range 0~5000 or 0~10000, unit mV;
    let amp_mode_0020: u16 = 0x0002; // Range 4~20mA, output range 4000~20000, unit uA;
    let amp_mode_0420: u16 = 0x0003; // Range 4~20mA, output range 4000~20000, unit uA;

    // 0x0000: Range 0~5V, output range 0~5000 or 0~10000, unit mV;
    // 0x0001: Range 1~5V, output range 1000~5000 or 2~10V, output range 2000~10000, unit mV;
    // 0x0002: Range 0~20mA, output range 0~20000, unit uA;
    // 0x0003: Range 4~20mA, output range 4000~20000, unit uA;
    // 0x0004: Direct output of numerical code, output range 0~4096, requires linear conversion to obtain actual measured voltage and current;z
    println!("select node: {NODEID_WA8TAI_IV_ADC:?}");
    ctx.set_slave(Slave(NODEID_WA8TAI_IV_ADC)); 
    ctx.write_single_register(0x1000, volt_mode_10v).await??;
    ctx.write_single_register(0x1001, amp_mode_0020).await??;
    ctx.write_single_register(0x1002, volt_mode_10v).await??;
    ctx.write_single_register(0x1003, amp_mode_0020).await??;
    
    ctx.write_single_register(0x1004, volt_mode_10v).await??;
    ctx.write_single_register(0x1005, amp_mode_0420).await??;
    ctx.write_single_register(0x1006, volt_mode_10v).await??;
    ctx.write_single_register(0x1007, amp_mode_0420).await??;

    let resp = ctx.read_holding_registers(0x1000, 8).await??;
    println!("wa8tai config response: {resp:?}");

    Ok(())
}

