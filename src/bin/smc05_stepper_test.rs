#![allow(unused)]

use std::os::macos::raw::stat;
use std::time::Duration;
use tokio::time::sleep;

use tokio_modbus::prelude::*;

use tokio_modbus::client::{Client, Reader, Writer};
use tokio_serial::SerialStream;
use craven_control::*;


async fn start_motion(ctx: &mut tokio_modbus::client::Context, motion: u16) 
-> Result<(), Box<dyn std::error::Error>> 

{
    println!("start motion: {}", motion);
    ctx.write_single_register(0x0030, motion).await?;
    sleep(Duration::from_millis(25)).await;
    let status_resp: Vec<u16> = ctx.read_holding_registers(0x001A, 5).await??;
    println!("> start status 0x1A: {:?}", status_resp);
    let opstatus = status_resp[0];
    let direction = status_resp[1];
    if opstatus == 0  {
        println!("0x0030 -> {} start ", motion);
        ctx.write_single_register(0x0030, 3).await?;
    }

    Ok(())
}

async fn start_forward(ctx: &mut tokio_modbus::client::Context) 
-> Result<(), Box<dyn std::error::Error>> 

{
    stop_motion(ctx).await;
    start_motion(ctx,0).await
}

async fn start_reverse(ctx: &mut tokio_modbus::client::Context) 
-> Result<(), Box<dyn std::error::Error>> 
{
    stop_motion(ctx).await;
    start_motion(ctx,1).await
}

async fn stop_motion(ctx: &mut tokio_modbus::client::Context) 
-> Result<(), Box<dyn std::error::Error>> 

{
    let status_resp: Vec<u16> = ctx.read_holding_registers(0x001A, 5).await??;
    println!("> stop status 0x1A: {:?}", status_resp);
    let opstatus = status_resp[0];
    let direction = status_resp[1];
    if opstatus != 0 {
        println!("0x0030 -> 3 stop_motion ");
        ctx.write_single_register(0x0030, 3).await?;
    }
    Ok(())
}

/// 
/// Verify that all the modules we expect to be connected to the RS-485 Modbus are,
/// in fact, connected.
/// 
async fn enumerate_required_modules(ctx: &mut tokio_modbus::client::Context) -> Result<(), Box<dyn std::error::Error>> 
{

    // measures dual type-K thermocouples
    ping_one_modbus_node_id(ctx, NODEID_YKKTC1202_DUAL_TK, REG_NODEID_YKKTC1202_DUAL_TK).await?;

    // measures voltage and current across the electrodes
    // TODO we can't ping WDCU3003M with a node ID read, because it doesn't expose node ID to Modbus
    // ping_one_modbus_node_id(ctx, NODEID_WDCU3003_IV_ADC, 0x00).await?;


    ctx.set_slave(Slave(NODEID_WDCU3003_IV_ADC));
    let wdc3003_vals: Vec<u16> = ctx.read_holding_registers(0, 10).await??;
    println!("wdc3003_vals: {:?}", wdc3003_vals);

    // supplies current to the cathode and anodes
    ping_one_modbus_node_id(ctx,NODEID_YKPVCCS010_CURR_SRC, REG_NODEID_YKPVCCS010_CURR_SRC).await?;

    // controls furnace on/off
    // controls 4-pair anode connection relays
    ping_one_modbus_node_id(ctx, NODEID_WAV_OCTO_RELAY, REG_NODEID_WAVESHARE_V2).await?;

    sleep(Duration::from_millis(100)).await;
    ctx.set_slave(Slave(NODEID_SMC05_STEP_DRIVER));
    let config_resp: Vec<u16> = ctx.read_holding_registers(0x0000, 7).await??;
    println!("> SMC05 motion config 0x000 (7):\r\n {:?}", config_resp);

    let config_resp: Vec<u16> = ctx.read_holding_registers(0x0018, 1).await??;
    println!("> SMC05 address:  {:?}", config_resp);

    // ping_one_modbus_node_id(ctx, NODEID_SMC05_STEP_DRIVER, REG_NODEID_SMC05).await?;
    // ctx.read_holding_registers(REG_NODEID_SMC05, 1).await??;

    Ok(())
}

#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<(), Box<dyn std::error::Error>> {

    // actual usb device ID of one USB-RS485 adapter
    // let tty_path = "/dev/cu.usbserial-BG02SI88";
    // let baud_rate = 115200;

    // let builder = tokio_serial::new(tty_path, baud_rate);
    // let mut ctx = rtu::attach_slave(SerialStream::open(&builder).unwrap(), Slave(NODEID_DEFAULT));
    let socket_addr: std::net::SocketAddr = "10.0.1.151:502".parse()?;

    println!("Connecting to: '{socket_addr:?}'");
    let mut ctx: client::Context = tcp::connect(socket_addr).await?;
    enumerate_required_modules(&mut ctx).await?;

    ctx.set_slave(Slave(NODEID_SMC05_STEP_DRIVER));
    // println!("waiting on set_slave..");
    sleep(Duration::from_millis(50)).await;

    // let config_resp: Vec<u16> = ctx.read_holding_registers(0x0000, 24).await??;
    // println!("> config 0x000 (24):\r\n {:?}", config_resp);

    let status_rsp: Vec<u16> = ctx.read_holding_registers(0x001A, 11).await??;
    println!("> start status 0x001A: {:?}", status_rsp);
    let orig_motion_direction = status_rsp[1];
    let orig_pulse_count = status_rsp[4];
    let orig_action_count = status_rsp[8];
    // const ACTION_COUNT_STATUS: u16 = 0;
    // let action_count  = (ctx.read_holding_registers(ACTION_COUNT_STATUS, 1).await??)[0];

    // // "Action process mode"
    ctx.write_single_register(0x0000, 6).await?;

    // // forward pulses
    // ctx.write_single_register(0x0001, 1600).await??;
    // // reverse pulses
    // ctx.write_single_register(0x0004, 1600).await??;
    // // number of cycles
    // ctx.write_single_register(0x0007, 3).await??;

    // // pulses per rotation?
    // ctx.write_single_register(0x0010, 1600).await??;

    // "start" the preprogrammed motion (which is like 25 mm forward and back, 3 cycles)
    let mut prior_motion_direction = orig_motion_direction;
    let mut prior_pulse_count = orig_pulse_count;
    let mut prior_action_count = orig_action_count;
    ctx.write_single_register(0x0030, 3).await?;

    for i in 0..2 {
        let status_rsp: Vec<u16> = ctx.read_holding_registers(0x001A, 11).await??;
        println!("> status 0x001A: {:?}", status_rsp);

        let op_status = status_rsp[0];
        let motion_direction = status_rsp[1];
        let pulse_count = status_rsp[4];
        let action_count = status_rsp[8];
        println!("op {} dir {} pulse {} action {}",op_status, motion_direction, pulse_count, action_count);

        if action_count != prior_action_count {
            println!("New Action started!");
        }

        // TODO TEST if preprogrammed motion has finished, start it again:
        if op_status == 0 { // "Stopped"
            if motion_direction == prior_motion_direction  {
                if pulse_count == prior_pulse_count {
                    // if action_count doesn't equal prior, then we're starting a new round?
                    if action_count == prior_action_count {
                        println!("Restarting preprogrammed motion!");
                        ctx.write_single_register(0x0030, 3).await?;
                    }
                }
            }
        }

        prior_motion_direction = motion_direction;
        prior_action_count = action_count;
        prior_pulse_count = pulse_count;

        // TODO this needs to align with the "reversal" pauses , and pauses between cycles
        sleep(Duration::from_millis(2000)).await;
        
    }


    // ctx.write_single_register(0x0030, 1).await??;
    // sleep(Duration::from_millis(250)).await;
    // let status_resp: Vec<u16> = ctx.read_holding_registers(0x001A, 11).await??;
    // println!("> status 0x1A: {:?}", status_resp);
    // let opstatus = status_resp[0];
    // let direction = status_resp[1];
    // if opstatus == 0 {
    //     println!("0x0030 -> 1 start ");
    //     ctx.write_single_register(0x0030, 1).await??;
    // }

    // sleep(Duration::from_millis(1000)).await;

    
    // println!("0x0030 -> 1 begin ");
    // ctx.write_single_register(0x0030, 1).await??;
    // sleep(Duration::from_millis(250)).await;

    // let status_resp: Vec<u16> = ctx.read_holding_registers(0x001A, 11).await??;
    // println!("> status 0x1A: {:?}", status_resp);
    // let opstatus = status_resp[0];
    // let direction = status_resp[1];
    // if opstatus == 0 {
    //     println!("0x0030 -> 3 start ");
    //     ctx.write_single_register(0x0030, 3).await??;
    // }
    // sleep(Duration::from_millis(1000)).await;


    // let status_resp: Vec<u16> = ctx.read_holding_registers(0x001A, 11).await??;
    // println!("> status 0x1A: {:?}", status_resp);
    // let opstatus = status_resp[0];
    // let direction = status_resp[1];
    // if opstatus != 0 {
    //     println!("0x0030 -> 3 stopping ");
    //     ctx.write_single_register(0x0030, 3).await??;
    //     sleep(Duration::from_millis(1000)).await;
    //     let status_resp: Vec<u16> = ctx.read_holding_registers(0x001A, 11).await??;
    //     println!("> status 0x1A: {:?}", status_resp);
    // }
    // let new_direction = if direction == 0 { 1 } else { 0 };

    // println!("0x0030 -> {} new ", new_direction);
    // ctx.write_single_register(0x0030, new_direction).await??;
    // sleep(Duration::from_millis(250)).await;
    // let status_resp: Vec<u16> = ctx.read_holding_registers(0x001A, 11).await??;
    // println!("> status 0x1A: {:?}", status_resp);
    // let opstatus = status_resp[0];
    // let direction = status_resp[1];
    // if (opstatus == 0) {
    //     println!("0x0030 -> 3 restart ");
    //     ctx.write_single_register(0x0030, 3).await??;
    // }
    // sleep(Duration::from_millis(1000)).await;

    // let status_resp: Vec<u16> = ctx.read_holding_registers(0x001A, 11).await??;
    // println!("> status 0x1A: {:?}", status_resp);
    // let opstatus = status_resp[0];
    // let direction = status_resp[1];
    // if opstatus != 0 {
    //     println!("0x0030 -> 3 stopping ");
    //     ctx.write_single_register(0x0030, 3).await??;
    //     let status_resp: Vec<u16> = ctx.read_holding_registers(0x001A, 11).await??;
    //     println!("> status 0x1A: {:?}", status_resp);
    // }


    // sleep(Duration::from_millis(1000)).await;

    // let status_resp: Vec<u16> = ctx.read_holding_registers(0x001A, 11).await??;
    // println!("> status 0x1A: {:?}", status_resp);
    // let opstatus = status_resp[0];
    // let direction = status_resp[1];
    // if opstatus != 0 {
    //     println!("0x0030 -> 3 stop ");
    //     ctx.write_single_register(0x0030, 3).await??;
    // }

    ctx.disconnect().await?;

    Ok(())
}




