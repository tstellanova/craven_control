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

#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<(), Box<dyn std::error::Error>> {

    // actual usb device ID of one USB-RS485 adapter
    let tty_path = "/dev/cu.usbserial-BG02SI88";
    let baud_rate = 115200;

    let builder = tokio_serial::new(tty_path, baud_rate);
    let mut ctx = rtu::attach_slave(SerialStream::open(&builder).unwrap(), Slave(NODEID_DEFAULT));

    ctx.set_slave(Slave(NODEID_SMC05_STEP_DRIVER));
    println!("waiting on set_slave..");
    sleep(Duration::from_millis(50)).await;

    // let config_resp: Vec<u16> = ctx.read_holding_registers(0x0000, 24).await??;
    // println!("> config 0x000 (24):\r\n {:?}", config_resp);

    let read_rsp: Vec<u16> = ctx.read_holding_registers(0x001A, 11).await??;
    println!("> config 0x001A: {:?}", read_rsp);


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

    start_reverse(&mut ctx).await?;
    sleep(Duration::from_millis(1100)).await;
    start_forward(&mut ctx).await?;
    sleep(Duration::from_millis(1000)).await;
    
    stop_motion(&mut ctx).await?;

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




