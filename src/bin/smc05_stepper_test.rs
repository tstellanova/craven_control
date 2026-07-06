#![allow(unused)]

use std::time::Duration;
use tokio::time::sleep;

use tokio_modbus::prelude::*;

use tokio_modbus::client::{Client, Reader, Writer};
use tokio_serial::SerialStream;
use craven_control::*;



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

    println!("r1 0x0000 ....");
    let read_rsp: Vec<u16> = ctx.read_holding_registers(0x0000, 37).await??;
    println!("> r1: {:?}", read_rsp);

    println!("r2 0x001A ....");
    let read_rsp: Vec<u16> = ctx.read_holding_registers(0x001A, 11).await??;
    println!("> r2: {:?}", read_rsp);



    // "Action process mode"
    ctx.write_single_register(0x0000, 3).await??;

    // forward pulses
    ctx.write_single_register(0x0001, 1600).await??;
    // reverse pulses
    ctx.write_single_register(0x0004, 1600).await??;
    // number of cycles
    ctx.write_single_register(0x0007, 3).await??;

    // pulses per rotation?
    ctx.write_single_register(0x0010, 1600).await??;


    let config_resp: Vec<u16> = ctx.read_holding_registers(0x0000, 24).await??;
    println!("> config 0x000 (24):\r\n {:?}", config_resp);


    // serial mode
    ctx.write_single_register(0x0030, 3).await??;

    for _i in 0..5 {
        let status_resp: Vec<u16> = ctx.read_holding_registers(0x001A, 11).await??;
        println!("> status 0x1A: {:?}", status_resp);
        sleep(Duration::from_millis(500)).await;
    }

    // "Action process mode"
    // ctx.write_single_register(0x0000, 0).await??;

    ctx.disconnect().await?;

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

