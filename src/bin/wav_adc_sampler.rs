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
    let baud_rate = 9600;

    let builder = tokio_serial::new(tty_path, baud_rate);
    let mut ctx = rtu::attach_slave(SerialStream::open(&builder).unwrap(), Slave(NODEID_DEFAULT));

    ping_one_modbus_node_id(&mut ctx,NODEID_WA8TAI_IV_ADC, REG_NODEID_WAVESHARE_V2).await?;

    // cofigure channel 1-8 input modes for WA8TAI_IV_AD: Even channels are current, odd channels are voltage
    // configure_wa8tai_mixed_adc_modes(&mut ctx).await?;

    for i in 0..10 {
        sleep(Duration::from_millis(500)).await;
        let _val = read_wa8tai_one_channel(&mut ctx, 4).await?;
        // println!("test  value: {val:?}");
    }
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

