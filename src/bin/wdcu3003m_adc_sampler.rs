#![allow(unused)]

use std::time::Duration;
use tokio::time::sleep;

use tokio_modbus::prelude::*;

use tokio_modbus::client::{Client, Reader, Writer};
use tokio_serial::SerialStream;
use craven_control::*;


/**
 * Configure ADC with odd channels Voltage, even channels Amps
 */
async fn configure_adc(ctx: &mut tokio_modbus::client::Context)
-> Result<(), Box<dyn std::error::Error>>  {

    // 0x0000: Measure voltage   
    // 0x0001: Measure current (high 16 bits)   
    // 0x0002: Measure current (low 16 bits)   
    // 0x0003: Measure power  
    // 0x0004: Set the power-on screen (1: Yes 0: None)   
    // 0x0005: Set the lower limit of the voltage alarm (value is 0: off)	5000
    // 0x0006: Set the upper limit of the voltage alarm (value is 0: off)	24000
    // 0x0007: 16 bits lower limit   0
    // 0x0008: low 16 bits Set the lower limit of the current alarm (value is 0: off)    1000
    // 0x0009: 16 bits high upper limit   45
    // 0x000A: low 16 bits Set the current alarm upper limit (value is 0: value is 0: off)   50880
    // 0x000B: internal use default 1   (1)
    // 0x000C: internal use default   (0)
    // 0x000D: internal use default   (0)
    // 0x000E: Set the shielding voltage  (30500)
    // 0x000F: Set the shielding current 16-bit high   (45)
    // 0x0010: Set the shielding current 16-bit low   (50980)
    // 0x0011: Set the alarm in the area or out of the area (the value is 0: out of the area, the value is 1: in the area)

    let resp = ctx.read_holding_registers(0x0000, 24).await??;
    println!("before: {resp:?}");

    ctx.set_slave(Slave(NODEID_WDCU3003M_IV_ADC)); 
    ctx.write_single_register(0x0004, 0).await??;
    ctx.write_single_register(0x0005, 0).await??;
    ctx.write_single_register(0x0006, 0).await??;
    ctx.write_single_register(0x0007, 8).await??;
    ctx.write_single_register(0x0008, 0).await??;
    ctx.write_single_register(0x0009, 0).await??;
    ctx.write_single_register(0x000A, 0).await??;
        // ---- 
    ctx.write_single_register(0x000E, 0).await??;
    ctx.write_single_register(0x000F, 0).await??;
    ctx.write_single_register(0x0010, 0).await??;

    let resp = ctx.read_holding_registers(0x0000, 24).await??;
    println!("after: {resp:?}");

    Ok(())
}

#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<(), Box<dyn std::error::Error>> {

    // actual usb device ID of one USB-RS485 adapter
    let tty_path = "/dev/cu.usbserial-BG02SI88";
    let baud_rate = 9600;

    let builder = tokio_serial::new(tty_path, baud_rate);
    let mut ctx = rtu::attach_slave(SerialStream::open(&builder).unwrap(), Slave(NODEID_DEFAULT));

    // TODO no way to read the address? unclear what register is readable
    // ping_one_modbus_node_id(&mut ctx, NODEID_WDCU3003M_IV_ADC, 0x00).await?;
    // poll_one_modbus_register(&mut ctx, NODEID_WDCU3003M_IV_ADC, 0x00).await?;

    ctx.set_slave(Slave(NODEID_WDCU3003M_IV_ADC)); 
    configure_adc(&mut ctx).await?;

    // 9 Volt source with 1 kOhm load should give 9 mA current
    for i in 0..300 {
        sleep(Duration::from_millis(1000)).await;
        let (volts, milliamps) = read_wdcu3003m_iv(&mut ctx).await?;
        println!("wdcu3003m  {:.4} V {:.4} mA ",volts, milliamps);
    }
    ctx.disconnect().await?;

    Ok(())
}





