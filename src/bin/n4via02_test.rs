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

    // set_one_modbus_node_id(tty_path, baud_rate, REG_NODEID_N4VIA02, NODEID_DEFAULT, NODEID_N4VIA02_IV_ADC).await?;

    ping_one_modbus_node_id(&mut ctx, NODEID_N4VIA02_IV_ADC, REG_NODEID_N4VIA02).await?;


    // cofigure channel 1-8 input modes for WA8TAI_IV_AD: Even channels are current, odd channels are voltage
    // configure_wa8tai_mixed_adc_modes(&mut ctx).await?;

    for i in 0..20 {
        sleep(Duration::from_millis(500)).await;
        let (voltage_vals, milliamp_vals) = read_n4via02_multimeter(&mut ctx).await?;
        println!(" milliamps: {milliamp_vals:?}");
        // let _val = read_wa8tai_iv(&mut ctx, 4).await?;
        // println!("test  value: {val:?}");
    }
    ctx.disconnect().await?;

    Ok(())
}



