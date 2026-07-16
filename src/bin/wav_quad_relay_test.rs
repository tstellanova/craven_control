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

    ping_one_modbus_node_id(&mut ctx, NODEID_WAV_OCTO_RELAY, REG_NODEID_WAVESHARE_V2).await?;

    let raw_status = ctx.read_coils(0x0000, 8).await??;
    println!("raw_status: {:?}",raw_status);

    ctx.write_single_coil(0x0006, true).await?;
    sleep(Duration::from_secs(3)).await;

    for i in 0..3 {
        ctx.write_multiple_coils(0x0000, &[true, false, false, false]).await?;
        sleep(Duration::from_secs(1)).await;

        ctx.write_multiple_coils(0x0000, &[false, true, false, false]).await?;
            sleep(Duration::from_secs(1)).await;

        ctx.write_multiple_coils(0x0000, &[false, false, true, false]).await?;
        sleep(Duration::from_secs(1)).await;

        ctx.write_multiple_coils(0x0000, &[false, false, false, true]).await?;
        sleep(Duration::from_secs(1)).await;

        ctx.write_multiple_coils(0x0000, &[false, false, false, false]).await?;
    }

    sleep(Duration::from_secs(3)).await;
    ctx.write_single_coil(0x0006, false).await?;

    ctx.disconnect().await?;

    Ok(())
}



