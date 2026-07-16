#![allow(unused)]

use std::time::Duration;
use tokio::time::sleep;

use tokio_modbus::prelude::*;

use tokio_modbus::client::{Client, Reader, Writer};
use tokio_serial::SerialStream;
use craven_control::*;


/// Max time to wait for series of modbus transactions to complete
const MODBUS_TRANSACTION_TIMEOUT: Duration = Duration::from_secs(5);

/// Pause in Modbus commands for "important" commands
const MODBUS_RW_DELAY: Duration = Duration::from_millis(20);


async fn toggle_furnace(ctx: &mut tokio_modbus::client::Context, active:bool)
-> Result<(), Box<dyn std::error::Error>> 
{
    const FURNACE_RELAY_CHANNEL: u8 = 7;
    sleep(MODBUS_RW_DELAY).await;
    toggle_wav_octo_relay(ctx, FURNACE_RELAY_CHANNEL, active).await
}

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

    toggle_furnace(&mut ctx, true).await?;
    sleep(Duration::from_secs(2)).await;

    let mut anode_channels = [false; 4];
    for i in 1..=3 {
        for j in 1..=4 {
            anode_channels.fill(false);
            anode_channels[j-1] = true;
            write_wav_octo_relays(&mut ctx, &anode_channels).await?;
            sleep(Duration::from_secs(1)).await;
        }
        sleep(Duration::from_secs(1)).await;
    }

    //disable anodes
    anode_channels.fill(false);
    write_wav_octo_relays(&mut ctx, &anode_channels).await?;

    sleep(Duration::from_secs(3)).await;
    toggle_furnace(&mut ctx, false).await?;


    ctx.disconnect().await?;

    Ok(())
}



