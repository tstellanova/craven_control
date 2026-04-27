#![allow(unused)]

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;
use tokio::time::sleep;

use tokio_modbus::prelude::*;

use tokio_modbus::client::{Client, Reader, Writer};
use tokio_serial::SerialStream;
use craven_control::*;


/// Update the given Exponential Weighted Moving Average with a new value
fn update_ewma(ewma: &mut f32, new_value: f32, alpha: f32) {
    *ewma = alpha * new_value + (1.0 - alpha) * *ewma;
}


async fn drive_current_and_measure_ccs1000(ctx: &mut tokio_modbus::client::Context,
 requested_ma: f32, current_ewma: &mut f32, wait_time: Duration)
-> Result<(), Box<dyn std::error::Error>> 
{
    set_ykpvccs1000_current_drive(ctx, requested_ma).await?;
    sleep( wait_time).await;
    let reported_ma =  read_ykpvccs1000_current_drive(ctx).await?;
    update_ewma(current_ewma, reported_ma, 0.1);
    Ok(())
}

async fn drive_current_and_measure_ccs0100(ctx: &mut tokio_modbus::client::Context,
 requested_ma: f32, current_ewma: &mut f32, wait_time: Duration)
-> Result<(), Box<dyn std::error::Error>> 
{
    set_ykpvccs0100_current_drive(ctx, requested_ma).await?;
    sleep( wait_time).await;
    let reported_ma =  read_ykpvccs0100_current_drive(ctx).await?;
    update_ewma(current_ewma, reported_ma, 0.1);
    Ok(())
}

#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<(), Box<dyn std::error::Error>> {

    // actual usb device ID of one USB-RS485 adapter
    let tty_path = "/dev/cu.usbserial-BG02SI88";
    let baud_rate = 115200;

    let builder = tokio_serial::new(tty_path, baud_rate);
    let mut ctx = rtu::attach_slave(SerialStream::open(&builder).unwrap(), Slave(NODEID_DEFAULT));

    ping_one_modbus_node_id(&mut ctx, NODEID_YKPVCCS010_CURR_SRC, REG_NODEID_YKPVCCS010_CURR_SRC).await?;

    // Create an AtomicBool flag protected by Arc for thread-safe sharing
    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();

    // Set the Ctrl+C handler
    ctrlc::set_handler(move || {
        println!("\n====> Received Ctrl+C, initiating shutdown...");
        r.store(false, Ordering::SeqCst);
    }).expect("Error setting Ctrl-C handler");

    const MEASURE_CURRENT_WAIT_TIME: Duration = Duration::from_millis(500);
    const REQUESTED_MA: f32 = 80.;
    let mut current_ewma: f32 = REQUESTED_MA;

    while running.load(Ordering::SeqCst) { 
        tokio::time::timeout(Duration::from_secs(2), 
        //drive_current_and_measure_ccs1000(&mut ctx, REQUESTED_MA, &mut current_ewma, MEASURE_CURRENT_WAIT_TIME)).await?;
        drive_current_and_measure_ccs0100(&mut ctx, REQUESTED_MA, &mut current_ewma, MEASURE_CURRENT_WAIT_TIME)).await?;

        println!("{} req {:.3} ewma {:.3} mA",chrono::Utc::now().timestamp_millis(), REQUESTED_MA, current_ewma);
    }

    println!("shutdown!");
    set_ykpvccs0100_current_drive(&mut ctx, 0.).await?;

    ctx.disconnect().await?;

    Ok(())
}



