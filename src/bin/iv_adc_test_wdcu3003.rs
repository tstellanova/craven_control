#![allow(unused)]

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;
use tokio::time::sleep;

use tokio_modbus::prelude::*;

use tokio_modbus::client::{Client, Reader, Writer};
use tokio_serial::SerialStream;
use craven_control::*;



///
/// Test the WDCU3003 IV ADC and display meter
/// 

#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<(), Box<dyn std::error::Error>> {

    // actual usb device ID of one USB-RS485 adapter
    let tty_path = "/dev/cu.usbserial-BG02SI88";
    let baud_rate = 115200;

    let builder = tokio_serial::new(tty_path, baud_rate);
    let mut ctx = rtu::attach_slave(SerialStream::open(&builder).unwrap(), Slave(NODEID_DEFAULT));

    //ping_one_modbus_node_id(&mut ctx, NODEID_YKPVCCS010_CURR_SRC, REG_NODEID_YKPVCCS010_CURR_SRC).await?;
    // println!("read IV");
    // let (volts, milliamps) =  tokio::time::timeout(Duration::from_secs(2), 
    //     read_wdcu3003_iv_adc(&mut ctx)).await??;

    // Create an AtomicBool flag protected by Arc for thread-safe sharing
    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();

    // Set the Ctrl+C handler
    ctrlc::set_handler(move || {
        println!("\n====> Received Ctrl+C, initiating shutdown...");
        r.store(false, Ordering::SeqCst);
    }).expect("Error setting Ctrl-C handler");

    const MEASURE_WAIT_TIME: Duration = Duration::from_millis(1500);


    ctx.set_slave(Slave(NODEID_WDCU3003_IV_ADC));

    while running.load(Ordering::SeqCst) { 
        let (volts, milliamps) =  tokio::time::timeout(Duration::from_secs(5),
        read_wdcu3003_iv_adc(&mut ctx)).await??;
        println!("{}  {:.3} V , {:.3} mA",chrono::Utc::now().timestamp_millis(), volts, milliamps);
        sleep(MEASURE_WAIT_TIME).await;
    }

    println!("shutdown!");
    ctx.disconnect().await?;

    Ok(())
}



