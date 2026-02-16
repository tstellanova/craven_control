#![allow(unused)]

use std::process::exit;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;
use tokio::time::sleep;

use tokio_modbus::prelude::*;

use tokio_modbus::client::{Client, Reader, Writer};
// use byteorder::{BigEndian, ByteOrder};
use std::fs::File;
use std::io::{BufWriter, Write};

use ctrlc;
use std::sync::mpsc::channel;

use craven_control::*;

/**
 * Read the dual thermocouple reader
 */
async fn read_dual_tk_temps(ctx: &mut tokio_modbus::client::Context)
-> Result<(Option<f32>, Option<f32>), Box<dyn std::error::Error>> 
{
    read_ykktc1202_dual_tk_temps(ctx).await
}


/**
 * Read the voltage across and current through active electrode pair.
 * 
 */
async fn read_electrode_pair_iv_adc(ctx: &mut tokio_modbus::client::Context)
-> Result<(f32, f32), Box<dyn std::error::Error>> 
{
   read_ykdaq1402_iv_adc(ctx).await
}

/**
 * Set the output drive current of the precision current source 
 */
async fn set_precision_current_drive(ctx: &mut tokio_modbus::client::Context, milliamps: f32) -> Result<f32, Box<dyn std::error::Error>> 
{
    set_ykpvccs0100_current_drive(ctx, milliamps).await
}


/**
 * Set the pyro simulator current loop controller (4-20 mA source) current value
 */
async fn set_pyro_420ma_drive(ctx: &mut tokio_modbus::client::Context,  milliamps: f32) 
-> Result<(), Box<dyn std::error::Error>> 
{
    set_n4ioa01_0420_current_loop_drive(ctx, milliamps).await
}

/**
 * Read the actual current flowing through the pyrometer loop
 */
async fn read_pyro_420ma_value(ctx: &mut tokio_modbus::client::Context)
-> Result<f32, Box<dyn std::error::Error>> 
{
    let resp = read_n4aia04_420_iv_adc(ctx).await?;
    Ok(resp.1)
}

/**
 * Verify that all the modules we expect to be connected to the RS-485 Modbus are,
 * in fact, connected.
 */
async fn enumerate_required_modules(ctx: &mut tokio_modbus::client::Context) -> Result<(), Box<dyn std::error::Error>> 
{
    // measures dual type K thermocouple signal
    ping_one_modbus_node_id(ctx, NODEID_YKKTC1202_DUAL_TK, REG_NODEID_YKKTC1202_DUAL_TK).await?;

    // pyro 4-20 mA current loop source (simulates pyrometer)
    ping_one_modbus_node_id(ctx,NODEID_N4IOA01_CURR_GEN, REG_NODEID_N4IOA01).await?;

    // pyro 4-20 mA current loop ammeter (verifies pyrometer simulation signal)
    ping_one_modbus_node_id(ctx,NODEID_N4AIA04_IV_ADC, REG_NODEID_N4AIA04).await?;


    // ping_one_modbus_node_id(ctx,NODEID_N4VIA02_IV_ADC, REG_NODEID_N4VIA02).await?;

    // supplies current to the electrode probe
    //TODO ping_one_modbus_node_id(ctx,NODEID_YKPVCCS010_CURR_SRC, REG_NODEID_YKPVCCS010_CURR_SRC).await?;

    // measures the voltage at and current flowing through electrode probe
    ping_one_modbus_node_id(ctx, NODEID_YKDAQ1402_IV_ADC, REG_NODEID_YKDAQ1402_IV_ADC).await?;

    Ok(())
}

#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    use tokio_serial::SerialStream;

    use tokio_modbus::prelude::*;

    // actual usb device ID of one USB-RS485 adapter
    let tty_path = "/dev/cu.usbserial-BG02SI88";
    let baud_rate = 9600;

    // Via TCP server bridge (a Wifi bridge on our local network)
    let socket_addr: std::net::SocketAddr = "10.0.1.151:502".parse()?;

    println!("Connecting to: '{socket_addr:?}'");
    let mut ctx: client::Context = tcp::connect(socket_addr).await?;
    enumerate_required_modules(&mut ctx).await?;


    let start_time = chrono::Utc::now().timestamp();
    let log_out_path = format!("{}_recorder.csv",start_time);
    println!("Recording data to {log_out_path:?} ...");
    let logfile = File::create(format!("{}_recorder.csv",start_time))?;
    let mut csv_writer = BufWriter::new(logfile);

    const CSV_HEADER: &str =  "epoch_secs,TK1_C,TK2_C,avg_C,elec_drive_mA,self_reported_mA,elec_V,elec_mA,pyro_mA";
    writeln!(csv_writer, "{}", CSV_HEADER)?;
    println!("{}",CSV_HEADER);

    const MIN_PYRO_MA:f32 = 4.0;
    const MAX_PYRO_MA:f32 = 20.0;
    const MAX_PYRO_TEMP_C:f32 = 1000.0;
    const PYRO_CELSIUS_RANGE:f32 = MAX_PYRO_TEMP_C - 0.0;
    const MILLIAMPS_PER_CELSIUS:f32 = (MAX_PYRO_MA - MIN_PYRO_MA)/PYRO_CELSIUS_RANGE; //TODO check correctness of this
    let mut cur_core_temperature = 20.0f32;
    let mut pyro_loop_sim_ma = MIN_PYRO_MA;
    let mut elec_drive_milliamps = 0.0f32;
    let mut self_reported_ma = 0f32;

    // Create an AtomicBool flag protected by Arc for thread-safe sharing
    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();

    // Set the Ctrl+C handler
    ctrlc::set_handler(move || {
        println!("\nReceived Ctrl+C, initiating shutdown...");
        r.store(false, Ordering::SeqCst);
    }).expect("Error setting Ctrl-C handler");

    while running.load(Ordering::SeqCst) { 
        sleep(Duration::from_millis(125)).await;
        let (elec_volts, elec_milliamps) = read_electrode_pair_iv_adc(&mut ctx).await?;
        // println!("electrode {elec_volts:?} V, {elec_milliamps:?} mA");

        sleep(Duration::from_millis(125));
        let (ch1_tk_opt, ch2_tk_opt) = read_dual_tk_temps(&mut ctx).await?;
        let cur_tk1 = ch1_tk_opt.unwrap_or(0.00);
        let cur_tk2 = ch2_tk_opt.unwrap_or(cur_tk1);
        let cur_core_temperature = 
            if cur_tk1 > 0.0 {
                (cur_tk1 + cur_tk2) / 2f32
            }
            else {
                MAX_PYRO_TEMP_C
            };
        // cur_core_temperature = ch1_tk_opt.unwrap_or(ch2_tk_opt.unwrap_or(1000.0));
        // println!("TK1 {cur_tk1:?}°C TK2 {cur_tk2:?}°C avg: {cur_core_temperature:?}°C");

        // TODO calculate correctly
        // pyro_loop_sim_ma = MIN_PYRO_MA + MILLIAMPS_PER_CELSIUS * cur_core_temperature;
        sleep(Duration::from_millis(125)).await;
        set_pyro_420ma_drive(&mut ctx, pyro_loop_sim_ma).await?;
        pyro_loop_sim_ma += 0.5f32; //TODO
        if pyro_loop_sim_ma > 20.0f32 {
            pyro_loop_sim_ma = 3.5f32;
        }

        let pyro_loop_ma = read_pyro_420ma_value(&mut ctx).await?;
        println!("pyro sim {pyro_loop_sim_ma:?}mA actual {pyro_loop_ma:?} mA");

        //TODO recalculate ideal electrode drive current
        // sleep(Duration::from_millis(125)).await;
        // self_reported_ma = set_precision_current_drive(&mut ctx,elec_drive_milliamps).await?;
        elec_drive_milliamps += 0.25f32;
        if elec_drive_milliamps > 5.0f32 { elec_drive_milliamps = 0f32;}

        let timestamp = chrono::Utc::now().timestamp();
        let log_line = format!( "{},{},{},{},{},{},{},{},{}",
            timestamp,
            cur_tk1, cur_tk2,cur_core_temperature,
            elec_drive_milliamps,self_reported_ma,
            elec_volts, elec_milliamps,
            pyro_loop_sim_ma,
            );
        println!("{}",log_line);
        writeln!(  csv_writer,"{}",  log_line)?;
        csv_writer.flush()?;
    }

    //TODO add ctrl-c handler
    println!("shutting down...");
    sleep(Duration::from_millis(125)).await;
    set_pyro_420ma_drive(&mut ctx, 0f32).await?;
    // sleep(Duration::from_millis(125)).await;
    // set_precision_current_drive(&mut ctx,0f32).await?;

    ctx.disconnect().await?;

    Ok(())
}



