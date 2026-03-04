#![allow(unused)]

use std::f32::INFINITY;
use std::process::exit;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;
use tokio::time::sleep;
use tokio_modbus::prelude::*;

use tokio_modbus::client::{Client, Reader, Writer};
use std::fs::File;
use std::io::{BufWriter, Write};

use ctrlc;
use std::sync::mpsc::channel;
use approx::{AbsDiff, abs_diff_ne};
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
 * Set the output drive current of the test electrodes 
 */
async fn set_electrode_current_drive(ctx: &mut tokio_modbus::client::Context, milliamps: f32) -> Result<f32, Box<dyn std::error::Error>> 
{
    set_ykpvccs0100_current_drive(ctx, milliamps).await
}


/**
 * Set the pyro simulator current loop controller (4-20 mA source) current value
 */
async fn set_pyro_420ma_drive(ctx: &mut tokio_modbus::client::Context,  milliamps: f64) 
-> Result<(), Box<dyn std::error::Error>> 
{
    // set_n4ioa01_0420_current_loop_drive(ctx, milliamps).await
    set_wa26419_0420_current_loop_drive(ctx, 4, milliamps).await
}

/**
 * Read the actual current flowing through the pyrometer loop
 */
async fn read_pyro_420ma_value(ctx: &mut tokio_modbus::client::Context)
-> Result<f64, Box<dyn std::error::Error>> 
{
    let val = read_wa8tai_iv(ctx,2).await?;
    // let adjusted_val = val - 0.014;
    Ok(val)
}

/**
 * Verify that all the modules we expect to be connected to the RS-485 Modbus are,
 * in fact, connected.
 */
async fn enumerate_required_modules(ctx: &mut tokio_modbus::client::Context) -> Result<(), Box<dyn std::error::Error>> 
{
    // measures dual type K thermocouple signal
    ping_one_modbus_node_id(ctx, NODEID_YKKTC1202_DUAL_TK, REG_NODEID_YKKTC1202_DUAL_TK).await?;

    // // pyro 4-20 mA current loop source (simulates pyrometer)
    // ping_one_modbus_node_id(ctx,NODEID_N4IOA01_CURR_GEN, REG_NODEID_N4IOA01).await?;

    // // pyro 4-20 mA current loop ammeter (verifies pyrometer simulation signal)
    // ping_one_modbus_node_id(ctx,NODEID_N4AIA04_IV_ADC, REG_NODEID_N4AIA04).await?;

    // pyro 4-20 mA current loop source (simulates pyrometer)
    ping_one_modbus_node_id(ctx,NODEID_WA26419_8CH_DAC, REG_NODEID_WAVESHARE_V2).await?;

    // pyro 4-20 mA current loop ammeter (verifies pyrometer simulation signal)
    ping_one_modbus_node_id(ctx,NODEID_WA8TAI_IV_ADC, REG_NODEID_WAVESHARE_V2).await?;

    // supplies current to the electrode probe
    ping_one_modbus_node_id(ctx,NODEID_YKPVCCS010_CURR_SRC, REG_NODEID_YKPVCCS010_CURR_SRC).await?;

    // measures the voltage at and current flowing through electrode probe
    ping_one_modbus_node_id(ctx, NODEID_YKDAQ1402_IV_ADC, REG_NODEID_YKDAQ1402_IV_ADC).await?;

    Ok(())
}

fn current_from_current_density(density: f32) -> f32 
{
    // cylindrical surface area of two parallel ideal cylindrical wires,
    // each 0.7 mm diameter, each with 10 mm submerged into the electrolyate
    const ELECTRODE_SA_FACTOR: f32 = 2E-5;

    density * ELECTRODE_SA_FACTOR
}

#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Connect to Modbus apparatus via TCP server bridge (a WiFi bridge on our local network)
    let socket_addr: std::net::SocketAddr = "10.0.1.151:502".parse()?;

    println!("Connecting to: '{socket_addr:?}'");
    let mut ctx: client::Context = tcp::connect(socket_addr).await?;
    enumerate_required_modules(&mut ctx).await?;

    let start_time = chrono::Utc::now().timestamp();
    let log_out_path = format!("{}_recorder.csv",start_time);
    println!("Recording data to {log_out_path:?} ...");
    let logfile = File::create(format!("./data/{}_recorder.csv",start_time))?;
    let mut csv_writer = BufWriter::new(logfile);

    const CSV_HEADER: &str =  "epoch_secs,TK1_C,TK2_C,avg_C,elecdrive_mA,elecdrive_actual_mA,elec_V,elec_mA,elec_R,pyro_sim_mA,pyro_actual_mA";
    writeln!(csv_writer, "{}", CSV_HEADER)?;
    println!("{}",CSV_HEADER);

    const MIN_PYRO_MA:f64 = 4f64;
    const MAX_PYRO_MA:f64 = 20f64;
    const MIN_PYRO_TEMP_C:f64 = 0f64;
    const MAX_PYRO_TEMP_C:f64 = 1000f64;
    const PYRO_CELSIUS_RANGE:f64 = MAX_PYRO_TEMP_C - MIN_PYRO_TEMP_C;
    const MILLIAMPS_PER_CELSIUS:f64 = (MAX_PYRO_MA - MIN_PYRO_MA)/PYRO_CELSIUS_RANGE; //TODO check correctness of this
    const PYRO_SIM_MA_CORRECTION: f64 = -0.375; // required to correct current loop value seen by induction heater
    let mut cur_core_temperature = 20f32;
    let mut pyro_loop_sim_ma = MIN_PYRO_MA;
    let mut pyro_loop_actual_ma = 0f64;
    let mut elecdrive_milliamps = 2.0f32; //TODO temporary static value
    let mut elecdrive_actual_ma = 0f32;

    // Create an AtomicBool flag protected by Arc for thread-safe sharing
    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();

    // Set the Ctrl+C handler
    ctrlc::set_handler(move || {
        println!("\nReceived Ctrl+C, initiating shutdown...");
        r.store(false, Ordering::SeqCst);
    }).expect("Error setting Ctrl-C handler");

    let mut last_pyro_sim_ma = 3f64;

    while running.load(Ordering::SeqCst) { 

        sleep(Duration::from_millis(125));
        let (ch1_tk_opt, ch2_tk_opt) = read_dual_tk_temps(&mut ctx).await?;
        let tk1_c = ch1_tk_opt.unwrap_or(0f32);
        let tk2_c = ch2_tk_opt.unwrap_or(0f32);
        let mut avg_core_tk_c: f32 = 
            if ch1_tk_opt.is_some() {
                if ch2_tk_opt.is_some() {  (tk1_c + tk2_c) / 2f32 }
                else { tk1_c.into()}
            }
            else if ch2_tk_opt.is_some() { tk2_c.into() }
            else { MAX_PYRO_TEMP_C as f32};
        // println!("TK1 {tk1_c:?}°C TK2 {tk2_c:?}°C avg: {avg_core_tk_c:?}°C");

        pyro_loop_sim_ma = MIN_PYRO_MA + MILLIAMPS_PER_CELSIUS * (avg_core_tk_c as f64) + PYRO_SIM_MA_CORRECTION; //TODO recalibrate instead??
        if pyro_loop_sim_ma < 4.0 { pyro_loop_sim_ma = 4.0 };
        // Setting a new current value may cause a short dropout in current:
        // we don't set a new current value unless it's significantly different than previous
        if abs_diff_ne!(last_pyro_sim_ma, pyro_loop_sim_ma, epsilon = 0.01) {
            sleep(Duration::from_millis(125)).await;
            set_pyro_420ma_drive(&mut ctx, pyro_loop_sim_ma).await?;
            last_pyro_sim_ma = pyro_loop_sim_ma;
        }
        sleep(Duration::from_millis(125)).await;
        pyro_loop_actual_ma = read_pyro_420ma_value(&mut ctx).await?;
        // if abs_diff_ne!(pyro_loop_sim_ma, pyro_loop_actual_ma, epsilon = 0.01) {
        //     println!("pyro sim {pyro_loop_sim_ma:?} mA actual {pyro_loop_actual_ma:?} mA");
        // }

        // recalculate ideal electrode drive current:
        // calculate current density for about 1 cm long, 0.5 - 0.8 mm OD wires
        if avg_core_tk_c >= 750.0 && avg_core_tk_c <= 800.0 {
            let current_density = 0.5 * 100. * 100.; // ideally 0.5 A/cm^2 == 5000 A/m^2
            elecdrive_milliamps = current_from_current_density(current_density);
            sleep(Duration::from_millis(125)).await;
            elecdrive_actual_ma = set_electrode_current_drive(&mut ctx, elecdrive_milliamps).await?;
        }

        sleep(Duration::from_millis(125)).await;
        let (elec_volts, elec_milliamps) = read_electrode_pair_iv_adc(&mut ctx).await?;
        let inter_electrode_resistance = 
            if elec_milliamps > 0. {
                // this also covers the case where volts = 0.0, i.e. zero resistance.
                elec_volts / (elec_milliamps/1000.) 
            }
            else {
                core::f32::INFINITY
            };

        // TODO terminate the current drive if we determine that resistance is zero (indicating
        // that the electrode-electrode gap has been bridged by conductive material.
        const MIN_INTER_ELECTRODE_OHMS: f32 = 0.25;
        if inter_electrode_resistance < MIN_INTER_ELECTRODE_OHMS  {
            set_electrode_current_drive(&mut ctx, 0.).await?;
        }

        let timestamp = chrono::Utc::now().timestamp();
        let log_line = format!( "{},{},{},{},{},{},{},{},{},{},{}",
            timestamp,
            tk1_c, tk2_c, avg_core_tk_c,
            elecdrive_milliamps, elecdrive_actual_ma,
            elec_volts, elec_milliamps, inter_electrode_resistance,
            pyro_loop_sim_ma, pyro_loop_actual_ma
            );
        println!("{}",log_line);
        writeln!(  csv_writer,"{}",  log_line)?;
        csv_writer.flush()?;
    }

    println!("Shutting down outputs...");
    sleep(Duration::from_millis(125)).await;
    set_pyro_420ma_drive(&mut ctx, 0f64).await?;
    sleep(Duration::from_millis(125)).await;
    set_electrode_current_drive(&mut ctx,0f32).await?;

    ctx.disconnect().await?;

    Ok(())
}



