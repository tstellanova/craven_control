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
//    read_ykdaq1402_iv_adc(ctx).await

  let potential  = read_wa8tai_iv(ctx,1).await?;
  let current = read_wa8tai_iv(ctx,2).await?;

  Ok((potential, current))
}


/**
 * Set the output drive current of the test electrodes 
 */
async fn set_electrode_current_drive(ctx: &mut tokio_modbus::client::Context, milliamps: f32) -> Result<f32, Box<dyn std::error::Error>> 
{
    println!("new eleco_ma: {milliamps:?}");
    set_ykpvccs0100_current_drive(ctx, milliamps).await
}


/**
 * Set the pyro simulator current loop controller (4-20 mA source) current value
 */
async fn set_pyro_420ma_drive(ctx: &mut tokio_modbus::client::Context,  milliamps: f32) 
-> Result<(), Box<dyn std::error::Error>> 
{
    // println!("set pyro sim mA: {milliamps:?}");
    // set_n4ioa01_0420_current_loop_drive(ctx, milliamps).await
    set_wa26419_0420_current_loop_drive(ctx, 4, milliamps).await
}

/**
 * Read the actual current flowing through the pyrometer loop
 */
async fn read_pyro_420ma_value(ctx: &mut tokio_modbus::client::Context)
-> Result<f32, Box<dyn std::error::Error>> 
{
    let val = read_wa8tai_iv(ctx,4).await?;
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
    // ping_one_modbus_node_id(ctx, NODEID_YKDAQ1402_IV_ADC, REG_NODEID_YKDAQ1402_IV_ADC).await?;

    Ok(())
}

fn current_from_current_density_ma(density_ma_cm2: f64, wire_od_mm: f64, wire_len_mm: f64) -> f32 
{
    // Surface Area = PI * diameter * length 
    // For 0.8 mm OD, 10 mm long, outer cylinder SA = 0.251 cm2
    // 200 mA/cm2 * 0.251 = 50.265 mA
    // For 0.8 mm OD,  3 mm long, SA = 0.024 cm2, current = 4.8 mA
    let electrode_surface_area: f64 = std::f64::consts::PI * (wire_od_mm/10. * wire_len_mm/10.); 
    (density_ma_cm2  * electrode_surface_area) as f32
}

enum DrivePhase {
    Init = 0,
    Anchoring = 1, /// Attach initial carbon atoms to cathode surface
    Growth = 2, /// Growth of carbon chains between electrodes
    Done = 3,
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

    const CSV_HEADER: &str =  "epoch_secs,TK1_C,TK2_C,avg_C,eleco_dmA,eleco_rmA,elecm_mA,elecm_V,elec_R,pyro_sim_mA,pyro_actual_mA";
    writeln!(csv_writer, "{}", CSV_HEADER)?;
    println!("{}",CSV_HEADER);

    const MIN_INTER_ELECTRODE_OHMS: f32 = 16. * 1.5;
    const INF_INTER_ELECTRODE_OHMS: f32 = 60E3;
    const MIN_PYRO_MA:f32 = 4.;
    const MAX_PYRO_MA:f32 = 20.;
    const MIN_PYRO_TEMP_C:f32 = 0.;
    const MAX_PYRO_TEMP_C:f32 = 1000.;
    const PYRO_CELSIUS_RANGE:f32 = MAX_PYRO_TEMP_C - MIN_PYRO_TEMP_C;
    const MILLIAMPS_PER_CELSIUS:f32 = (MAX_PYRO_MA - MIN_PYRO_MA)/PYRO_CELSIUS_RANGE; 
    const PYRO_SIM_MA_CORRECTION: f32 = -0.375; // required to correct current loop value seen by induction heater
    const NOMINAL_GROWTH_MA: f32 = 5.;
    let mut cur_core_temperature: f64 = 20.;
    let mut pyro_sim_ma = MIN_PYRO_MA;
    let mut pyro_actual_ma = 0.;
    let mut last_pyro_sim_ma = 3.;
    let mut eleco_dma = 0.; // electrode drive current (a value we set)
    let mut last_eleco_dma = 0.; // prior loop electrode drive current
    let mut eleco_rma = 0.; // electrode reported drive current (current driver provides this)
    let mut prior_elecm_volts = 0.; //prior measured volts across electrodes
    let mut prior_elecm_ma = 0.;
    let mut drive_phase = DrivePhase::Init;
    let mut anchoring_phase_start = 0;
    let mut growth_phase_start = 0;
    let mut constant_current_target_ma = 0.;
    let mut prior_inter_electrode_resistance = INF_INTER_ELECTRODE_OHMS;
    // Create an AtomicBool flag protected by Arc for thread-safe sharing
    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();

    // Set the Ctrl+C handler
    ctrlc::set_handler(move || {
        println!("\nReceived Ctrl+C, initiating shutdown...");
        r.store(false, Ordering::SeqCst);
    }).expect("Error setting Ctrl-C handler");

    // 200 mA / cm^2 is ideal according to Y. Chen et al
    // const IDEAL_CURRENT_DENSITY_MA_CM2: f64 = 200.;
    // let ideal_current = current_from_current_density_ma(IDEAL_CURRENT_DENSITY_MA_CM2, 0.8, 10.);
    // println!("IDEAL_CURRENT: {ideal_current:?} mA");

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

        pyro_sim_ma = MIN_PYRO_MA + MILLIAMPS_PER_CELSIUS * avg_core_tk_c  + PYRO_SIM_MA_CORRECTION; //TODO recalibrate instead??
        if pyro_sim_ma < 4.0 { pyro_sim_ma = 4.0 };
        // Setting a new current value may cause a short dropout in current:
        // we don't set a new current value unless it's significantly different than previous
        if abs_diff_ne!(last_pyro_sim_ma, pyro_sim_ma, epsilon = 0.01) {
            sleep(Duration::from_millis(125)).await;
            set_pyro_420ma_drive(&mut ctx, pyro_sim_ma).await?;
            last_pyro_sim_ma = pyro_sim_ma;
        }
        sleep(Duration::from_millis(125)).await;
        pyro_actual_ma = read_pyro_420ma_value(&mut ctx).await?;

        // recalculate ideal electrode drive current:
        // calculate current density for about 10 mm long, 0.7 mm average OD wires
        if avg_core_tk_c > 740. && avg_core_tk_c < 780. {

            match drive_phase {
                DrivePhase::Init => {
                    drive_phase = DrivePhase::Anchoring;
                    eleco_dma = 5.;
                    anchoring_phase_start = chrono::Utc::now().timestamp();
                    println!("start anchor phase at: {anchoring_phase_start:?}");
                }
                DrivePhase::Anchoring => { // anchoring phase -- constant voltage
                    if prior_elecm_volts > 1.2 {
                        eleco_dma -= 0.05;
                    }
                    else if prior_elecm_volts < 1.0 {
                        eleco_dma += 0.05;
                    }
                    let current_gap = last_eleco_dma - eleco_rma;
                    if current_gap >= 0.3 {
                        println!("end anchor phase with eleco dma {last_eleco_dma:?} rma {eleco_rma:?}");
                        // TODO verify: transition to the next phase once we've achieved target voltage
                        constant_current_target_ma = eleco_rma + 0.1;
                        eleco_dma = constant_current_target_ma;
                        drive_phase = DrivePhase::Growth;
                        growth_phase_start = chrono::Utc::now().timestamp();
                        println!("start growth phase at: {growth_phase_start:?}");
                    }
                }
                DrivePhase::Growth => {  // whisker growth phase: constant current
                    eleco_dma = constant_current_target_ma;

                    // Terminate the current drive if we determine that resistance is close to zero (indicating
                    // that the electrode-electrode gap has been bridged by conductive material).
                    if prior_inter_electrode_resistance < MIN_INTER_ELECTRODE_OHMS  {
                        println!("end growth phase with V {prior_elecm_volts:?} R {prior_inter_electrode_resistance:?}");
                        drive_phase = DrivePhase::Done;
                        eleco_dma = 0.;
                        let growth_phase_end = chrono::Utc::now().timestamp();
                        println!("end growth phase at: {growth_phase_end:?}");
                    }
                }
                _ => {
                    eleco_dma = 0.;
                }
            };
        }
        else {
            eleco_dma = 0.;
        }

        if abs_diff_ne!(last_eleco_dma, eleco_dma, epsilon = 0.01) {
            sleep(Duration::from_millis(125)).await;
            eleco_rma = set_electrode_current_drive(&mut ctx, eleco_dma).await?;
            last_eleco_dma = eleco_dma;
        }

        sleep(Duration::from_millis(125)).await;
        let (elecm_volts, elecm_ma) = read_electrode_pair_iv_adc(&mut ctx).await?;
        let estd_electrode_ma: f32 = 
            if eleco_dma > 0. {
                if elecm_ma > 0. { elecm_ma } else { eleco_rma}
            }  else { 0. };
        let inter_electrode_resistance = 
            if estd_electrode_ma > 0. {
                // this also covers the case where volts = 0.0, i.e. zero resistance.
                elecm_volts / (estd_electrode_ma/1000.) 
            }
            else {
                INF_INTER_ELECTRODE_OHMS // arbitrary value based on measurement
            };
        prior_inter_electrode_resistance = inter_electrode_resistance;
        prior_elecm_volts = elecm_volts;
        prior_elecm_ma = elecm_ma;


        let timestamp = chrono::Utc::now().timestamp();
        let log_line = format!( "{},{},{},{},{},{},{},{},{},{},{}",
            timestamp,
            tk1_c, tk2_c, avg_core_tk_c,
            eleco_dma, eleco_rma, elecm_ma,
            elecm_volts, inter_electrode_resistance,
            pyro_sim_ma as f32, pyro_actual_ma as f32
            );
        println!("{}",log_line);
        writeln!(  csv_writer,"{}",  log_line)?;
        csv_writer.flush()?;
    }

    println!("Shutting down outputs...");
    sleep(Duration::from_millis(125)).await;
    set_pyro_420ma_drive(&mut ctx, 0.).await?;
    sleep(Duration::from_millis(125)).await;
    set_electrode_current_drive(&mut ctx,0.).await?;

    ctx.disconnect().await?;

    Ok(())
}



