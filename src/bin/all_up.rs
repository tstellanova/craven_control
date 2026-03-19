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


const MAX_PROBE_TEMP_C:f32 = 1000.;


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
    println!("new eleco_ma: {milliamps:.3}");
    set_ykpvccs0100_current_drive(ctx, milliamps).await
}


/**
 * Verify that all the modules we expect to be connected to the RS-485 Modbus are,
 * in fact, connected.
 */
async fn enumerate_required_modules(ctx: &mut tokio_modbus::client::Context) -> Result<(), Box<dyn std::error::Error>> 
{
    // measures dual type K thermocouple signal
    ping_one_modbus_node_id(ctx, NODEID_YKKTC1202_DUAL_TK, REG_NODEID_YKKTC1202_DUAL_TK).await?;

    // measures voltage and current across the electrodes
    ping_one_modbus_node_id(ctx,NODEID_WA8TAI_IV_ADC, REG_NODEID_WAVESHARE_V2).await?;

    // supplies current to the electrode probe
    ping_one_modbus_node_id(ctx,NODEID_YKPVCCS010_CURR_SRC, REG_NODEID_YKPVCCS010_CURR_SRC).await?;

    // controls furnace on/off
    ping_one_modbus_node_id(ctx, NODEID_R4DVI04_QRELAY_ADC, REG_NODEID_R4DVI04).await?;

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
    Probing = 1, // check that the electrode is immersed in conductive melt
    Anchoring = 2, /// Attach initial carbon atoms to cathode surface
    Growth = 3, /// Growth of carbon chains between electrodes
    Done = 4,
} 

#[derive(Debug, Clone)]
pub struct FurnaceState {
    /// Prior maximum temperature
    pub prior_max_temp_c: f32,

    /// Temperature set point
    pub setupoint_c: f32,

    /// Most recently measured temperature
    pub measured_temp_c: f32,

    pub heater_on: bool,

}

async fn toggle_furnace(ctx: &mut tokio_modbus::client::Context, active:bool)
-> Result<(), Box<dyn std::error::Error>> 
{
    toggle_r4dvi04_relay(ctx,4, active).await
}

async fn control_furnace(ctx: &mut tokio_modbus::client::Context, state: &mut FurnaceState) 
-> Result<(), Box<dyn std::error::Error>> 
{
    // first, measure current temperatur
    sleep(Duration::from_millis(125));
    let (ch1_tk_opt, ch2_tk_opt) = read_dual_tk_temps(ctx).await?;
    let tk1_c = ch1_tk_opt.unwrap_or(0f32);
    let tk2_c = ch2_tk_opt.unwrap_or(0f32);
    let mut avg_core_tk_c: f32 = 
        if ch1_tk_opt.is_some() {
            if ch2_tk_opt.is_some() {  (tk1_c + tk2_c) / 2f32 }
            else { tk1_c.into()}
        }
        else if ch2_tk_opt.is_some() { tk2_c.into() }
        else { MAX_PROBE_TEMP_C as f32};
    
    if abs_diff_ne!(tk1_c, tk2_c, epsilon=10.) {
        println!("TK1 {tk1_c:?}°C TK2 {tk2_c:?}°C avg: {avg_core_tk_c:?}°C");
    }

    state.measured_temp_c = avg_core_tk_c;

    // TODO proper bangbang controller
    if state.measured_temp_c < ( state.setupoint_c  - 20.) {
        println!("set heater on at: {:.3} < {:.3}", state.measured_temp_c, state.setupoint_c);
        toggle_furnace(ctx, true).await?;
        state.heater_on = true;
        state.prior_max_temp_c = 0.; //reset
    }
    else if state.measured_temp_c > state.setupoint_c {
        println!("set heater off at: {:.3} >= {:.3}", state.measured_temp_c, state.setupoint_c);
        toggle_furnace(ctx, false).await?;
        state.heater_on = false;
    }

    if state.measured_temp_c > state.prior_max_temp_c {
        state.prior_max_temp_c = state.measured_temp_c;
    }
    
    Ok(())
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

    const CSV_HEADER: &str =  "epoch_secs,heating,avg_C,eleco_dmA,eleco_rmA,elecm_mA,elecm_V,elec_R";
    writeln!(csv_writer, "{}", CSV_HEADER)?;
    println!("{}",CSV_HEADER);

    const MIN_INTER_ELECTRODE_OHMS: f32 = 16. * 1.5;
    const INF_INTER_ELECTRODE_OHMS: f32 = 60E3;

    const NOMINAL_GROWTH_MA: f32 = 5.;
    const PROBE_CURRENT_MA: f32 = 1.;

    let mut eleco_dma = 0.; // electrode drive current (a value we set)
    let mut last_eleco_dma = 0.; // prior loop electrode drive current
    let mut eleco_rma = 0.; // electrode reported drive current (current driver provides this)
    let mut prior_elecm_volts = 0.; //prior measured volts across electrodes
    let mut prior_elecm_ma = 0.;
    let mut drive_phase = DrivePhase::Init;

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

    let mut furnace_state = FurnaceState  { 
        prior_max_temp_c: 0., 
        setupoint_c: 750., 
        measured_temp_c: 0., 
        heater_on: false 
    };

    while running.load(Ordering::SeqCst) { 

        control_furnace(&mut ctx, &mut furnace_state).await?;

        if prior_inter_electrode_resistance < 8000. {

            match drive_phase {
                DrivePhase::Init => {
                    drive_phase = DrivePhase::Probing;
                    eleco_dma = PROBE_CURRENT_MA;
                    last_eleco_dma = 0.;
                    eleco_rma = 0.;
                    let probe_phase_start = chrono::Utc::now().timestamp();
                    println!("start probing phase at: {probe_phase_start:?}");
                }
                DrivePhase::Probing => {
                    if eleco_rma >= PROBE_CURRENT_MA  {
                        // TODO experiment with target current density 
                        // constant_current_target_ma = current_from_current_density_ma(200., 0.8, 10.);
                        constant_current_target_ma = 19.; //TODO tmp
                        eleco_dma = constant_current_target_ma;
                        drive_phase = DrivePhase::Growth;
                        let growth_phase_start = chrono::Utc::now().timestamp();
                        println!("start growth phase at: {growth_phase_start:?} with eleco_dma: {eleco_dma:.3}");
                    }
                    else {
                        eleco_dma = PROBE_CURRENT_MA;
                    }
                }
                DrivePhase::Anchoring => { // anchoring phase -- constant voltage
                    if prior_elecm_volts > 1.7 {
                        eleco_dma -= 0.02;
                    }
                    else if prior_elecm_volts < 1.55 {
                        eleco_dma += 0.02;
                    }
                    let current_gap: f32 = last_eleco_dma - eleco_rma;
                    if current_gap >= 0.3 {
                        println!("end anchor phase with eleco dma {last_eleco_dma:?} rma {eleco_rma:.3}");
                        // transition to the next phase once we've achieved target voltage
                        constant_current_target_ma = eleco_rma + 0.1;
                        eleco_dma = constant_current_target_ma;
                        drive_phase = DrivePhase::Growth;
                        let growth_phase_start = chrono::Utc::now().timestamp();
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
            eleco_dma = 0.1;
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
                INF_INTER_ELECTRODE_OHMS // arbitrary value based on previous experiments
            };
        prior_inter_electrode_resistance = inter_electrode_resistance;
        prior_elecm_volts = elecm_volts;
        prior_elecm_ma = elecm_ma;

        let timestamp = chrono::Utc::now().timestamp();
        let log_line = format!( "{},{},{:.2},{:.3},{:.3},{:.3},{:.3},{:.1}",
            timestamp,
            furnace_state.heater_on as u8,
            furnace_state.measured_temp_c,
            eleco_dma, eleco_rma, elecm_ma,
            elecm_volts, inter_electrode_resistance,
            );
        println!("{}",log_line);
        writeln!(  csv_writer,"{}",  log_line)?;
        csv_writer.flush()?;
    }

    println!("Shutting down outputs...");
    toggle_furnace(&mut ctx, false);

    sleep(Duration::from_millis(125)).await;
    set_electrode_current_drive(&mut ctx,0.).await?;

    ctx.disconnect().await?;

    Ok(())
}



