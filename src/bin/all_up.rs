#![allow(unused)]

use std::f32::INFINITY;
use std::process::exit;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;
use tokio::time::{sleep, sleep_until};
use tokio_modbus::prelude::*;

use tokio_modbus::client::{Client, Reader, Writer};
use std::fs::File;
use std::io::{BufWriter, Write};

use ctrlc;
use std::sync::mpsc::channel;
use approx::{AbsDiff, abs_diff_ne};
use craven_control::*;


const ONE_SEC_DURATION: Duration = Duration::from_millis(1000);
const MODBUS_RW_DELAY: Duration = Duration::from_millis(100);

/// Rated maximum temperature of thermocouples (in this case, Type K)
const MAX_PROBE_TEMP_C:f32 = 1000.;
/// Temp at which we attempt to submerge thermo probes in electrolyte melt
const PROBE_CHECK_TEMP_C:f32 = 550.; 
/// Temp we expect to see when probe is succesfully inserted into melt
const PROBE_INSERTED_TEMP_C:f32 = 600.;
/// The center temperature we are trying to achieve for the electrolyte melt
const ELECTROLYTE_TARGET_TEMP_C:f32 = 780.;

/// If the electrodes were shorted together at room temperature, what resistance do we expect?
const MIN_INTER_ELECTRODE_OHMS: f32 = 5.;
/// A guess at what a stable dendrite resistance would be when it approaches the anode
const STABLE_DENDRITE_OHMS: f32 = MIN_INTER_ELECTRODE_OHMS*2.;
/// Arbitrary value for "infinite" resistance (open circuit) between electrodes
const INF_INTER_ELECTRODE_OHMS: f32 = 666E2;
/// The measured gap between requested and actual current supplied by the current source, when they diverge. 
const PLATEAU_CURRENT_GAP_MA: f32 = 2.5;
/// Highest potential provided by current source (measured as 10.689) minus some uncertainty
const OPEN_CIRCUIT_VOLTS: f32 = 9.; 

/// Midrange for 4-20 mA measurement
const DENDRITE_CREEP_MA: f32 = 12.; 
/// Used to probe for electrolyte or carbon bridge conductivity
const PROBE_CURRENT_MA: f32 = 1.;
/// Used after we think we've achieved a solid carbon bridge 
const COOLDOWN_PROBE_CURRENT_MA: f32 = 0.5;
/// The minimum increment for drive current, as specified in the current source docs
const MIN_DRIVE_CURRENT_INCR_MA: f32 = 0.1;
/// The range of the ammeter
const MAX_AMMETER_VAL: f32 = 20.0;
/**
 * Read the dual thermocouple reader
 */
async fn read_dual_tk_temps(ctx: &mut tokio_modbus::client::Context)
-> Result<(Option<f32>, Option<f32>), Box<dyn std::error::Error>> 
{
    sleep(MODBUS_RW_DELAY);
    read_ykktc1202_dual_tk_temps(ctx).await
}


/**
 * Read the voltage across and current through active electrode pair.
 * 
 */
async fn read_electrode_pair_iv_adc(ctx: &mut tokio_modbus::client::Context)
-> Result<(f32, f32), Box<dyn std::error::Error>> 
{
    sleep(Duration::from_millis(100)).await;
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
    sleep(MODBUS_RW_DELAY).await;
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

#[repr(u8)]
#[derive(Debug, Clone, PartialEq)]
enum DrivePhase {
    /// check that the electrode is immersed in conductive melt
    Warmup = 0, 
    /// Attach initial carbon atoms to cathode surface
    Anchoring = 1, 
     /// Growth of carbon chains between electrodes
    Growth = 2,
    /// Stable dendrite growth?
    Dendrite = 3, 
    /// Monitor the inter-electrode conductivity
    Cooldown = 4, 
} 

/**
 * Redirect furnace on/off to actual modbus device.
 */
async fn toggle_furnace(ctx: &mut tokio_modbus::client::Context, active:bool)
-> Result<(), Box<dyn std::error::Error>> 
{
    sleep(MODBUS_RW_DELAY).await;
    toggle_r4dvi04_relay(ctx,4, active).await?;
    Ok(())
}

/**
 * Shut off the furnace heater, shut off any current drive.
 */
async fn zero_control_outputs(ctx: &mut tokio_modbus::client::Context)
-> Result<(), Box<dyn std::error::Error>> 
{
    println!("Shutting down outputs...");
    toggle_furnace(ctx, false).await?;
    set_electrode_current_drive(ctx,0.).await?;
    println!("Outputs disabled.");
    Ok(())
}

#[derive(Debug, Clone)]
pub struct FurnaceState {
    /// Prior maximum temperature
    pub prior_max_temp_c: f32,

    /// Temperature set point
    pub setpoint_c: f32,

    /// Most recently measured temperature
    pub measured_temp_c: f32,

    pub heater_on: bool,

}

/**
 * Turn the furnace heating on/off based on setpoint and temperature
 */
async fn control_furnace(ctx: &mut tokio_modbus::client::Context, state: &mut FurnaceState) 
-> Result<(), Box<dyn std::error::Error>> 
{
    let mut new_temp_setpoint_c: f32 = 0.;

    // first, measure temperature
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
    
    if abs_diff_ne!(tk1_c, tk2_c, epsilon=15.) {
        println!("TK1 {tk1_c:?}°C TK2 {tk2_c:?}°C avg: {avg_core_tk_c:?}°C");
    }

    state.measured_temp_c = avg_core_tk_c;

    // update the temperature setpoint based on which phase of electrolyte melting we're at
    if state.measured_temp_c < ELECTROLYTE_TARGET_TEMP_C {
        if state.measured_temp_c < PROBE_CHECK_TEMP_C {
            new_temp_setpoint_c = PROBE_CHECK_TEMP_C;
        }
        else if state.measured_temp_c < PROBE_INSERTED_TEMP_C {
            new_temp_setpoint_c = PROBE_INSERTED_TEMP_C;
        }
        else {
            new_temp_setpoint_c = ELECTROLYTE_TARGET_TEMP_C;
        }
    }
    else {
        new_temp_setpoint_c = ELECTROLYTE_TARGET_TEMP_C;
    }

    if new_temp_setpoint_c != state.setpoint_c {
        println!("setpoint old {:.3} new {:.3}", state.setpoint_c, new_temp_setpoint_c);
    }

    // TODO proper bangbang controller?
    if state.heater_on {
        if state.measured_temp_c > (new_temp_setpoint_c + 15.) {
            println!("set heater off at: {:.3} >= {:.3}", state.measured_temp_c, new_temp_setpoint_c);
            toggle_furnace(ctx, false).await?;
            state.heater_on = false;
        }
    }
    else {
        if state.measured_temp_c < (new_temp_setpoint_c  + 7.5) {
            println!("set heater on at: {:.3} (target {:.3} )", state.measured_temp_c, new_temp_setpoint_c);
            toggle_furnace(ctx, true).await?;
            state.heater_on = true;
            state.prior_max_temp_c = 0.; //reset
        }
    }

    state.setpoint_c = new_temp_setpoint_c;

    if state.measured_temp_c > state.prior_max_temp_c {
        state.prior_max_temp_c = state.measured_temp_c;
    }
    
    Ok(())
}

#[derive(Debug, Clone)]
pub struct ElectrodeState {
    drive_phase: DrivePhase,
    inter_electrode_ohms: f32,
    target_drive_ma: f32,
    reported_drive_ma: f32,
    measured_ma: f32,
    measured_volts: f32,
    growth_ma: f32,
}

/// State the electrode controller is reset to when we're in "warmup" mode below the active melt temperature
const WARMUP_ELECTRODE_STATE: ElectrodeState = 
    ElectrodeState {
            drive_phase:DrivePhase::Warmup,
            inter_electrode_ohms:INF_INTER_ELECTRODE_OHMS,
            target_drive_ma:0.,
            reported_drive_ma:0.,
            measured_ma:0.,
            measured_volts:0., 
            growth_ma: 40. 
        }; 
/**
 * Adjust the electrode current based on melt condition and drive phase
 */
async fn control_electrodes(ctx: &mut tokio_modbus::client::Context, 
    state: &mut ElectrodeState,
) 
-> Result<(), Box<dyn std::error::Error>> 
{
    let mut new_drive_ma: f32 = state.target_drive_ma;
    
    let (elecm_volts, elecm_ma) = read_electrode_pair_iv_adc(ctx).await?;
    let measured_electrode_ma: f32 = 
        if state.target_drive_ma > 0. {
            if elecm_volts < OPEN_CIRCUIT_VOLTS {
                if elecm_ma < MAX_AMMETER_VAL && elecm_ma > MIN_DRIVE_CURRENT_INCR_MA { elecm_ma }
                else { state.reported_drive_ma }
            } else { 0. }
        }  else { 0. };
    let inter_electrode_ohms = 
        if measured_electrode_ma > 0. {
            // this also covers the case where volts = 0.0, i.e. zero resistance.
            (1000. * elecm_volts) / measured_electrode_ma 
        }
        else {
            INF_INTER_ELECTRODE_OHMS // arbitrary value based on previous experiments
        };
    state.inter_electrode_ohms = inter_electrode_ohms;
    state.measured_volts = elecm_volts;
    state.measured_ma = measured_electrode_ma;

    let reported_gap = state.target_drive_ma - state.reported_drive_ma;
    let measured_gap = state.target_drive_ma - measured_electrode_ma;

    let current_gap: f32 = 
        if state.target_drive_ma >= PROBE_CURRENT_MA {
            measured_gap
        }
        else { 100. }; // outrageously large current gap

    match state.drive_phase {
        DrivePhase::Warmup => {
            new_drive_ma = PROBE_CURRENT_MA;
            if current_gap < 0.2 {
                // the measured current is about the same as probe current
                state.drive_phase = DrivePhase::Anchoring;
                println!("start anchor phase at: {:?}",chrono::Utc::now().timestamp());
            }
        }
        DrivePhase::Anchoring => { 
            if current_gap > PLATEAU_CURRENT_GAP_MA {
                // the actual current has diverged from desired current
                state.growth_ma =  state.target_drive_ma;
                println!("end Anchoring phase with target {:.3} mA :: actual {:.3} mA", 
                    state.target_drive_ma, state.measured_ma);
                // transition to the next phase now that current has slightly diverged
                state.drive_phase = DrivePhase::Growth;
                println!("end Anchoring phase at: {:?}",chrono::Utc::now().timestamp());
            }
            else {
                // slowly increase the desired drive current until measured current diverges
                new_drive_ma = state.target_drive_ma + 2.*MIN_DRIVE_CURRENT_INCR_MA;
            }
        }
        DrivePhase::Growth => {  
            if state.inter_electrode_ohms < STABLE_DENDRITE_OHMS {
                // continue steady dendrite growth
                println!("end Growth phase with V {:?} R {:?}", state.measured_volts, state.inter_electrode_ohms);
                state.drive_phase = DrivePhase::Dendrite;
                println!("end Growth phase at: {:?}",chrono::Utc::now().timestamp());
            }
            else if reported_gap > PLATEAU_CURRENT_GAP_MA {
                // try nudging down the drive current a bit
                new_drive_ma = (2.*state.target_drive_ma + state.reported_drive_ma) / 3.;
            }
        }
        DrivePhase::Dendrite => {
            if state.inter_electrode_ohms < MIN_INTER_ELECTRODE_OHMS  {
                // inter-electrode resistance is close to zero, indicating that the electrode-electrode gap has been bridged
                println!("end Dendrite phase with V {:?} R {:?}", state.measured_volts, state.inter_electrode_ohms);
                state.drive_phase = DrivePhase::Cooldown;
                println!("end Dendrite phase at: {:?}",chrono::Utc::now().timestamp());
            }
            else if state.inter_electrode_ohms > STABLE_DENDRITE_OHMS {
                // restart growth phase
                println!("restart Growth phase with V {:?} R {:?}", state.measured_volts, state.inter_electrode_ohms);
                new_drive_ma =  state.growth_ma;
                state.drive_phase = DrivePhase::Growth;
                println!("restart Growth phase at: {:?}",chrono::Utc::now().timestamp());
            }
            else {
                new_drive_ma = DENDRITE_CREEP_MA;
            }
        }
        DrivePhase::Cooldown => {
            new_drive_ma = COOLDOWN_PROBE_CURRENT_MA;
        }
        _ => {
        }
    };

    if abs_diff_ne!(new_drive_ma, state.target_drive_ma, epsilon = 0.01) {
        state.reported_drive_ma = set_electrode_current_drive(ctx, new_drive_ma).await?;
    }

    state.target_drive_ma = new_drive_ma;

    Ok(())
}



/**
 * Entry point
 */
#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Connect to Modbus apparatus via TCP server bridge (a WiFi bridge on our local network)
    let socket_addr: std::net::SocketAddr = "10.0.1.151:502".parse()?;

    println!("Connecting to: '{socket_addr:?}'");
    let mut ctx: client::Context = tcp::connect(socket_addr).await?;
    enumerate_required_modules(&mut ctx).await?;

    zero_control_outputs(&mut ctx).await?;

    let start_time = chrono::Utc::now().timestamp();
    let log_out_filename = format!("{}_recorder.csv",start_time);
    println!("Recording data to {log_out_filename:?} ...");
    let logfile = File::create(format!("./data/{}",log_out_filename))?;
    let mut csv_writer = BufWriter::new(logfile);

    const CSV_HEADER: &str =  "epoch_secs,heat,avg_C,eleco_dmA,eleco_rmA,elecm_mA,elecm_V,elec_R";
    println!("{}",CSV_HEADER);
    writeln!(csv_writer, "{}", CSV_HEADER)?;

    // Create an AtomicBool flag protected by Arc for thread-safe sharing
    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();

    // Set the Ctrl+C handler
    ctrlc::set_handler(move || {
        println!("\n====> Received Ctrl+C, initiating shutdown...");
        r.store(false, Ordering::SeqCst);
    }).expect("Error setting Ctrl-C handler");

    let mut furnace_state = FurnaceState  { 
        prior_max_temp_c: 0., 
        setpoint_c: PROBE_CHECK_TEMP_C, /// Heat until we can attempt probe insertion into melt
        measured_temp_c: 0., 
        heater_on: false 
    };

    let mut electrode_state = 
        ElectrodeState {
            drive_phase:DrivePhase::Warmup,
            inter_electrode_ohms:INF_INTER_ELECTRODE_OHMS,
            target_drive_ma:0.,
            reported_drive_ma:0.,
            measured_ma:0.,
            measured_volts:0., 
            growth_ma: 40. 
        };

    while running.load(Ordering::SeqCst) { 
        let current_instant = tokio::time::Instant::now();
        let current_utc_dt = chrono::Utc::now();
        let next_run_instant = current_instant + ONE_SEC_DURATION;

        control_furnace(&mut ctx, &mut furnace_state).await?;
        if electrode_state.drive_phase == DrivePhase::Cooldown ||
            furnace_state.measured_temp_c > PROBE_INSERTED_TEMP_C 
        {
            control_electrodes(&mut ctx, &mut electrode_state).await?;
        }
        else {
            electrode_state = WARMUP_ELECTRODE_STATE;
        }

        let log_line = format!( "{},{},{:.2},{:.3},{:.3},{:.3},{:.3},{:.1}",
            current_utc_dt.timestamp(),
            furnace_state.heater_on as u8,
            furnace_state.measured_temp_c,
            electrode_state.target_drive_ma, electrode_state.reported_drive_ma, electrode_state.measured_ma,
            electrode_state.measured_volts, electrode_state.inter_electrode_ohms,
        );
        println!("{}",log_line);
        writeln!(  csv_writer,"{}",  log_line)?;
        csv_writer.flush()?;
        
        // During warmup, the state does not change very quickly
        // This is an attempt to sync to about 1 Hz measurements
        sleep_until(next_run_instant).await;
    }

    // Attempt to shut off all outputs before exiting
    zero_control_outputs(&mut ctx).await?;
    ctx.disconnect().await?;

    Ok(())
}



