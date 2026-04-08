#![allow(unused)]

use std::default;
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


const INTER_LOOP_DELAY: Duration = Duration::from_millis(1000);
const MODBUS_RW_DELAY: Duration = Duration::from_millis(25);
const CURRENT_SOURCE_STABILIZATION_TIME: Duration = Duration::from_millis(25);
const HOLD_ZERO_PULSE_TIME:Duration = Duration::from_millis(100);

/// Rated maximum temperature of thermocouples (in this case, Type K)
const MAX_PROBE_TEMP_C:CelsiusF32 = 1000.;
/// Temp at which we attempt to submerge thermo probes in electrolyte melt
const PROBE_CHECK_TEMP_C:CelsiusF32 = 550.; 
/// Temp we expect to see when probe is succesfully inserted into melt
const PROBE_INSERTED_TEMP_C:CelsiusF32 = 600.;
/// The center temperature we are trying to achieve for the electrolyte melt
const ELECTROLYTE_TARGET_TEMP_C:CelsiusF32 = 780.;

/// If the electrodes were shorted together at room temperature, what resistance do we expect?
const MIN_INTER_ELECTRODE_OHMS: OhmsF32 = 10.;
/// The EWMA of dR/dt drops below this value when a bridge forms
const BRIDGE_OHM_RATE_CLIFF: f32 = -0.75;
/// A guess at what a stable bridge resistance would be when it approaches the anode
const STABLE_BRIDGE_OHMS: OhmsF32 = 15.;
/// Arbitrary value for "infinite" resistance (open circuit) between electrodes
const INF_INTER_ELECTRODE_OHMS: OhmsF32 = 666E2;

/// The measured gap between requested and actual current supplied by the current source, when they diverge. 
const PLATEAU_CURRENT_GAP_MA: MilliampsF32 = 6.0;
/// Measured anchoring current for small (~1 mm) electrode separation
const SMALL_SEP_ANCHOR_CURRENT_MA: MilliampsF32 = 25.;
/// Brute force anchoring overcurrent (based on measured values for small gaps)
const BRUTE_ANCHOR_CURRENT_MA: MilliampsF32 = 40.; 
/// Brute force anchoring expected current gap
const BRUTE_CURRENT_GAP_MA: MilliampsF32 = BRUTE_ANCHOR_CURRENT_MA; 
/// Highest potential provided by current source (measured as 10.689) minus some uncertainty
const OPEN_CIRCUIT_VOLTS: VoltsF32 = 9.; 

/// Used when dendrite has formed
const DENDRITE_CREEP_MA: MilliampsF32 = 4.; 

/// Exponential decay constant for decreasing current after bridge forms, Higher = faster decay
const BRIDGE_CURRENT_DECAY: f64 = 0.02; 


/// Used to probe for electrolyte or carbon bridge conductivity
const PROBE_CURRENT_MA: MilliampsF32 = 1.;
/// Used after we think we've achieved a solid carbon bridge 
const COOLDOWN_PROBE_CURRENT_MA: MilliampsF32 = 0.5;
/// The minimum increment for drive current, as specified in the current source docs
const MIN_DRIVE_CURRENT_INCR_MA: MilliampsF32 = 0.1;
/// The range of the ammeter
const MAX_AMMETER_VAL: MilliampsF32 = 20.0;



/// Weighting alpha for calculating Exponential Weighted Moving Average of resistance
const RESISTANCE_EWMA_ALPHA: f32 = 0.1;

/// Update the given Exponential Weighted Moving Average with a new value
fn update_ewma(ewma: &mut f32, new_value: f32, alpha: f32) {
    *ewma = alpha * new_value + (1.0 - alpha) * *ewma;
}

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
-> Result<(VoltsF32, MilliampsF32), Box<dyn std::error::Error>> 
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
    set_ykpvccs0100_current_drive(ctx, milliamps).await
}

/**
 * Verify that all the modules we expect to be connected to the RS-485 Modbus are,
 * in fact, connected.
 */
async fn enumerate_required_modules(ctx: &mut tokio_modbus::client::Context) -> Result<(), Box<dyn std::error::Error>> 
{

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
#[derive(Debug, Copy, Clone, PartialEq)]
enum DrivePhase {
    /// check that the electrode is immersed in conductive melt
    Warmup = 0, 
    /// Attach initial carbon atoms to cathode surface
    SlowRiseAnchoring = 1, 
    /// Force high current pulses to achieve anchoring quickly
    BruteAnchoring = 2,
    /// Growth of carbon dendrites from cathode to anode
    Growth = 3,
    /// We've detected a dendrite bridge across the electrodes
    StabilizeBridge = 4, 
    /// Monitor the inter-electrode conductivity
    MonitorBridge = 5, 
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
    // set_electrode_current_drive(ctx,0.).await?;
    println!("Outputs disabled.");
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


    // Create an AtomicBool flag protected by Arc for thread-safe sharing
    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();

    // Set the Ctrl+C handler
    ctrlc::set_handler(move || {
        println!("\n====> Received Ctrl+C, initiating shutdown...");
        r.store(false, Ordering::SeqCst);
    }).expect("Error setting Ctrl-C handler");

    // enable the furnace to begin with
    let base_heating_utc_dt = chrono::Utc::now();
    let mut heat_cycle_start_dt = base_heating_utc_dt;
    println!("{} start heating", base_heating_utc_dt.timestamp());

    let mut heat_on = true;
    toggle_furnace(&mut ctx, heat_on);

    const MOLTEN_FROM_COLD_TIME_MINUTES: i64 = 15;
    const COOLDOWN_TIME_SECS: i64 = 60;
    const HEAT_MAINT_TIME_SECS: i64 = COOLDOWN_TIME_SECS * 4;

    while running.load(Ordering::SeqCst) { 
        let current_instant = tokio::time::Instant::now();
        let current_utc_dt = chrono::Utc::now();
        let next_run_instant = current_instant + INTER_LOOP_DELAY;

        let base_delta_dt = current_utc_dt.signed_duration_since(base_heating_utc_dt);
        let cycle_delta_dt = current_utc_dt.signed_duration_since(heat_cycle_start_dt);

        // about as long as it takes to fully melt from cold
        if base_delta_dt.num_minutes() > 20 {
            let cycle_seconds = cycle_delta_dt.num_seconds();
            if heat_on {
                 // turn off heat after some number of seconds
                if cycle_seconds > HEAT_MAINT_TIME_SECS {
                    println!("{} toggle heat off",current_utc_dt.timestamp());
                    heat_on = false;
                    heat_cycle_start_dt = current_utc_dt;
                }
            }
            else {
                if cycle_seconds > COOLDOWN_TIME_SECS {
                    println!("{} toggle heat on",current_utc_dt.timestamp());
                    heat_on = true;
                    heat_cycle_start_dt = current_utc_dt;
                }
            }
            toggle_furnace(&mut ctx, heat_on);
        }

        println!("{} {} {:.2} furnace heating: {}",current_utc_dt.timestamp(), base_delta_dt.num_minutes(), cycle_delta_dt.as_seconds_f32(), heat_on );

        // sync to about 1 Hz loops
        sleep_until(next_run_instant).await;
    }

    // Attempt to shut off all outputs before exiting
    zero_control_outputs(&mut ctx).await?;
    ctx.disconnect().await?;

    Ok(())
}



