#![allow(unused)]

use std::f32::INFINITY;
use std::f32::consts::{PI, TAU};
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

/// How long to wait for series of modbus transactions to complete
const MODBUS_TRANSACTION_TIMEOUT: Duration = Duration::from_secs(4);

const MODBUS_RW_DELAY: Duration = Duration::from_millis(10);
/// minimum current stabilization time supported by the current source
const CURRENT_SOURCE_STABILIZATION_TIME: Duration = Duration::from_millis(25);
/// time we allow the current to settle before measuring
const CURRENT_SOURCE_WAIT_TIME: Duration = Duration::from_millis(250);
/// the guaranteed minimum "on" time for positive drive pulses
const CURRENT_PULSE_ON_TIME: Duration = Duration::from_millis(50); 
/// the guaranteed minimum "off" time between positive drive pulses
const CURRENT_PULSE_OFF_TIME: Duration = Duration::from_millis(5);

/// Average peak-to-peak interval for heating after warmup
const AVG_HEAT_CYCLE_DURATION_SEC: u64 = 260;
const AVG_PKPK_HEAT_CYCLE_MS: u64 = AVG_HEAT_CYCLE_DURATION_SEC * 1000;

/// Time limite for inter-electrode resistance gauging phase
const GAUGE_RESISTANCE_PHASE_DUR_MS: u64 = AVG_PKPK_HEAT_CYCLE_MS;
/// Time limit for minimum resistance to drop during Growth phase
const GROWTH_PHASE_MINR_LIMIT_MS: u64 = (3*AVG_PKPK_HEAT_CYCLE_MS)/2;

/// Rated maximum temperature of thermocouples (in this case, Type K)
const MAX_PROBE_TEMP_C:f32 = 1000.;
/// Temp at which we attempt to submerge thermo probes in electrolyte melt
const PROBE_CHECK_TEMP_C:f32 = 550.; 
/// Temp we expect to see when probe is succesfully inserted into melt
const PROBE_INSERTED_TEMP_C:f32 = 600.;
/// The center temperature we are trying to achieve for the electrolyte melt
const ELECTROLYTE_TARGET_TEMP_C:f32 = 777.;
/// Above this temperature the furnace heat is out of control
const EXCESSIVE_HEAT_TEMP_C:f32 = 800.;

const CUT_OUT_ABOVE_TARGET_TEMP_C: f32 = 12.5;
const CUT_IN_ABOVE_TARGET_TEMP_C: f32 = 7.5;


/// If the electrodes were shorted together at room temperature, what resistance do we expect?
const MIN_INTER_ELECTRODE_OHMS: f32 = 9.;

/// The limit of dR/dt magnitude indicating that we're reaching a transition
const OHM_RATE_SETTLING_LIMIT: f32 = 5.0;

/// The EWMA of dR/dt drops below this value when a bridge forms across electrodes
const BRIDGE_OHM_RATE_CLIFF: f32 = -0.6;

/// A guess at what a stable dendrite resistance would be when it approaches the anode
const STABLE_DENDRITE_OHMS: f32 = 20.;
/// Arbitrary value for "infinite" resistance (open circuit) between electrodes
const INF_INTER_ELECTRODE_OHMS: f32 = 666E2;

/// Duty cycle percentage for the external hardware trigger on the current source
const CURRENT_SOURCE_DUTY_CYCLE: f32 = 0.2;

/// Maximum current the current source can provide
const MAX_CURRENT_SOURCE_MA: f32 = 100.;
/// The measured gap between requested and actual current supplied by the current source, when they diverge. 
const PLATEAU_CURRENT_GAP_MA: f32 = 18.0;

/// If reported current is greater than requested current, we may have some concerns
const PLATEAU_NEG_CURRENT_GAP_MA: f32 = -0.7;

/// Highest potential provided by current source (measured as 10.689) minus some uncertainty
const OPEN_CIRCUIT_VOLTS: f32 = 10.; 


/// Used to probe for electrolyte or carbon bridge conductivity
const PROBE_CURRENT_MA: f32 = 1.;
/// Used to gauge the initial (presumably zero-growth) inter-electrode resistance
const GAUGE_CURRENT_MA: f32 = 15. ; //5.;
/// Used when bridge has formed across electrodes
const BRIDGE_CREEP_MA: f32 = GAUGE_CURRENT_MA ;
/// Current to use to start anchoring phase
const INITIAL_ANCHORING_CURRENT_MA: f32 = 25.;

/// Growth phase variable current amplitude +/- added to mean value
const GROWTH_PHASE_VARIABLE_MA: f32 = MAX_CURRENT_SOURCE_MA / 5.;
/// Growth phase mean current value
const GROWTH_PHASE_MEAN_MA: f32 = MAX_CURRENT_SOURCE_MA * 0.8;

const GROWTH_PHASE_PERIOD_SEC: f32 = 20.; // 0.05 Hz  -- 20 second cycle
/// Growth phase sweep frequency
const GROWTH_PHASE_SWEEP_FREQUENCY: f32 = (1./GROWTH_PHASE_PERIOD_SEC); 

/// Used after we think we've achieved a solid carbon bridge 
const HOLDING_PROBE_CURRENT_MA: f32 = 1.0;
/// The minimum increment for drive current, as specified in the current source docs
const MIN_DRIVE_CURRENT_INCR_MA: f32 = 0.1;

const REPORTED_CURRENT_THRESHOLD_MA: f32 = 2.*MIN_DRIVE_CURRENT_INCR_MA;

/// Weighting alpha for calculating Exponential Weighted Moving Average of resistance
const RESISTANCE_EWMA_ALPHA: f32 = 0.05;


/// Update the given Exponential Weighted Moving Average with a new value
fn update_ewma(ewma: &mut f32, new_value: f32, alpha: f32) {
    *ewma = alpha * new_value + (1.0 - alpha) * *ewma;
}

/// 
/// Read the dual thermocouple ADC
/// 
async fn read_dual_tk_temps(ctx: &mut tokio_modbus::client::Context)
-> Result<(Option<f32>, Option<f32>), Box<dyn std::error::Error>> 
{
    read_ykktc1202_dual_tk_temps(ctx).await
}


/// 
/// Read the voltage across and current through active electrode pair.
/// 
async fn read_electrode_pair_iv_adc(ctx: &mut tokio_modbus::client::Context)
-> Result<(f32, f32), Box<dyn std::error::Error>> 
{
    read_wa8tai_volts_milliamps(ctx).await
}

///
/// Read just the voltage across the electrodes using IV ADC
/// 
async fn read_electrode_pair_volts(ctx: &mut tokio_modbus::client::Context)
-> Result<f32, Box<dyn std::error::Error>> 
{
    read_wa8tai_one_channel(ctx, 1).await
}

 /// 
 /// Set the output drive current of the test electrodes 
 /// 
async fn set_electrode_current_drive(ctx: &mut tokio_modbus::client::Context, milliamps: f32) -> Result<(), Box<dyn std::error::Error>> 
{
    set_ykpvccs0100_current_drive(ctx, milliamps).await
}

/// 
/// Verify that all the modules we expect to be connected to the RS-485 Modbus are,
/// in fact, connected.
/// 
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

// fn current_from_current_density_ma(density_ma_cm2: f64, wire_od_mm: f64, wire_len_mm: f64) -> f32 
// {
//     // Surface Area = PI * diameter * length 
//     // For 0.8 mm OD, 10 mm long, outer cylinder SA = 0.251 cm2
//     // 200 mA/cm2 * 0.251 = 50.265 mA
//     // For 0.8 mm OD,  3 mm long, SA = 0.024 cm2, current = 4.8 mA
//     let electrode_surface_area: f64 = std::f64::consts::PI * (wire_od_mm/10. * wire_len_mm/10.); 
//     (density_ma_cm2  * electrode_surface_area) as f32
// }

#[repr(u8)]
#[derive(Debug, Clone, PartialEq)]
enum DrivePhase {
    /// check that the electrode is immersed in conductive melt
    Warmup = 0, 
    /// Measure initial resistance between electrodes
    GaugeResistance = 1,
    /// Attach initial carbon atoms to cathode surface
    Anchoring = 2, 
     /// Growth of carbon chains between electrodes
    Growth = 3,
    /// Stable bridge formed across electrodes
    Bridged = 4, 
    /// Monitor the inter-electrode conductivity
    Holding = 5, 
} 

 /// 
 /// Redirect furnace on/off to actual modbus device.
 /// 
async fn toggle_furnace(ctx: &mut tokio_modbus::client::Context, active:bool)
-> Result<(), Box<dyn std::error::Error>> 
{
    sleep(MODBUS_RW_DELAY).await;
    toggle_r4dvi04_relay(ctx,4, active).await?;
    Ok(())
}

/// Shut off the furnace heater, shut off any current drive.
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

    /// Temperature set point
    pub setpoint_c: f32,

    /// Most recently measured temperature
    pub measured_temp_c: f32,

    pub heater_on: bool,

}

const INITIAL_FURNACE_STATE: FurnaceState = FurnaceState  { 
        setpoint_c: PROBE_CHECK_TEMP_C, 
        measured_temp_c: 0., 
        heater_on: false 
    };

///
/// Turn the furnace heating on/off based on setpoint and temperature
/// 
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
            if ch2_tk_opt.is_some() {  
                if abs_diff_ne!(tk1_c, tk2_c, epsilon=CUT_OUT_ABOVE_TARGET_TEMP_C) {
                    println!("TK1 {tk1_c:?}°C TK2 {tk2_c:?}°C ");
                }
                (tk1_c + tk2_c) / 2f32 
            }
            else { tk1_c.into()}
        }
        else if ch2_tk_opt.is_some() { tk2_c.into() }
        else { MAX_PROBE_TEMP_C as f32};
    
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

    // Dirt simple bangbang controller:
    // - Cut out the heater when temperature exceeds a cut out point above the target temperature
    // - Cut in the heater when temperature drops below a cut in point (above the target temperature)
    if state.heater_on {
        if state.measured_temp_c > (new_temp_setpoint_c + CUT_OUT_ABOVE_TARGET_TEMP_C) {
            //println!("set heater off at: {:.3} >= {:.3}", state.measured_temp_c, new_temp_setpoint_c);
            toggle_furnace(ctx, false).await?;
            state.heater_on = false;
        }
    }
    else { // !state.heater_on
        if state.measured_temp_c < (new_temp_setpoint_c  + CUT_IN_ABOVE_TARGET_TEMP_C) {
            //println!("set heater on at: {:.3} (target {:.3} )", state.measured_temp_c, new_temp_setpoint_c);
            toggle_furnace(ctx, true).await?;
            state.heater_on = true;
        }
    }

    state.setpoint_c = new_temp_setpoint_c;
    
    Ok(())
}


#[derive(Debug, Clone)]
pub struct ElectrodeState {
    drive_phase: DrivePhase,
    last_update_ms: i64,
    /// Time at which this drive phase started
    phase_start_ms: i64,
    /// Measured inter-electrode resistance (or infinite)
    measured_ohms: f32,
    /// Exponential moving average of inter-electrode resistance
    ohms_ewma: f32,
    /// Minimum of ohms EWMA
    min_ohms_ewma: f32,
    /// The last time the min_ohms_ewma changed
    minr_update_ms: i64,
    /// Maximum value of inter-electrode resistance at start of growth phase
    max_ohms_ewma: f32,
    /// Finite difference of prior ohms_ewma and new
    ohms_rate: f32,
    /// The drive current to send to the electrodes
    target_drive_ma: f32,
    /// The actual drive current reported by the current source
    reported_drive_ma: f32,
    /// The actual measured drive currrent (may be same as reported_drive_ma above 20 mA) 
    measured_ma: f32,
    /// The actual measured potential across the electrodes 
    measured_volts: f32,
    
    /// Timestamp when GaugeResistance phase started 
    phase_gauge_start_utc: i64,
    /// Timestamp when GaugeResistance ended
    phase_gauge_end_utc: i64,
    /// Timestamp when Anchoring phase ended
    phase_anchoring_end_utc: i64,
    /// Timestamp when Growth phase ended
    phase_growth_end_utc: i64,
    /// Timestamp when Bridged phase ended
    phase_bridged_end_utc: i64,
}

/// State the electrode controller is reset to when we're in "warmup" mode below the active melt temperature
const INITIAL_ELECTRODE_STATE: ElectrodeState = 
    ElectrodeState {
            drive_phase:DrivePhase::Warmup,
            last_update_ms:0,
            phase_start_ms:0,
            measured_ohms:INF_INTER_ELECTRODE_OHMS,
            ohms_ewma:0.,
            min_ohms_ewma: INF_INTER_ELECTRODE_OHMS,
            minr_update_ms:0,
            max_ohms_ewma: 0.,
            ohms_rate:0.,
            target_drive_ma: MIN_DRIVE_CURRENT_INCR_MA,
            reported_drive_ma:0.,
            measured_ma:0.,
            measured_volts:0., 
            phase_gauge_start_utc: 0,
            phase_gauge_end_utc: 0,
            phase_anchoring_end_utc: 0,
            phase_growth_end_utc: 0,
            phase_bridged_end_utc: 0,
        }; 


///  
/// Send one current drive pulse to the electrodes,
///  consisting of a drive to a positive potential for high_time, 
///  followed by holding zero DC for a measured low_time.
///
///  Note that after this function returns,
///  the output drive current should be ZERO.
/// 
async fn drive_one_pulse(ctx: &mut tokio_modbus::client::Context,
    state: &mut ElectrodeState, high_time: Duration, low_time: Duration,
)
-> Result<(f32, f32, f32), Box<dyn std::error::Error>> 
{
    // Drive output current pulse based on prior settings, and measure result
    set_electrode_current_drive(ctx, state.target_drive_ma).await?;
    sleep(high_time).await;

    let mut total_volts = 0.;
    // Measure the resulting induced current and potential across the electrodes
    state.reported_drive_ma = read_ykpvccs0100_current_drive(ctx).await?;

    let measured_volts = read_electrode_pair_volts(ctx).await?;

    let measured_milliamps: f32 = 
        if state.target_drive_ma > 0.  && measured_volts < OPEN_CIRCUIT_VOLTS {  state.reported_drive_ma }  
        else { 0. };
    let measured_ohms = 
        if measured_milliamps > 0. {
            // this also covers the case where volts = 0.0, i.e. zero resistance.
            (1000. * measured_volts) / measured_milliamps 
        }
        else {
            INF_INTER_ELECTRODE_OHMS // arbitrary value based on previous experiments
        };

    // zero output after pulse
    set_electrode_current_drive(ctx, 0.).await?;
    sleep(low_time).await;

    return Ok((measured_volts, measured_milliamps, measured_ohms))

}


/// 
/// Set the electrode current and measure its response
/// 
async fn drive_current_and_measure(ctx: &mut tokio_modbus::client::Context,
    state: &mut ElectrodeState, settling_time: Duration
)
-> Result<(f32, f32, f32), Box<dyn std::error::Error>> 
{
    // Drive output current pulse based on prior settings, and measure result
    set_electrode_current_drive(ctx, state.target_drive_ma).await?;
    sleep(settling_time).await;

    // Measure the resulting induced current and potential across the electrodes
    state.reported_drive_ma = read_ykpvccs0100_current_drive(ctx).await?;

    let mut total_volts = 0.;
    for i in 0..3 {
        total_volts += read_electrode_pair_volts(ctx).await?;
        sleep(Duration::from_millis(25));
    }
    let measured_volts = total_volts / 3.;

    let measured_milliamps: f32 = 
        if state.target_drive_ma > 0.  && state.reported_drive_ma > REPORTED_CURRENT_THRESHOLD_MA  {  state.reported_drive_ma }  
        else { 0. };
    let measured_ohms = 
        if measured_milliamps > 0. {
            // this also covers the case where volts = 0.0, i.e. zero resistance.
            (1000. * measured_volts) / measured_milliamps 
        }
        else {
            INF_INTER_ELECTRODE_OHMS // arbitrary value based on previous experiments
        };

    return Ok((measured_volts, measured_milliamps, measured_ohms))
}


/// 
/// Adjust the electrode current based on melt condition and drive phase
/// 
async fn control_electrodes(ctx: &mut tokio_modbus::client::Context, 
    state: &mut ElectrodeState,
) 
-> Result<(), Box<dyn std::error::Error>> 
{
    let mut start_drive_utc_dt = chrono::Utc::now();
    let mut start_drive_ms = start_drive_utc_dt.timestamp_millis();
    
    // Drive output current pulse based on prior settings, and measure result
    let (measured_volts, measured_milliamps, measured_ohms) = 
        drive_current_and_measure(ctx, state, CURRENT_SOURCE_WAIT_TIME).await?;

    let mut end_drive_utc_dt = chrono::Utc::now();
    let mut end_drive_ms = end_drive_utc_dt.timestamp_millis();
    let mut drive_duration_ms = end_drive_ms - start_drive_ms;

    let drive_duration_sec = (drive_duration_ms as f32)/1000.;
    let phase_duration_ms = 
        if state.phase_start_ms <  end_drive_ms {  (end_drive_ms - state.phase_start_ms) as u64 } 
        else { 0 };

    // if  state.measured_ma== 0. {
    //     println!("{} phase {} :  drive {:.3} mA  measured 0 mA", 
    //     start_drive_utc_dt.timestamp(), state.drive_phase.clone() as u8, state.target_drive_ma);
    // }

    // reuse old drive current until instructed otherwise
    let mut new_drive_ma: f32 = state.target_drive_ma;
    
    // Check for significant drops in resistance
    if state.measured_ma != 0. && measured_ohms != INF_INTER_ELECTRODE_OHMS   {
        let prior_ohms_ewma = state.ohms_ewma;
        update_ewma(&mut state.ohms_ewma, measured_ohms, RESISTANCE_EWMA_ALPHA);

        let dr_dt = 
            if prior_ohms_ewma != INF_INTER_ELECTRODE_OHMS { state.ohms_ewma - prior_ohms_ewma }
            else { 0. };
        state.ohms_rate = dr_dt;
 
        // only evaluate minimum phase ohms when rate is not extreme 
        if phase_duration_ms > 30000 {
            // if state.ohms_rate_ewma != 0. 
            //     && state.ohms_rate_ewma.abs() < OHM_RATE_SETTLING_LIMIT  {
                if state.ohms_ewma < state.min_ohms_ewma {
                    println!("{} minR {:.3} -> {:.3}", end_drive_utc_dt.timestamp(), state.min_ohms_ewma, state.ohms_ewma);
                    state.min_ohms_ewma = state.ohms_ewma;
                    state.minr_update_ms = end_drive_ms;
                }
            // }
        }
    }

    state.measured_ohms = measured_ohms;
    state.measured_volts = measured_volts;
    state.measured_ma = measured_milliamps;

    let measured_gap = state.target_drive_ma - measured_milliamps;

    let current_gap: f32 = 
        if state.target_drive_ma >= PROBE_CURRENT_MA {
            measured_gap
        }
        else { 100. }; // outrageously large current gap

    // Then calculate any drive phase transitions
    match state.drive_phase {
        DrivePhase::Warmup => {
            new_drive_ma = GAUGE_CURRENT_MA;
            // println!("gap: {:0.3}", current_gap);

            // TODO TMP! -- skip to Growth phase directly
            if state.measured_ma > 0.7 && phase_duration_ms > 60000 {
                state.drive_phase = DrivePhase::Growth;
                new_drive_ma = GROWTH_PHASE_MEAN_MA;
                state.phase_start_ms = end_drive_ms;
                state.phase_gauge_end_utc = end_drive_utc_dt.timestamp();
                println!("{} end GaugeResistance phase: Ohms min {:.3} max {:.3} Ohms ({} ms) ",
                    state.phase_gauge_end_utc, state.min_ohms_ewma, state.max_ohms_ewma, 
                    phase_duration_ms);
                // reset min-max for next phase
                state.min_ohms_ewma = INF_INTER_ELECTRODE_OHMS;
                state.max_ohms_ewma = 5.;
                state.minr_update_ms = end_drive_ms;
            }
        }
        DrivePhase::GaugeResistance => {
            new_drive_ma = GAUGE_CURRENT_MA;
            if state.max_ohms_ewma < state.ohms_ewma {
                state.max_ohms_ewma = state.ohms_ewma;
            }
            // continue probing over multiple heat/cool cycles to characterize resistance
            if phase_duration_ms > GAUGE_RESISTANCE_PHASE_DUR_MS {
                state.drive_phase = DrivePhase::Anchoring;
                new_drive_ma = INITIAL_ANCHORING_CURRENT_MA;
                state.phase_start_ms = end_drive_ms;
                state.phase_gauge_end_utc = end_drive_utc_dt.timestamp();
                println!("{} end GaugeResistance phase: Ohms min {:.3} max {:.3} Ohms ({} ms) ",
                    state.phase_gauge_end_utc, state.min_ohms_ewma, state.max_ohms_ewma, 
                    phase_duration_ms);
                // reset min-max for next phase
                state.min_ohms_ewma = state.ohms_ewma;
                state.max_ohms_ewma = state.ohms_ewma;
            }
        }
        DrivePhase::Anchoring => { 
            // TODO current gap is unreliable at the moment for kHz triggered output
            if state.target_drive_ma > 60. {
            // if current_gap > PLATEAU_CURRENT_GAP_MA || (current_gap < PLATEAU_NEG_CURRENT_GAP_MA ) {
                // the actual current has diverged from desired current
                state.drive_phase = DrivePhase::Growth;
                state.phase_start_ms = end_drive_ms;
                state.ohms_rate = 0.; //reset because it's scrambled by prior descent
                state.phase_anchoring_end_utc = end_drive_utc_dt.timestamp();
                println!("{} end Anchoring phase with min {:.3} max {:.3} Ohms, gap {:.3} mA ({} ms)", 
                    state.phase_anchoring_end_utc, 
                    state.min_ohms_ewma, state.max_ohms_ewma, current_gap,
                    phase_duration_ms
                );
                state.min_ohms_ewma = state.ohms_ewma;
                state.minr_update_ms = end_drive_ms;
                state.max_ohms_ewma = state.ohms_ewma;
            }
            else if current_gap >= 0. {
                if current_gap > 1.0 {
                    // slow down the rate of increasing current
                    new_drive_ma = state.target_drive_ma + MIN_DRIVE_CURRENT_INCR_MA;
                }
                else {
                    // increase the desired drive current until reported current diverges
                    new_drive_ma = state.target_drive_ma + 5.*MIN_DRIVE_CURRENT_INCR_MA;
                }
            }
        }
        DrivePhase::Growth => {  
            let minr_drop_delay = if end_drive_ms > state.minr_update_ms { end_drive_ms - state.minr_update_ms } else { 0};
            let growth_stalled = (minr_drop_delay > GROWTH_PHASE_MINR_LIMIT_MS as i64);
            // TODO check for bridge formation, somehow
            // let bridge_formed =   state.ohms_rate_ewma < BRIDGE_OHM_RATE_CLIFF ;

            if growth_stalled {
                // switch to bridge preservation
                state.drive_phase = DrivePhase::Bridged;
                state.phase_start_ms = end_drive_ms;
                state.phase_growth_end_utc = end_drive_utc_dt.timestamp();
                println!("{:?} end Growth phase with V {:.2} Rew {:.2} dR/dt {:.3} ({} ms)", 
                    state.phase_growth_end_utc, state.measured_volts, 
                    state.ohms_ewma, state.ohms_rate,
                    phase_duration_ms
                );
            }
            else {
                new_drive_ma = growth_current_at_time_ms(end_drive_ms , state.phase_start_ms);
            }
        }
        DrivePhase::Bridged => {
            if state.measured_ohms < MIN_INTER_ELECTRODE_OHMS  {
                // inter-electrode resistance is close to zero, indicating that the electrode-electrode gap has been bridged
                state.drive_phase = DrivePhase::Holding;
                state.phase_start_ms = end_drive_ms;
                state.phase_bridged_end_utc = end_drive_utc_dt.timestamp();
                new_drive_ma = HOLDING_PROBE_CURRENT_MA;
                println!("{} end Bridge phase with V {:.2} R {:.2} dR/dt {:.3} ({} ms)", 
                    state.phase_bridged_end_utc, state.measured_volts, state.measured_ohms, state.ohms_rate,
                    phase_duration_ms
                );
            }
            else if new_drive_ma > BRIDGE_CREEP_MA {
                new_drive_ma -= 5.*MIN_DRIVE_CURRENT_INCR_MA;
            }
        }
        DrivePhase::Holding => {
            new_drive_ma = HOLDING_PROBE_CURRENT_MA;
        }
        _ => {
        }
    };

    // Now, update the drive current for the next main loop iteration
    // state.reported_drive_ma = set_electrode_current_drive(ctx, new_drive_ma).await?;
    state.target_drive_ma = new_drive_ma;
    state.last_update_ms = chrono::Utc::now().timestamp_millis();

    Ok(())
}

///
/// Calculate the value of the desired growth current at a given time
fn growth_current_at_time_ms(timestamp_ms: i64, zero_ms: i64) -> f32
{
    let offset_time_sec: f32 = ((timestamp_ms  - zero_ms) as f32)/1000.;
    let current = GROWTH_PHASE_MEAN_MA + GROWTH_PHASE_VARIABLE_MA * ((TAU * GROWTH_PHASE_SWEEP_FREQUENCY * offset_time_sec ).sin());
    current
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

    const CSV_HEADER: &str =  "epoch_secs,heat,avg_C,eleco_mA,elecm_mA,elecm_V,elec_R,Rew,MinRew,dRdTew";
    macro_rules! CSV_LINE_FORMAT { () => { "{},{},{:.2},{:.2},{:.2},{:.3},{:.3},{:.3},{:.3},{:.3}" } }
    
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

    let mut furnace_state = INITIAL_FURNACE_STATE;
    let mut electrode_state =  INITIAL_ELECTRODE_STATE;
    electrode_state.phase_start_ms =  chrono::Utc::now().timestamp_millis();

    let mut loop_count = 0;

    // First run main loop at next second
    let mut now_instant = tokio::time::Instant::now();
    let first_loop_instant = now_instant + INTER_LOOP_DELAY - Duration::from_millis(now_instant.elapsed().subsec_millis() as u64);
    sleep_until(first_loop_instant).await;

    while running.load(Ordering::SeqCst) { 
        now_instant = tokio::time::Instant::now();
        let current_utc_dt = chrono::Utc::now();
        let next_run_instant = now_instant + INTER_LOOP_DELAY - Duration::from_millis(now_instant.elapsed().subsec_millis() as u64);

        let furnace_res = 
            tokio::time::timeout(MODBUS_TRANSACTION_TIMEOUT,control_furnace(&mut ctx, &mut furnace_state)).await;
        if !furnace_res.is_ok() { 
            running.store(false, Ordering::SeqCst);
            eprintln!("control_furnace timeout: {:?}",furnace_res);
            continue;
        }

        if (furnace_state.measured_temp_c > PROBE_INSERTED_TEMP_C && 
            furnace_state.measured_temp_c < EXCESSIVE_HEAT_TEMP_C) ||
            electrode_state.drive_phase == DrivePhase::Holding 
        {
            let elec_res = 
                tokio::time::timeout(MODBUS_TRANSACTION_TIMEOUT, control_electrodes(&mut ctx, &mut electrode_state)).await;
            if !elec_res.is_ok() { 
                running.store(false, Ordering::SeqCst);
                eprintln!("control_electrodes timeout: {:?}",elec_res);
                continue;
            }
        }
        else {
            electrode_state = INITIAL_ELECTRODE_STATE;
            electrode_state.phase_start_ms = current_utc_dt.timestamp_millis();
        }

        let log_line = format!( CSV_LINE_FORMAT!(),
            current_utc_dt.timestamp(),
            furnace_state.heater_on as u8,
            furnace_state.measured_temp_c,
            electrode_state.target_drive_ma, electrode_state.measured_ma,
            electrode_state.measured_volts, 
            electrode_state.measured_ohms, electrode_state.ohms_ewma, electrode_state.min_ohms_ewma,
            electrode_state.ohms_rate
        );
        println!("{}",log_line);
        writeln!(  csv_writer,"{}",  log_line)?;
        loop_count = (loop_count + 1) % 4;
        if loop_count == 0 { csv_writer.flush(); }

        // Attempt to sync to about 1 Hz measurements
        sleep_until(next_run_instant).await;
    }

    println!("Flushing log file...");
    csv_writer.flush()?;

    // Attempt to shut off all outputs before exiting.
    // We disconnect and reconnect to modbus to flush any cruft at the WiFi bridge
    ctx.disconnect().await?;
    sleep(Duration::from_secs(2));
    println!("Reconnecting to: '{socket_addr:?}'");
    ctx = tcp::connect(socket_addr).await?;
    zero_control_outputs(&mut ctx).await?;
    ctx.disconnect().await?;

    // dump logged timeline
    if electrode_state.phase_gauge_start_utc > 0 {
        println!("{} Gauge started",electrode_state.phase_gauge_start_utc);
    }
    if electrode_state.phase_gauge_end_utc > 0 {
        println!("{} Gauge ended",electrode_state.phase_gauge_end_utc);
    }
    if electrode_state.phase_anchoring_end_utc > 0 {
        println!("{} Anchoring ended",electrode_state.phase_anchoring_end_utc);
    }
    if electrode_state.phase_growth_end_utc > 0 {
        println!("{} Growth ended",electrode_state.phase_growth_end_utc);
    }
    if electrode_state.phase_bridged_end_utc > 0 {
        println!("{} Bridged ended", electrode_state.phase_bridged_end_utc);
    }

    Ok(())
}



