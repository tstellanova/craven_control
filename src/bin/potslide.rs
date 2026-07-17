// #![allow(unused)]


use std::time::Duration;
use tokio::time::{sleep, sleep_until};
use tokio_modbus::prelude::*;

use tokio_modbus::client::{Client};
use tokio::io::{AsyncBufReadExt, BufReader};

use std::fs::File;
use std::io::{BufWriter, Write};

use approx::{abs_diff_ne};
use craven_control::*;

/// This dictates, on average, how often the main loop runs 
const INTER_LOOP_DELAY: Duration = Duration::from_millis(1000);

/// Max time to wait for series of modbus transactions to complete
const MODBUS_TRANSACTION_TIMEOUT: Duration = Duration::from_secs(5);

/// Pause in Modbus commands for "important" commands
const MODBUS_RW_DELAY: Duration = Duration::from_millis(10);

const MAINLOOP_DELAY: Duration = Duration::from_millis(100);

/// minimum current stabilization time supported by the current source
const CURRENT_SOURCE_STABILIZATION_MS: u64 = 25;
/// time we allow the current to settle, after driving, before measuring
const CURRENT_SOURCE_WAIT_TIME: Duration = Duration::from_millis(CURRENT_SOURCE_STABILIZATION_MS);

// /// Empirically-derived average peak-to-peak interval for heating after warmup
// const AVG_HEAT_CYCLE_DURATION_SEC: u64 = 260;
// const AVG_PKPK_HEAT_CYCLE_MS: u64 = AVG_HEAT_CYCLE_DURATION_SEC * 1000;

/// Minimum time for Warmup phase
const WARMUP_PHASE_DUR_MS: u64 = 30*1000; 

/// Rated maximum temperature of thermocouples (in this case, Type K)
const MAX_PROBE_TEMP_C:f32 = 1000.;
/// Temp at which we attempt to submerge thermo probes in electrolyte melt
const PROBE_CHECK_TEMP_C:f32 = 550.; 
/// Temp we expect to see when probe is succesfully inserted into melt
const PROBE_INSERTED_TEMP_C:f32 = 600.;
/// The center temperature we are trying to achieve for the electrolyte melt
const ELECTROLYTE_TARGET_TEMP_C:f32 = 777.;
/// Below this temperature we don't drive start driving current through the electrodes.
const MIN_ELECTRODE_CHECK_TEMP_C:f32 = ELECTROLYTE_TARGET_TEMP_C - 12.;
/// The temperature at which the heater should cut in (turn on)
const CUT_IN_ABOVE_TARGET_TEMP_C: f32 = 5.;
/// The temperature at which the heater should cut out (turn off)
const CUT_OUT_ABOVE_TARGET_TEMP_C: f32 = 10.;
/// Above this temperature the furnace heat is out of control
const EXCESSIVE_HEAT_TEMP_C:f32 = ELECTROLYTE_TARGET_TEMP_C + 22.5;

/// Arbitrary value for "infinite" resistance (open circuit) between electrodes
const INF_INTER_ELECTRODE_OHMS: f32 = 666.;
/// Below this resistance value we terminate the Cyclic phase
const CYCLIC_LOWV_TERMINATION_OHMS: f32 = 5.0;

// /// Maximum current the current source can provide
// const MAX_CURRENT_SOURCE_MA: f32 = 1000.;
// /// Highest potential provided by current source (measured as 10.689) minus some uncertainty
// const OPEN_CIRCUIT_VOLTS: f32 = 10.; 


/// Pre-estimated surface area of electrode probe
const ELECTRODE_SURFACE_MM2:f32 = std::f32::consts::PI*(1.)*25.; // 1 mm diameter, about 25 mm long

/// Ideal current density for establishing nucleation sites on the cathode surface
const NUCLEATION_CURRENT_DENSITY_AMPS_CM2:f32 = 0.01;
const NUCLEATION_CURRENT_DENSITY_MA_MM2:f32 = (NUCLEATION_CURRENT_DENSITY_AMPS_CM2 * 1000.)/100.;
/// Maximum allowed current density during Nucleation phase
const MAX_NUCLEATION_CURRENT_MA:f32 =  ELECTRODE_SURFACE_MM2 * NUCLEATION_CURRENT_DENSITY_MA_MM2;

/// Ideal current density for growing elongated CNTs from the nucleation sites
const ELONGATION_CURRENT_DENSITY_AMPS_CM2:f32 = 0.4;
const ELONGATION_CURRENT_DENSITY_MA_MM2:f32 = (ELONGATION_CURRENT_DENSITY_AMPS_CM2 * 1000.)/100.;
/// Maximum allowed current density during Cyclic growth phase
const MAX_ELONGATION_CURRENT_MA:f32 =  ELECTRODE_SURFACE_MM2 * ELONGATION_CURRENT_DENSITY_MA_MM2;


/// Highest voltage potential to use during Cyclic drive phase, where carbon growth is driven. 
const CYCLIC_GROWTH_PEAK_V: f32 = 2.2;
/// Lowest voltage to use during Cycling phase, where true inter-electrode resistance can be measured. 
const CYCLIC_GROWTH_FLOOR_V: f32 = 1.4;
/// Voltage at which to measure "Low V" minimum resistance
const CYCLIC_LOWV_MINR_MEASURE_V: f32 = 1.6;

/// The duration of the High voltage growth segment of the Cyclic phase
const CYCLIC_HIGHV_DURATION_MS: u64 = 80*1000;
/// The duration of the Low voltage measurement segment of the Cyclic phase
const CYCLIC_LOWV_DURATION_MS: u64 = 20*1000;
/// Total duration of the combined high/low Cyclic phase drive cycle
const CYCLIC_PERIOD_MS: u64 = CYCLIC_LOWV_DURATION_MS + CYCLIC_HIGHV_DURATION_MS;

/// The minimum increment for drive current, as specified in the current source docs
const MIN_DRIVE_CURRENT_INCR_MA: f32 = 1.0;
/// Used after we think we've achieved a robust carbon bridge 
const HOLDING_PROBE_CURRENT_MA: f32 = 2. * MIN_DRIVE_CURRENT_INCR_MA;
/// We only recognize current values reported by the current source above this threshold
const REPORTED_CURRENT_THRESHOLD_MA: f32 = MIN_DRIVE_CURRENT_INCR_MA;
/// Fixed Warmup phase current
const WARMUP_CURRENT_MA: f32 = 5.;
/// Fall back to this current value during Cyclic drive phase when resistance is unknown.
const CYCLIC_PHASE_FALLBACK_MA: f32 = 100.;

/// Weighting alpha for Exponential Weighted Moving Average of resistance
const RESISTANCE_EWMA_ALPHA: f32 = 0.4;

/// Update the given Exponential Weighted Moving Average with a new value
fn update_ewma(ewma: &mut f32, new_value: f32, alpha: f32) {
    *ewma = alpha * new_value + (1.0 - alpha) * *ewma;
}

/// Read the dual thermocouple ADC
async fn read_dual_tk_temps(ctx: &mut tokio_modbus::client::Context)
-> Result<(Option<f32>, Option<f32>), Box<dyn std::error::Error>> 
{
    read_ykktc1202_dual_tk_temps(ctx).await
}

 /// Set the output drive current of the test electrodes 
async fn set_electrode_current_drive(ctx: &mut tokio_modbus::client::Context, milliamps: f32) -> Result<(), Box<dyn std::error::Error>> 
{
    // set_ykpvccs0100_current_drive(ctx, milliamps).await
    set_ykpvccs1000_current_drive(ctx, milliamps).await
}

/// Read the reported current from the current source
async fn read_electrode_current_drive(ctx: &mut tokio_modbus::client::Context) -> Result<f32, Box<dyn std::error::Error>> 
{
    //read_ykpvccs0100_current_drive(ctx).await
    read_ykpvccs1000_current_drive(ctx).await
}


/// 
/// Verify that all the modules we expect to be connected to the RS-485 Modbus are,
/// in fact, connected.
/// 
async fn enumerate_required_modules(ctx: &mut tokio_modbus::client::Context) -> Result<(), Box<dyn std::error::Error>> 
{
    // measures dual type-K thermocouples
    ping_one_modbus_node_id(ctx, NODEID_YKKTC1202_DUAL_TK, REG_NODEID_YKKTC1202_DUAL_TK).await?;

    // measures voltage and current across the electrodes
    // TODO we can't ping WDCU3003M with a node ID read, because it doesn't expose node ID to Modbus
    // ping_one_modbus_node_id(ctx, NODEID_WDCU3003_IV_ADC, 0x00).await?;


    ctx.set_slave(Slave(NODEID_WDCU3003_IV_ADC));
    let wdc3003_vals: Vec<u16> = ctx.read_holding_registers(0, 10).await??;
    print!("wdc3003_vals: {:?}", wdc3003_vals);

    // supplies current to the cathode and anodes
    ping_one_modbus_node_id(ctx,NODEID_YKPVCCS010_CURR_SRC, REG_NODEID_YKPVCCS010_CURR_SRC).await?;

    // controls furnace on/off
    // controls 4-pair anode connection relays
    ping_one_modbus_node_id(ctx, NODEID_WAV_OCTO_RELAY, REG_NODEID_WAVESHARE_V2).await?;

    Ok(())
}

#[repr(u8)]
#[derive(Debug, Clone, PartialEq)]
enum DrivePhase {
    /// No prior state
    Fresh = 0,
    /// Check that the electrode is immersed in conductive melt
    Warmup = 1, 
    /// Establishing nucleation sites on the cathode
    Nucleation = 2,
    /// Alternating measure/grow cycle
    Elongation = 3,
    /// Monitor the inter-electrode conductivity
    Holding = 4, 
    /// Number of drive phases
    Max,
} 

 /// 
 /// Redirect furnace on/off to actual modbus device.
 /// 
async fn toggle_furnace(ctx: &mut tokio_modbus::client::Context, active:bool)
-> Result<(), Box<dyn std::error::Error>> 
{
    const FURNACE_RELAY_CHANNEL: u8 = 7;
    sleep(MODBUS_RW_DELAY).await;
    toggle_wav_octo_relay(ctx, FURNACE_RELAY_CHANNEL, active).await
}

/// Shut off the furnace heater, shut off any current drive.
async fn zero_control_outputs(ctx: &mut tokio_modbus::client::Context)
-> Result<(), Box<dyn std::error::Error>> 
{
    println!("Shutting down outputs...");
    toggle_furnace(ctx, false).await?;
    set_electrode_current_drive(ctx,0.).await?;

    let  anode_channels= [false; 4];
    write_wav_octo_relays(ctx, &anode_channels).await?;

    println!("Outputs disabled.");
    Ok(())
}

#[derive(Debug, Clone)]
pub struct FurnaceState {

    /// Temperature set point
    pub setpoint_c: f32,
    /// Most recently measured temperature
    pub measured_temp_c: f32,
    /// Whether the furnace heater is turned on
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
    let new_temp_setpoint_c: f32;

    // first, measure temperature
    let (ch1_tk_opt, ch2_tk_opt) = read_dual_tk_temps(ctx).await?;
    let tk1_c = ch1_tk_opt.unwrap_or(0f32);
    let tk2_c = ch2_tk_opt.unwrap_or(0f32);
    let avg_core_tk_c: f32 = 
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
    /// UTC epoch milliseconds at which the state was last updated
    last_update_ms: i64,
    /// UTC epoch milliseconds at which this drive phase started
    phase_start_ms: i64,
    /// Measured inter-electrode resistance (or infinite)
    measured_ohms: f32,
    /// Exponential moving average of inter-electrode resistance
    ohms_ewma: f32,

    /// Minimum resistance measured during Low-voltage drive
    lowv_minr_ohms: f32,
    /// The last time lowv_minr changed
    lowv_minr_update_ms: i64,
    /// Minimum resistance measured during High-voltage drive
    highv_minr_ohms: f32,
    /// The last time highv_minr changed
    highv_minr_update_ms: i64,

    /// Maximum value of inter-electrode resistance
    max_ohms_ewma: f32,

    /// The drive current to send to the electrodes
    target_drive_ma: f32,
    /// The actual drive current reported by the current source
    reported_drive_ma: f32,
    /// The actual measured drive currrent (may be same as reported_drive_ma above 20 mA) 
    measured_ma: f32,
    /// The actual measured potential across the electrodes 
    measured_volts: f32,

    /// Timestamp when each phase started
    phase_starts_utc_ms: [i64; DrivePhase::Max as usize],
    /// Whether a given anode is connected to the current supply 
    anode_connections: [bool; 4],
}

/// State the electrode controller is reset to when we're in "warmup" mode below the active melt temperature
const INITIAL_ELECTRODE_STATE: ElectrodeState = 
    ElectrodeState {
            drive_phase:DrivePhase::Fresh,
            last_update_ms:0,
            phase_start_ms:0,
            measured_ohms:INF_INTER_ELECTRODE_OHMS,
            ohms_ewma:0.,
            lowv_minr_ohms: INF_INTER_ELECTRODE_OHMS,
            lowv_minr_update_ms:0,
            highv_minr_ohms: INF_INTER_ELECTRODE_OHMS,
            highv_minr_update_ms:0,
            max_ohms_ewma: 0.,
            target_drive_ma: MIN_DRIVE_CURRENT_INCR_MA,
            reported_drive_ma:0.,
            measured_ma:0.,
            measured_volts:0., 
            anode_connections: [false; 4],
            phase_starts_utc_ms: [0; DrivePhase::Max as usize],
        }; 


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
    state.reported_drive_ma = read_electrode_current_drive(ctx).await?;

    // println!("drive_phase: {:?} r_ma: {:.3}", state.drive_phase, state.reported_drive_ma);

    // Measure the average resulting induced current and potential across the electrodes
    let mut total_volts = 0.;
    let mut total_milliamps = 0.;
    const NUM_IV_READ_STEPS: i32 = 3;
    const AVG_IV_FACTOR: f32 = NUM_IV_READ_STEPS as f32;

    for _i in 0..NUM_IV_READ_STEPS {
        let (step_volts, step_milliamps) = read_wdcu3003_iv_adc(ctx).await?;
        total_volts += step_volts;
        total_milliamps += step_milliamps;
        sleep(settling_time).await;
    }
    // average multiple potential samples
    let measured_volts = total_volts / AVG_IV_FACTOR;
    // average multiple current samples
    let mut measured_milliamps: f32 = 
        if state.target_drive_ma > 0.  && state.reported_drive_ma > REPORTED_CURRENT_THRESHOLD_MA  
        {  total_milliamps / AVG_IV_FACTOR }  
        else { 0. };

    // sanity check that measured current is close to (current source reported) drive current
    if state.reported_drive_ma > 2.*MIN_DRIVE_CURRENT_INCR_MA {
        let mr_current_gap_frac = (state.reported_drive_ma - measured_milliamps)/state.reported_drive_ma;
        if mr_current_gap_frac > 0.05 {
            println!("mr_current_gap : {:.3}", mr_current_gap_frac);
            measured_milliamps = state.reported_drive_ma;
        }
    }

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

/// Transition to Warmup drive phase
fn trans_warmup_phase(state: &mut ElectrodeState, trans_utc_ms: i64)
-> f32
{
    state.drive_phase = DrivePhase::Warmup;
    state.phase_start_ms = trans_utc_ms;
    state.phase_starts_utc_ms[DrivePhase::Warmup as usize] = trans_utc_ms;
    println!("{} start Warmup phase", 
        trans_utc_ms, 
    );
    WARMUP_CURRENT_MA
}

/// Transition to Nucleation drive phase
fn trans_nucleation_phase(state: &mut ElectrodeState, trans_utc_ms: i64)
-> f32
{
    state.drive_phase = DrivePhase::Nucleation;
    state.phase_start_ms = trans_utc_ms;
    state.phase_starts_utc_ms[DrivePhase::Nucleation as usize] = trans_utc_ms;
    println!("{} start Nucleation phase w/Rewma {:.2} min {:.2} max {:.2} Ohms", 
        trans_utc_ms, 
        state.ohms_ewma, state.lowv_minr_ohms, state.max_ohms_ewma, 
    );
    MAX_NUCLEATION_CURRENT_MA
}

/// Transition to Elongation drive phase
fn trans_elongation_phase(state: &mut ElectrodeState, trans_utc_ms: i64, prior_duration_ms: u64)
-> f32
{
    state.drive_phase = DrivePhase::Elongation;
    state.phase_start_ms = trans_utc_ms;
    state.phase_starts_utc_ms[DrivePhase::Elongation as usize] = trans_utc_ms;
    println!("{} start Elongation phase w/Rewma {:.2} min {:.2} max {:.2} Ohms ({} ms)", 
        trans_utc_ms, 
        state.ohms_ewma, state.lowv_minr_ohms, state.max_ohms_ewma, 
        prior_duration_ms
    );
    CYCLIC_PHASE_FALLBACK_MA
}

/// Switch to Holding drive phase
fn trans_holding_phase(state: &mut ElectrodeState, trans_utc_ms: i64, prior_duration_ms: u64)
-> f32
{
    state.drive_phase = DrivePhase::Holding;
    state.phase_start_ms = trans_utc_ms;
    state.phase_starts_utc_ms[DrivePhase::Holding as usize] = trans_utc_ms;

    println!("{} start Holding phase w/Rewma {:.2} min {:.2} max {:.2} Ohms ({} ms)", 
        trans_utc_ms, 
        state.ohms_ewma, state.lowv_minr_ohms, state.max_ohms_ewma, 
        prior_duration_ms
    );
    HOLDING_PROBE_CURRENT_MA
}

/// 
/// Adjust the electrode current based on melt condition and drive phase
/// 
async fn control_electrodes(ctx: &mut tokio_modbus::client::Context, 
    state: &mut ElectrodeState,
) 
-> Result<(), Box<dyn std::error::Error>> 
{    
    // Drive output current pulse based on prior settings, and measure result
    let (measured_volts, measured_milliamps, measured_ohms) = 
        drive_current_and_measure(ctx, state, CURRENT_SOURCE_WAIT_TIME).await?;

    let after_drive_utc_dt = chrono::Utc::now();
    let after_drive_utc_ms = after_drive_utc_dt.timestamp_millis();

    let phase_duration_ms = 
        if state.phase_start_ms <  after_drive_utc_ms {  (after_drive_utc_ms - state.phase_start_ms) as u64 } 
        else { 0 };

    // reuse old drive current until instructed otherwise
    let mut new_drive_ma: f32;

    // update the resistance moving average if possible
    let ohms_ewma_valid =
        if state.measured_ma != 0. && measured_ohms != INF_INTER_ELECTRODE_OHMS  {
            if measured_ohms == 0. {
                println!("{} warn: 0 ohms resistance!", after_drive_utc_ms);
            }
            update_ewma(&mut state.ohms_ewma, measured_ohms, RESISTANCE_EWMA_ALPHA);
            if state.max_ohms_ewma < state.ohms_ewma {
                state.max_ohms_ewma = state.ohms_ewma;
            }
            true
        }
        else { false };

    state.measured_ohms = measured_ohms;
    state.measured_volts = measured_volts;
    state.measured_ma = measured_milliamps;

    anode_connections_at_time_ms(phase_duration_ms, &mut state.anode_connections);
    
    match state.drive_phase {
        DrivePhase::Fresh => {
            // just transition to next phase
            new_drive_ma = trans_warmup_phase(state, after_drive_utc_ms);
        }
        DrivePhase::Warmup => {
            // while the melt is warming up, monitor the current throughput 
            new_drive_ma = WARMUP_CURRENT_MA;
            if phase_duration_ms > WARMUP_PHASE_DUR_MS 
                && state.measured_ma > (new_drive_ma / 2.)  
                && ohms_ewma_valid 
            {
                // capture the resistance EWMA at this moment as both min and max
                state.lowv_minr_ohms = state.ohms_ewma;
                state.highv_minr_ohms = state.ohms_ewma;

                // TODO for now we require manually transition to next state
                // new_drive_ma = 
                //     trans_nucleation_phase(state, after_drive_utc_ms);
            }
            println!("Warmup: {} sec {:.1} Ω", phase_duration_ms/1000, state.measured_ohms);
        }
        DrivePhase::Nucleation => {
            new_drive_ma = MAX_NUCLEATION_CURRENT_MA;
        }
        DrivePhase::Elongation => {
            let goal_drive_volts = cyclic_voltage_at_time_ms(phase_duration_ms);
            // calculate current value for (nearly) constant voltage
            if ohms_ewma_valid {
                new_drive_ma = (goal_drive_volts * 1000.) / state.measured_ohms;

                // cap at some reasonable limit
                if new_drive_ma > MAX_ELONGATION_CURRENT_MA {
                    println!("Maxed {:.2} --> {:.2} mA", new_drive_ma, MAX_ELONGATION_CURRENT_MA);
                    new_drive_ma = MAX_ELONGATION_CURRENT_MA;
                }

                // check for cyclic growth termination condition
                if state.measured_volts <= CYCLIC_LOWV_MINR_MEASURE_V {
                    if state.ohms_ewma < state.lowv_minr_ohms {
                        println!("{} lowv {:.2} V,  LV_MinR -> {:.3} Ω", 
                            after_drive_utc_ms, state.measured_volts, state.ohms_ewma);
                        state.lowv_minr_ohms = state.ohms_ewma;
                        state.lowv_minr_update_ms = after_drive_utc_ms;
                    }

                    if state.ohms_ewma < CYCLIC_LOWV_TERMINATION_OHMS {
                        new_drive_ma =
                            trans_holding_phase(state, 
                                after_drive_utc_ms,
                                phase_duration_ms);
                    }
                }
                else {
                    if state.ohms_ewma < state.highv_minr_ohms {
                        println!("{} highv {:.2} V,  HV_MinR -> {:.3} Ω", 
                            after_drive_utc_ms, state.measured_volts, state.ohms_ewma);
                        state.highv_minr_ohms = state.ohms_ewma;
                        state.highv_minr_update_ms = after_drive_utc_ms;
                    }
                }
                
            }
            else {
                if goal_drive_volts == CYCLIC_GROWTH_PEAK_V {
                    new_drive_ma = CYCLIC_PHASE_FALLBACK_MA;
                }
                else {
                    new_drive_ma = 25.;
                }
                println!("{} cyclic fallback at {:.2} Ω : {:.1}", after_drive_utc_ms, state.ohms_ewma,new_drive_ma);
            }
        }
        DrivePhase::Holding => {
            new_drive_ma = HOLDING_PROBE_CURRENT_MA;
        }
        DrivePhase::Max => {
            unreachable!("DrivePhase::Max s/b unused");
        }
    };

    // Now, update the drive current for the next main loop iteration
    // state.reported_drive_ma = set_electrode_current_drive(ctx, new_drive_ma).await?;
    state.target_drive_ma = new_drive_ma;
    state.last_update_ms = chrono::Utc::now().timestamp_millis();

    Ok(())
}


///
/// Calculate the LowV/HighV voltage at a given time 
fn cyclic_voltage_at_time_ms(phase_duration_ms: u64) -> f32 
{
    let remainder: u64 = phase_duration_ms % CYCLIC_PERIOD_MS;

    if remainder < CYCLIC_LOWV_DURATION_MS {
        // begin with LOWV drive on each cycle
        CYCLIC_GROWTH_FLOOR_V
    }
    else {
        // end with HIGHV drive on each cycle
        CYCLIC_GROWTH_PEAK_V
    }
}

///
/// Calculate which anodes are connected (via relay switch) to the current supply at the given time
fn anode_connections_at_time_ms(phase_duration_ms: u64, connections: &mut [bool]) 
{
    const CONNECTION_PERIOD_MS:usize = 1000;
    let full_cycle_duration_ms  = connections.len() * CONNECTION_PERIOD_MS;
    // let cycle_count = phase_duration_ms / full_cycle_duration_ms;
    let cycle_modulo_ms = (phase_duration_ms as usize) % full_cycle_duration_ms;
    let active_idx = cycle_modulo_ms / CONNECTION_PERIOD_MS;
    connections.fill(false);
    connections[active_idx] = true;
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

    let start_time_secs = chrono::Utc::now().timestamp();
    let log_out_filename = format!("{}_log.csv",start_time_secs);
    println!("Recording data to {log_out_filename:?} ...");

    println!("Nucleate: Imax {:.2} mA", MAX_NUCLEATION_CURRENT_MA);
    println!("Elongate: Imax {:.2} mA Term {:.1} Ω ", MAX_ELONGATION_CURRENT_MA, CYCLIC_LOWV_TERMINATION_OHMS);


    let logfile = File::create(format!("./data/{}",log_out_filename))?;
    let mut csv_writer = BufWriter::new(logfile);

    const CSV_HEADER: &str =  "epoch_ms,heat,avg_C,eleco_mA,elecm_mA,elecm_V,elec_R,Rew,hvMinR,lvMinR";
    macro_rules! CSV_LINE_FORMAT { () => { "{},{},{:.2},{:.2},{:.2},{:.3},{:.3},{:.3},{:.3},{:.3}" } }
    
    println!("{}",CSV_HEADER);
    writeln!(csv_writer, "{}", CSV_HEADER)?;

    // setup command handling
    let mut lines = BufReader::new(tokio::io::stdin()).lines();

    let mut furnace_state = INITIAL_FURNACE_STATE;
    let mut electrode_state =  INITIAL_ELECTRODE_STATE;

    electrode_state.phase_start_ms =  chrono::Utc::now().timestamp_millis();

    let mut loop_count = 0;

    // First run main loop at next second
    let now_instant = tokio::time::Instant::now();
    let first_loop_instant = now_instant + INTER_LOOP_DELAY - Duration::from_millis(now_instant.elapsed().subsec_millis() as u64);
    sleep_until(first_loop_instant).await;

    let ctrl_c_fut = tokio::signal::ctrl_c();
    tokio::pin!(ctrl_c_fut);

    // main control loop
    loop { 
        let current_utc_dt = chrono::Utc::now();
        let current_utc_ms = current_utc_dt.timestamp_millis();
        let sleep_timer = sleep(Duration::from_millis(50));
        tokio::pin!(sleep_timer);

        tokio::select! {
            _ = &mut ctrl_c_fut => {
                eprintln!("\nCtrl-C: Shutting down...");
                break;
            }
            cmd_line =  lines.next_line() => {
                match cmd_line {
                    Ok(Some(cmd_line)) => {
                        match cmd_line.trim() {
                            "hello" => println!("Hello!"),
                            "q" | "quit" => {
                                println!("Quitting...");
                                break;
                            }
                            "w" | "warmup" => {
                                trans_warmup_phase(&mut electrode_state, current_utc_ms);
                            },
                            "n" | "nucleate" => {
                                trans_nucleation_phase(&mut electrode_state, current_utc_ms);
                            }
                            "e" | "elongate" => {
                                trans_elongation_phase(&mut electrode_state, current_utc_ms, 0);
                            },
                            "h" | "holding" => {
                                trans_holding_phase(&mut electrode_state,  current_utc_ms, 0);
                            }
                            other => println!("Unknown commannd: {other:?}"),
                        }
                    }
                    Ok(None) => {
                        eprintln!("EOF (stdin closed)");
                        break;
                    }
                    Err(e) if e.kind() == std::io::ErrorKind::Interrupted => {
                        eprintln!("Interrupted!");
                        break;
                    }
                    Err(e) => {
                        eprintln!("Error: {:?}",e);
                        return Err(e.into())
                    }
                }
            }
            _ = &mut sleep_timer  => {
                // no commands received: continue the main loop
            }
        }


        let furnace_res = 
            tokio::time::timeout(MODBUS_TRANSACTION_TIMEOUT,control_furnace(&mut ctx, &mut furnace_state)).await;
        if !furnace_res.is_ok() { 
            eprintln!("control_furnace timeout: {:?}",furnace_res);
            break;
        }

        if (furnace_state.measured_temp_c > MIN_ELECTRODE_CHECK_TEMP_C &&  furnace_state.measured_temp_c < EXCESSIVE_HEAT_TEMP_C) ||
            electrode_state.drive_phase != DrivePhase::Fresh 
        {
            let elec_res = 
                tokio::time::timeout(MODBUS_TRANSACTION_TIMEOUT, control_electrodes(&mut ctx, &mut electrode_state)).await;
            if !elec_res.is_ok() { 
                eprintln!("control_electrodes timeout: {:?}",elec_res);
                break;
            }
        }
        else {
            // println!("drive_phase: {:?} temp: {:.2}", electrode_state.drive_phase, furnace_state.measured_temp_c);
            electrode_state = INITIAL_ELECTRODE_STATE;
            electrode_state.phase_start_ms = current_utc_dt.timestamp_millis();
        }

        let log_line = format!( CSV_LINE_FORMAT!(),
            current_utc_dt.timestamp_millis(),
            furnace_state.heater_on as u8,
            furnace_state.measured_temp_c,
            electrode_state.target_drive_ma, electrode_state.measured_ma,
            electrode_state.measured_volts, 
            electrode_state.measured_ohms, electrode_state.ohms_ewma, 
            electrode_state.highv_minr_ohms, electrode_state.lowv_minr_ohms,
        );
        println!("{}",log_line);
        writeln!(  csv_writer,"{}",  log_line)?;
        loop_count = (loop_count + 1) % 5;
        if loop_count == 0 { let _ = csv_writer.flush(); }
        else { sleep(MAINLOOP_DELAY).await; }

    }

    println!("Flushing log file...");
    csv_writer.flush()?;

    // Attempt to shut off all outputs before exiting.
    // We reconnect to modbus to flush any cruft buffered at the WiFi bridge.
    println!("Disconnecting...");
    ctx.disconnect().await?;
    sleep(Duration::from_secs(2)).await;
    println!("Reconnecting to: '{socket_addr:?}' ...");
    ctx = tcp::connect(socket_addr).await?;
    zero_control_outputs(&mut ctx).await?;
    println!("Disconnecting again...");
    let foomp = ctx.disconnect().await;
    if foomp.is_err() {
        eprintln!("disconnect failed: {:?}", foomp);
    }

    // dump logged timeline
    println!("phase_starts_ms: {:?}", electrode_state.phase_starts_utc_ms);

    println!("finished...");
    std::process::exit(0); 

}



