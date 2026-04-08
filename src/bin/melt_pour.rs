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
const MODBUS_RW_DELAY: Duration = Duration::from_millis(125);

/// Rated maximum temperature of thermocouples (in this case, Type K)
const MAX_PROBE_TEMP_C:CelsiusF32 = 1000.;
/// Temp at which we attempt to submerge thermo probes in electrolyte melt
const PROBE_CHECK_TEMP_C:CelsiusF32 = 550.; 
/// Temp we expect to see when probe is succesfully inserted into melt
const PROBE_INSERTED_TEMP_C:CelsiusF32 = 600.;
/// The center temperature we are trying to achieve for the electrolyte melt
const ELECTROLYTE_TARGET_TEMP_C:CelsiusF32 = 780.;






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
    toggle_furnace(&mut ctx, heat_on).await?;

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
        }

        toggle_furnace(&mut ctx, heat_on).await?;

        println!("{} {} {:.1} furnace heating: {}",current_utc_dt.timestamp(), base_delta_dt.num_seconds(), cycle_delta_dt.as_seconds_f32(), heat_on );

        // sync to about 1 Hz loops
        sleep_until(next_run_instant).await;
    }

    // Attempt to shut off all outputs before exiting
    zero_control_outputs(&mut ctx).await?;
    ctx.disconnect().await?;

    Ok(())
}



