#![allow(unused)]

use std::os::macos::raw::stat;
use std::time::Duration;
use tokio::time::sleep;

use tokio_modbus::prelude::*;

use tokio_modbus::client::{Client, Reader, Writer};
use tokio_serial::SerialStream;
use craven_control::*;

/// Tells the SMC05 to start (or stop) the preprogrammed loop
const START_STOP_OP_COMMAND: u16 = 3;


#[derive(Debug, Clone)]
pub struct StepperDriverState {
    /// Whether or not the SMC05 dipper is enabled
    dipper_enabled: bool, 
    /// Last time the driver status was checked
    dipper_last_status_check_ms: i64,
    /// The prior direction of the SMC05 dipper motion
    dipper_prior_motion_direction: u16,
    /// The prior SMC05 pulse count (which indicates distance traveled)
    dipper_prior_pulse_count: u16,
    /// The prior SMC05 action count (which indicates how many actions have run)
    dipper_prior_action_count: u16,
}



async fn start_motion(ctx: &mut tokio_modbus::client::Context, motion: u16) 
-> Result<(), Box<dyn std::error::Error>> 
{
    println!("start motion: {}", motion);
    ctx.write_single_register(REG_SMC05_OPERATION_MODE, motion).await?;
    sleep(Duration::from_millis(25)).await;
    let status_resp: Vec<u16> = ctx.read_holding_registers(REG_SMC05_CUR_MOTOR_STATUS, 5).await??;
    println!("> start status 0x1A: {:?}", status_resp);
    let opstatus = status_resp[0];
    let direction = status_resp[1];
    if opstatus == 0  {
        println!("0x0030 -> {} start ", motion);
        ctx.write_single_register(REG_SMC05_OPERATION_MODE, START_STOP_OP_COMMAND).await?;
    }

    Ok(())
}

async fn start_forward(ctx: &mut tokio_modbus::client::Context) 
-> Result<(), Box<dyn std::error::Error>> 

{
    stop_motion(ctx).await;
    start_motion(ctx,0).await
}

async fn start_reverse(ctx: &mut tokio_modbus::client::Context) 
-> Result<(), Box<dyn std::error::Error>> 
{
    stop_motion(ctx).await;
    start_motion(ctx,1).await
}

async fn stop_motion(ctx: &mut tokio_modbus::client::Context) 
-> Result<(), Box<dyn std::error::Error>> 

{
    let status_resp: Vec<u16> = ctx.read_holding_registers(REG_SMC05_CUR_MOTOR_STATUS, 5).await??;
    println!("> stop status 0x1A: {:?}", status_resp);
    let opstatus = status_resp[0];
    let direction = status_resp[1];
    if opstatus != 0 {
        println!("0x0030 -> 3 stop_motion ");
        ctx.write_single_register(REG_SMC05_OPERATION_MODE, START_STOP_OP_COMMAND).await?;
    }
    Ok(())
}

/// 
/// Verify that all the modules we expect to be connected to the RS-485 Modbus are,
/// in fact, connected.
/// 
async fn enumerate_required_modules(ctx: &mut tokio_modbus::client::Context) -> Result<(), Box<dyn std::error::Error>> 
{
    // measures voltage and current across the electrodes
    // TODO we can't ping WDCU3003M with a node ID read, because it doesn't expose node ID to Modbus
    ctx.set_slave(Slave(NODEID_WDCU3003_IV_ADC));
    let wdc3003_vals: Vec<u16> = ctx.read_holding_registers(0, 10).await??;
    println!("wdc3003_vals: {:?}", wdc3003_vals);

    // supplies current to the cathode and anodes
    ping_one_modbus_node_id(ctx,NODEID_YKPVCCS010_CURR_SRC, REG_NODEID_YKPVCCS010_CURR_SRC).await?;

    // controls dipping motion of cathode
    ping_one_modbus_node_id(ctx, NODEID_SMC05_STEP_DRIVER, REG_NODEID_SMC05).await?;

    Ok(())
}



fn disable_dipper_monitor(state: &mut StepperDriverState) {
    state.dipper_enabled = false;
    state.dipper_last_status_check_ms = 0;
    println!("Dipper monitor canceling...");
}

fn toggle_dipper_enabled(state: &mut StepperDriverState) {
    let old_enabled = state.dipper_enabled ;
    state.dipper_enabled = !old_enabled;
    state.dipper_last_status_check_ms = 0;
    println!("Toggled dipper_enabled {} -> {}", old_enabled, state.dipper_enabled);
}

/// How often should we check whether the dipper has finished its preprogrammed loop?
const DIPPER_PROGRESS_PERIOD_MS: i64 = 5000;

/// Monitor the dip cycle preprogrammed into the stepper controller.
/// If the program has finished, restart it.
async fn dipper_cycle_check(ctx: &mut tokio_modbus::client::Context, 
    state: &mut StepperDriverState, current_utc_ms: i64)
    -> Result<(), Box<dyn std::error::Error>> 
{
    /// Action Mode that runs a distance cycle (by pulses) and then stops
    const ACTION_PROCESS_MODE_DISTANCE_LOOP: u16 = 6;

    let (op_status, motion_direction, pulse_count, action_count) = read_smc05_motor_status(ctx).await?;
    // println!("{} > op {} dir {} pulse {} action {}", current_utc_ms, op_status, motion_direction, pulse_count, action_count);

    if state.dipper_last_status_check_ms == 0 {
        println!("{} Fresh Dipper",current_utc_ms);
        // "Action process mode" -- running preprogrammed loop
        ctx.set_slave(Slave(NODEID_SMC05_STEP_DRIVER));
        ctx.write_single_register(REG_SMC05_ACTION_PROCESS_MODE, ACTION_PROCESS_MODE_DISTANCE_LOOP).await??;
        state.dipper_prior_motion_direction = motion_direction;
        state.dipper_prior_action_count = action_count;
        state.dipper_prior_pulse_count = pulse_count;
    }

    // only check the status periodically, because there can be some pauses and delays between reversals and loops
    if (current_utc_ms - state.dipper_last_status_check_ms) > DIPPER_PROGRESS_PERIOD_MS {
        println!("{} > op {} dir {} pulse {} action {}", current_utc_ms, op_status, motion_direction, pulse_count, action_count);

        // if action_count != state.dipper_prior_action_count {
        //     println!("{} New Action started!",current_utc_ms);
        // }
        // If preprogrammed motion loop has finished, start it again:
        if op_status == 0 { // "Stopped"
            if motion_direction == state.dipper_prior_motion_direction  {
                if pulse_count == state.dipper_prior_pulse_count {
                    // if action_count doesn't equal prior, that would indicate we're starting a new cycle of the loop
                    if action_count == state.dipper_prior_action_count {
                        println!("{} Next dip cycle", current_utc_ms);
                        start_smc05_action_loop(ctx).await?;
                    }
                }
            }
        }

        state.dipper_prior_motion_direction = motion_direction;
        state.dipper_prior_action_count = action_count;
        state.dipper_prior_pulse_count = pulse_count;
        state.dipper_last_status_check_ms = current_utc_ms;
    }

    Ok(())
}



#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<(), Box<dyn std::error::Error>> {

    // actual usb device ID of one USB-RS485 adapter
    // let tty_path = "/dev/cu.usbserial-BG02SI88";
    // let baud_rate = 115200;
    // let builder = tokio_serial::new(tty_path, baud_rate);
    // let mut ctx = rtu::attach_slave(SerialStream::open(&builder).unwrap(), Slave(NODEID_DEFAULT));
    
    let socket_addr: std::net::SocketAddr = "10.0.1.151:502".parse()?;

    println!("Connecting to: '{socket_addr:?}'");
    let mut ctx: client::Context = tcp::connect(socket_addr).await?;
    enumerate_required_modules(&mut ctx).await?;

    ctx.set_slave(Slave(NODEID_SMC05_STEP_DRIVER));
    sleep(Duration::from_millis(50)).await;

    let mut driver_state: StepperDriverState = StepperDriverState { 
        dipper_enabled: false,
        dipper_last_status_check_ms: 0, 
        dipper_prior_motion_direction: 0, 
        dipper_prior_pulse_count: 0, 
        dipper_prior_action_count: 0, 
    };

    let start_time_ms = chrono::Utc::now().timestamp_millis();

    // "start" the preprogrammed motion loop (which is like 25 mm forward and back, multiple cycles)
    let mut continue_running = true;
    while continue_running {
        let current_utc_dt = chrono::Utc::now();
        let current_utc_ms = current_utc_dt.timestamp_millis();

        if driver_state.dipper_enabled {
            dipper_cycle_check(&mut ctx, &mut driver_state, current_utc_ms).await?;
        }

        if (current_utc_ms - start_time_ms) > 60000 {
            continue_running = false;
        }
        else {
            // TODO this needs to align with the "reversal" pauses , and any pauses between cycles
            sleep(Duration::from_millis(2000)).await;
        }
    }


    ctx.disconnect().await?;

    Ok(())
}




