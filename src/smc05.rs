use tokio_modbus::{Slave, client::{Reader, Writer}, slave::SlaveContext};

use crate::*;

/// Register holding node ID (address) for SMC05
pub const REG_NODEID_SMC05: u16 = 0x0018; 
/// SMC05 action mode, such as stepping forward and back or running a preprogrammed action loop
pub const REG_SMC05_SPORT_MODE: u16 = 0x0000; 

/// SMC05 forward rotation speed (Rotations Per Minute)
pub const REG_SMC05_FWD_RPM: u16 = 0x0003;

/// SMC05 reverse rotation speed (Rotations Per Minute)
pub const REG_SMC05_REV_RPM: u16 = 0x0006;

/// SMC05 Current motor operating status: 0 stop, 1 acceleration, 2 deceleration , 3 uniform speed 
pub const REG_SMC05_CUR_MOTOR_STATUS:u16  = 0x001A;
/// SMC05 fwd/rev/start/stop operations
pub const REG_SMC05_OPERATION_MODE: u16 = 0x0030; 

/// In this sport mode , run either fwd or rev on command: stop on same command
const SMC05_SPORT_MODE_03_FWD_REV_RUNTIL: u16 = 3;

/// In this "sport mode" run fwd,rev, with delays at direction changes
const SMC05_SPORT_MODE_06_FWD_REV_LOOP: u16 = 6;

/// Tells the SMC05 to start (or stop) the preprogrammed loop
const START_STOP_OP_COMMAND: u16 = 3;

const ROTATION_DIR_FWD_CMD: u16 = 1;
const ROTATION_DIR_REV_CMD: u16 = 2;

// TODO this documentation seems to be incorrect:
// Controlling sport mode action with RS485 serial / Modbus:
// 00 - forward rotation 
// 01 - reverse rotation 
// 03 - stop / start


/// How often should we check whether the dipper has finished its preprogrammed loop?
const SPORT_MODE06_PROGRESS_PERIOD_MS: i64 = 5000;

#[derive(Debug, Clone)]
pub struct StepperDriverState {
    /// Whether or not the SMC05 dipper is enabled
    pub dipper_enabled: bool, 
    /// Last time the driver status was checked
    pub dipper_last_status_check_ms: i64,
    /// The prior direction of the SMC05 dipper motion
    pub dipper_prior_motion_direction: u16,
    /// The prior SMC05 pulse count (which indicates distance traveled)
    pub dipper_prior_pulse_count: u16,
    /// The prior SMC05 action count (which indicates how many actions have run)
    pub dipper_prior_action_count: u16,
}

impl Default for StepperDriverState {
    fn default() -> Self { Self { 
            dipper_enabled: false,
            dipper_last_status_check_ms: 0, 
            dipper_prior_motion_direction: 0, 
            dipper_prior_pulse_count: 0, 
            dipper_prior_action_count: 0, 
        }
    }
}



/// # Returns 
/// (motion_direction, pulse_count, action_count) 
pub async fn read_stepper_driver_status(ctx: &mut tokio_modbus::client::Context)
-> Result<(u16, u16, u16, u16), Box<dyn std::error::Error>> 
{
    ctx.set_slave(Slave(NODEID_SMC05_STEP_DRIVER));
    let status_rsp: Vec<u16> = ctx.read_holding_registers(REG_SMC05_CUR_MOTOR_STATUS, 9).await??;
    // println!("> SMC05 status: {:?}", status_rsp);
    let op_status = status_rsp[0];
    let motion_direction = status_rsp[1];
    let pulse_count = status_rsp[4];
    let action_count = status_rsp[8];
    Ok((op_status, motion_direction, pulse_count, action_count))
}

pub async fn start_sport_mode06_sequence(ctx: &mut tokio_modbus::client::Context) 
-> Result<(), Box<dyn std::error::Error>> 
{
    send_smc05_start_stop_cmd(ctx).await
}

pub async fn report_smc05_motor_status(ctx: &mut tokio_modbus::client::Context) 
-> Result<(u16, u16), Box<dyn std::error::Error>>
{
    let (op_status, motor_direction, pulse_count, action_count) = read_stepper_driver_status(ctx).await?;
    println!("{} > op {} dir {} pulse {} action {}", 
        chrono::Utc::now().timestamp_millis(), op_status, motor_direction, pulse_count, action_count);
    Ok((op_status, motor_direction))
}

const SMC05_ACCEL_TIME_MS: u64 = 1000;

pub const SMC05_ROTATION_DIR_FWD: u16 = 0;
pub const SMC05_ROTATION_DIR_REV: u16 = 1;

pub async fn start_smc05_fwd_rotation(ctx: &mut tokio_modbus::client::Context) 
-> Result<(), Box<dyn std::error::Error>>
{
    let (_op_status, motor_direction) = report_smc05_motor_status(ctx).await?;
    if motor_direction != SMC05_ROTATION_DIR_FWD {
        println!("FLIP -> Fwd");
        send_smc05_fwd_rotation_cmd(ctx).await?;
        sleep(Duration::from_millis(SMC05_ACCEL_TIME_MS)).await;
    }
    
    let (op_status, motor_direction) = report_smc05_motor_status(ctx).await?;
    if op_status == 0 { //still stopped?
        println!("START Fwd {} ", motor_direction);
        send_smc05_start_stop_cmd(ctx).await?;
    }

    report_smc05_motor_status(ctx).await?;
    Ok(())
}

pub async fn start_smc05_rev_rotation(ctx: &mut tokio_modbus::client::Context) 
-> Result<(), Box<dyn std::error::Error>>
{
    let (_op_status, motor_direction) = report_smc05_motor_status(ctx).await?;
    if motor_direction != SMC05_ROTATION_DIR_REV {
        println!("FLIP -> Rev");
        send_smc05_rev_rotation_cmd(ctx).await?;
        sleep(Duration::from_millis(SMC05_ACCEL_TIME_MS)).await;
    }
    
    let (op_status, motor_direction) = report_smc05_motor_status(ctx).await?;
    if op_status == 0 { //still stopped?
        println!("START Rev {}", motor_direction);
        send_smc05_start_stop_cmd(ctx).await?;
    }

    report_smc05_motor_status(ctx).await?;
    Ok(())
}

pub async fn stop_smc05_rotation(ctx: &mut tokio_modbus::client::Context) 
-> Result<(), Box<dyn std::error::Error>>
{
    loop {
        let (op_status, motion_direction) = report_smc05_motor_status(ctx).await?; 
        if 0 != op_status {
            println!("STOP dir {}", motion_direction);
            send_smc05_start_stop_cmd(ctx).await?;
            sleep(Duration::from_millis(500)).await;
        }
        else { break };
    }
    Ok(())
}

pub async fn send_smc05_serial_op_cmd(ctx: &mut tokio_modbus::client::Context, op_cmd: u16) 
-> Result<(), Box<dyn std::error::Error>> 
{
    ctx.set_slave(Slave(NODEID_SMC05_STEP_DRIVER));
    // println!("0x0030 -> opcmd: {}", op_cmd);
    ctx.write_single_register(REG_SMC05_OPERATION_MODE, op_cmd).await??;
    // sleep(Duration::from_millis(100)).await;
    Ok(())
}

pub async fn send_smc05_fwd_rotation_cmd(ctx: &mut tokio_modbus::client::Context) 
-> Result<(), Box<dyn std::error::Error>> 
{
    send_smc05_serial_op_cmd(ctx, ROTATION_DIR_FWD_CMD).await
}

pub async fn send_smc05_rev_rotation_cmd(ctx: &mut tokio_modbus::client::Context) 
-> Result<(), Box<dyn std::error::Error>> 
{
    send_smc05_serial_op_cmd(ctx, ROTATION_DIR_REV_CMD).await
}

pub async fn send_smc05_start_stop_cmd(ctx: &mut tokio_modbus::client::Context) 
-> Result<(), Box<dyn std::error::Error>> 
{
    send_smc05_serial_op_cmd(ctx, START_STOP_OP_COMMAND).await
}

/// Disable the dipper monitor
pub fn disable_dipper_monitor(state: &mut StepperDriverState) {
    state.dipper_enabled = false;
    state.dipper_last_status_check_ms = 0;
    println!("Dipper monitor canceling...");
}

/// If the dipper monitor is enabled, disable it (or vice-versa)
pub fn toggle_dipper_monitor(state: &mut StepperDriverState) {
    let old_enabled = state.dipper_enabled ;
    state.dipper_enabled = !old_enabled;
    state.dipper_last_status_check_ms = 0;
    println!("Toggled dipper_enabled {} -> {}", old_enabled, state.dipper_enabled);
}


/// 
pub async fn report_system_config(ctx: &mut tokio_modbus::client::Context)
    -> Result<(), Box<dyn std::error::Error>> 
{
    ctx.set_slave(Slave(NODEID_SMC05_STEP_DRIVER));
    let status_resp: Vec<u16> = ctx.read_holding_registers(REG_SMC05_SPORT_MODE, 12).await??;
    println!("sysconfig: {:?}", status_resp);
    Ok(())
}

pub async fn set_fwd_speed(ctx: &mut tokio_modbus::client::Context, rpm: f32)
    -> Result<(), Box<dyn std::error::Error>> 
{
    let fxp_rpm: u16 = (10. * rpm).round() as u16;
    ctx.set_slave(Slave(NODEID_SMC05_STEP_DRIVER));
    ctx.write_single_register(REG_SMC05_FWD_RPM, fxp_rpm).await??;
    Ok(())
}

pub async fn set_rev_speed(ctx: &mut tokio_modbus::client::Context, rpm: f32)
    -> Result<(), Box<dyn std::error::Error>> 
{
    let fxp_rpm = (10. * rpm).round() as u16;
    ctx.set_slave(Slave(NODEID_SMC05_STEP_DRIVER));
    ctx.write_single_register(REG_SMC05_REV_RPM, fxp_rpm).await??;
    Ok(())
}

///
/// Set the sport mode of the SMC05 stepper driver
/// 
pub async fn set_smc05_sport_mode(ctx: &mut tokio_modbus::client::Context,  mode: u16)
    -> Result<(), Box<dyn std::error::Error>> 
{
    println!("Set SMC05 Sport Mode {}...", mode);
    ctx.set_slave(Slave(NODEID_SMC05_STEP_DRIVER));
    ctx.write_single_register(REG_SMC05_SPORT_MODE, mode).await??;
    Ok(())
}


pub async fn enable_sport_mode03(ctx: &mut tokio_modbus::client::Context)
    -> Result<(), Box<dyn std::error::Error>> 
{
    set_smc05_sport_mode(ctx, SMC05_SPORT_MODE_03_FWD_REV_RUNTIL).await
}

/// Setup stepper driver to run repeated dip cycle
pub async fn enable_sport_mode06(ctx: &mut tokio_modbus::client::Context, 
    state: &mut StepperDriverState)
    -> Result<(), Box<dyn std::error::Error>> 
{
    ctx.set_slave(Slave(NODEID_SMC05_STEP_DRIVER));
    let (_op_status, motor_direction, pulse_count, action_count) = read_stepper_driver_status(ctx).await?;            
    set_smc05_sport_mode(ctx, SMC05_SPORT_MODE_06_FWD_REV_LOOP).await?;
    state.dipper_prior_motion_direction = motor_direction;
    state.dipper_prior_action_count = action_count;
    state.dipper_prior_pulse_count = pulse_count;

    Ok(())
}


/// Monitor the dip cycle preprogrammed into the stepper controller.
/// If the program has finished, restart it.
/// Note that state.dipper_enabled must be enabled, else this fn does nothing.
pub async fn dipper_cycle_check(ctx: &mut tokio_modbus::client::Context, 
    state: &mut StepperDriverState, current_utc_ms: i64)
    -> Result<(), Box<dyn std::error::Error>> 
{
    if !state.dipper_enabled {return Ok(()) };

    if state.dipper_last_status_check_ms == 0 {
        println!("{} Fresh Dipper",current_utc_ms);
        enable_sport_mode06(ctx,state).await?;
    }

    // only check the status periodically, because there can be some pauses and delays between reversals and loops
    if (current_utc_ms - state.dipper_last_status_check_ms) > SPORT_MODE06_PROGRESS_PERIOD_MS {
        let (op_status, motor_direction, pulse_count, action_count) = read_stepper_driver_status(ctx).await?;
        println!("{} > op {} dir {} pulse {} action {}", current_utc_ms, op_status, motor_direction, pulse_count, action_count);

        // If preprogrammed motion loop has finished, start it again:
        if op_status == 0 { // "Stopped"
            if motor_direction == state.dipper_prior_motion_direction  {
                if pulse_count == state.dipper_prior_pulse_count {
                    // if action_count doesn't equal prior, that would indicate we're starting a new cycle of the loop
                    if action_count == state.dipper_prior_action_count {
                        println!("{} Next dip cycle", current_utc_ms);
                        start_sport_mode06_sequence(ctx).await?;
                    }
                }
            }
        }

        state.dipper_prior_motion_direction = motor_direction;
        state.dipper_prior_action_count = action_count;
        state.dipper_prior_pulse_count = pulse_count;
        state.dipper_last_status_check_ms = current_utc_ms;
    }

    Ok(())
}


