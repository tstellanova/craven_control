use std::time::Duration;
use tokio::time::sleep;

use tokio_modbus::prelude::*;

use tokio_modbus::client::{Client};
use craven_control::*;
use craven_control::smc05::*;


/// 
/// Verify that all the modules we expect to be connected to the RS-485 Modbus are,
/// in fact, connected.
/// 
async fn enumerate_required_modules(ctx: &mut tokio_modbus::client::Context) -> Result<(), Box<dyn std::error::Error>> 
{
    // measures voltage and current across the electrodes
    ping_one_modbus_node_register(ctx, NODEID_WDCU3003_IV_ADC, 0, 1).await?;

    // supplies current to the cathode and anodes
    ping_one_modbus_node_id(ctx,NODEID_YKPVCCS010_CURR_SRC, REG_NODEID_YKPVCCS010_CURR_SRC).await?;

    // controls dipping motion of cathode
    ping_one_modbus_node_id(ctx, NODEID_SMC05_STEP_DRIVER, REG_NODEID_SMC05).await?;

    Ok(())
}


/// This function steps the cathode down until it makes solid contact with electrolyte
///
async fn step_down_to_contact_surface(ctx: &mut tokio_modbus::client::Context) -> Result<(), Box<dyn std::error::Error>> 
{
    const DRIVE_CURRENT_MA: f32 = 4.;
    const CHECK_CURRENT_MA: f32 = DRIVE_CURRENT_MA - 0.25;

    // Enable all of the anode connections
    let  anode_channels= [true; 4];
    write_wav_octo_relays(ctx, &anode_channels).await?;

    set_smc05_sport_mode(ctx, 06);
    loop {
        let (measured_volts, measured_ma, measured_ohms) = 
            drive_current_and_measure(ctx, DRIVE_CURRENT_MA).await?;
        let cur_time_ms = chrono::Utc::now().timestamp_millis();
        println!("{} {:.2} V {:.2} mA {:.2} Ohms", measured_volts, measured_ma, measured_ohms);
        if measured_ma > CHECK_CURRENT_MA {
            break;
        }
        else {
            let (op_status, motion_direction, pulse_count, action_count) = read_stepper_driver_status(ctx).await?;
            println!("{} > op {} dir {} pulse {} action {}", current_utc_ms, op_status, motion_direction, pulse_count, action_count);

            // Move some increment FWD / down into the crucible
            send_smc05_fwd_rotation_cmd(ctx).await?;
            sleep(Duration::from_millis(1000)).await?;
        }
    }
    Ok(())
}


#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<(), Box<dyn std::error::Error>> {

    // // actual USB device ID of one USB-RS485 adapter
    // let tty_path = "/dev/cu.usbserial-BG02SI88";
    // let baud_rate = 115200;
    // let builder = tokio_serial::new(tty_path, baud_rate);
    // let mut ctx = rtu::attach_slave(tokio_serial::SerialStream::open(&builder).unwrap(), Slave(NODEID_DEFAULT));
    
    // In this mode we're using a WiFi bridge to a Modbus RTU (RS485) bus
    let socket_addr: std::net::SocketAddr = "10.0.1.151:502".parse()?;
    println!("Connecting to: '{socket_addr:?}'");
    let mut ctx: client::Context = tcp::connect(socket_addr).await?;
    enumerate_required_modules(&mut ctx).await?;

    ctx.set_slave(Slave(NODEID_SMC05_STEP_DRIVER));

    let mut driver_state = StepperDriverState::default();

    let start_time_ms = chrono::Utc::now().timestamp_millis();

    let mut continue_running = true;
    while continue_running {
        let current_utc_dt = chrono::Utc::now();
        let current_utc_ms = current_utc_dt.timestamp_millis();

        dipper_cycle_check(&mut ctx, &mut driver_state, current_utc_ms).await?;

        if (current_utc_ms - start_time_ms) > 60000 {
            continue_running = false;
        }
        else {
            // Arbitrary
            sleep(Duration::from_millis(4000)).await;
        }
    }


    ctx.disconnect().await?;

    Ok(())
}




