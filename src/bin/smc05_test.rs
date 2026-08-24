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




/// 
/// This function steps the cathode down until it makes solid contact with electrolyte
///
pub async fn step_down_to_contact_surface(ctx: &mut tokio_modbus::client::Context) -> Result<(), Box<dyn std::error::Error>> 
{
    const DRIVE_CURRENT_MA: f32 = 4.;

    // Enable specific anode connections
    let  anode_channels= [false, false, false, true];
    write_wav_octo_relays(ctx, &anode_channels).await?;

    setup_cathode_surface_probe(ctx).await?;

    let mut stepper_state = StepperDriverState::default();

    loop {
        let (measured_volts, measured_ma, measured_ohms) = 
            drive_current_and_measure(ctx, DRIVE_CURRENT_MA).await?;
        let cur_time_utc_ms = chrono::Utc::now().timestamp_millis();
        if measured_ma > 0. {
            println!("{} {:.2} V {:.2} mA {:.2} Ohms", cur_time_utc_ms, measured_volts, measured_ma, measured_ohms);
        }

        surface_contact_monitor(ctx, cur_time_utc_ms, &mut stepper_state, 2.0, measured_ma).await?;
        if stepper_state.surface_contact_start_ms != 0 {
            let contact_duration = cur_time_utc_ms - stepper_state.surface_contact_start_ms;
            if contact_duration > 20000 {
                println!("Contact duration: {}", contact_duration);
                break;
            }
        }
        sleep(Duration::from_millis(500)).await;
    }

    stop_smc05_rotation(ctx).await?;

    Ok(())
}

pub async fn sport_modes_test(ctx: &mut tokio_modbus::client::Context) -> Result<(), Box<dyn std::error::Error>> 
{
    // Enable a specific anode connection
    let  anode_channels= [false, false, false, false];
    write_wav_octo_relays(ctx, &anode_channels).await?;

    set_smc05_sport_mode(ctx, 3).await?;

    stop_smc05_rotation(ctx).await?;

    start_smc05_rev_rotation(ctx).await?;
    sleep(Duration::from_millis(1000)).await;
    start_smc05_fwd_rotation(ctx).await?;
    sleep(Duration::from_millis(1000)).await;
    start_smc05_rev_rotation(ctx).await?;
    sleep(Duration::from_millis(1000)).await;

    
    stop_smc05_rotation(ctx).await?;

    Ok(())
}


 /// Set the output drive current of the test electrodes 
async fn set_electrode_current_drive(ctx: &mut tokio_modbus::client::Context, milliamps: f32) -> Result<(), Box<dyn std::error::Error>> 
{
    // set_ykpvccs0100_current_drive(ctx, milliamps).await
    set_ykpvccs1000_current_drive(ctx, milliamps).await
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

    let start_time_ms = chrono::Utc::now().timestamp_millis();
    // sport_modes_test(&mut ctx).await?;
    step_down_to_contact_surface(&mut ctx).await?;
    let duration =  chrono::Utc::now().timestamp_millis() - start_time_ms;
    println!("Finished in {} ms", duration);

    set_electrode_current_drive(&mut ctx, 0.).await?;
    ctx.disconnect().await?;

    Ok(())
}




