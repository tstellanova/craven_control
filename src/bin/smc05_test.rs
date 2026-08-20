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
    const CHECK_CURRENT_MA: f32 = DRIVE_CURRENT_MA - 0.25;

    // Enable specific anode connections
    let  anode_channels= [false, false, false, true];
    write_wav_octo_relays(ctx, &anode_channels).await?;

    report_system_config(ctx).await?;
    set_fwd_speed(ctx, 60.).await?;
    set_rev_speed(ctx, 5.).await?;
    enable_sport_mode03(ctx).await?;
    report_system_config(ctx).await?;

    let mut contact_start_time_ms = 0;
    loop {
        let (measured_volts, measured_ma, measured_ohms) = 
            drive_current_and_measure(ctx, DRIVE_CURRENT_MA).await?;
        let cur_time_utc_ms = chrono::Utc::now().timestamp_millis();
        if measured_ma > 0. {
            println!("{} {:.2} V {:.2} mA {:.2} Ohms", cur_time_utc_ms, measured_volts, measured_ma, measured_ohms);
        }
        if measured_ma > CHECK_CURRENT_MA {
            println!("touchdown!");
            if contact_start_time_ms == 0 {
                stop_smc05_rotation(ctx).await?;
                contact_start_time_ms = cur_time_utc_ms;
            }
            else {
                start_smc05_rev_rotation(ctx).await?;
                let contact_duration_ms = cur_time_utc_ms - contact_start_time_ms;
                if contact_duration_ms > 10000 {
                    println!("Finished contact duration: {}", contact_duration_ms);
                    break;
                }
            }
        }
        else {
            // not in contact
            if contact_start_time_ms != 0 {
                println!("{} Lost contact!",cur_time_utc_ms);
                contact_start_time_ms = 0;
            }
            // Move some increment FWD / down into the crucible
            start_smc05_fwd_rotation(ctx).await?;
q        }
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

    let (op_status, _motor_direction) = report_smc05_motor_status(ctx).await?;
    if op_status != 0 {
        stop_smc05_rotation(ctx).await?;
    }

    start_smc05_rev_rotation(ctx).await?;
    sleep(Duration::from_millis(1000)).await;
    start_smc05_fwd_rotation(ctx).await?;
    sleep(Duration::from_millis(1000)).await;
    start_smc05_rev_rotation(ctx).await?;
    sleep(Duration::from_millis(1000)).await;

    
    stop_smc05_rotation(ctx).await?;

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

    let start_time_ms = chrono::Utc::now().timestamp_millis();
    // sport_modes_test(&mut ctx).await?;
    step_down_to_contact_surface(&mut ctx).await?;
    let duration =  chrono::Utc::now().timestamp_millis() - start_time_ms;
    println!("Finished in {} ms", duration);

    ctx.disconnect().await?;

    Ok(())
}




