use voltage_modbus::{ModbusRtuClient, ModbusClient, ModbusResult};
use std::time::Duration;

#[tokio::main]
async fn main() -> ModbusResult<()> {
    // TODO: auto-find and confirm port, baud rate, etc
    // full configuration (data bits, stop bits, parity, timeout)
    let mut client = ModbusRtuClient::with_config_and_logging(
        "/dev/cu.usbserialXXX",
        38400,
        tokio_serial::DataBits::Eight,
        tokio_serial::StopBits::One,
        tokio_serial::Parity::Even,  // Even parity
        Duration::from_secs(1),
        None,
    )?;

    // (Modbus unit ID, register address, number of registers)
    let read_tk1_tk2 = client.read_03(1, 0, 2).await?;
    println!("Temp TK1_TK2: {:?}", read_tk1_tk2);

    client.close().await?;
    Ok(())
}


