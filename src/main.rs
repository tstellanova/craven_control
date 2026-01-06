use std::time::Duration;
use tokio::time::sleep;

// TODO read from temperature dual input sensor

//     let read_temp = client.read_03(1, 0x00, 6).await;
//     println!(" read_temp: {:?}", read_temp);
//     // let read_temp = client.read_03(1, 0x00, 2).await;
//     // println!(" read_temp: {:?}", read_temp);

//     // let read_state = client.read_03(1, 0x10, 2).await;
//     // println!("read_state: {:?}", read_state);

//     // let read_cfg = client.read_03(1, 0x20, 3).await;
//     // println!("read_cfg: {:?}", read_cfg);


#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    use tokio_serial::SerialStream;

    use tokio_modbus::prelude::*;

    // actual usb device ID of one USB-RS485 adapter
    let tty_path = "/dev/cu.usbserial-BG02SI88";
    let slave = Slave(0x01);

    let builder = tokio_serial::new(tty_path, 9600);
    let port = SerialStream::open(&builder).unwrap();
    let milliamp_reg_addr = 1;
    let mut ctx = rtu::attach_slave(port, slave);

    // ----- TODO currently we're reading and writing the value of a 4-20 mA current loop controller
    println!("Reading a sensor value");
    let read_rsp: Vec<u16> = ctx.read_holding_registers(milliamp_reg_addr, 1).await??;
    println!("read_rsp: {read_rsp:?}");
    let original_ma_val = read_rsp[0];

    let out_ma_setting = 80;
    println!("writing val of : {}", out_ma_setting);
    let w_rsp = ctx.write_single_register(milliamp_reg_addr, out_ma_setting).await?;
    println!("w_rsp: {w_rsp:?}");
    let read_rsp: Vec<u16> = ctx.read_holding_registers(milliamp_reg_addr, 1).await??;
    println!("new val read_rsp: {read_rsp:?}");

    sleep(Duration::from_millis(500)).await;

    let out_ma_setting = original_ma_val;
    println!("Resetting val of : {}", out_ma_setting);
    let w_rsp = ctx.write_single_register(milliamp_reg_addr, out_ma_setting).await?;
    println!("w_rsp: {w_rsp:?}");
    let read_rsp = ctx.read_holding_registers(milliamp_reg_addr, 1).await??;
    println!("read_rsp: {read_rsp:?}");
    // ----- TODO


    println!("Disconnecting");
    ctx.disconnect().await?;

    Ok(())
}



