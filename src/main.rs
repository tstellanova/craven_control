// use std::time::Duration;
// use tokio::time::sleep;
// use tokio_modbus::client::Reader;
// use tokio_modbus::Result;
// use tokio_modbus::client::Client;

    
/// # Modbus node address assignments
///
/// | Address | Description |
/// |------|----------|
/// | 0x1F  | Current-Voltage 2 channel ADC   |
/// | 0x2F  | Precision current source (0-1000 mA)   |
/// | 0x3F  | Dual Type-K Thermocouple Reader |
///
/// 
/// 

const NODEID_DEFAULT: u8 = 0x01;
const NODEID_ADDR_IV_ADC: u8 = 0x1F;
const NODEID_ADDR_CURR_SRC: u8 =  0x2F;
const NODEID_DUAL_TK: u8 = 0x3F;
const NODEID_PYRO_CURR_GEN: u8 = 0x4F;
const NODEID_QUAD_RELAY: u8 = 0x5F;
const NODEID_MAX: u8 = 0x7F;


const REG_ADDR_IV_START: u16 = 0x00;

// --- TODO example of overwriting the dual TK address:
    // let first_node: Slave = Slave(0x01);
    // let port: SerialStream = SerialStream::open(&builder).unwrap();
    // let mut ctx = rtu::attach_slave(port, first_node);

    // const DEVICE_ADDRESS_REG:u16 = 0x20;
    // println!("Connecting first... ");
    // let read_rsp: Vec<u16> = ctx.read_holding_registers(DEVICE_ADDRESS_REG, 3).await??;
    // println!(" read_rsp: {:?}", read_rsp);

    // const NEW_DEVICE_ADDR:u8 = 0x3F;
    // let w_resp = ctx.write_single_register(DEVICE_ADDRESS_REG, NEW_DEVICE_ADDR.into()).await?;
    // println!(" w_resp: {:?}", w_resp);

    // println!("Disconnecting first");
    // ctx.disconnect().await?;


    // println!("Connecting second... ");
    // let second_node: Slave = Slave(NEW_DEVICE_ADDR);
    // let port = SerialStream::open(&builder).unwrap();
    // let mut ctx = rtu::attach_slave(port, second_node);
    // let read_rsp: Vec<u16> = ctx.read_holding_registers(DEVICE_ADDRESS_REG, 3).await??;
    // println!(" read_rsp: {:?}", read_rsp);

// --- TODO example of overwriting the IV ADC address:
    // const DEVICE_ADDRESS_REG:u16 = 0x40; // IV
    // println!("Connecting first... ");
    // let read_rsp: Vec<u16> = ctx.read_holding_registers(DEVICE_ADDRESS_REG, 3).await??;
    // println!(" read_rsp: {:?}", read_rsp);
    // const NEW_DEVICE_ADDR:u8 = NODEID_ADDR_IV_ADC;
    // let w_resp = ctx.write_single_register(DEVICE_ADDRESS_REG, NEW_DEVICE_ADDR.into()).await?;
    // println!(" w_resp: {:?}", w_resp);

    // println!("Disconnecting first");
    // ctx.disconnect().await?;

    // println!("Connecting second... ");
    // let second_node: Slave = Slave(NEW_DEVICE_ADDR);
    // let port = SerialStream::open(&builder).unwrap();
    // let mut ctx = rtu::attach_slave(port, second_node);
    // let read_rsp: Vec<u16> = ctx.read_holding_registers(DEVICE_ADDRESS_REG, 3).await??;
    // println!(" read_rsp: {:?}", read_rsp);

#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    use tokio_serial::SerialStream;

    use tokio_modbus::prelude::*;

    // actual usb device ID of one USB-RS485 adapter
    let tty_path = "/dev/cu.usbserial-BG02SI88";
    let builder = tokio_serial::new(tty_path, 9600);


    let first_node: Slave = Slave(NODEID_ADDR_IV_ADC); // NODEID_DEFAULT); // or eg NODEID_DUAL_TK
    let port: SerialStream = SerialStream::open(&builder).unwrap();
    let mut ctx = rtu::attach_slave(port, first_node);


    // const DEVICE_ADDRESS_REG:u16 = 0x20; // TK
    // const DEVICE_ADDRESS_REG:u16 = 0x40; // IV
    // println!("Connecting first... ");
    // let read_rsp: Vec<u16> = ctx.read_holding_registers(DEVICE_ADDRESS_REG, 3).await??;
    // println!(" read_rsp: {:?}", read_rsp);
    // const NEW_DEVICE_ADDR:u8 = NODEID_ADDR_IV_ADC;
    // let w_resp = ctx.write_single_register(DEVICE_ADDRESS_REG, NEW_DEVICE_ADDR.into()).await?;
    // println!(" w_resp: {:?}", w_resp);

    // println!("Disconnecting first");
    // ctx.disconnect().await?;

    // println!("Connecting second... ");
    // let second_node: Slave = Slave(NEW_DEVICE_ADDR);
    // let port = SerialStream::open(&builder).unwrap();
    // let mut ctx = rtu::attach_slave(port, second_node);
    // let read_rsp: Vec<u16> = ctx.read_holding_registers(DEVICE_ADDRESS_REG, 3).await??;
    // println!(" read_rsp: {:?}", read_rsp);

    // ----- TODO example of reading all the IV ADC registers
    let read_rsp: Vec<u16> = ctx.read_holding_registers(REG_ADDR_IV_START, 4).await??;
    println!(" read_rsp: {:?}", read_rsp);

    // // ----- TODO example of reading all the dual TK registers:
    // let read_rsp: Vec<u16> = ctx.read_holding_registers(0x00, 2).await??;
    // println!(" read_rsp: {:?}", read_rsp);

    // // let read_state = client.read_03(1, 0x10, 2).await;
    // let read_rsp: Vec<u16> = ctx.read_holding_registers(0x10, 2).await??;
    // println!(" read_rsp: {:?}", read_rsp);

    // // let read_cfg = client.read_03(1, 0x20, 3).await;
    // let read_rsp: Vec<u16> = ctx.read_holding_registers(0x20, 3).await??;
    // println!(" read_rsp: {:?}", read_rsp);
    
    // ----- TODO currently we're reading and writing the value of a 4-20 mA current loop controller
    // let milliamp_reg_addr: u16 = 1;

    // println!("Reading a sensor value");
    // let read_rsp: Vec<u16> = ctx.read_holding_registers(milliamp_reg_addr, 1).await??;
    // println!("read_rsp: {read_rsp:?}");
    // let original_ma_val = read_rsp[0];

    // let out_ma_setting = 80;
    // println!("writing val of : {}", out_ma_setting);
    // let w_rsp = ctx.write_single_register(milliamp_reg_addr, out_ma_setting).await?;
    // println!("w_rsp: {w_rsp:?}");
    // let read_rsp: Vec<u16> = ctx.read_holding_registers(milliamp_reg_addr, 1).await??;
    // println!("new val read_rsp: {read_rsp:?}");

    // sleep(Duration::from_millis(500)).await;

    // let out_ma_setting = original_ma_val;
    // println!("Resetting val of : {}", out_ma_setting);
    // let w_rsp = ctx.write_single_register(milliamp_reg_addr, out_ma_setting).await?;
    // println!("w_rsp: {w_rsp:?}");
    // let read_rsp = ctx.read_holding_registers(milliamp_reg_addr, 1).await??;
    // println!("read_rsp: {read_rsp:?}");
    // ----- TODO


    println!("Disconnecting");
    ctx.disconnect().await?;

    Ok(())
}



