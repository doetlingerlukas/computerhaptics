use std::{
  io,
  process::exit,
  time::Duration,
};

use serialport::*;

fn main() {
  let port_name = "COM3";
  let baud_rate = 57600;

  let settings = SerialPortSettings {
    baud_rate: baud_rate,
    data_bits: DataBits::Eight,
    flow_control: FlowControl::None,
    parity: Parity::None,
    stop_bits: StopBits::One,
    timeout: Duration::from_millis(10),
  };

  match open_with_settings(port_name, &settings) {
    Ok(mut port) => {
      let mut serial_buf: Vec<u8> = vec![0; 1000];
      println!("Receiving data on {} at {} baud:", &port_name, &baud_rate);

      loop {
        match port.read(serial_buf.as_mut_slice()) {
          Ok(t) => {
            let buffer = &serial_buf[..t];

            println!("{}", String::from_utf8_lossy(buffer));
          }
          Err(ref e) if e.kind() == io::ErrorKind::TimedOut => (),
          Err(e) => eprintln!("{:?}", e),
        }
      }
    }
    Err(e) => {
      eprintln!(
        "Failed to open serial port '{}' with error: {}",
        port_name, e
      );
      exit(1);
    }
  }
}
