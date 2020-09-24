use std::{
  io,
  process::exit,
  sync::{Arc, RwLock},
  time::Duration,
};

use serialport::SerialPortSettings;

mod volume;

mod listener;
use listener::init_listener;

fn main() {
  let port_name = "COM3";
  let baud_rate = 57600;

  let mut settings = SerialPortSettings::default();
  settings.baud_rate = baud_rate;
  settings.timeout = Duration::from_millis(10);

  let volume_lock = Arc::new(RwLock::new(0.0));
  init_listener(volume_lock.clone());

  match serialport::open_with_settings(port_name, &settings) {
    Ok(mut port) => {
      let mut serial_buf: Vec<u8> = vec![0; 10];
      println!("Receiving data on {} at {} baud:", &port_name, &baud_rate);

      loop {
        match port.read(serial_buf.as_mut_slice()) {
          Ok(t) => {
            let buffer = &serial_buf[..t];

            if let Ok(val) = String::from_utf8_lossy(buffer).trim().parse::<f32>() {
              if val > 0.0 && val < 1.0 {
                let mut volume_ref = volume_lock.write().unwrap();
                *volume_ref = val;
              }
            };
          }
          Err(ref e) if e.kind() == io::ErrorKind::TimedOut => (),
          Err(e) => eprintln!("{:?}", e),
        }
      }
    }
    Err(e) => {
      eprintln!("Failed to open serial port '{}' with error: {}", port_name, e);
      exit(1);
    }
  }
}
