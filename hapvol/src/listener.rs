use std::{
  sync::{Arc, RwLock},
  thread,
};

use crate::volume::AudioDevice;

pub fn init_listener(volume_value: Arc<RwLock<f32>>) {
  thread::spawn(move || {
    let audio_device = AudioDevice::default();

    let mut last_volume_value = 0.0;

    loop {
      let new_value = volume_value.read().unwrap();

      if *new_value != last_volume_value {
        audio_device.set_volume(*new_value);
      }

      last_volume_value = *new_value;
    }
  });
}
