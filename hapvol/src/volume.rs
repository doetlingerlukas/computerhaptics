#[cfg(windows)]
extern crate winapi;

use winapi::shared::*;
use winapi::um::endpointvolume::*;
use winapi::um::mmdeviceapi::*;
use winapi::um::objbase::*;
use winapi::Interface;

pub struct AudioDevice {
  endpoint: *mut IAudioEndpointVolume,
}

impl AudioDevice {
  pub fn set_volume(&self, volume: f32) {
    unsafe { (*self.endpoint).SetMasterVolumeLevelScalar(volume, std::ptr::null_mut()) };
  }
}

impl Default for AudioDevice {
  fn default() -> Self {
    Self { endpoint: get_audio_endpoint_volume() }
  }
}

fn get_device_enumerator() -> *mut IMMDeviceEnumerator {
  let cls_mm_device_enum: guiddef::GUID = CLSID_MMDeviceEnumerator;
  let iid_imm_device_enumerator = IMMDeviceEnumerator::uuidof();

  let mut device_enumerator: *mut IMMDeviceEnumerator = unsafe { std::mem::zeroed() };

  unsafe {
    winapi::um::combaseapi::CoCreateInstance(
      &cls_mm_device_enum,
      std::ptr::null_mut(),
      wtypesbase::CLSCTX_INPROC_SERVER,
      &iid_imm_device_enumerator,
      &mut device_enumerator as *mut *mut IMMDeviceEnumerator as *mut *mut winapi::ctypes::c_void,
    );
  }
  return device_enumerator;
}

fn get_imm_device(device_enumerator: *mut IMMDeviceEnumerator) -> *mut IMMDevice {
  let mut pp_device: *mut winapi::um::mmdeviceapi::IMMDevice = unsafe { std::mem::zeroed() };
  unsafe {
    (*device_enumerator).GetDefaultAudioEndpoint(
      winapi::um::mmdeviceapi::eRender,
      winapi::um::mmdeviceapi::eConsole,
      &mut pp_device,
    );
  }
  return pp_device;
}

fn get_iaudio_endpoint_volume(pp_device: *mut IMMDevice) -> *mut IAudioEndpointVolume {
  let cls_iaudio_endpoint_volume = IAudioEndpointVolume::uuidof();

  let mut endpoint_device: *mut IAudioEndpointVolume = unsafe { std::mem::zeroed() };

  unsafe {
    (*pp_device).Activate(
      &cls_iaudio_endpoint_volume,
      wtypesbase::CLSCTX_INPROC_SERVER,
      std::ptr::null_mut(),
      &mut endpoint_device as *mut *mut winapi::um::endpointvolume::IAudioEndpointVolume
        as *mut *mut winapi::ctypes::c_void,
    );
  }
  return endpoint_device;
}

fn get_audio_endpoint_volume() -> *mut IAudioEndpointVolume {
  unsafe { CoInitialize(std::ptr::null_mut()) };
  let device_enumerator: *mut IMMDeviceEnumerator = get_device_enumerator();
  let pp_device: *mut IMMDevice = get_imm_device(device_enumerator);
  return get_iaudio_endpoint_volume(pp_device);
}
