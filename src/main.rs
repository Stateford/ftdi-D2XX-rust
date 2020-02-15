mod bindings;
use std::ptr::null_mut;
use std::os::raw::{c_ulong, c_char, c_void, c_uchar};
use std::ffi::CString;

fn main() {
    unsafe{
        let mut num_devices: c_ulong = 0;
        bindings::FT_CreateDeviceInfoList(&mut num_devices);

        println!("{}", num_devices)
    }
    println!("Hello, world!");
}
