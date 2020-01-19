use std::os::raw::{c_ulong, c_char, c_void, c_uchar};
use std::ffi::CString;

// #[allow(non_camel_case_types)]
type FT_STATUS = c_ulong;
type FT_HANDLE = *mut c_void;

// FT_OPENEX FLAGS

const FT_OPEN_BY_SERIAL_NUMBER: i32 = 1;
const FT_OPEN_BY_DESCRIPTION: i32 = 2;
const FT_OPEN_BY_LOCATION: i32 = 4;

const FT_OPEN_MASK: i32 = FT_OPEN_BY_SERIAL_NUMBER | FT_OPEN_BY_DESCRIPTION | FT_OPEN_BY_LOCATION;

// FT_LIST_DEVICES FLAGS
// used with FT_OpenEx Flags
const FT_LIST_NUMBER_ONLY: i32 = 0x80000000;
const FT_LIST_BY_INDEX: i32 = 0x40000000;
const FT_LIST_ALL: i32 = 0x20000000;

const FT_LIST_MASK: i32 = FT_LIST_NUMBER_ONLY | FT_LIST_BY_INDEX | FT_LIST_ALL;

// BAUD RATES
const FT_BAUD_300: i32 = 300;
const FT_BAUD_600: i32 = 600;
const FT_BAUD_1200: i32 = 1200;
const FT_BAUD_2400: i32 = 2400;
const FT_BAUD_4800: i32 = 4800;
const FT_BAUD_9600: i32 = 9600;
const FT_BAUD_14400: i32 = 14400;
const FT_BAUD_19200: i32 = 19200;
const FT_BAUD_38400: i32 = 38400;
const FT_BAUD_57600: i32 = 57600;
const FT_BAUD_115200: i32 = 115200;
const FT_BAUD_230400: i32 = 230400;
const FT_BAUD_460800: i32 = 460800;
const FT_BAUD_921600: i32 = 921600;

// word lengths
const FT_BITS_8: c_uchar = 8;
const FT_BITS_7: c_uchar = 7;

// Stop Bits
const FT_STOP_BITS_1: c_uchar = 0;
const FT_STOP_BITS_2: c_uchar = 2;

// Parity
const FT_PARITY_NONE: c_uchar = 0;
const FT_PARITY_ODD: c_uchar = 0;
const FT_PARITY_EVEN: c_uchar = 0;
const FT_PARITY_MARK: c_uchar = 0;
const FT_PARITY_SPACE: c_uchar = 0;

// FLOW CONTROL
const FT_FLOW_NONE: i32     = 0x0000;
const FT_FLOW_RTS_CTS: i32  = 0x0100;
const FT_FLOW_DTR_DSR: i32  = 0x0200;
const FT_FLOW_XON_XOFF: i32 = 0x0400;

// purge rx and tx buffers
const FT_PURGE_RX: i32 = 1;
const FT_PURGE_TX: i32 = 2;

// events
// TODO: check callback
type PFT_EVEN_HANDLER = *mut c_void(c_ulong, c_ulong);

const FT_EVENT_RXCHAR: i32 = 1;
const FT_EVENT_MODEM_STATUS: i32 = 2;
const FT_EVENT_LINE_STATUS: i32 = 4;

// timeouts
const FT_DEFAULT_RX_TIMEOUT: i32 = 300;
const FT_DEFAULT_TX_TIMEOUT: i32 = 300;

type FT_DEVICE = c_ulong;

#[repr(C)]
enum FT_DEVICE_ENUM {
    FT_DEVICE_BM,
	FT_DEVICE_AM,
	FT_DEVICE_100AX,
	FT_DEVICE_UNKNOWN,
	FT_DEVICE_2232C,
	FT_DEVICE_232R,
	FT_DEVICE_2232H,
	FT_DEVICE_4232H,
	FT_DEVICE_232H,
	FT_DEVICE_X_SERIES,
	FT_DEVICE_4222H_0,
	FT_DEVICE_4222H_1_2,
	FT_DEVICE_4222H_3,
    FT_DEVICE_4222_PROG,
    FT_DEVICE_900,
    FT_DEVICE_930,
    FT_DEVICE_UMFTPD3A
}

// BIT MODES
const FT_BITMODE_RESET: i32 = 0x00;
const FT_BITMODE_ASYNC_BITBANG: i32 = 0x01;
const FT_BITMODE_MPSSE: i32 = 0x02;
const FT_BITMODE_SYNC_BITBANG: i32 = 0x04;
const FT_BITMODE_SYNC_MCU_HOST: i32 = 0x08;
const FT_BITMODE_FAST_SERIAL: i32 = 0x10;
const FT_BITMODE_CPUS_BITBANG: i32 = 0x20;
const FT_BITMODE_SYNC_FIFO: i32 = 0x40;

// FT232R CBUS Options EEPROM values
const FT_232R_CBUS_TXDEN: i32 = 0x00;
const FT_232R_CBUS_PWRON: i32 = 0x01;	//	Power On
const FT_232R_CBUS_RXLED: i32 = 0x02;	//	Rx LED
const FT_232R_CBUS_TXLED:i32 = 0x03;	//	Tx LED
const FT_232R_CBUS_TXRXLED: i32 = 0x04;	//	Tx and Rx LED
const FT_232R_CBUS_SLEEP: i32 = 0x05;	//	Sleep
const FT_232R_CBUS_CLK48: i32 = 0x06;	//	48MHz clock
const FT_232R_CBUS_CLK24: i32 = 0x07;	//	24MHz clock
const FT_232R_CBUS_CLK12: i32 = 0x08;	//	12MHz clock
const FT_232R_CBUS_CLK6: i32 = 0x09;	//	6MHz clock
const FT_232R_CBUS_IOMODE: i32 = 0x0A;	//	IO Mode for CBUS bit-bang
const FT_232R_CBUS_BITBANG_WR: i32 = 0x0B;	//	Bit-bang write strobe
const FT_232R_CBUS_BITBANG_RD: i32 = 0x0C;	//	Bit-bang read strobe

// FT232H CBUS Options EEPROM values
const FT_232H_CBUS_TRISTATE: i32 = 0x00;	//	Tristate
const FT_232H_CBUS_TXLED: i32 = 0x01;	//	Tx LED
const FT_232H_CBUS_RXLED: i32 = 0x02;	//	Rx LED
const FT_232H_CBUS_TXRXLED: i32 = 0x03;	//	Tx and Rx LED
const FT_232H_CBUS_PWREN: i32 = 0x04;	//	Power Enable
const FT_232H_CBUS_SLEEP: i32 = 0x05;	//	Sleep
const FT_232H_CBUS_DRIVE_0: i32 = 0x06;	//	Drive pin to logic 0
const FT_232H_CBUS_DRIVE_1: i32 = 0x07;	//	Drive pin to logic 1
const FT_232H_CBUS_IOMODE: i32 = 0x08;	//	IO Mode for CBUS bit-bang
const FT_232H_CBUS_TXDEN: i32 = 0x09;	//	Tx Data Enable
const FT_232H_CBUS_CLK30: i32 = 0x0A;	//	30MHz clock
const FT_232H_CBUS_CLK15: i32 = 0x0B;	//	15MHz clock
const FT_232H_CBUS_CLK7_5: i32 = 0x0C;	//	7.5MHz clock

// FT X Series CBUS Options EEPROM values
const FT_X_SERIES_CBUS_TRISTATE: i32 = 0x00;	//	Tristate
const FT_X_SERIES_CBUS_TXLED: i32 = 0x01;	//	Tx LED
const FT_X_SERIES_CBUS_RXLED: i32 = 0x02;	//	Rx LED
const FT_X_SERIES_CBUS_TXRXLED: i32 = 0x03;	//	Tx and Rx LED
const FT_X_SERIES_CBUS_PWREN: i32 = 0x04;	//	Power Enable
const FT_X_SERIES_CBUS_SLEEP: i32 = 0x05;	//	Sleep
const FT_X_SERIES_CBUS_DRIVE_0: i32 = 0x06;	//	Drive pin to logic 0
const FT_X_SERIES_CBUS_DRIVE_1: i32 = 0x07;	//	Drive pin to logic 1
const FT_X_SERIES_CBUS_IOMODE: i32 = 0x08;	//	IO Mode for CBUS bit-bang
const FT_X_SERIES_CBUS_TXDEN: i32 = 0x09;	//	Tx Data Enable
const FT_X_SERIES_CBUS_CLK24: i32 = 0x0A;	//	24MHz clock
const FT_X_SERIES_CBUS_CLK12: i32 = 0x0B;	//	12MHz clock
const FT_X_SERIES_CBUS_CLK6: i32 = 0x0C;	//	6MHz clock
const FT_X_SERIES_CBUS_BCD_CHARGER: i32 = 0x0D;	//	Battery charger detected
const FT_X_SERIES_CBUS_BCD_CHARGER_N: i32 = 0x0E;	//	Battery charger detected inverted
const FT_X_SERIES_CBUS_I2C_TXE: i32 = 0x0F;	//	I2C Tx empty
const FT_X_SERIES_CBUS_I2C_RXF: i32 = 0x10;	//	I2C Rx full
const FT_X_SERIES_CBUS_VBUS_SENSE: i32 = 0x11;	//	Detect VBUS
const FT_X_SERIES_CBUS_BITBANG_WR: i32 = 0x12;	//	Bit-bang write strobe
const FT_X_SERIES_CBUS_BITBANG_RD: i32 = 0x13;	//	Bit-bang read strobe
const FT_X_SERIES_CBUS_TIMESTAMP: i32 = 0x14;	//	Toggle output when a USB SOF token is received
const FT_X_SERIES_CBUS_KEEP_AWAKE: i32 = 0x15;	//	

//driver types
const FT_DRIVER_TYPE_D2XX: i32 = 0;
const FT_DRIVER_TYPE_VCP: i32 = 1;

#[repr(C)]
#[allow(dead_code, non_camel_case_types)]
enum FT_STATUS_ENUM {
    FT_OK,
    FT_INVALID_HANDLE,
	FT_DEVICE_NOT_FOUND,
	FT_DEVICE_NOT_OPENED,
	FT_IO_ERROR,
	FT_INSUFFICIENT_RESOURCES,
	FT_INVALID_PARAMETER,
	FT_INVALID_BAUD_RATE,

	FT_DEVICE_NOT_OPENED_FOR_ERASE,
	FT_DEVICE_NOT_OPENED_FOR_WRITE,
	FT_FAILED_TO_WRITE_DEVICE,
	FT_EEPROM_READ_FAILED,
	FT_EEPROM_WRITE_FAILED,
	FT_EEPROM_ERASE_FAILED,
	FT_EEPROM_NOT_PRESENT,
	FT_EEPROM_NOT_PROGRAMMED,
	FT_INVALID_ARGS,
	FT_NOT_SUPPORTED,
	FT_OTHER_ERROR,
	FT_DEVICE_LIST_NOT_READY
}

#[repr(C)]
struct FT_DEVICE_LIST_INFO_NODE {
    flags: c_ulong,
    ttype: c_ulong,
    id: c_ulong,
    loc_id: c_ulong,
    serial_number: [c_char; 16],
    description: [c_char; 64],
    ft_handle: FT_HANDLE
}

#[repr(C)]
enum FT_FLAGS {
    FT_FLAGS_OPENED = 1,
    FT_FLAGS_HISPEED = 2
}

// #[allow(dead_code)]
extern "C" {
    fn FT_SetVIDPID(
        dw_vid: c_ulong,
        dw_pid: c_ulong) -> FT_STATUS;
}

extern "C" {
    fn FT_GetVIDPID(
        pdw_vid: *mut c_ulong,
        pdw_pid: *mut c_ulong
    ) -> FT_STATUS;
}

extern "C" {
    fn FT_CreateDeviceInfoList(
        lpdw_num_devs: *mut c_ulong
    ) -> FT_STATUS;
}

extern "C" {
    fn FT_GetDeviceInfoList(
        p_dest: *mut FT_DEVICE_LIST_INFO_NODE,
        lpdw_num_devs: *mut c_ulong
    ) -> FT_STATUS;
}