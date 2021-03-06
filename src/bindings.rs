use std::os::raw::{c_ulong, c_char, c_void, c_uchar, c_ushort, c_uint, c_long};
use std::ffi::CString;

// #[allow(non_camel_case_types)]
type FT_STATUS = c_ulong;
type FT_HANDLE = *mut c_void;

type DWORD = c_ulong;
type WORD = c_uint;

// FT_OPENEX FLAGS

const FT_OPEN_BY_SERIAL_NUMBER: i32 = 1;
const FT_OPEN_BY_DESCRIPTION: i32 = 2;
const FT_OPEN_BY_LOCATION: i32 = 4;

const FT_OPEN_MASK: i32 = FT_OPEN_BY_SERIAL_NUMBER | FT_OPEN_BY_DESCRIPTION | FT_OPEN_BY_LOCATION;

// FT_LIST_DEVICES FLAGS
// used with FT_OpenEx Flags
const FT_LIST_NUMBER_ONLY: i64 = 0x80000000;
const FT_LIST_BY_INDEX: i64 = 0x40000000;
const FT_LIST_ALL: i64 = 0x20000000;

const FT_LIST_MASK: i64 = FT_LIST_NUMBER_ONLY | FT_LIST_BY_INDEX | FT_LIST_ALL;

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
//type PFT_EVEN_HANDLER = *mut c_void(c_ulong, c_ulong);

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

#[link(name="ftd2xx", kind="dylib")]
extern "C" {
    fn FT_Initialise(
        pvoid: c_void,
    ) -> FT_STATUS;

    fn FT_Finalise(
        pvoid: c_void
    ) -> c_void;

    fn FT_Open(
        device_number: i32,
        p_handle: *mut FT_HANDLE
    ) -> FT_STATUS;

    fn FT_OpenEx(
        parg: *mut c_void,
        flags: c_ulong,
        phandle: *mut FT_HANDLE
    ) -> FT_STATUS;

    fn FT_ListDevices(
        parg1: *mut c_void,
        parg2: *mut c_void,
        flags: c_ulong
    ) -> FT_STATUS;

    fn FT_Close(
        handle: FT_HANDLE
    ) -> FT_STATUS;

    fn FT_Read(
        handle: FT_HANDLE,
        lpbuffer: *mut c_void,
        bytes_to_read: c_ulong,
        bytes_returned: *mut c_ulong
    ) -> FT_STATUS;

    pub fn FT_Write(
        handle: FT_HANDLE,
        lpbuffer: *mut c_void,
        bytes_to_write: c_ulong,
        bytes_written: *mut c_ulong
    ) -> FT_STATUS;

/*
    pub fn FT_IoCtl(
        handle: FT_HANDLE,
        io_control_code: c_ulong,
        in_buffer: *mut c_void,
        in_buffer_size: c_ulong,
        out_buffer: *mut c_void,
        out_buffer_size: c_ulong,
        bytes_returned: *mut c_void,
        overlapped: *mut OVERLAPPED
    ) -> FT_STATUS;
*/

    pub fn FT_SetBaudRate(
        handle: FT_HANDLE,
        baud_rate: c_ulong
    ) -> FT_STATUS;

    pub fn FT_SetDivisor(
        handle: FT_HANDLE,
        divisor: c_ushort
    ) -> FT_STATUS;

    pub fn FT_SetDataCharacteristics(
        handle: FT_HANDLE,
        word_length: c_uchar,
        stop_bits: c_uchar,
        parity: c_uchar
    ) -> FT_STATUS;

    pub fn FT_SetFlowControl(
        handle: FT_HANDLE,
        flow_control: c_ushort,
        x_on_char: c_uchar,
        x_off_char: c_uchar
    ) -> FT_STATUS;

    pub fn FT_ResetDevice(
        handle: FT_HANDLE
    ) -> FT_STATUS;

    pub fn FT_SetDtr(
        handle: FT_HANDLE
    ) -> FT_STATUS;

    pub fn FT_ClrDtr(
        handle: FT_HANDLE
    ) -> FT_STATUS;

    pub fn FT_SetRts(
        handle: FT_HANDLE
    ) -> FT_STATUS;

    pub fn FT_ClrRts(
        handle: FT_HANDLE
    ) -> FT_STATUS;

    pub fn FT_GetModemStatus(
        handle: FT_HANDLE,
        modem_status: *mut c_ulong
    ) -> FT_STATUS;

    pub fn FT_SetChars(
        handle: FT_HANDLE,
        event_char: c_uchar,
        event_char_enabled: c_uchar,
        error_char: c_uchar,
        error_char_enabled: c_char
    ) -> FT_STATUS;

    pub fn FT_Purge(
        handle: FT_HANDLE,
        mask: c_ulong
    );

    pub fn FT_SetTimeouts(
        handle: FT_HANDLE,
        read_timeout: c_ulong,
        write_timeout: c_ulong
    ) -> FT_STATUS;

    pub fn GT_GetQueueStatus(
        handle: FT_HANDLE,
        rx_bytes: *mut c_ulong
    ) -> FT_STATUS;

    pub fn FT_SetEventNotification(
        handle: FT_HANDLE,
        mask: c_ulong,
        param: *mut c_void
    ) -> FT_STATUS;

    pub fn FT_GetStatus(
        handle: FT_HANDLE,
        rx_bytes: *mut c_ulong,
        tx_bytes: *mut c_ulong,
        event_dword: *mut c_ulong
    ) -> FT_STATUS;

    pub fn FT_SetBreakOn(
        handle: FT_HANDLE
    ) -> FT_STATUS;

    pub fn FT_SetBreakOff(
        handle: FT_HANDLE
    ) -> FT_STATUS;

    pub fn FT_SetWaitMask(
        handle: FT_HANDLE,
        mask: c_ulong
    ) -> FT_STATUS;

    pub fn FT_WaitOnMask(
        handle: FT_HANDLE,
        mask: *mut c_ulong
    ) -> FT_STATUS;

    pub fn FT_GetEventStatus(
        handle: FT_HANDLE,
        event_dword: *mut c_ulong
    ) -> FT_STATUS;

    pub fn FT_ReadEE(
        handle: FT_HANDLE,
        word_offset: c_ulong,
        value: *mut c_ulong
    ) -> FT_STATUS;

    pub fn FT_WriteEE(
        handle: FT_HANDLE,
        word_offset: c_ulong,
        value: c_uint
    ) -> FT_STATUS;

    pub fn FT_EraseEE(
        handle: FT_HANDLE
    ) -> FT_STATUS;
}

#[repr(C)]
pub struct FT_PROGRAM_DATA {
    Signature1: c_ulong,			// Header - must be 0x00000000 
    Signature2: c_ulong,			// Header - must be 0xffffffff
    Version: c_ulong,				// Header - FT_PROGRAM_DATA version
    //			0 = original
    //			1 = FT2232 extensions
    //			2 = FT232R extensions
    //			3 = FT2232H extensions
    //			4 = FT4232H extensions
    //			5 = FT232H extensions

    VendorId: c_uint,				// 0x0403
    ProductId: c_uint,				// 0x6001
    Manufacturer: *mut char,			// "FTDI"
    ManufacturerId: *mut char,		// "FT"
    Description: *mut char,			// "USB HS Serial Converter"
    SerialNumber: *mut char,			// "FT000001" if fixed, or NULL
    MaxPower: c_uint,				// 0 < MaxPower <= 500
    PnP: c_uint,					// 0 = disabled, 1 = enabled
    SelfPowered: c_uint,			// 0 = bus powered, 1 = self powered
    RemoteWakeup: c_uint,			// 0 = not capable, 1 = capable
    //
    // Rev4 (FT232B) extensions
    //
    Rev4: c_uchar,					// non-zero if Rev4 chip, zero otherwise
    IsoIn: c_uchar,				// non-zero if in endpoint is isochronous
    IsoOut: c_uchar,				// non-zero if out endpoint is isochronous
    PullDownEnable: c_uchar,		// non-zero if pull down enabled
    SerNumEnable: c_uchar,			// non-zero if serial number to be used
    USBVersionEnable: c_uchar,		// non-zero if chip uses USBVersion
    USBVersion: c_uchar,			// BCD (0x0200 => USB2)
    //
    // Rev 5 (FT2232) extensions
    //
    Rev5: c_uchar,					// non-zero if Rev5 chip, zero otherwise
    IsoInA: c_uchar,				// non-zero if in endpoint is isochronous
    IsoInB: c_uchar,				// non-zero if in endpoint is isochronous
    IsoOutA: c_uchar,				// non-zero if out endpoint is isochronous
    IsoOutB: c_uchar,				// non-zero if out endpoint is isochronous
    PullDownEnable5: c_uchar,		// non-zero if pull down enabled
    SerNumEnable5: c_uchar,		// non-zero if serial number to be used
    USBVersionEnable5: c_uchar,	// non-zero if chip uses USBVersion
    USBVersion5: c_uchar,			// BCD (0x0200 => USB2)
    AIsHighCurrent: c_uchar,		// non-zero if interface is high current
    BIsHighCurrent: c_uchar,		// non-zero if interface is high current
    IFAIsFifo: c_uchar,			// non-zero if interface is 245 FIFO
    IFAIsFifoTar: c_uchar,			// non-zero if interface is 245 FIFO CPU target
    IFAIsFastSer: c_uchar,			// non-zero if interface is Fast serial
    AIsVCP: c_uchar,				// non-zero if interface is to use VCP drivers
    IFBIsFifo: c_uchar,			// non-zero if interface is 245 FIFO
    IFBIsFifoTar: c_uchar,			// non-zero if interface is 245 FIFO CPU target
    IFBIsFastSer: c_uchar,			// non-zero if interface is Fast serial
    BIsVCP: c_uchar,				// non-zero if interface is to use VCP drivers
    //
    // Rev 6 (FT232R) extensions
    //
    UseExtOsc: c_uchar,			// Use External Oscillator
    HighDriveIOs: c_uchar,			// High Drive I/Os
    EndpointSize: c_uchar,			// Endpoint size
    PullDownEnableR: c_uchar,		// non-zero if pull down enabled
    SerNumEnableR: c_uchar,		// non-zero if serial number to be used
    InvertTXD: c_uchar,			// non-zero if invert TXD
    InvertRXD: c_uchar,			// non-zero if invert RXD
    InvertRTS: c_uchar,			// non-zero if invert RTS
    InvertCTS: c_uchar,			// non-zero if invert CTS
    InvertDTR: c_uchar,			// non-zero if invert DTR
    InvertDSR: c_uchar,			// non-zero if invert DSR
    InvertDCD: c_uchar,			// non-zero if invert DCD
    InvertRI: c_uchar,				// non-zero if invert RI
    Cbus0: c_uchar,				// Cbus Mux control
    Cbus1: c_uchar,				// Cbus Mux control
    Cbus2: c_uchar,				// Cbus Mux control
    Cbus3: c_uchar,				// Cbus Mux control
    Cbus4: c_uchar,				// Cbus Mux control
    RIsD2XX: c_uchar,				// non-zero if using D2XX driver
    //
    // Rev 7 (FT2232H) Extensions
    //
    PullDownEnable7: c_uchar,		// non-zero if pull down enabled
    SerNumEnable7: c_uchar,		// non-zero if serial number to be used
    ALSlowSlew: c_uchar,			// non-zero if AL pins have slow slew
    ALSchmittInput: c_uchar,		// non-zero if AL pins are Schmitt input
    ALDriveCurrent: c_uchar,		// valid values are 4mA, 8mA, 12mA, 16mA
    AHSlowSlew: c_uchar,			// non-zero if AH pins have slow slew
    AHSchmittInput: c_uchar,		// non-zero if AH pins are Schmitt input
    AHDriveCurrent: c_uchar,		// valid values are 4mA, 8mA, 12mA, 16mA
    BLSlowSlew: c_uchar,			// non-zero if BL pins have slow slew
    BLSchmittInput: c_uchar,		// non-zero if BL pins are Schmitt input
    BLDriveCurrent: c_uchar,		// valid values are 4mA, 8mA, 12mA, 16mA
    BHSlowSlew: c_uchar,			// non-zero if BH pins have slow slew
    BHSchmittInput: c_uchar,		// non-zero if BH pins are Schmitt input
    BHDriveCurrent: c_uchar,		// valid values are 4mA, 8mA, 12mA, 16mA
    IFAIsFifo7: c_uchar,			// non-zero if interface is 245 FIFO
    IFAIsFifoTar7: c_uchar,		// non-zero if interface is 245 FIFO CPU target
    IFAIsFastSer7: c_uchar,		// non-zero if interface is Fast serial
    AIsVCP7: c_uchar,				// non-zero if interface is to use VCP drivers
    IFBIsFifo7: c_uchar,			// non-zero if interface is 245 FIFO
    IFBIsFifoTar7: c_uchar,		// non-zero if interface is 245 FIFO CPU target
    IFBIsFastSer7: c_uchar,		// non-zero if interface is Fast serial
    BIsVCP7: c_uchar,				// non-zero if interface is to use VCP drivers
    PowerSaveEnable: c_uchar,		// non-zero if using BCBUS7 to save power for self-powered designs
    //
    // Rev 8 (FT4232H) Extensions
    //
    PullDownEnable8: c_uchar,		// non-zero if pull down enabled
    SerNumEnable8: c_uchar,		// non-zero if serial number to be used
    ASlowSlew: c_uchar,			// non-zero if A pins have slow slew
    ASchmittInput: c_uchar,		// non-zero if A pins are Schmitt input
    ADriveCurrent: c_uchar,		// valid values are 4mA, 8mA, 12mA, 16mA
    BSlowSlew: c_uchar,			// non-zero if B pins have slow slew
    BSchmittInput: c_uchar,		// non-zero if B pins are Schmitt input
    BDriveCurrent: c_uchar,		// valid values are 4mA, 8mA, 12mA, 16mA
    CSlowSlew: c_uchar,			// non-zero if C pins have slow slew
    CSchmittInput: c_uchar,		// non-zero if C pins are Schmitt input
    CDriveCurrent: c_uchar,		// valid values are 4mA, 8mA, 12mA, 16mA
    DSlowSlew: c_uchar,			// non-zero if D pins have slow slew
    DSchmittInput: c_uchar,		// non-zero if D pins are Schmitt input
    DDriveCurrent: c_uchar,		// valid values are 4mA, 8mA, 12mA, 16mA
    ARIIsTXDEN: c_uchar,			// non-zero if port A uses RI as RS485 TXDEN
    BRIIsTXDEN: c_uchar,			// non-zero if port B uses RI as RS485 TXDEN
    CRIIsTXDEN: c_uchar,			// non-zero if port C uses RI as RS485 TXDEN
    DRIIsTXDEN: c_uchar,			// non-zero if port D uses RI as RS485 TXDEN
    AIsVCP8: c_uchar,				// non-zero if interface is to use VCP drivers
    BIsVCP8: c_uchar,				// non-zero if interface is to use VCP drivers
    CIsVCP8: c_uchar,				// non-zero if interface is to use VCP drivers
    DIsVCP8: c_uchar,				// non-zero if interface is to use VCP drivers
    //
    // Rev 9 (FT232H) Extensions
    //
    PullDownEnableH: c_uchar,		// non-zero if pull down enabled
    SerNumEnableH: c_uchar,		// non-zero if serial number to be used
    ACSlowSlewH: c_uchar,			// non-zero if AC pins have slow slew
    ACSchmittInputH: c_uchar,		// non-zero if AC pins are Schmitt input
    ACDriveCurrentH: c_uchar,		// valid values are 4mA, 8mA, 12mA, 16mA
    ADSlowSlewH: c_uchar,			// non-zero if AD pins have slow slew
    ADSchmittInputH: c_uchar,		// non-zero if AD pins are Schmitt input
    ADDriveCurrentH: c_uchar,		// valid values are 4mA, 8mA, 12mA, 16mA
    Cbus0H: c_uchar,				// Cbus Mux control
    Cbus1H: c_uchar,				// Cbus Mux control
    Cbus2H: c_uchar,				// Cbus Mux control
    Cbus3H: c_uchar,				// Cbus Mux control
    Cbus4H: c_uchar,				// Cbus Mux control
    Cbus5H: c_uchar,				// Cbus Mux control
    Cbus6H: c_uchar,				// Cbus Mux control
    Cbus7H: c_uchar,				// Cbus Mux control
    Cbus8H: c_uchar,				// Cbus Mux control
    Cbus9H: c_uchar,				// Cbus Mux control
    IsFifoH: c_uchar,				// non-zero if interface is 245 FIFO
    IsFifoTarH: c_uchar,			// non-zero if interface is 245 FIFO CPU target
    IsFastSerH: c_uchar,			// non-zero if interface is Fast serial
    IsFT1248H: c_uchar,			// non-zero if interface is FT1248
    FT1248CpolH: c_uchar,			// FT1248 clock polarity - clock idle high (1) or clock idle low (0)
    FT1248LsbH: c_uchar,			// FT1248 data is LSB (1) or MSB (0)
    FT1248FlowControlH: c_uchar,	// FT1248 flow control enable
    IsVCPH: c_uchar,				// non-zero if interface is to use VCP drivers
    PowerSaveEnableH: c_uchar,		// non-
}

#[link(name="ftd2xx", kind="dylib")]
extern "C" {
    pub fn FT_EE_Program(
        handle: FT_HANDLE,
        data: *mut FT_PROGRAM_DATA
    ) -> FT_STATUS;

    pub fn FT_EE_ProgramEx(
        handle: FT_HANDLE,
        data: *mut FT_PROGRAM_DATA,
        manufacturer: *mut c_char,
        manufacturer_id: *mut c_char,
        description: *mut c_char,
        serial_number: *mut c_char
    ) -> FT_STATUS;

    pub fn FT_EE_Read(
        handle: FT_HANDLE,
        data: *mut FT_PROGRAM_DATA
    ) -> FT_STATUS;

    pub fn FT_EE_ReadEx(
        handle: FT_HANDLE,
        data: *mut FT_PROGRAM_DATA,
        manufacturer: *mut c_char,
        manufacturer_id: *mut c_char,
        description: *mut c_char,
        serial_number: *mut c_char
    ) -> FT_STATUS;

    pub fn FT_EE_UASize(
        handle: FT_HANDLE,
        size: *mut c_ulong
    ) -> FT_STATUS;

    pub fn FT_EE_UAWrite(
        handle: FT_HANDLE,
        data: *mut c_uchar,
        data_len: c_ulong
    ) -> FT_STATUS;

    pub fn FT_EE_UARead(
        handle: FT_HANDLE,
        data: *mut c_uchar,
        data_len: DWORD,
        bytes_read: *mut DWORD
    ) -> FT_STATUS;
}

#[repr(C)]
struct ft_eeprom_header {
    device_type: FT_DEVICE,
    vendor_id: WORD,
    product_id: WORD,
    serial_number_enable: c_uchar,
    max_power: WORD,
    self_powered: c_uchar,
    remote_wakeup: c_uchar,
    pull_down_enable: c_uchar
}

type FT_EEPROM_HEADER = ft_eeprom_header;
type PFT_EEPROM_HEADER = *mut ft_eeprom_header;

#[repr(C)]
struct ft_eeprom_232b {
    common: FT_EEPROM_HEADER
}

type FT_EEPROM_232B = ft_eeprom_232b;
type PFT_EEPROM_232B = *mut ft_eeprom_232b; 


#[repr(C)]
struct ft_eeprom_2232 {
    common: FT_EEPROM_HEADER,
    a_is_high_current: c_uchar,
    b_is_high_current: c_uchar,
    
    a_is_fifo: c_uchar,
    a_is_fifo_tar: c_uchar,
    a_is_fast_ser: c_char,

    b_is_fifo: c_uchar,
    b_is_fifo_tar: c_uchar,
    b_is_fast_ser: c_char,

    a_driver_type: c_uchar,
    b_driver_type: c_char
}

type FT_EEPROM_2232 = ft_eeprom_2232;
type PFT_EEPROM_2232 = *mut ft_eeprom_2232;

#[repr(C)]
struct ft_eeprom_232r {
    common: FT_EEPROM_HEADER,
    is_high_current: c_uchar,
    use_ext_osc: c_uchar,
    invert_txd: c_uchar,
    invert_rxd: c_uchar,
    invert_rts: c_uchar,
    invert_cts: c_uchar,
    invert_dtr: c_uchar,
    invert_dsr: c_uchar,
    invert_dcd: c_uchar,
    invert_ri: c_uchar,
    c_bus_0: c_uchar,
    c_bus_1: c_uchar,
    c_bus_2: c_uchar,
    c_bus_3: c_uchar,
    c_bus_4: c_uchar,
    driver_type: c_uchar
}

type FT_EEPROM_232R = ft_eeprom_232r;
type PFT_EEPROM_232R = *mut ft_eeprom_232r;


#[repr(C)]
struct ft_eeprom_2232h {
    common: FT_EEPROM_HEADER,
    al_slow_slew: c_uchar,
    al_schmitt_input: c_uchar,
    al_drive_current: c_uchar,
    ah_slow_slew: c_uchar,
    ah_drive_current: c_uchar,
    bl_slow_slew: c_uchar,
    bl_schmitt_input: c_uchar,
    bl_drive_current: c_uchar,
    bh_slow_slew: c_uchar,
    bh_schmitt_input: c_uchar,
    bh_drive_current: c_uchar,

    a_is_fifo: c_uchar,
    a_is_fifo_tar: c_uchar,
    a_is_fast_ser: c_uchar,
    b_is_fifo: c_uchar,
    b_is_fifo_tar: c_uchar,
    b_is_fast_ser: c_uchar,
    power_save_enable: c_uchar,

    a_driver_type: c_uchar,
    b_driver_type: c_uchar
}

type FT_EEPROM_2232H = ft_eeprom_2232h;
type PFT_EEPROM_2232H = *mut ft_eeprom_2232h;

#[repr(C)]
struct ft_eeprom_4232h {
    common: FT_EEPROM_HEADER,

    a_slow_slew: c_uchar,
    a_schmitt_input: c_uchar,
    a_drive_current: c_uchar,

    b_slow_slew: c_uchar,
    b_schmitt_input: c_uchar,
    b_drive_current: c_uchar,

    c_slow_slew: c_uchar,
    c_schmitt_input: c_uchar,
    c_drive_current: c_uchar,

    d_slow_slew: c_uchar,
    d_schmitt_input: c_uchar,
    d_drive_current: c_uchar,

    ariis_tx_den: c_uchar,
    briis_tx_den: c_uchar,
    criis_tx_den: c_uchar,
    driis_tx_den: c_uchar,

    a_driver_type: c_uchar,
    b_driver_type: c_uchar,
    c_driver_type: c_uchar,
    d_driver_type: c_uchar
}

type FT_EEPROM_4232H = ft_eeprom_4232h;
type PFT_EEPROM_4232H = *mut ft_eeprom_4232h;


#[repr(C)]
struct ft_eeprom_232h {
    common: FT_EEPROM_HEADER,
    ac_slow_slew: c_uchar,
    ac_schmitt_input: c_uchar,
    ac_drive_current: c_uchar,
    ad_slow_slew: c_uchar,
    ad_schmitt_input: c_uchar,
    ad_drive_current: c_uchar,

    cbus0: c_uchar,
    cbus1: c_uchar,
    cbus2: c_uchar,
    cbus3: c_uchar,
    cbus4: c_uchar,
    cbus5: c_uchar,
    cbus6: c_uchar,
    cbus7: c_uchar,
    cbus8: c_uchar,
    cbus9: c_uchar,

    ft1248_cpol: c_uchar,
    ft1248_lsb: c_uchar,
    ft1248_flow_control: c_uchar,

    is_fifo: c_uchar,
    is_fifo_tar: c_uchar,
    is_fast_ser: c_uchar,
    is_ft1248: c_uchar,
    power_save_enable: c_uchar,

    driver_type: c_uchar
}

type FT_EEPROM_232H = ft_eeprom_232h;
type PFT_EEPROM_232H = *mut ft_eeprom_232h;

#[repr(C)]
struct ft_eeprom_x_series {
    common: FT_EEPROM_HEADER,

    ac_slow_slew: c_uchar,
    ac_schmitt_input: c_uchar,
    ac_drive_current: c_uchar,
    ad_slow_slew: c_uchar,
    ad_schmitt_input: c_uchar,
    ad_drive_current: c_uchar,

    cbus0: c_uchar,
    cbus1: c_uchar,
    cbus2: c_uchar,
    cbus3: c_uchar,
    cbus4: c_uchar,
    cbus5: c_uchar,
    cbus6: c_uchar,

    invert_txd: c_uchar,
    invert_rxd: c_uchar,
    invert_rts: c_uchar,
    invert_cts: c_uchar,
    invert_dtr: c_uchar,
    invert_dsr: c_uchar,
    invert_dcd: c_uchar,
    invert_ri: c_uchar,

    bcd_enable: c_uchar,
    bcd_force_cbus_pwr_en: c_uchar,
    bcd_disable_sleep: c_uchar,

    i2c_slace_address: WORD,
    i2c_device_id: DWORD,
    i2c_disable_schmitt: c_uchar,

    ft1248_cpol: c_uchar,
    ft1248_lsb: c_uchar,
    ft1248_flow_control: c_uchar,

    rs485_echo_suppress: c_uchar,
    power_save_enable: c_uchar,

    driver_type: c_uchar
}

type FT_EEPROM_X_SERIES = ft_eeprom_x_series;
type PFT_EEPROM_X_SERIES = *mut ft_eeprom_x_series;

#[link(name="ftd2xx", kind="dylib")]
extern "C" {
    pub fn FT_EEPROM_Read(
        ft_handle: FT_HANDLE,
        eeprom_data: *mut c_void,
        eeprom_data_size: DWORD,
        manufacturer: *mut c_char,
        manufacturer_id: *mut c_char,
        description: *mut c_char,
        serial_number: *mut c_char
    ) -> FT_STATUS;

    pub fn FT_EEPROM_Program(
        ft_handle: FT_HANDLE,
        eeprom_data: *mut c_void,
        eeprom_data_size: DWORD,
        manufacturer: *mut c_char,
        manufacturer_id: *mut c_char,
        description: *mut c_char,
        serial_number: *mut c_char
    ) -> FT_STATUS;

    pub fn FT_SetLatencyTimer(
        handle: FT_HANDLE,
        latency: c_uchar
    ) -> FT_STATUS;

    pub fn FT_GetLatencyTimer(
        handle: FT_HANDLE,
        latency: *mut c_uchar
    ) -> FT_STATUS;

    pub fn FT_SetBitMode(
        handle: FT_HANDLE,
        mask: c_uchar,
        enable: c_uchar
    ) -> FT_STATUS;

    pub fn FT_GetBitMode(
        handle: FT_HANDLE,
        mode: *mut c_uchar
    ) -> FT_STATUS;

    pub fn FT_SetUSBParameters(
        handle: FT_HANDLE,
        in_transfer_size: c_ulong,
        out_transfer_size: c_ulong
    ) -> FT_STATUS;

    pub fn FT_SetDeadmanTimeout(
        handle: FT_HANDLE,
        deadman_timeout: c_ulong
    ) -> FT_STATUS;
}


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
pub struct FT_DEVICE_LIST_INFO_NODE {
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
#[link(name="ftd2xx", kind="dylib")]
extern "C" {
    pub fn FT_SetVIDPID(
        vid: DWORD,
        pid: DWORD
    ) -> FT_STATUS;

    pub fn FT_GetVIDPID(
        vid: *mut DWORD,
        pid: *mut DWORD
    ) -> FT_STATUS;

    pub fn FT_GetDeviceLocId(
        handle: FT_HANDLE,
        loc_id: *mut DWORD
    ) -> FT_STATUS;

    pub fn FT_CreateDeviceInfoList(
        lpdw_num_devs: *mut c_ulong
    ) -> FT_STATUS;

    pub fn FT_GetDeviceInfoList(
        p_dest: *mut FT_DEVICE_LIST_INFO_NODE,
        lpdw_num_devs: *mut c_ulong
    ) -> FT_STATUS;

    pub fn FT_GetDeviceInfoDetail(
        index: DWORD,
        flags: *mut DWORD,
        dw_type: *mut DWORD,
        id: *mut DWORD,
        loc_id: *mut DWORD,
        serial_number: *mut c_void,
        description: *mut c_void,
        handle: FT_HANDLE
    ) -> FT_STATUS;

    pub fn FT_GetDriverVersion(
        handle: FT_HANDLE,
        version: *mut DWORD
    ) -> FT_STATUS;

    pub fn FT_GetLibraryVersion(
        version: *mut DWORD
    ) -> FT_STATUS;

    pub fn FT_Rescan() -> FT_STATUS;

    pub fn FT_Reload(
        vid: WORD,
        pid: WORD
    ) -> FT_STATUS;

    pub fn FT_GetComPortNumber(
        handle: FT_HANDLE,
        com_port_number: *mut c_long
    ) -> FT_STATUS;

    pub fn FT_EE_ReadConfig(
        handle: FT_HANDLE,
        address: c_uchar,
        value: *mut c_uchar
    ) -> FT_STATUS;

    pub fn FT_EE_WriteConfig(
        handle: FT_HANDLE,
        address: c_uchar,
        value: c_uchar
    ) -> FT_STATUS;

    pub fn FT_EE_ReadECC(
        handle: FT_HANDLE,
        options: c_uchar,
        value: *mut WORD
    ) -> FT_STATUS;

    pub fn FT_GetQueueStatusEx(
        handle: FT_HANDLE,
        rx_bytes: *mut DWORD
    ) -> FT_STATUS;

    pub fn FT_ComPortIdle(
        handle: FT_HANDLE
    ) -> FT_STATUS;

    pub fn FT_ComPortCancelIdle(
        handle: FT_HANDLE
    ) -> FT_STATUS;

    pub fn FT_VendorCmdGet(
        handle: FT_HANDLE,
        request: c_uchar,
        buf: *mut c_uchar,
        len: c_ushort
    ) -> FT_STATUS;

    pub fn FT_VendorCmdSet(
        handle: FT_HANDLE,
        request: c_uchar,
        buf: *mut c_uchar,
        len: c_ushort
    ) -> FT_STATUS;

    pub fn FT_VendorCmdGetEx(
        handle: FT_HANDLE,
        value: c_ushort,
        buf: *mut c_uchar,
        len: c_ushort
    ) -> FT_STATUS;
    
    pub fn FT_VendorCmdSetEx(
        handle: FT_HANDLE,
        value: c_ushort,
        buf: *mut c_uchar,
        len: c_ushort
    ) -> FT_STATUS;

    pub fn FT_GetDeviceInfo(
        handle: FT_HANDLE,
        device: *mut FT_DEVICE,
        id: *mut DWORD,
        serial_number: *mut c_char,
        description: *mut c_char,
        dummy: *mut c_void
    ) -> FT_STATUS;

    pub fn FT_StopInTask(
        handle: FT_HANDLE
    ) -> FT_STATUS;

    pub fn FT_RestartInTask(
        handle: FT_HANDLE
    ) -> FT_STATUS;

    pub fn FT_SetResetPipeRetryCount(
        handle: FT_HANDLE,
        count: DWORD
    ) -> FT_STATUS;

    pub fn FT_ResetPort(
        handle: FT_HANDLE
    ) -> FT_STATUS;

    pub fn FT_CyclePort(
        handle: FT_HANDLE
    ) -> FT_STATUS;
}