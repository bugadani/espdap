#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use dap_rs::{
    dap::{Dap, DapLeds, DapVersion},
    swd::{self, APnDP, DPRegister},
    swj::{Dependencies, Pins},
    swo::Swo,
};
use defmt::{info, todo, unwrap, warn};
#[cfg(feature = "esp32s3")]
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_usb::{
    class::cdc_acm::{CdcAcmClass, State},
    driver::{Endpoint, EndpointError, EndpointIn, EndpointOut},
    msos::{self, windows_version},
    types::StringIndex,
    Builder, Handler, UsbDevice,
};
use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    gpio::{Flex, GpioPin, Level, Pull},
    otg_fs::{
        asynch::{Config, Driver},
        Usb,
    },
    prelude::*,
    timer::systimer::SystemTimer,
};
#[cfg(feature = "esp32s2")]
use esp_println as _;
use static_cell::StaticCell;

type MyDriver = Driver<'static>;

// Pin configuration
type NresetPin = GpioPin<15>;
type TmsSwdioPin = GpioPin<16>;
type TckSwclkPin = GpioPin<17>;
type TdiPin = GpioPin<18>;
type TdoPin = GpioPin<8>;

#[embassy_executor::task]
async fn usb_task(mut device: UsbDevice<'static, MyDriver>) {
    device.run().await;
}

struct OldDelay;

impl embedded_hal_02::blocking::delay::DelayUs<u32> for OldDelay {
    fn delay_us(&mut self, us: u32) {
        Delay::new().delay_micros(us);
    }
}

#[main]
async fn main(spawner: Spawner) -> () {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    info!("Init!");

    esp_hal_embassy::init(SystemTimer::new(peripherals.SYSTIMER).alarm0);

    // Pinout
    let t_nrst = peripherals.GPIO15;
    let t_jtms_swdio = peripherals.GPIO16;
    let t_jtck_swclk = peripherals.GPIO17;
    let t_jtdi = peripherals.GPIO18;
    let t_jtdo = peripherals.GPIO8;
    //let t_nrst = peripherals.GPIO4;
    //let t_swo = peripherals.GPIO5;

    let usb = Usb::new(peripherals.USB0, peripherals.GPIO20, peripherals.GPIO19);

    // Create the driver, from the HAL.
    static STATIC_EP_OUT_BUFFER: StaticCell<[u8; 1024]> = StaticCell::new();
    let ep_out_buffer = STATIC_EP_OUT_BUFFER.init([0u8; 1024]);
    let config = Config::default();
    let driver = Driver::new(usb, ep_out_buffer, config);

    // Create embassy-usb Config.
    let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("me");
    config.product = Some("ESP32 Probe CMSIS-DAP");
    config.serial_number = Some("12345678");
    config.device_class = 0xEF;
    config.device_sub_class = 0x02;
    config.device_protocol = 0x01;
    config.composite_with_iads = true;

    // Create embassy-usb DeviceBuilder using the driver and config.
    static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static MSOS_DESC: StaticCell<[u8; 196]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 128]> = StaticCell::new();
    let mut builder = Builder::new(
        driver,
        config,
        &mut CONFIG_DESC.init([0; 256])[..],
        &mut BOS_DESC.init([0; 256])[..],
        &mut MSOS_DESC.init([0; 196])[..],
        &mut CONTROL_BUF.init([0; 128])[..],
    );

    builder.msos_descriptor(windows_version::WIN8_1, 0);

    // DAP - Custom Class 0
    let iface_string = builder.string();
    let mut function = builder.function(0xFF, 0, 0);
    function.msos_feature(msos::CompatibleIdFeatureDescriptor::new("WINUSB", ""));
    function.msos_feature(msos::RegistryPropertyFeatureDescriptor::new(
        "DeviceInterfaceGUIDs",
        // CMSIS-DAP standard GUID, from https://arm-software.github.io/CMSIS_5/DAP/html/group__DAP__ConfigUSB__gr.html
        msos::PropertyData::RegMultiSz(&["{CDB3B5AD-293B-4663-AA36-1AAE46463776}"]),
    ));
    let mut interface = function.interface();
    let mut altsetting = interface.alt_setting(0xFF, 0, 0, Some(iface_string));
    let mut read_ep = altsetting.endpoint_bulk_out(64);
    let mut write_ep = altsetting.endpoint_bulk_in(64);
    drop(function);

    static CONTROL_HANDLER: StaticCell<Control> = StaticCell::new();
    builder.handler(CONTROL_HANDLER.init(Control { iface_string }));

    // CDC - dummy class to get things working for now. Windows needs more than one interface
    // to load usbccgp.sys, which is necessary for nusb to be able to list interfaces.
    static STATIC_STATE: StaticCell<State> = StaticCell::new();
    let state = STATIC_STATE.init(State::new());
    _ = CdcAcmClass::new(&mut builder, state, 64);

    // Start USB.
    let usb = builder.build();
    unwrap!(spawner.spawn(usb_task(usb)));

    // Process DAP commands in a loop.
    let deps = Deps::new(
        t_nrst,
        t_jtdi,
        t_jtms_swdio,
        t_jtck_swclk,
        t_jtdo,
        Delay::new(),
    );

    let mut dap = Dap::new(deps, Leds, OldDelay, None::<NoSwo>, "Embassy CMSIS-DAP");

    let mut req = [0u8; 1024];
    let mut resp = [0u8; 1024];
    loop {
        read_ep.wait_enabled().await;

        let req_len = match read_packet(&mut read_ep, &mut req).await {
            Ok(n) => n,
            Err(e) => {
                warn!("failed to read from USB: {:?}", e);
                continue;
            }
        };
        let resp_len = dap.process_command(&req[..req_len], &mut resp, DapVersion::V2);

        if let Err(e) = write_packet(&mut write_ep, &resp[..resp_len]).await {
            warn!("failed to write to USB: {:?}", e);
            continue;
        }
    }
}

async fn read_packet(ep: &mut impl EndpointOut, buf: &mut [u8]) -> Result<usize, EndpointError> {
    let mut n = 0;

    loop {
        let i = ep.read(&mut buf[n..]).await?;
        n += i;
        if i < 64 {
            return Ok(n);
        }
    }
}

async fn write_packet(ep: &mut impl EndpointIn, buf: &[u8]) -> Result<(), EndpointError> {
    for chunk in buf.chunks(64) {
        ep.write(chunk).await?;
    }
    if buf.len() % 64 == 0 {
        ep.write(&[]).await?;
    }
    Ok(())
}

struct Control {
    iface_string: StringIndex,
}

impl Handler for Control {
    fn get_string(&mut self, index: StringIndex, _lang_id: u16) -> Option<&str> {
        if index == self.iface_string {
            Some("CMSIS-DAP v2 Interface")
        } else {
            warn!("unknown string index requested");
            None
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum Dir {
    Write = 0,
    Read = 1,
}

struct Deps {
    nreset: Flex<'static, NresetPin>,
    tdi: Flex<'static, TdiPin>,
    tms_swdio: Flex<'static, TmsSwdioPin>,
    tck_swclk: Flex<'static, TckSwclkPin>,
    tdo: Flex<'static, TdoPin>,
    delay: Delay,
}

impl Deps {
    pub fn new(
        nreset: NresetPin,
        tdi: TdiPin,
        tms_swdio: TmsSwdioPin,
        tck_swclk: TckSwclkPin,
        tdo: TdoPin,
        delay: Delay,
    ) -> Self {
        let mut nreset = Flex::new_typed(nreset);
        let mut tdi = Flex::new_typed(tdi);
        let mut tms_swdio = Flex::new_typed(tms_swdio);
        let mut tck_swclk = Flex::new_typed(tck_swclk);
        let mut tdo = Flex::new_typed(tdo);

        nreset.set_as_output();
        nreset.set_high();

        //io.set_pull(Pull::Up);
        tms_swdio.set_as_output();
        tms_swdio.set_high();

        //ck.set_pull(Pull::None);
        tck_swclk.set_as_output();

        tdi.set_as_input(Pull::None);
        tdo.set_as_output();

        Self {
            nreset,
            tdi,
            tms_swdio,
            tck_swclk,
            tdo,
            delay,
        }
    }

    fn req(&mut self, port: APnDP, dir: Dir, addr: DPRegister) {
        let req = (port as u32) | (dir as u32) << 1 | (addr as u32) << 2;
        let parity = req.count_ones() % 2;
        self.shift_out(0b10000001 | req << 1 | parity << 5, 8);
    }

    pub fn read(&mut self, port: APnDP, addr: DPRegister) -> swd::Result<u32> {
        self.req(port, Dir::Read, addr);

        self.shift_in(1); // turnaround

        let ack = self.shift_in(3);
        match ack {
            0b001 => {} // ok
            0b010 => {
                self.shift_in(1); // turnaround
                return Err(swd::Error::AckWait);
            }
            0b100 => {
                self.shift_in(1); // turnaround
                return Err(swd::Error::AckFault);
            }
            _ => {
                self.shift_in(1); // turnaround
                return Err(swd::Error::AckUnknown(ack as u8));
            }
        }

        let data = self.shift_in(32);
        let parity = self.shift_in(1);
        if parity != data.count_ones() % 2 {
            return Err(swd::Error::BadParity);
        }

        self.shift_in(1); // turnaround

        Ok(data)
    }

    pub fn write(&mut self, port: APnDP, addr: DPRegister, data: u32) -> swd::Result<()> {
        self.req(port, Dir::Write, addr);

        self.shift_in(1); // turnaround

        let ack = self.shift_in(3);
        self.shift_in(1); // turnaround
        match ack {
            0b001 => {} // ok
            0b010 => return Err(swd::Error::AckWait),
            0b100 => return Err(swd::Error::AckFault),
            _ => return Err(swd::Error::AckUnknown(ack as _)),
        }

        self.shift_out(data, 32);
        self.shift_out(data.count_ones() % 2, 1);

        Ok(())
    }

    fn shift_out(&mut self, val: u32, n: u32) {
        self.tms_swdio.set_as_output();
        for i in 0..n {
            self.tms_swdio.set_level(Level::from(val & (1 << i) != 0));
            self.clock_pulse();
        }
    }

    fn shift_in(&mut self, n: u32) -> u32 {
        self.tms_swdio.set_as_input(Pull::None);
        let mut val = 0;
        for i in 0..n {
            val |= (self.tms_swdio.is_high() as u32) << i;
            self.clock_pulse();
        }
        val
    }

    fn clock_pulse(&mut self) {
        wait(&mut self.delay);
        self.tck_swclk.set_high();
        wait(&mut self.delay);
        self.tck_swclk.set_low();
    }
}

fn wait(delay: &mut Delay) {
    delay.delay_nanos(500);
}

impl Dependencies<Deps, Deps> for Deps {
    fn high_impedance_mode(&mut self) {
        // todo
    }

    fn process_swj_clock(&mut self, _max_frequency: u32) -> bool {
        true // TODO
    }

    fn process_swj_pins(&mut self, output: Pins, mask: Pins, wait_us: u32) -> Pins {
        if mask.contains(Pins::SWCLK) {
            self.tck_swclk
                .set_level(Level::from(output.contains(Pins::SWCLK)));
        }
        if mask.contains(Pins::SWDIO) {
            self.tms_swdio
                .set_level(Level::from(output.contains(Pins::SWDIO)));
        }
        if mask.contains(Pins::NRESET) {
            self.nreset
                .set_level(Level::from(output.contains(Pins::NRESET)));
        }
        if mask.contains(Pins::TDO) {
            self.tdo.set_level(Level::from(output.contains(Pins::TDO)));
        }

        if wait_us != 0 {
            self.delay.delay_micros(wait_us);
        }

        let mut read = Pins::empty();

        read.set(Pins::SWCLK, self.tck_swclk.is_high());
        read.set(Pins::SWDIO, self.tms_swdio.is_high());
        read.set(Pins::NRESET, self.nreset.is_high());
        read.set(Pins::TDO, self.tdo.is_high());
        read.set(Pins::TDI, self.tdi.is_high());
        read.set(Pins::NTRST, true);

        read
    }

    fn process_swj_sequence(&mut self, data: &[u8], mut nbits: usize) {
        for &b in data {
            if nbits == 0 {
                break;
            }
            let bits = nbits.min(8);
            self.shift_out(b as u32, bits as u32);
            nbits -= bits;
        }
    }
}

impl dap_rs::swd::Swd<Deps> for Deps {
    const AVAILABLE: bool = true;

    fn read_inner(&mut self, port: APnDP, addr: DPRegister) -> swd::Result<u32> {
        self.read(port, addr)
    }

    fn write_inner(&mut self, port: APnDP, addr: DPRegister, data: u32) -> swd::Result<()> {
        self.write(port, addr, data)
    }

    fn set_clock(&mut self, _max_frequency: u32) -> bool {
        // todo
        true
    }
}

impl dap_rs::jtag::Jtag<Deps> for Deps {
    const AVAILABLE: bool = false;

    fn sequences(&mut self, _data: &[u8], _rxbuf: &mut [u8]) -> u32 {
        todo!()
    }

    fn set_clock(&mut self, _max_frequency: u32) -> bool {
        todo!()
    }
}

struct Leds;

impl DapLeds for Leds {
    fn react_to_host_status(&mut self, _host_status: dap_rs::dap::HostStatus) {
        // TODO
    }
}

struct NoSwo;

impl Swo for NoSwo {
    fn set_transport(&mut self, _transport: dap_rs::swo::SwoTransport) {
        todo!()
    }

    fn set_mode(&mut self, _mode: dap_rs::swo::SwoMode) {
        todo!()
    }

    fn set_baudrate(&mut self, _baudrate: u32) -> u32 {
        todo!()
    }

    fn set_control(&mut self, _control: dap_rs::swo::SwoControl) {
        todo!()
    }

    fn polling_data(&mut self, _buf: &mut [u8]) -> u32 {
        todo!()
    }

    fn streaming_data(&mut self) {
        todo!()
    }

    fn is_active(&self) -> bool {
        todo!()
    }

    fn bytes_available(&self) -> u32 {
        todo!()
    }

    fn buffer_size(&self) -> u32 {
        todo!()
    }

    fn support(&self) -> dap_rs::swo::SwoSupport {
        todo!()
    }

    fn status(&mut self) -> dap_rs::swo::SwoStatus {
        todo!()
    }
}
