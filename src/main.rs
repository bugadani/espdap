#![no_std]
#![no_main]

use bitbang_dap::{BitbangAdapter, DelayCycles, InputOutputPin};
use dap_rs::{
    dap::{Dap, DapLeds, DapVersion, DelayNs},
    jtag::TapConfig,
    swo::Swo,
};
use defmt::{info, todo, unwrap, warn};
#[cfg(feature = "defmt-rtt")]
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
    clock::CpuClock,
    delay::Delay,
    gpio::{Flex, InputPin, OutputPin, Pull},
    otg_fs::{
        asynch::{Config, Driver},
        Usb,
    },
    timer::timg::TimerGroup,
};
#[cfg(feature = "esp-println")]
use esp_println as _;
use static_cell::{ConstStaticCell, StaticCell};

type MyDriver = Driver<'static>;

struct BitDelay;

impl DelayNs for BitDelay {
    fn delay_ns(&mut self, ns: u32) {
        Delay::new().delay_ns(ns)
    }
}

impl DelayCycles for BitDelay {
    fn delay_cycles(&mut self, cycles: u32) {
        xtensa_lx::timer::delay(cycles)
    }

    fn cpu_clock(&self) -> u32 {
        240_000_000
    }
}

#[embassy_executor::task]
async fn usb_task(mut device: UsbDevice<'static, MyDriver>) {
    device.run().await;
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) -> () {
    info!("Init!");
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

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
    static SCAN_CHAIN: ConstStaticCell<[TapConfig; 8]> = ConstStaticCell::new([TapConfig::INIT; 8]);
    let deps = BitbangAdapter::new(
        IoPin::new(t_nrst),
        IoPin::new(t_jtdi),
        IoPin::new(t_jtms_swdio),
        IoPin::new(t_jtck_swclk),
        IoPin::new(t_jtdo),
        BitDelay,
        SCAN_CHAIN.take(),
    );

    let mut dap = Dap::new(deps, Leds, Delay::new(), None::<NoSwo>, "Embassy CMSIS-DAP");

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

struct IoPin<'a> {
    pin: Flex<'a>,
}

impl<'a> IoPin<'a> {
    fn new(pin: impl InputPin + OutputPin + 'a) -> Self {
        Self {
            pin: Flex::new(pin),
        }
    }
}

impl InputOutputPin for IoPin<'_> {
    fn set_as_output(&mut self) {
        self.pin.set_as_output();
    }

    fn set_high(&mut self, high: bool) {
        match high {
            true => self.pin.set_high(),
            false => self.pin.set_low(),
        }
    }

    fn set_as_input(&mut self) {
        self.pin.set_as_input(Pull::None);
    }

    fn is_high(&mut self) -> bool {
        self.pin.is_high()
    }
}

struct Leds;

impl DapLeds for Leds {
    fn react_to_host_status(&mut self, host_status: dap_rs::dap::HostStatus) {
        info!("Host status: {:?}", host_status);
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
