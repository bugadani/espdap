#![no_std]
#![no_main]

use bitbang_dap::{BitbangAdapter, DelayCycles, InputOutputPin};
use dap_rs::{
    dap::{Dap, DapLeds, DapVersion, DelayNs},
    jtag::TapConfig,
    swo::Swo,
};
use defmt::{info, todo, warn};
#[cfg(feature = "defmt-rtt")]
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_usb::{
    class::cdc_acm::{CdcAcmClass, State},
    msos::windows_version,
    Builder,
};
use esp_backtrace as _;
use esp_hal::{
    clock::{Clock, CpuClock},
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

use crate::usb_driver::{CmsisDapV2Class, CmsisDapV2State};

mod usb_driver;

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
        CpuClock::max().frequency().as_hz()
    }
}

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) -> () {
    info!("Init!");
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    // Configuration options:
    // 1. Pinout
    let t_nrst = peripherals.GPIO15;
    let t_jtms_swdio = peripherals.GPIO16;
    let t_jtck_swclk = peripherals.GPIO17;
    let t_jtdi = peripherals.GPIO18;
    let t_jtdo = peripherals.GPIO8;
    //let t_nrst = peripherals.GPIO4;
    //let t_swo = peripherals.GPIO5;

    // 2. Max JTAG scan chain
    const MAX_SCAN_CHAIN_LENGTH: usize = 8;

    // 3. USB configuration
    const MANUFACTURER: &str = "me";
    const PRODUCT: &str = "ESP32 Probe CMSIS-DAP";

    let usb = Usb::new(peripherals.USB0, peripherals.GPIO20, peripherals.GPIO19);

    // Create the driver, from the HAL.
    static STATIC_EP_OUT_BUFFER: ConstStaticCell<[u8; 1024]> = ConstStaticCell::new([0u8; 1024]);
    let driver = Driver::new(usb, STATIC_EP_OUT_BUFFER.take(), Config::default());

    // Create embassy-usb Config
    let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some(MANUFACTURER);
    config.product = Some(PRODUCT);
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
    static DAP_STATE: ConstStaticCell<CmsisDapV2State> =
        ConstStaticCell::new(CmsisDapV2State::new());
    let mut dap_class = CmsisDapV2Class::new(&mut builder, DAP_STATE.take(), 64);

    // CDC - dummy class to get things working for now. Windows needs more than one interface
    // to load usbccgp.sys, which is necessary for nusb to be able to list interfaces.
    static CDC_STATE: ConstStaticCell<State> = ConstStaticCell::new(State::new());
    _ = CdcAcmClass::new(&mut builder, CDC_STATE.take(), 64);

    // Build the builder.
    let mut usb = builder.build();

    // Run the USB device.
    let usb_fut = usb.run();

    // Process DAP commands in a loop.
    static SCAN_CHAIN: ConstStaticCell<[TapConfig; MAX_SCAN_CHAIN_LENGTH]> =
        ConstStaticCell::new([TapConfig::INIT; MAX_SCAN_CHAIN_LENGTH]);
    let deps = BitbangAdapter::new(
        IoPin::new(t_nrst),
        IoPin::new(t_jtdi),
        IoPin::new(t_jtms_swdio),
        IoPin::new(t_jtck_swclk),
        IoPin::new(t_jtdo),
        BitDelay,
        SCAN_CHAIN.take(),
    );

    let mut dap = Dap::new(
        deps,
        Leds,
        Delay::new(),
        None::<NoSwo>,
        concat!("2.1.0, Adaptor version ", env!("CARGO_PKG_VERSION")),
    );

    let dap_fut = async {
        let mut req = [0u8; 1024];
        let mut resp = [0u8; 1024];
        loop {
            dap_class.wait_connection().await;

            let Ok(req_len) = dap_class.read_packet(&mut req).await.inspect_err(|e| {
                warn!("failed to read from USB: {:?}", e);
            }) else {
                continue;
            };

            let resp_len = dap.process_command(&req[..req_len], &mut resp, DapVersion::V2);

            if let Err(e) = dap_class.write_packet(&resp[..resp_len]).await {
                warn!("failed to write to USB: {:?}", e);
                continue;
            }
        }
    };

    // Run everything concurrently.
    // If we had made everything `'static` above instead, we could do this using separate tasks instead.
    join(usb_fut, dap_fut).await;
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
