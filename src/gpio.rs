//! Easy way to communicate with the GPIO functions.
//!
//! Wraps the `unsafe` functions.
//!
//! # TODO
//!
//! Need to clean up the Mode, Speed, Type, Output, and PuPd resistors.
//! Only uses bit flags for the pins, everything else is handled with an
//! `enum`. Allows enabling/disabling multiple pins at once.
//!
//! # Example
//!
//! ``` ignore,no_run
//! #![feature(used)]
//! #![no_std]
//!
//! extern crate cortex_m;
//! extern crate cortex_m_rt;
//! extern crate f4;
//!
//! use cortex_m::asm;
//! use f4::stm32f429x::{GPIOG, RCC};
//! use f4::gpio::{self, GPIO};
//!
//! fn main() {
//!     cortex_m::interrupt::free(
//!         |cs| {
//!             let gpiog = GPIOG.borrow(cs);
//!             let rcc = RCC.borrow(cs);
//!
//!             rcc.ahb1enr.modify(|_, w| w.gpiogen().enabled());
//!             gpiog.pin_enable(
//!                 gpio::PIN13,
//!                 gpio::Mode::Out,
//!                 gpio::Speed::Low,
//!                 gpio::Output::PushPull,
//!                 gpio::PuPd::NoPull
//!             );
//!
//!             let count = 13 * 5000;
//!
//!             loop {
//!                 if gpiog.odr.read().odr13().bits() != 0u8 {
//!                     gpiog.pin_reset(gpio::PIN13);
//!                 } else {
//!                     gpiog.pin_set(gpio::PIN13);
//!                 }
//!                 delay(count);
//!             }
//!         },
//!     );
//! }
//!
//! fn delay(count: u32) {
//!     for _ in 0..count { cortex_m::asm::nop() }
//! }
//!
//! #[allow(dead_code)]
//! #[link_section = ".rodata.interrupts"]
//! #[used]
//! static INTERRUPTS: [extern "C" fn(); 240] = [default_handler; 240];
//!
//! extern "C" fn default_handler() {
//!     asm::bkpt();
//! }
//! ```

use stm32f429x::{Gpioa, Gpiob, Gpioc, Gpiod, Gpioe, Gpiof, Gpiog, Gpioh, Gpioi, Gpioj, Gpiok};
use cast::u32;

bitflags! {
    /// Bit value assignment for each of the pins
    ///
    /// Allows for using multiple pins through bitwise or
    pub flags Pins: u16 {
        const PIN0  = 0x0001,
        const PIN1  = 0x0002,
        const PIN2  = 0x0004,
        const PIN3  = 0x0008,
        const PIN4  = 0x0010,
        const PIN5  = 0x0020,
        const PIN6  = 0x0040,
        const PIN7  = 0x0080,
        const PIN8  = 0x0100,
        const PIN9  = 0x0200,
        const PIN10 = 0x0400,
        const PIN11 = 0x0800,
        const PIN12 = 0x1000,
        const PIN13 = 0x2000,
        const PIN14 = 0x4000,
        const PIN15 = 0x8000,
    }
}

/// Output mode for the pin
#[derive(Copy, Clone)]
pub enum Mode {
    In = 0x00,
    Out = 0x01,
    AltFunction = 0x02,
    Analog = 0x03,
}

/// Output speed for the GPIO pins
#[derive(Copy, Clone)]
pub enum Speed {
    /// Legacy 2MHz
    Low = 0x00,
    /// Legacy 25MHz
    Medium = 0x01,
    /// Legacy 50MHz
    Fast = 0x02,
    /// Legacy 100MHz
    High = 0x03,
}

/// Hardware output configuration
#[derive(Copy, Clone)]
pub enum Output {
    PushPull = 0x00,
    OpenDrain = 0x01,
}

/// Resistors configured for the pin
#[derive(Copy, Clone)]
pub enum PuPd {
    NoPull = 0x00,
    Up = 0x01,
    Down = 0x02,
}

/// GPIO trait
///
/// Allows the setting of specific pins, as well as determining the alternate
/// functions for each of the pins.
pub trait GPIO {
    /// Set a specific pin up properly.
    ///
    /// Expects the GPIO to be enabled on the clock separately.
    fn pin_enable(
        &self,
        pin: Pins,
        mode: Mode,
        speed: Speed,
        output: Output,
        pupd: PuPd
    ) -> &Self;

    /// Lock the pin configuration for the GPIO
    fn pin_lock(&self, pin: Pins) -> &Self;

    /// Set the pin
    fn pin_set(&self, pin: Pins) -> &Self;

    /// Reset the pin
    fn pin_reset(&self, pin: Pins) -> &Self;
}

macro_rules! impl_gpio {
    ($gpio:ty) => {
        impl GPIO for $gpio {
            fn pin_enable(
                &self,
                pin: Pins,
                mode: Mode,
                speed: Speed,
                output: Output,
                pupd: PuPd
            ) -> &Self {
                for pinpos in 0..16 {
                    let pos: u16 = 0x01 << pinpos;
                    if pos & pin.bits() != 0 {
                        self.moder.write(|w| unsafe {
                            w.bits(!(0b11 << (pinpos * 2)))
                                .bits((mode as u32) << (pinpos * 2))
                        });

                        if mode as u32 & (Mode::Out as u32 | Mode::AltFunction as u32) != 0 {
                            self.ospeedr.write(|w| unsafe {
                                w.bits(!(0b11 << (pinpos * 2)))
                                    .bits((speed as u32) << (pinpos * 2))
                            });
                            self.otyper.write(|w| unsafe {
                                w.bits(!(0b1 << pinpos))
                                    .bits((output as u32) << pinpos)
                            });
                        }

                        self.pupdr.write(|w| unsafe {
                            w.bits(!(0b11 << (pinpos * 2)))
                                .bits((pupd as u32) << (pinpos * 2))
                        });
                    }
                }

                self
            }

            fn pin_lock(&self, pin: Pins) -> &Self {
                let tmp: u32 = 0x00010000 | u32(pin.bits());
                self.lckr.write(|w| unsafe { w.bits(tmp) });
                self.lckr.write(|w| unsafe { w.bits(u32(pin.bits())) });
                self.lckr.write(|w| unsafe { w.bits(tmp) });
                self.lckr.read().bits();
                self.lckr.read().bits();
                self
            }

            fn pin_set(&self, pin: Pins) -> &Self {
                self.bsrr.write(|w| unsafe { w.bits(u32(pin.bits())) });
                self
            }

            fn pin_reset(&self, pin: Pins) -> &Self {
                self.bsrr.write(|w| unsafe { w.bits(u32(pin.bits()) << 16) });
                self
            }
        }
    }
}

impl_gpio!(Gpioa);
impl_gpio!(Gpiob);
impl_gpio!(Gpioc);
impl_gpio!(Gpiod);
impl_gpio!(Gpioe);
impl_gpio!(Gpiof);
impl_gpio!(Gpiog);
impl_gpio!(Gpioh);
impl_gpio!(Gpioi);
impl_gpio!(Gpioj);
impl_gpio!(Gpiok);

