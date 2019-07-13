// Copyright 2018-2019 First Rust Competition Developers.
// Licensed under the Apache License, Version 2.0 <LICENSE-APACHE or
// http://www.apache.org/licenses/LICENSE-2.0> or the MIT license
// <LICENSE-MIT or http://opensource.org/licenses/MIT>, at your
// option. This file may not be copied, modified, or distributed
// except according to those terms.

use wpilib_sys::usage::{instances, resource_types};
use wpilib_sys::*;

/// Check if a PWM channel is valid.
fn check_pwm_channel(channel: i32) -> bool {
    unsafe { HAL_CheckPWMChannel(channel) != 0 }
}

/// Represents the amount to multiply the minimum servo-pulse pwm period by.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum PeriodMultiplier {
    /// Don't skip pulses. PWM pulses occur every 5.005 ms
    Multiplier1x = 0,
    /// Skip every other pulse. PWM pulses occur every 10.010 ms
    Multiplier2x = 1,
    /// Skip three out of four pulses. PWM pulses occur every 20.020 ms
    Multiplier4x = 3,
}

#[derive(Debug)]
/// A PWM output pin.
pub struct Pwm {
    channel: i32,
    handle: HAL_DigitalHandle,
}
#[deprecated(since = "0.5.0", note = "renamed to Pwm")]
pub type PWM = Pwm;

impl Pwm {
    pub fn new(channel: i32) -> HalResult<Self> {
        if !check_pwm_channel(channel) {
            return Err(HalError(0));
        }

        let handle = hal_call!(HAL_InitializePWMPort(HAL_GetPort(channel)))?;
        let mut pwm = Pwm { channel, handle };

        pwm.set_disabled()?;
        pwm.enable_deadband_elimination(false)?;

        usage::report(resource_types::PWM, channel as instances::Type);

        Ok(pwm)
    }

    /// Set the PWM value directly to the hardware.
    pub fn set_raw(&mut self, value: i32) -> HalResult<()> {
        hal_call!(HAL_SetPWMRaw(self.handle, value))
    }

    /// Get the PWM value directly from the hardware.
    pub fn raw(&self) -> HalResult<i32> {
        hal_call!(HAL_GetPWMRaw(self.handle))
    }

    /// Set the PWM value based on a position. `pos` must be between 0 and 1.
    pub fn set_position(&mut self, pos: f64) -> HalResult<()> {
        hal_call!(HAL_SetPWMPosition(self.handle, pos))
    }

    /// Get the PWM value in terms of a position.
    pub fn position(&self) -> HalResult<f64> {
        hal_call!(HAL_GetPWMPosition(self.handle))
    }

    /// Set the PWM value based on a speed between -1 and 1.
    pub fn set_speed(&mut self, speed: f64) -> HalResult<()> {
        hal_call!(HAL_SetPWMSpeed(self.handle, speed))
    }

    /// Get the PWM value in terms of speed.
    pub fn speed(&self) -> HalResult<f64> {
        hal_call!(HAL_GetPWMSpeed(self.handle))
    }

    /// Temporarily disables the PWM output. The next set call will re-enable.
    pub fn set_disabled(&mut self) -> HalResult<()> {
        hal_call!(HAL_SetPWMDisabled(self.handle))
    }

    /// Slow down the PWM signal for old devices.
    pub fn set_period_multiplier(&mut self, mult: PeriodMultiplier) -> HalResult<()> {
        hal_call!(HAL_SetPWMPeriodScale(self.handle, mult as i32))
    }

    /// Honestly, I have no idea what this does and it isn't documented in wpilib.
    pub fn set_zero_latch(&mut self) -> HalResult<()> {
        hal_call!(HAL_LatchPWMZero(self.handle))
    }

    /// Optionally eliminate the deadband from a speed controller.
    pub fn enable_deadband_elimination(&mut self, eliminate_deadband: bool) -> HalResult<()> {
        hal_call!(HAL_SetPWMEliminateDeadband(
            self.handle,
            eliminate_deadband as i32
        ))
    }

    /// Set the bounds on the PWM pulse widths. This sets the bounds on the PWM values for a
    /// particular type of controller. The values determine the upper and lower speeds as well as
    /// the deadband bracket.
    pub fn set_bounds(
        &mut self,
        max: f64,
        deadband_max: f64,
        center: f64,
        deadband_min: f64,
        min: f64,
    ) -> HalResult<()> {
        hal_call!(HAL_SetPWMConfig(
            self.handle,
            max,
            deadband_max,
            center,
            deadband_min,
            min
        ))
    }

    /// Set the bounds on the PWM values. This sets the bounds on the PWM values for a particular
    /// each type of controller. The values determine the upper and lower speeds as well as the
    /// deadband bracket.
    pub fn set_raw_bounds(
        &mut self,
        max: i32,
        deadband_max: i32,
        center: i32,
        deadband_min: i32,
        min: i32,
    ) -> HalResult<()> {
        hal_call!(HAL_SetPWMConfigRaw(
            self.handle,
            max,
            deadband_max,
            center,
            deadband_min,
            min
        ))
    }

    /// Get the bounds on the PWM values. This Gets the bounds on the PWM values for a particular
    /// each type of controller. The values determine the upper and lower speeds as well as the
    /// deadband bracket.
    pub fn raw_bounds(
        &self,
        max: &mut i32,
        deadband_max: &mut i32,
        center: &mut i32,
        deadband_min: &mut i32,
        min: &mut i32,
    ) -> HalResult<()> {
        hal_call!(HAL_GetPWMConfigRaw(
            self.handle,
            max,
            deadband_max,
            center,
            deadband_min,
            min
        ))
    }

    /// Get the channel of this device.
    pub fn channel(&self) -> i32 {
        self.channel
    }
}

impl Drop for Pwm {
    fn drop(&mut self) {
        hal_call!(HAL_SetPWMDisabled(self.handle)).ok();
        hal_call!(HAL_FreePWMPort(self.handle)).ok();
    }
}

/// Initialises a PWM for use as a speed controller.
/// Assumes the bounds have been set.
fn init_speed_controller(pwm: &mut Pwm) -> HalResult<()> {
    pwm.set_period_multiplier(PeriodMultiplier::Multiplier1x)?;
    pwm.set_speed(0.0)?;
    pwm.set_zero_latch()
}

#[derive(Debug)]
/// A PWM motor controller.
pub struct PwmSpeedController {
    pwm: Pwm,
    inverted: bool,
}

impl PwmSpeedController {
    /// Creates a PwmSpeedController from a Pwm without configuring it.
    ///
    /// This is useful if you need to calibrate the PWM bounds.
    pub fn new(pwm: Pwm) -> Self {
        PwmSpeedController {
            pwm,
            inverted: false,
        }
    }

    /// Creates a new PWM Talon SRX, Victor SPX, Victor SP, or DMC 60.
    fn new_common(channel: i32, resource: usage::resource_types::Type) -> HalResult<Self> {
        let mut pwm = Pwm::new(channel)?;

        /*
         * Note that the above use the following bounds for PWM values. These
         * values should work reasonably well for most controllers, but if users
         * experience issues such as asymmetric behavior around the deadband or
         * inability to saturate the controller in either direction, calibration
         * is recommended. Refer to the manufacturer's user manual for details.
         *
         *   2.004ms = full "forward"
         *   1.52ms = the "high end" of the deadband range
         *   1.50ms = center of the deadband range (off)
         *   1.48ms = the "low end" of the deadband range
         *   0.997ms = full "reverse"
         */
        pwm.set_bounds(2.004, 1.52, 1.5, 1.48, 0.997)?;

        init_speed_controller(&mut pwm)?;
        usage::report(resource, channel as _);
        Ok(Self::new(pwm))
    }

    /// Creates a Digilent DMC 60.
    pub fn new_dmc60(channel: i32) -> HalResult<Self> {
        Self::new_common(channel, usage::resource_types::DigilentDMC60)
    }

    /// Creates a CTRE Talon SRX over PWM.
    pub fn new_talon_srx(channel: i32) -> HalResult<Self> {
        Self::new_common(channel, usage::resource_types::PWMTalonSRX)
    }

    /// Creates a Vex Victor SP.
    pub fn new_victor_sp(channel: i32) -> HalResult<Self> {
        Self::new_common(channel, usage::resource_types::VictorSP)
    }

    /// Creates a CTRE Victor SPX over PWM.
    pub fn new_victor_spx(channel: i32) -> HalResult<Self> {
        Self::new_common(channel, usage::resource_types::PWMVictorSPX)
    }

    /// Set the PWM value. The PWM value is set using a range of -1.0 to 1.0, appropriately scaling
    /// the value for the FPGA.
    pub fn set(&mut self, speed: f64) -> HalResult<()> {
        self.pwm
            .set_speed(if self.inverted { -speed } else { speed })
    }

    /// Get the recently set value of the PWM.
    pub fn get(&self) -> HalResult<f64> {
        if self.inverted {
            Ok(-self.pwm.speed()?)
        } else {
            self.pwm.speed()
        }
    }

    /// Sets if the provided speed is inverted by default when calling set.
    pub fn set_inverted(&mut self, inverted: bool) {
        self.inverted = inverted;
    }

    /// Gets if the PWM is being inverted.
    pub fn inverted(&self) -> bool {
        self.inverted
    }

    /// Disabled the PWM until the next update.
    pub fn disable(&mut self) -> HalResult<()> {
        self.pwm.set_disabled()
    }
}

#[derive(Debug)]
/**
 * Standard hobby style servo.
 *
 * The range parameters default to the appropriate values for the Hitec HS-322HD
 * servo provided in the FIRST Kit of Parts in 2008.
 */
pub struct Servo(Pwm);

impl Servo {
    pub const MAX_ANGLE: f64 = 180.0;
    pub const MIN_ANGLE: f64 = 0.0;
    const ANGLE_RANGE: f64 = Self::MAX_ANGLE - Self::MIN_ANGLE;

    pub const DEFAULT_MAX_PWM: f64 = 2.4;
    pub const DEFAULT_MIN_PWM: f64 = 0.6;

    /// Creates a Hitec HS-322HD servo on the specified PWM channel.
    pub fn new(channel: i32) -> HalResult<Self> {
        Self::with_bounds(channel, Self::DEFAULT_MAX_PWM, Self::DEFAULT_MIN_PWM)
    }

    fn with_bounds(channel: i32, max: f64, min: f64) -> HalResult<Self> {
        let mut pwm = Pwm::new(channel)?;

        pwm.set_bounds(max, 0.0, 0.0, 0.0, min)?;
        pwm.set_period_multiplier(PeriodMultiplier::Multiplier4x)?;

        usage::report(usage::resource_types::Servo, channel as _);
        Ok(Self(pwm))
    }

    /// Creates a Servo from a Pwm without configuring it.
    pub fn from_pwm(pwm: Pwm) -> Self {
        Self(pwm)
    }

    /// Set the servo position. Servo values range from 0.0 to 1.0.
    pub fn set(&mut self, value: f64) -> HalResult<()> {
        self.0.set_position(value)
    }

    pub fn get(&self) -> HalResult<f64> {
        self.0.position()
    }

    pub fn set_angle(&mut self, degrees: f64) -> HalResult<()> {
        self.set(
            (degrees.max(Self::MIN_ANGLE).min(Self::MAX_ANGLE) - Self::MIN_ANGLE)
                / Self::ANGLE_RANGE,
        )
    }

    pub fn angle(&self) -> HalResult<f64> {
        Ok(self.get()? * Self::ANGLE_RANGE + Self::MIN_ANGLE)
    }
}
