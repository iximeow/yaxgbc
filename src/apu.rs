
/// cpu hz / audio hz. comes out to 95, actual value is 95.01133786... we'll see what 0.01% does in
/// practice..
const CLOCKS_PER_SAMPLE: u64 = 4_190_000 / 44100;

pub struct Apu {
    apu_active: bool,
    rendered_sound: Vec<f32>,
    clocks_since_length_tick: u64,
    /// the number of 256-sample buffers enqueued for playing, last we checked. the deeper this
    /// buffer, the more the cpu is running ahead of real time. if this buffer fully drains,
    /// though, we're running too slow and audio will be choppy.
    pub(crate) audio_buffer_depth: u64,
    /// the number of clocks since the last sample that was rendered out.
    ///
    /// at a clock speed of 4.19mhz and a 44100hz output, that leaves about 95 clocks per sample.
    /// every 256 samples, we flush rendered_sound out to the audio sink (if one exists), meaning
    /// the buffering delay introduced here is at worst 0.58ms (1 / 44.1khz * 256).
    sample_clock: u64,
    nr50: u8,
    nr51: u8,
    channel_active: [bool; 4],
    channel_1_dac_enable: bool,
    channel_1_clocks_since_sample: u64,
    channel_1_clocks_since_envelope_tick: u64,
    channel_1_clocks_since_sweep_tick: u64,
    channel_1_length_timer: u8,
    channel_1_period_sweep_pace: u8,
    channel_1_period_sweep_direction: bool,
    channel_1_period_step: u8,
    channel_1_wave_duty: u8,
    channel_1_initial_length: u8,
    channel_1_initial_volume: u8,
    channel_1_index: u8,
    channel_1_volume: u8,
    channel_1_envelope_direction: bool,
    channel_1_sweep_pace: u8,
    channel_1_wavelength: u16,
    channel_1_length_enable: bool,
    channel_1_trigger: bool,
    channel_2_dac_enable: bool,
    channel_2_clocks_since_sample: u64,
    channel_2_clocks_since_envelope_tick: u64,
    channel_2_length_timer: u8,
    channel_2_pace: i8,
    channel_2_wave_duty: u8,
    channel_2_initial_length: u8,
    channel_2_initial_volume: u8,
    channel_2_index: u8,
    channel_2_volume: u8,
    channel_2_envelope_direction: bool,
    channel_2_sweep_pace: u8,
    /// this is also called "period value" in the pan docs
    channel_2_wavelength: u16,
    channel_2_length_enable: bool,
    channel_2_trigger: bool,
    channel_3_dac_enable: bool,
    channel_3_index: u8,
    channel_3_clocks_since_sample: u64,
    channel_3_length_timer: u8,
    channel_3_output_level: u8,
    channel_3_wavelength: u16,
    channel_3_trigger: bool,
    channel_3_length_enable: bool,
    channel_3_wave_ram: [u8; 16],
    channel_4_dac_enable: bool,
    channel_4_clocks_since_sample: u64,
    channel_4_clocks_since_envelope_tick: u64,
    channel_4_last_sample: bool,
    channel_4_length_timer: u8,
    channel_4_length_enable: bool,
    channel_4_initial_volume: u8,
    channel_4_volume: u8,
    channel_4_envelope_direction: bool,
    channel_4_sweep_pace: u8,
    channel_4_clock_shift: u8,
    channel_4_lfsr_width: u8,
    channel_4_clock_divider: u8,
    channel_4_wavelength: u16,
    channel_4_trigger: bool,
}

impl Apu {
    pub fn store(&mut self, address: usize, value: u8) {
        match address {
            crate::NR10 => {
                let period_sweep_pace = (value & 0x70) >> 4;
                let period_sweep_direction = (value & 0x08) >> 3;
                let period_step = (value & 0x07);
                self.channel_1_period_sweep_pace = period_sweep_pace;
                self.channel_1_period_sweep_direction = period_sweep_direction != 0;
                self.channel_1_period_step = period_step;
            }
            crate::NR11 => {
                let wave_duty = (value & 0xc0) >> 6;
                let initial_length_timer = value & 0x3f;
                self.channel_1_wave_duty = wave_duty;
                self.channel_1_initial_length = initial_length_timer;
            },
            crate::NR12 => {
                let initial_volume = (value & 0xf0) >> 4;
                let envelope_direction = ((value & 0x08) >> 3) != 0;
                let sweep_pace = value & 0x07;
                self.channel_1_initial_volume = initial_volume;
                self.channel_1_envelope_direction = envelope_direction;
                self.channel_1_sweep_pace = sweep_pace;
            }
            crate::NR13 => {
                self.channel_1_wavelength =
                    (self.channel_1_wavelength & 0xff00) |
                    (value as u16);
            }
            crate::NR14 => {
                let trigger = (value & 0x80) != 0;
                let sound_length_enable = (value & 0x40) != 0;
                let wavelength_hi = (value & 0x07) as u16;

                self.channel_1_length_enable = sound_length_enable;
                self.channel_1_wavelength =
                    (self.channel_1_wavelength & 0x00ff) |
                    (wavelength_hi << 8);

                self.channel_1_trigger = trigger;
                if trigger {
                    self.channel_1_dac_enable = true;
                    self.channel_1_clocks_since_envelope_tick = 0;
                    self.channel_1_clocks_since_sweep_tick = 0;
                    self.channel_1_clocks_since_sample = 0;
//                    eprintln!("channel 1 activated, vol={}, wavelength={:03x}", self.channel_1_initial_volume, self.channel_1_wavelength);
                    self.channel_1_volume = self.channel_1_initial_volume;
                    self.channel_1_length_timer = self.channel_1_initial_length;
                }
            }
            0x115 => {
                // not used? not documented at least
            }
            0x11f => {
                // not used? not documented at least
            }
            crate::NR21 => {
                let wave_duty = (value & 0xc0) >> 6;
                let initial_length_timer = value & 0x3f;
                self.channel_2_wave_duty = wave_duty;
                self.channel_2_initial_length = initial_length_timer;
            },
            crate::NR22 => {
                let initial_volume = (value & 0xf0) >> 4;
                let envelope_direction = ((value & 0x08) >> 3) != 0;
                let sweep_pace = value & 0x07;
                self.channel_2_initial_volume = initial_volume;
                self.channel_2_envelope_direction = envelope_direction;
                self.channel_2_sweep_pace = sweep_pace;
            }
            crate::NR23 => {
                self.channel_2_wavelength =
                    (self.channel_2_wavelength & 0xff00) |
                    (value as u16);
            }
            crate::NR24 => {
                let trigger = (value & 0x80) != 0;
                let sound_length_enable = (value & 0x40) != 0;
                let wavelength_hi = (value & 0x07) as u16;

                self.channel_2_length_enable = sound_length_enable;
                self.channel_2_wavelength =
                    (self.channel_2_wavelength & 0x00ff) |
                    (wavelength_hi << 8);

                self.channel_2_trigger = trigger;
                if trigger {
                    self.channel_2_dac_enable = true;
                    self.channel_2_clocks_since_envelope_tick = 0;
                    self.channel_2_clocks_since_sample = 0;
//                    eprintln!("channel 2 activated, vol={}, wavelength={:03x}", self.channel_2_initial_volume, self.channel_2_wavelength);
                    self.channel_2_volume = self.channel_2_initial_volume;
                    self.channel_2_length_timer = self.channel_2_initial_length;
                }
            }
            crate::NR30 => {
                self.channel_3_dac_enable = (value & 0x80) != 0;
            },
            crate::NR31 => {
                self.channel_3_length_timer = value;
            }
            crate::NR32 => {
                let output_level = (value & 0x60) >> 5;
                self.channel_3_output_level = output_level;
            }
            crate::NR33 => {
                // TODO: pan docs says that channel 3 wavelength changes take effect the next time
                // wave RAM is read
                self.channel_3_wavelength =
                    (self.channel_3_wavelength & 0xff00) |
                    (value as u16);
            }
            crate::NR34 => {
                let trigger = (value & 0x80) != 0;
                let sound_length_enable = (value & 0x40) != 0;
                let wavelength_hi = (value & 0x07) as u16;

                self.channel_3_trigger = trigger;
                if trigger {
                    self.channel_3_dac_enable = true;
                    self.channel_3_clocks_since_sample = 0;
                }
                self.channel_3_length_enable = sound_length_enable;
                self.channel_3_wavelength =
                    (self.channel_3_wavelength & 0x00ff) |
                    (wavelength_hi << 8);
            }
            crate::NR41 => {
                self.channel_4_length_timer = value & 0x3f;
            }
            crate::NR42 => {
                let initial_volume = (value & 0xf0) >> 4;
                let envelope_direction = ((value & 0x08) >> 3) != 0;
                let sweep_pace = value & 0x07;
                self.channel_4_initial_volume = initial_volume;
                self.channel_4_envelope_direction = envelope_direction;
                self.channel_4_sweep_pace = sweep_pace;
            }
            crate::NR43 => {
                self.channel_4_clock_shift = (value >> 4);
                self.channel_4_lfsr_width = if (value >> 3) & 1 == 0 {
                    15
                } else {
                    7
                };
                self.channel_4_clock_divider = value & 0x07;
            }
            crate::NR44 => {
                let trigger = (value & 0x80) != 0;
                let sound_length_enable = (value & 0x40) != 0;

                self.channel_4_trigger = trigger;
                if trigger {
                    self.channel_4_dac_enable = true;
                    self.channel_4_clocks_since_envelope_tick = 0;
                    self.channel_4_clocks_since_sample = 0;
                }
                self.channel_4_volume = self.channel_4_initial_volume;
                self.channel_4_length_enable = sound_length_enable;
            }
            crate::NR50 => {
//                eprintln!("support master volume...");
                self.nr50 = value;
            }
            crate::NR51 => {
//                eprintln!("support sound panning...");
                self.nr51 = value;
            }
            crate::NR52 => {
                self.apu_active = (value & 0x80 != 0) as bool;
            }
            0x130..=0x13f => {
                self.channel_3_wave_ram[address - 0x130] = value;
            }
            _ => {
                panic!("store unhandled register {:04x} (set to {:02x})", address, value);
            }
        }
    }
    pub fn load(&self, address: usize) -> u8 {
        match address {
            crate::NR30 => {
                if self.channel_3_dac_enable {
                    0x80
                } else {
                    0x00
                }
            },
            crate::NR50 => {
                self.nr50
            }
            crate::NR51 => {
                self.nr51
            }
            crate::NR52 => {
                (self.apu_active as u8) << 7 |
                (self.channel_4_dac_enable as u8) << 3 |
                (self.channel_3_dac_enable as u8) << 2 |
                (self.channel_2_dac_enable as u8) << 1 |
                (self.channel_1_dac_enable as u8) << 0
            },
            _ => {
                0
                // panic!("load unhandled register {:04x}", address);
            }
        }
    }
/*
const NR10: usize = 0x110;
const NR11: usize = 0x111;
const NR12: usize = 0x112;
const NR13: usize = 0x113;
const NR14: usize = 0x114;
const NR21: usize = 0x116;
const NR22: usize = 0x117;
const NR23: usize = 0x118;
const NR24: usize = 0x119;
const NR30: usize = 0x11a;
const NR31: usize = 0x11b;
const NR32: usize = 0x11c;
const NR33: usize = 0x11d;
const NR34: usize = 0x11e;
const NR41: usize = 0x120;
const NR42: usize = 0x121;
const NR43: usize = 0x122;
const NR44: usize = 0x123;
const NR50: usize = 0x124;
const NR51: usize = 0x125;
const NR52: usize = 0x126;
const WAVE_RAM_END: usize = 0x13f;
const WAVE_RAM_START: usize = 0x130;
*/
    pub fn new() -> Self {
        Self {
            rendered_sound: Vec::new(),
            sample_clock: 0,
            clocks_since_length_tick: 0,
            audio_buffer_depth: 0,
            apu_active: false,
            nr50: 0,
            nr51: 0,
            channel_active: [false; 4],
            channel_1_dac_enable: false,
            channel_1_clocks_since_sample: 0,
            channel_1_clocks_since_envelope_tick: 0,
            channel_1_clocks_since_sweep_tick: 0,
            channel_1_length_timer: 0,
            channel_1_period_sweep_pace: 0,
            channel_1_period_sweep_direction: false,
            channel_1_period_step: 0,
            channel_1_wave_duty: 0,
            channel_1_initial_length: 0,
            channel_1_initial_volume: 0,
            channel_1_index: 0,
            channel_1_volume: 0,
            channel_1_envelope_direction: false,
            channel_1_sweep_pace: 0,
            channel_1_wavelength: 0,
            channel_1_length_enable: false,
            channel_1_trigger: false,
            channel_2_dac_enable: false,
            channel_2_clocks_since_sample: 0,
            channel_2_clocks_since_envelope_tick: 0,
            channel_2_length_timer: 0,
            channel_2_pace: 0,
            channel_2_wave_duty: 0,
            channel_2_initial_length: 0,
            channel_2_initial_volume: 0,
            channel_2_index: 0,
            channel_2_volume: 0,
            channel_2_envelope_direction: false,
            channel_2_sweep_pace: 0,
            channel_2_wavelength: 0,
            channel_2_length_enable: false,
            channel_2_trigger: false,
            channel_3_dac_enable: false,
            channel_3_index: 0,
            channel_3_clocks_since_sample: 0,
            channel_3_length_timer: 0,
            channel_3_output_level: 0,
            channel_3_wavelength: 0,
            channel_3_trigger: false,
            channel_3_length_enable: false,
            channel_3_wave_ram: [0; 16],
            channel_4_dac_enable: false,
            channel_4_last_sample: false,
            channel_4_clocks_since_envelope_tick: 0,
            channel_4_length_timer: 0,
            channel_4_length_enable: false,
            channel_4_initial_volume: 0,
            channel_4_volume: 0,
            channel_4_envelope_direction: false,
            channel_4_sweep_pace: 0,
            channel_4_clock_shift: 0,
            channel_4_lfsr_width: 0,
            channel_4_clock_divider: 0,
            channel_4_wavelength: 0,
            channel_4_trigger: false,
            channel_4_clocks_since_sample: 0,
        }
    }

    fn clocks_per_ch1_sweep_tick(&self) -> u64 {
        (4_190_000 / 128) * self.channel_1_period_sweep_pace as u64
    }

    fn clocks_per_envelope_tick(&self) -> u64 {
        4_190_000 / 64
    }

    fn update_channel_1(&mut self, clocks: u64) -> f32 {
        if !self.channel_1_dac_enable {
            return 0.0f32;
        }

        // pace, not "sweep pace" here, even though this is implementation of "channel 1 sweep"
        // functionality. probably should be sweep pace with others describing "envelope pace"
        // or something...
        if self.channel_1_period_sweep_pace != 0 {
            self.channel_1_clocks_since_sweep_tick += clocks;
            if self.channel_1_clocks_since_sweep_tick > self.clocks_per_ch1_sweep_tick() {
                self.channel_1_clocks_since_sweep_tick -= self.clocks_per_ch1_sweep_tick();
                // note direction is opposite of the meaning of envelope direction elsewhere...
                if !self.channel_1_period_sweep_direction {
                    let new_wavelength = self.channel_1_wavelength as i32 + (self.channel_1_wavelength / (2 << self.channel_1_period_step)) as i32;
//                    eprintln!("sweep tick up: wavelength={}", new_wavelength);
                    if new_wavelength > 0x7ff {
//                        eprintln!("disabled channel 1 because wavelength overflow");
                        self.channel_1_dac_enable = false;
                    } else {
                        self.channel_1_wavelength = new_wavelength as u16;
                    }
                } else {
                    let new_wavelength = self.channel_1_wavelength as i32 - (self.channel_1_wavelength / (2 << self.channel_1_period_step)) as i32;
//                    eprintln!("sweep tick down: wavelength={}", new_wavelength);
                    if new_wavelength < 0 {
                        self.channel_1_wavelength = 0;
                    } else {
                        self.channel_1_wavelength = new_wavelength as u16;
                    }
                };
            }
        }
        // see channel 2 docs about overkill here
        //
        if self.channel_1_sweep_pace != 0 {
            self.channel_1_clocks_since_envelope_tick += clocks;
            if self.channel_1_clocks_since_envelope_tick > self.clocks_per_envelope_tick() * self.channel_1_sweep_pace as u64 {
                self.channel_1_clocks_since_envelope_tick -= self.clocks_per_envelope_tick() * self.channel_1_sweep_pace as u64;
                self.channel_1_volume = if self.channel_1_envelope_direction {
                    std::cmp::min(15, self.channel_1_volume + 1)
                } else {
                    std::cmp::max(0, self.channel_1_volume as i8 - 1) as u8
                };
//                eprintln!("channel 1 vol={}, wavelength={:03x}", self.channel_1_volume, self.channel_1_wavelength);
            }
        }

        self.channel_1_clocks_since_sample += clocks;
        if self.channel_1_clocks_since_sample > self.channel_1_clocks_per_sample() {
            self.channel_1_clocks_since_sample -= self.channel_1_clocks_per_sample();
            self.channel_1_index = (self.channel_1_index + 1) & 0x7;
        }

        // yoinked from the NR11 docs here:
        // https://gbdev.io/pandocs/Audio_Registers.html#ff11--nr11-channel-1-length-timer--duty-cycle
        const WAVEFORMS: &'static [[u8; 8]; 4] = &[
            [1, 1, 1, 1, 1, 1, 1, 0],
            [0, 1, 1, 1, 1, 1, 1, 0],
            [0, 1, 1, 1, 1, 0, 0, 0],
            [1, 0, 0, 0, 0, 0, 0, 1],
        ];

        let sample = WAVEFORMS[self.channel_1_wave_duty as usize][self.channel_1_index as usize];

        if sample == 0 {
            0.0
        } else {
            (self.channel_1_volume as f32) / 16.0f32
        }
    }

    fn channel_1_clocks_per_sample(&self) -> u64 {
        4 * (2048 - self.channel_1_wavelength as u64)
    }


    fn update_channel_2(&mut self, clocks: u64) -> f32 {
        if !self.channel_2_dac_enable {
            return 0.0f32;
        }
        // TODO: a more precise accounting would be to consume clocks one at a time and figure out
        // all the timers. but i'm going to assume clocks is fairly low (4-16 typically ???) and
        // that is overkill.

        if self.channel_2_sweep_pace != 0 {
            self.channel_2_clocks_since_envelope_tick += clocks;
            if self.channel_2_clocks_since_envelope_tick > self.clocks_per_envelope_tick() * self.channel_2_sweep_pace as u64 {
                self.channel_2_clocks_since_envelope_tick -= self.clocks_per_envelope_tick() * self.channel_2_sweep_pace as u64;
                self.channel_2_volume = if self.channel_2_envelope_direction {
                    std::cmp::min(15, self.channel_2_volume + 1)
                } else {
                    std::cmp::max(0, self.channel_2_volume as i8 - 1) as u8
                };
//                eprintln!("channel 2 vol={}", self.channel_2_volume);
            }
        }

        self.channel_2_clocks_since_sample += clocks;
        if self.channel_2_clocks_since_sample > self.channel_2_clocks_per_sample() {
            self.channel_2_clocks_since_sample -= self.channel_2_clocks_per_sample();
            self.channel_2_index = (self.channel_2_index + 1) & 0x7;
        }

        // yoinked from the NR11 docs here:
        // https://gbdev.io/pandocs/Audio_Registers.html#ff11--nr11-channel-1-length-timer--duty-cycle
        const WAVEFORMS: &'static [[u8; 8]; 4] = &[
            [1, 1, 1, 1, 1, 1, 1, 0],
            [0, 1, 1, 1, 1, 1, 1, 0],
            [0, 1, 1, 1, 1, 0, 0, 0],
            [1, 0, 0, 0, 0, 0, 0, 1],
        ];

        let sample = WAVEFORMS[self.channel_2_wave_duty as usize][self.channel_2_index as usize];

        if sample == 0 {
            0.0
        } else {
            (self.channel_2_volume as f32) / 16.0f32
        }
    }

    fn channel_2_clocks_per_sample(&self) -> u64 {
        4 * (2048 - self.channel_2_wavelength as u64)
    }

    fn channel_3_clocks_per_sample(&self) -> u64 {
        2 * (2048 - self.channel_3_wavelength as u64)
    }

    // what happens if in one `clocks` we update both the position in the channel 3 buffer *and*
    // sample the buffer? which happens first?
    //
    // TODO: assuming this ordering isn't interesting and that we can update the position first.
    // TODO: same for a race with sampling and the channel length. currently returns a sample then
    // stops the channel. may stop the channel a few clocks late?
    fn update_channel_3(&mut self, clocks: u64) -> f32 {
        if !self.channel_3_dac_enable {
            return 0.0f32;
        }

        self.channel_3_clocks_since_sample += clocks;
        if self.channel_3_clocks_since_sample > self.channel_3_clocks_per_sample() {
            self.channel_3_clocks_since_sample -= self.channel_3_clocks_per_sample();
            self.channel_3_index = (self.channel_3_index + 1) & 0x1f;
        }

        let ch3_subindex = self.channel_3_index & 1;
        let ch3_sample = ((self.channel_3_wave_ram[self.channel_3_index as usize / 2]) >> (4 * (1 - ch3_subindex))) & 0x0f;

        let ch3_sample = match self.channel_3_output_level {
            0b00 => 0,
            0b01 => ch3_sample,
            0b10 => ch3_sample >> 1,
            _ => ch3_sample >> 2,
        };

        (ch3_sample as f32) / 16.0f32
    }

    fn channel_4_clocks_per_sample(&self) -> u64 {
        let denominator = if self.channel_4_clock_divider == 0 {
            (2 << (self.channel_4_clock_shift as u64)) >> 1
        } else {
            (2 << self.channel_4_clock_shift as u64) * self.channel_4_clock_divider as u64
        };
        // magic numbers from
        // https://gbdev.io/pandocs/Audio_Registers.html#ff13--nr13-channel-1-period-low-write-only
        // 2^^22 / 262144 == 16 cycles per clock
        16 * denominator
    }

    fn update_channel_4(&mut self, clocks: u64) -> f32 {
        if !self.channel_4_dac_enable {
            return 0.0f32;
        }

        if self.channel_4_sweep_pace != 0 {
            self.channel_4_clocks_since_envelope_tick += clocks;
            if self.channel_4_clocks_since_envelope_tick > self.clocks_per_envelope_tick() * self.channel_4_sweep_pace as u64 {
                self.channel_4_clocks_since_envelope_tick -= self.clocks_per_envelope_tick() * self.channel_4_sweep_pace as u64;
                self.channel_4_volume = if self.channel_4_envelope_direction {
                    std::cmp::min(15, self.channel_4_volume + 1)
                } else {
                    std::cmp::max(0, self.channel_4_volume as i8 - 1) as u8
                };
//                eprintln!("channel 4 vol={}", self.channel_4_volume);
            }
        }

        self.channel_4_clocks_since_sample += clocks;
        if self.channel_4_clocks_since_sample > self.channel_4_clocks_per_sample() {
            self.channel_4_clocks_since_sample -= self.channel_4_clocks_per_sample();
            use rand::RngCore;
            self.channel_4_last_sample = (rand::rngs::OsRng.next_u32() & 1) != 0;
        }

        let ch4_sample = if self.channel_4_last_sample {
            self.channel_4_volume
        } else {
            0
        };

        (ch4_sample as f32) / 16.0f32
    }

    // TODO: this should actually track on writes to DIV, but i really really do not want to plumb
    // that through right now...
    fn update_lengths(&mut self, clocks: u64) {
        let mut length_clocks = self.clocks_since_length_tick + clocks;
        const DIV_APU_CLOCK_HACK: u64 = 4_190_000 / 256;
        while length_clocks > DIV_APU_CLOCK_HACK {
            if self.channel_1_length_enable {
                self.channel_1_length_timer += 1;
                if self.channel_1_length_timer == 64 {
//                    eprintln!("channel 1 length complete");
                    // TODO: should also clear wave ram? maybe?
                    self.channel_1_length_enable = false;
                    self.channel_1_length_timer = 0;
                    self.channel_1_dac_enable = false;
                }
            }

            if self.channel_2_length_enable {
                self.channel_2_length_timer += 1;
                if self.channel_2_length_timer == 64 {
//                    eprintln!("channel 2 length complete");
                    self.channel_2_length_enable = false;
                    self.channel_2_length_timer = 0;
                    self.channel_2_dac_enable = false;
                }
            }

            if self.channel_3_length_enable {
                if self.channel_3_length_timer == 255 {
//                    eprintln!("channel 3 length complete");
                    // TODO: should also clear wave ram? maybe?
                    self.channel_3_length_enable = false;
                    self.channel_3_length_timer = 0;
                    self.channel_3_dac_enable = false;
                } else {
                    self.channel_3_length_timer += 1;
                }
            }

            if self.channel_4_length_enable {
                if self.channel_4_length_timer >= 64 {
//                    eprintln!("channel 4 length complete");
                    self.channel_4_length_enable = false;
                    self.channel_4_length_timer = 0;
                    self.channel_4_dac_enable = false;
                } else {
                    self.channel_4_length_timer += 1;
                }
            }

            length_clocks -= DIV_APU_CLOCK_HACK;
        }

        self.clocks_since_length_tick = length_clocks;
    }

    // TODO: NR51
    // TODO: NR50
    // TODO: DIV-APU?
    pub fn advance_clock(&mut self, sink: Option<&rodio::Sink>, clocks: u64, turbo: bool) {
        struct RenderedSample {
            buf: Vec<f32>,
            pos: usize,
        }

        impl rodio::Source for RenderedSample {
            fn current_frame_len(&self) -> Option<usize> {
                Some(self.buf.len())
            }

            fn channels(&self) -> u16 {
                1
            }

            fn sample_rate(&self) -> u32 {
                44100
            }

            fn total_duration(&self) -> Option<std::time::Duration> {
                None
            }
        }

        impl Iterator for RenderedSample {
            type Item = f32;

            fn next(&mut self) -> Option<Self::Item> {
                let res = self.buf.get(self.pos).copied();
                if res.is_some() {
                    self.pos += 1;
                }
                res
            }
        }

        if !self.apu_active {
            return;
        }

        self.sample_clock += clocks;
        while self.sample_clock > CLOCKS_PER_SAMPLE {
            let ch1_sample = self.update_channel_1(CLOCKS_PER_SAMPLE);
            let ch2_sample = self.update_channel_2(CLOCKS_PER_SAMPLE);
            let ch3_sample = self.update_channel_3(CLOCKS_PER_SAMPLE);
            let ch4_sample = self.update_channel_4(CLOCKS_PER_SAMPLE);
            let sample = ch1_sample + ch2_sample + ch3_sample + ch4_sample;

            // let ch3_sample = ch3_sample * 100.0;
            // render a sample, advance forward
            self.sample_clock -= CLOCKS_PER_SAMPLE;

            self.rendered_sound.push(sample / 8.0f32); // samples are just way too loud and idk why yet

            // TODO: more precisely, should update by the difference from sample_clock to
            // CLOCKS_PER_SAMPLE, then update after each sample, and update one more time at the
            // end for however far into the next sample period `clocks` leaves us.
            self.update_lengths(CLOCKS_PER_SAMPLE);
        }

        if self.rendered_sound.len() >= 256 {
            let rendered = std::mem::replace(&mut self.rendered_sound, Vec::new());
            if let Some(sink) = sink {
                self.audio_buffer_depth = sink.len() as u64;
                if sink.len() == 0 {
                    eprintln!("sink was drained? sorry, audio pop :(");
                } else if self.audio_buffer_depth > 10 && turbo {
                    // there's audio buffered and we're in turbo mode. we'll actually just skip
                    // this buffer so as to not get an overly-deep audio buffer if and when turbo
                    // stops happening.
                    return;
                } else {
//                    eprintln!("sink depth {}", sink.len());
                }
                // eprintln!("sample: {:?}", rendered);
                sink.append(RenderedSample { buf: rendered, pos: 0 });
                sink.play();
            }
        }
    }

    fn set_nr52(&mut self, v: u8) {
        // bit 7 controls the APU state, buts 0-3 are channel 1-4 state but are read-only
        // TODO: what happens on writes to bits 0-6? assuming those writes are discarded.
        self.apu_active = (v & 0x80) != 0;
    }
}
