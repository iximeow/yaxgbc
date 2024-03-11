use clap::Parser;

use std::fmt;
use std::fs::File;
use std::io::{Read, Seek, SeekFrom, Write};
use std::fmt::{Write as FmtWrite};
use std::sync::Arc;
use std::sync::Mutex;
use std::time::Duration;
use std::time::SystemTime;

use yaxpeax_arch::{Decoder, ReadError};

mod apu;
use crate::apu::Apu;
mod cpu;
use crate::cpu::Cpu;
use crate::cpu::DecorateExt;

mod frontend;
mod timer;

#[derive(Parser)]
#[clap(about, version, author)]
struct Args {
    file: String
}

fn main() {
    let args = Args::parse();

    let rom = std::fs::read(&args.file).unwrap();

    let save_path = format!("{}.sav", args.file);
    let save_path = std::path::Path::new(&save_path);
    let save = if true {
        Some(std::fs::OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .open(&save_path)
            .expect("can create save file"))
    } else {
        None
    };

    let cart = GBCCart::new(rom, save).unwrap();

    let bootrom = GBCCart::boot_rom(std::fs::read("cgb_boot.bin").unwrap(), &cart);
//    let bootrom = GBCCart::boot_rom(std::fs::read("dmg_boot.bin").unwrap(), &cart);

    let mut gb = GBC::new(bootrom);

    let (_stream, stream_handle) = rodio::OutputStream::try_default().unwrap();
    let audio_sink = rodio::Sink::try_new(&stream_handle).expect("can build audio sink");

    gb.set_cart(cart);
    gb.set_audio(audio_sink);

//    stream_handle.play_raw(audio_sink);

    let sins = Arc::new(Mutex::new(gb));

    let sins_ref = Arc::clone(&sins);

    std::thread::spawn(|| {
//        frontend::tui::do_ui(sins_ref)
        frontend::gui::do_ui(sins_ref)
    });

    /*
    let (tx, rx) = std::sync::mpsc::channel();

    std::thread::spawn(|| {
        timer::run(tx)
    });
    */
    /// safety: timeBeginPeriod can't actually cause unsoundness.
    #[cfg(target_os="windows")]
    {
        let time_res = unsafe { windows::Win32::Media::timeBeginPeriod(1) };
        if time_res != windows::Win32::Media::TIMERR_NOERROR {
            panic!("failed to set timeBeginPeriod: {}", time_res);
        }
    }

    let mut i = 0;
    let mut clock_total = 0;
    const CLOCKS_PER_FRAME: u64 = 4_190_000 / 60;
    let mut frame_target = SystemTime::now() + Duration::from_millis(16);
    let mut last_overshoot = 0;

    loop {
        let mut gb = sins.lock().unwrap();
        let mut vblank_fired = false;

        let mut vblank_before = gb.management_bits[IF] & 1;

        for _ in 0..20 {
            gb.run();

            let vblank_after = gb.management_bits[IF] & 1;
            vblank_fired |= vblank_before == 0 && vblank_after == 1;
            vblank_before = vblank_after;
        }

        // this entire block comes from... thinking really hard, and still not really understanding
        // it all the way.
        //
        // the idea i was trying to capture is the result of a few observations.
        // * on linux, having frame targets computed as "now + 16ms" appeared to have the emulator
        // get a little slwoer each frame - beating the target by 16ms, then 15.8ms, then 15.6,
        // 15.4, ... until some frame loses w.r.t frame target and resets to 16ms. this was almost
        // certainly sleep() sleeping more than the requested time, and that overshoot turning
        // into a growing error over time.
        // * on windows, this same thing, but several milliseconds of error every frame.
        //
        // so, now, this tries to do three things.
        // * have next frame target be constantly computed with respect to the current frame time, not
        // from some independently-managed timer.
        // * sleep just enough to produce one frame every 16.666ms (e.g. 60fps)
        // * correctly account for sleep() sleeping too much (or too little)
        //
        // the way this is handled... is best shown in a diagram. the simplest case of "the first
        // frame" is ignored here, and the Y-axis is time:
        //
        //  990.0ms | frame N vblank
        //  990.1ms | sleep until frame N target (9.9ms)
        // 1000.0ms |-------- frame N target
        // ...
        // 1004.0ms | wake from frame N sleep (actual sleep: 13.9ms)
        // 1004.1ms | work work work
        // ...
        // 1006.0ms | frame N+1 vblank
        // 1006.3ms | sleep until frame N target (9.7ms)
        // ...
        // 1016.0ms |-------- frame N+1 target
        // ...
        // 1021.1ms | wake from frame N sleep (actual sleep: 14.7ms)
        // 1021.1ms | work work work
        //
        // which is to say, count both time-to-frame and time-sleep-overshot to calculate future
        // frame times, and subtract overshoot time from the time we permit the current frame to be
        // computed.
        //
        // if we consistently run out of time for the current frame, probably stop trying to sleep
        // at all. on some platforms that seems to have a minimum of a few milliseconds delay, and
        // that might be make-or-break...???
        if vblank_fired {
            /*
            let now = SystemTime::now();
            let times = std::mem::replace(&mut gb.frame_times, Vec::new());
            let times = times.into_iter().filter(|x| {
                now.duration_since(*x).unwrap() < std::time::Duration::from_millis(1000)
            }).collect();
            gb.frame_times = times;
            gb.frame_times.push(now);
            std::mem::drop(gb);

            rx.recv().expect("TODO: sender is still running");
            */
            let now = SystemTime::now();
            if now < frame_target {
                let times = std::mem::replace(&mut gb.frame_times, Vec::new());
                let times = times.into_iter().filter(|x| {
                    now.duration_since(*x).unwrap() < std::time::Duration::from_millis(1000)
                }).collect();
                gb.frame_times = times;
                gb.frame_times.push(now);

                let apu_queue_depth = gb.apu.audio_buffer_depth;

                let micros_to_sleep = frame_target.duration_since(now).unwrap().as_micros();
                if last_overshoot > micros_to_sleep {
                    eprintln!("overshoot too much? last sleep overslept by {}, this frame has {} to waste", last_overshoot, micros_to_sleep);
                    last_overshoot = 0;
                } else {
                    let to_sleep = std::time::Duration::from_micros((micros_to_sleep - last_overshoot) as u64);
//                    eprintln!("beat frame time by {:0.4}ms", to_sleep.as_micros() as f64 / 1000.0);
                    let turbo = gb.turbo;
                    std::mem::drop(gb);

                    if !turbo {
                        std::thread::sleep(to_sleep);
                    }
                    let slept = now.elapsed().unwrap();
                    if slept > to_sleep {
                        let overshoot = slept.as_micros() - to_sleep.as_micros();
//                        eprintln!("overshot by {}micros, frame budget is {}", overshoot, 16666 - overshoot);
                        last_overshoot = overshoot;
                    } else {
                        last_overshoot = 0;
                    }
                }
                // try mixing some extra sleep in if we're behind on audio playback. the idea here
                // is to mix in a little sleep if we're a little behind, a lot of sleep if we're a
                // lot behind, but never too much to cause obvious visible delay.
                //
                // all in all, we're relying on accurate audio playback to calibrate times within a
                // few milliseconds...
                //
                // audio buffers are 256 samples, so each buffer we're behind represents 0.58ms of
                // playback we're behind on. call <5ms delay fine. at 100ms of delay, though, we'll
                // add a full two milliseconds to every frame to try catching up.
                let extra_sleep_micros = if apu_queue_depth < 4 {
                    0
                } else if apu_queue_depth < 30 {
                    (5000 * apu_queue_depth) / 30
                } else {
                    5000
                };
                frame_target = SystemTime::now() + (Duration::from_micros(16666 - last_overshoot as u64 + extra_sleep_micros));
            } else {
                eprintln!("yike, by {} micros", now.duration_since(frame_target).unwrap().as_micros());
                frame_target = now + Duration::from_millis(16);
            }
        }
    }
}

struct Lcd {
    // HBlank, VBlank, Searching OAM, Transferring Data to LCD Controller
    mode: u8,
    lcdc: u8,
    ly: u8,
    lcd_clock: u64,
    current_line_start: u64,
    current_draw_start: u64,
    initial_scx: u8,
    // when, in dots, we'll be at the next line. this is the end of the current line's HBlank.
    next_line: u64,
    next_draw_time: u64,
    oam_scan_items: Vec<OamItem>,
    oam: [u8; 0xa0],
    background_palettes_data: [u8; 0x40],
    background_palette_data_cache: [[u8; 4]; 0x20],
    object_palettes_data: [u8; 0x40],
    object_palette_data_cache: [[u8; 4]; 0x20],
    background_pixels: [Pixel; 160],
    curr_background_pixel: u8,
    oam_pixels: [Pixel; 160],
    display: Box<[u8; 4 * 144 * 160]>,
    toggle_background_sprite_bank: bool,
    toggle_oam_sprite_bank: bool,
}

#[derive(Copy, Clone, Default, PartialEq, Debug)]
struct Pixel {
    rgb: [u8; 4],
    pixel: u8,
    bg_priority: bool,
    priority: bool,
    // pointless field, but having implicit padding can result in poor codegen for array
    // initializers, see https://github.com/rust-lang/rust/issues/122274
    _pad: u8,
}

struct OamItem {
    selected_line: u8,
    x: u8,
    tile_index: u8,
    oam_attrs: OamAttributes,
}

struct OamAttributes(u8);

impl OamAttributes {
    fn bg_priority(&self) -> bool {
        (self.0 & 0b1000_0000) != 0
    }

    fn flip_vertical(&self) -> bool {
        (self.0 & 0b0100_0000) != 0
    }

    fn flip_horizontal(&self) -> bool {
        (self.0 & 0b0010_0000) != 0
    }

    fn palette_number(&self) -> u16 {
        ((self.0 & 0b0001_0000) >> 4) as u16
    }

    fn vram_bank(&self) -> u16 {
        ((self.0 & 0b0000_1000) >> 3) as u16
    }

    fn bg_palette(&self) -> u8 {
        self.0 & 0b0000_0111
    }
}

struct TileAttributes(u8);

impl TileAttributes {
    fn flip_vertical(&self) -> bool {
        (self.0 & 0b0100_0000) != 0
    }

    fn flip_horizontal(&self) -> bool {
        (self.0 & 0b0010_0000) != 0
    }

    fn vram_bank(&self) -> u16 {
        ((self.0 & 0b0000_1000) >> 3) as u16
    }

    fn bg_palette(&self) -> u8 {
        self.0 & 0b0000_0111
    }

    fn priority(&self) -> bool {
        (self.0 & 0b1000_000) != 0
    }
}

// lifted from SameBoy, i'm certain they've figured out color correction in a way i never
// will get to..
static CHANNEL_CORRECTION: [u8; 32] = [
      0,   6,  12,  20,  28,  36,  45,  56,
     66,  76,  88, 100, 113, 125, 137, 149,
    161, 172, 182, 192, 202, 210, 218, 225,
    232, 238, 243, 247, 250, 252, 254, 255
];

impl Lcd {
    const OAM_SCAN: u64 = 80;
    const LINE_TIME: u64 = 376 + Self::OAM_SCAN;
    const VBLANK_TIME: u64 = 4560; // vblank is 10 scan lines
    const SCREEN_TIME: u64 = 154 * Self::LINE_TIME;

    fn new() -> Self {
        Self {
            mode: 1,
            lcdc: 0,
            ly: 0,
            lcd_clock: 0,
            current_line_start: 0,
            current_draw_start: 0,
            initial_scx: 0,
            next_line: Self::LINE_TIME,
            next_draw_time: Self::SCREEN_TIME,
            oam_scan_items: Vec::new(),
            oam: [0u8; 0xa0],
            background_palettes_data: [0u8; 0x40],
            background_palette_data_cache: [[0u8; 4]; 0x20],
            object_palettes_data: [0u8; 0x40],
            object_palette_data_cache: [[0u8; 4]; 0x20],
            background_pixels: [Pixel::default(); 160],
            curr_background_pixel: 0,
            oam_pixels: [Pixel::default(); 160],
            display: Box::new([0u8; 4 * 144 * 160]),
            toggle_background_sprite_bank: false,
            toggle_oam_sprite_bank: false,
        }
    }

    fn recompute_palette_cache(&mut self) {
        // indexing into color palettes as the gameboy is designed involves some unfortunate index
        // construction and shifting. additionally, the color spectrum of the CGB LCD does not map
        // to contemporary displays well, so the literal colors have to go through a second
        // intensity correction LUT that makes color calculation a double-dereference *plus* index
        // shifting and masking and ...
        //
        // to avoid all that, cache the corrected color palettes as they're fed into the lcd,
        // rather than rerunning these calculations JIT style whenever the palette is accessed.
        // this causes more work when loading color palettes, but hopefully that is significantly
        // less common than using palettes.
        //
        // this function computes palette caches for both background and object palettes only
        // because i didn't feel like making two functions.

        for palette_nr in 0..8 {
            for px in 0..4 {
                let color_lo = self.background_palettes_data[palette_nr * 8 + px * 2];
                let color_hi = self.background_palettes_data[palette_nr * 8 + px * 2 + 1];
                let color = ((color_hi as u16) << 8) | (color_lo as u16);

                let r = color & 0x1f;
                let g = (color >> 5) & 0x1f;
                let b = (color >> 10) & 0x1f;
                let rgba = [
                    CHANNEL_CORRECTION[r as usize],
                    CHANNEL_CORRECTION[g as usize],
                    CHANNEL_CORRECTION[b as usize],
                    0xff
                ];
                self.background_palette_data_cache[palette_nr * 4 + px] = rgba;
            }
        }

        for palette_nr in 0..8 {
            for px in 0..4 {
                let color_lo = self.object_palettes_data[palette_nr * 8 + px * 2];
                let color_hi = self.object_palettes_data[palette_nr * 8 + px * 2 + 1];
                let color = ((color_hi as u16) << 8) | (color_lo as u16);

                let r = color & 0x1f;
                let g = (color >> 5) & 0x1f;
                let b = (color >> 10) & 0x1f;
                let rgba = [
                    CHANNEL_CORRECTION[r as usize],
                    CHANNEL_CORRECTION[g as usize],
                    CHANNEL_CORRECTION[b as usize],
                    0xff
                ];
                self.object_palette_data_cache[palette_nr * 4 + px] = rgba;
            }
        }
    }

    fn px2rgba(palette_data_cache: &[[u8; 4]; 0x20], palette_nr: u8, px: u8) -> [u8; 4] {
        palette_data_cache[(palette_nr * 4 + px) as usize]
    }

    fn window_tile_lookup_by_nr<'a>(&self, vram: &'a [u8], tile_nr: u16) -> (&'a [u8], TileAttributes) {
        // look up the tile number in vram bank 0
        // then attrs for that tile number in vram bank 1
        //
        // tile attrs can pick between vram banks 0 and 1 for tile data itself, so we need both to
        // find the right data.
        let tile_map_base = self.window_tile_base() as usize;
        let tile_id = vram[tile_map_base + tile_nr as usize];
        let tile_attrs = TileAttributes(vram[tile_map_base + tile_nr as usize + 0x2000]);

        let mut tile_data_addr = self.tile_addr_translate(tile_id);

        tile_data_addr += tile_attrs.vram_bank() as usize * 0x2000;

        (&vram[tile_data_addr..][..16], tile_attrs)
    }
    fn tile_lookup_by_nr<'a>(&self, vram: &'a [u8], tile_nr: u16) -> (&'a [u8], TileAttributes) {
        // look up the tile number in vram bank 0
        // then attrs for that tile number in vram bank 1
        //
        // tile attrs can pick between vram banks 0 and 1 for tile data itself, so we need both to
        // find the right data.
        let tile_map_base = self.background_tile_base() as usize;
        let tile_id = vram[tile_map_base + tile_nr as usize];
        let tile_attrs = TileAttributes(vram[tile_map_base + tile_nr as usize + 0x2000]);

        let mut tile_data_addr = self.tile_addr_translate(tile_id);

        tile_data_addr += tile_attrs.vram_bank() as usize * 0x2000;

        (&vram[tile_data_addr..][..16], tile_attrs)
    }
    fn tile_addr_translate<'a>(&self, tile_id: u8) -> usize {
        let data_offset = if self.lcdc & 0b0001_0000 == 0 {
            let addr = ((tile_id as i8).wrapping_add(-0x80)) as u8 as u16 * 16;
            0x800 + addr as usize
        } else {
            tile_id as usize * 16
        };
        data_offset
    }

    fn background_tile_base(&self) -> u16 {
        if (self.lcdc & 0b0000_1000 == 0) {
            0x1800
        } else {
            0x1c00
        }
    }

    fn window_tile_base(&self) -> u16 {
        if self.lcdc & 0b0100_0000 == 0 {
            0x1800
        } else {
            0x1c00
        }
    }

    fn on(&self) -> bool {
        self.lcdc & 0b1000_0000 != 0
    }

    fn window_enable(&self) -> bool {
        self.lcdc & 0b0010_0000 != 0
    }

    fn sprite_double_size(&self) -> bool {
        self.lcdc & 0b0000_0100 != 0
    }

    fn sprite_enable(&self) -> bool {
        self.lcdc & 0b0000_0010 != 0
    }

    fn set_lcdc(&mut self, new_lcdc: u8) {
        if self.mode != 1 {
//            eprintln!("set lcdc to {:08b} outside mode 1?", new_lcdc);
        }
        self.lcdc = new_lcdc;
    }

    fn load(&self, address: u16) -> u8 {
        // TODO: if we're in a mode where you can read OAM, allow it, otherwise return $ff
        assert!(address < self.oam.len() as u16, "invalid oam load address: {:#02x}", address);
        self.oam[address as usize]
    }

    fn store(&mut self, address: u16, value: u8) {
        // TODO: if we're in a mode where you can write OAM, allow it, otherwise ignore write
        assert!(address < self.oam.len() as u16, "invalid oam store address: {:#02x}", address);
//        eprintln!("oam ({}) write item {} = {:02x}", address / 4, address % 4, value);
        self.oam[address as usize] = value
    }

    fn render_sprite_debug(&self, vram: &[u8], display: &mut [u8; 4 * 42 * 182]) {
        let sprite_height = if self.lcdc & 0b100 == 0 {
            8
        } else {
            16
        };

        for i in 0..self.oam.len() / 4 {
            let obj_x = 2 + (i % 4) * (8 + 2);
            let obj_y = 2 + (i / 4) * (16 + 2);

            // disregard lcdc.3; we'll draw both tiles of 8x16 images, just.. not in the right
            // layout.
            let tile_index = self.oam[i * 4 + 2];
            let oam_attrs = OamAttributes(self.oam[i * 4 + 3]);

            let bank = oam_attrs.vram_bank() as usize * 0x2000;

            for selected_line in 0..sprite_height {
                let y_addr = if oam_attrs.flip_vertical() {
                    (sprite_height - 1) - selected_line
                } else {
                    selected_line
                };
                let (tile_row_lo, tile_row_hi) = {
                    let mut oam_tile_addr = bank + tile_index as usize * 16;
                    let mut tile_line = y_addr;
                    if y_addr >= 8 {
                        oam_tile_addr += 16;
                        tile_line -= 8;
                    }
                    let oam_tile_data = &vram[oam_tile_addr..][..16];
                    let lo = oam_tile_data[tile_line as usize * 2];
                    let hi = oam_tile_data[tile_line as usize * 2 + 1];
                    (lo, hi)
                };

                for x in 0..8 {
                    let x = if oam_attrs.flip_horizontal() {
                        7 - x
                    } else {
                        x
                    };

                    let px =
                        (((tile_row_hi >> (7 - x)) & 1) << 1) |
                        (((tile_row_lo >> (7 - x)) & 1) << 0);

                    // 102 = OAM_DEBUG_PANEL_WIDTH
                    let addr = obj_x + x + (obj_y + selected_line) * 42;

                    if px != 0 {
                        let rgb = Self::px2rgba(&self.object_palette_data_cache, oam_attrs.bg_palette(), px);

                        display[addr as usize * 4..][..4].copy_from_slice(&rgb);
                    } else {
                        display[addr as usize * 4..][..4].copy_from_slice(&[0, 0, 0, 0]);
                    }
                }
            }
        }
    }

    // advance lcd state by `clocks` ticks, using `lcd_stat` to determine if we should generate an
    // interrupt.
//    #[inline(never)]
    fn advance_clock(&mut self, vram: &[u8], lcd_stat: u8, lyc: u8, clocks: u64, scx: u8, scy: u8, wx: u8, wy: u8) -> (bool, bool) {
        if !self.on() {
            return (false, false);
        }
        // the number of dots (LCD clocks) to display one line. `HBlank` is whatever time is
        // necessary to meet this time, after completing mode 3.
        self.lcd_clock = self.lcd_clock.wrapping_add(clocks);
        if clocks > 40 {
            panic!("update Lcd::advance_clock to handle huge jumps");
        }
        let mut screen_time = self.lcd_clock - self.current_draw_start;

        let mut should_interrupt = false;

        if screen_time > Self::SCREEN_TIME {
            // ok, screen's done and we're resetting to mode 2 (exiting mode 1)
//            eprintln!("screen done");
            /*
            for i in 0..self.display.len() {
                self.display[i] = 0;
            }
            */
            self.current_draw_start += (screen_time / Self::SCREEN_TIME) * Self::SCREEN_TIME;
            self.ly = 0;
            // fire LYC=LY
            if self.ly == lyc && (lcd_stat & 0b0100_0000 != 0) {
//                eprintln!("firing lyc=ly at ly={}", self.ly);
                should_interrupt = true;
            }

            self.current_line_start = self.current_draw_start;
            screen_time -= screen_time % Self::SCREEN_TIME;
        }

        let mut line_time = self.lcd_clock - self.current_line_start;

        let mut vblank_hit = false;

        if line_time > Self::LINE_TIME {
            // ok, line's done and we're resetting to mode 2 (exiting mode 0)
//            eprintln!("line {} done", self.ly);
            if self.ly < 144 {
                for px in 0..(self.curr_background_pixel as usize) {
                    assert!(self.curr_background_pixel == 160);
                    let addr = (self.ly as usize * 160 + px) * 4;
                    self.display[addr..][..4].copy_from_slice(&self.background_pixels[px].rgb[..]);
                    if self.oam_pixels[px].pixel != 0 && (!(self.oam_pixels[px].bg_priority && self.background_pixels[px].pixel != 0)) {
                        self.display[addr..][..4].copy_from_slice(&self.oam_pixels[px].rgb[..]);
                    }
                }
                self.curr_background_pixel = 0;
                self.oam_pixels = [Pixel::default(); 160];
            }

            self.current_line_start += (line_time / Self::LINE_TIME) * Self::LINE_TIME;
            self.ly += 1;

            // fire LYC=LY
            if self.ly == lyc && (lcd_stat & 0b0100_0000 != 0) {
//                eprintln!("firing lyc=ly at ly={}", self.ly);
                should_interrupt = true;
            }

            if self.ly == 144 {
                vblank_hit = true;
            }
            line_time -= Self::LINE_TIME;
        }

        if self.ly >= 144 {
            // so ... does lcd_stat.vblank imply that vblank is suppressed entirely, or only that
            // STAT does not fire on vblank, but IF.vblank is still set?
            if self.mode != 1 && (lcd_stat & 0b0001_0000 != 0) {
                should_interrupt = true;
            }
            self.mode = 1;
            (vblank_hit, should_interrupt)
        } else {
            if line_time < 80 {
                let prior_mode = self.mode;
                if self.mode != 2 && (lcd_stat & 0b0010_0000 != 0) {
                    should_interrupt = true;
                }
                self.mode = 2;

                if prior_mode != self.mode {
                    self.initial_scx = scx;
                    // do a full OAM scan up front. there might be benefits to driving the OAM reads in
                    // a more cycle-accurate manner, but i'm skimping on that for now.
                    self.oam_scan_items.clear();
                    for i in 0..40 {
                        if self.lcdc & 0b010 == 0 {
                            break;
                        }
                        let object_addr = 4 * i;
                        let y_end = self.oam[object_addr + 0];
                        let x_end = self.oam[object_addr + 1];

                        let sprite_height = if self.lcdc & 0b100 == 0 {
                            8
                        } else {
                            16
                        };

                        let selected_line = self.ly as i16 - ((y_end as i16) - 16);
                        if selected_line < 0 || selected_line >= sprite_height {
                            continue;
                        }

                        if self.ly == 88 {
//                            eprintln!("item {} x_end,y_end[height={}]=({}, {}), selected line {} of tile {}", i, sprite_height, x_end, y_end, selected_line, self.oam[object_addr + 2]);
                        }

                        // low bit of tile index is masked to 0 in 8x16 mode
                        let tile_index = if self.lcdc & 0b100 == 0 {
                            self.oam[object_addr + 2]
                        } else {
                            self.oam[object_addr + 2] & 0xfe
                        };

                        self.oam_scan_items.push(OamItem {
                            selected_line: selected_line as u8,
                            x: x_end,
                            tile_index,
                            oam_attrs: OamAttributes(self.oam[object_addr + 3]),
                        });
                        if self.oam_scan_items.len() == 10 {
                            break;
                        }
                    }
                    /*
                if self.ly == 88 {
                    for px in 0..160 {
                        eprint!("{:03}={:02},{}", px, self.oam_pixels[px].pixel, self.oam_pixels[px].rgb);
                        if px % 16 == 0 {
                            eprintln!("");
                        }
                    }
                    eprintln!("");
                }
                */
                    /*
                    self.oam_scan_items.sort_by_key(|item| item.x);
                    while self.oam_scan_items.len() > 10 {
                        self.oam_scan_items.pop();
                    }
                    */
                }
            } else if line_time < 80 + 168 {
                // the exact timing here depends on how many OBJs were found. 168 is a minimum.
                let prior_mode = self.mode;
                self.mode = 3;

                // we just entered mode 3, draw the pixels. again, this should be made pixel-accurate in
                // the future
                if self.mode != prior_mode {
                    for item in self.oam_scan_items.iter().rev() {
                        if item.x >= 168 {
                            continue;
                        }
                        let bank = (item.oam_attrs.vram_bank() as usize * 0x2000) ^ (if self.toggle_oam_sprite_bank { 0x2000 } else { 0 });
                        let oam_tile_addr = bank + item.tile_index as usize * 16;
                        let y_addr = if item.oam_attrs.flip_vertical() {
                            let oam_height = if self.lcdc & 0b100 == 0 {
                                8
                            } else {
                                16
                            };
                            (oam_height - 1) - item.selected_line
                        } else {
                            item.selected_line
                        };
                        let (tile_row_lo, tile_row_hi) = if y_addr < 8 {
                            let tile_line = y_addr;
                            let oam_tile_data = &vram[oam_tile_addr..][..16];
                            let lo = oam_tile_data[tile_line as usize * 2];
                            let hi = oam_tile_data[tile_line as usize * 2 + 1];
                            (lo, hi)
                        } else {
                            let tile_line = y_addr - 8;
                            let oam_tile_data = &vram[oam_tile_addr + 16..][..16];
                            let lo = oam_tile_data[tile_line as usize * 2];
                            let hi = oam_tile_data[tile_line as usize * 2 + 1];
                            (lo, hi)
                        };

                        for x in 0..8 {
                            let x_addr = x as i32 + (item.x as i32 - 8);
                            if x_addr < 0 || x_addr >= 160 {
                                continue;
                            }

                            let x = if item.oam_attrs.flip_horizontal() {
                                7 - x
                            } else {
                                x
                            };

                            let px =
                                (((tile_row_hi >> (7 - x)) & 1) << 1) |
                                (((tile_row_lo >> (7 - x)) & 1) << 0);

                            if px != 0 {
                                let rgb = Self::px2rgba(&self.object_palette_data_cache, item.oam_attrs.bg_palette(), px);
                                /*
                                const COLORS: &[u32] = &[
                                    0x0000ff, 0x000080, 0x000040,
                                    0x00ff00, 0x008000, 0x004000,
                                    0xff0000, 0x800000, 0x400000,
                                    0x00ffff, 0x00ff80, 0x00ff40,
                                    0x0080ff, 0x008080, 0x008040,
                                    0x0040ff, 0x004080, 0x004040,
                                    0xff00ff, 0xff0080, 0xff0040,
                                    0x8000ff, 0x800080, 0x800040,
                                    0x4000ff, 0x400080, 0x400040,
                                ];
                                let px = COLORS[(px as usize) % COLORS.len()].wrapping_mul(item.x as u32 + y_addr as u32);
                                */

                                self.oam_pixels[x_addr as usize] = Pixel {
                                    pixel: px,
                                    rgb,
                                    bg_priority: item.oam_attrs.bg_priority(),
                                    priority: false,
                                    _pad: 0,
                                };
//                            } else {
//                                self.oam_pixels[x_addr as usize] = Some(0xff0000);
                            }
                        }
                    }

    //                eprintln!("{} sprites to draw", self.oam_scan_items.len());
                    self.curr_background_pixel = 0;
                    let tile_base = self.background_tile_base();
                    let window_tile_base = self.window_tile_base();
//                    eprintln!("tile base: {:04x}", tile_base);
//                    eprintln!("tile y: {}.{}", self.ly / 8, self.ly % 8);
                    let background_y = scy.wrapping_add(self.ly);
                    let background_x = scx + 0;
                    let tile_y = (background_y / 8) as u16;
                    let tile_yoffs = background_y as u16 % 8;
                    // if `ly < wy` this will be `None`, and our sign that we're not drawing the
                    // window on this line.
                    // TODO: handle window ly separately given the window might not always be
                    // visible..?
                    let window_y = if self.window_enable() {
                        self.ly.checked_sub(wy)
                    } else {
                        // and if the window is not enabled, `None` here bypasses later logic
                        None
                    };
                    let window_coords = window_y.map(|window_y| {
                        ((window_y / 8) as u16, (window_y as u16 % 8))
                    });
                    for i in 0..160u8 {
                        let (line_x, tile_data, attributes) = window_coords.and_then(|(window_y, window_y_offset)| {
                            if i + 7 < wx {
                                // the window y-line was visible, but still too far left to draw
                                // it.
                                return None;
                            }

                            let window_x = i - wx + 7;
                            let window_tile_x = (window_x / 8) as u16;
                            let window_tile_nr = window_y * 32 + window_tile_x;
                            let (tile_data, attributes) = self.window_tile_lookup_by_nr(vram, window_tile_nr);
                            Some((window_x, tile_data, attributes))
                        }).unwrap_or_else(|| {
                            // NOTE: if the screen is scrolled such that x would overflow past the end
                            // of the tile map, x waps back around to the left. i think.
                            let line_x = i.wrapping_add(background_x);
                            let tile_x = (line_x / 8) as u16;
                            let tile_nr = tile_y * 32 + tile_x;
                            let (tile_data, attributes) = self.tile_lookup_by_nr(vram, tile_nr);
                            (line_x, tile_data, attributes)
                        });

                        let y_idx = if attributes.flip_vertical() {
                            7 - tile_yoffs
                        } else {
                            tile_yoffs
                        };
                        let tile_row_lo = tile_data[y_idx as usize * 2];
                        let tile_row_hi = tile_data[y_idx as usize * 2 + 1];
                        let tile_xoffs = (line_x % 8);
                        let x_idx = if attributes.flip_horizontal() {
                            7 - tile_xoffs
                        } else {
                            tile_xoffs
                        };
                        let px =
                            (((tile_row_hi >> (7 - x_idx)) & 1) << 1) |
                            (((tile_row_lo >> (7 - x_idx)) & 1) << 0);

                        /*
                        if self.ly > 42 && self.ly < 49 {
                            eprintln!("px in = {}", px);
                        }
                        */
                        let rgb = Self::px2rgba(&self.background_palette_data_cache, attributes.bg_palette(), px);
                        /*
                        if self.ly > 42 && self.ly < 49 {
                            eprintln!("px out = {} via palette {}, tile x,y=({}, {})", px, attributes.bg_palette(), tile_x, tile_y);
                        }
                        */

                        let px = Pixel {
                            pixel: px,
                            rgb,
                            bg_priority: attributes.priority(),
                            priority: true,
                            _pad: 0,
                        };

                        self.background_pixels[self.curr_background_pixel as usize] = px;
                        self.curr_background_pixel += 1;
                    }
/*
                    if self.background_pixels.iter().all(|px| px.pixel == 0) {
//                        eprintln!("clear line... {}", self.ly);
                        self.background_pixels[0] = Pixel {
                            pixel: 1,
                            rgb: 0x00_ff_00_00,
                            bg_priority: true,
                            priority: true,
                        };
                    } else {
//                        eprintln!("normal line");
                    }
*/
                    assert_eq!(self.curr_background_pixel, 160);
                }
            } else if line_time < 80 + 168 + 208 {
                if self.mode != 0 && (lcd_stat & 0b0000_1000 != 0) {
                    should_interrupt = true;
                }
                self.mode = 0;
            }
            (false, should_interrupt)
        }
    }
}

struct BankReader<'memory> {
    start: u16,
    mark: u16,
    current: u16,
    storage: &'memory dyn MemoryBanks,
}

impl<'a> BankReader<'a> {
    fn read_at(data: &'a dyn MemoryBanks, addr: u16) -> Self {
        BankReader {
            start: addr,
            mark: addr,
            current: addr,
            storage: data
        }
    }
}

impl yaxpeax_arch::Reader<u16, u8> for BankReader<'_> {
    fn next(&mut self) -> Result<u8, ReadError> {
        let b = self.storage.load(self.current);
        self.current += 1;
        Ok(b)
    }

    fn next_n(&mut self, buf: &mut [u8]) -> Result<(), ReadError> {
        for i in 0..buf.len() {
            buf[i] = self.next()?;
        }
        Ok(())
    }

    fn mark(&mut self) {
        self.mark = self.current;
    }

    fn offset(&mut self) -> u16 {
        self.current - self.mark
    }

    fn total_offset(&mut self) -> u16 {
        self.current - self.start
    }
}

struct GBC {
    frame_times: Vec<SystemTime>,
    cpu: Cpu,
    lcd: Lcd,
    sprite_debug_panel: Box<[u8; 4 * 182 * 42]>,
    apu: Apu,
    audio_sink: Option<rodio::Sink>,
    boot_rom: GBCCart,
    cart: GBCCart,
    in_boot: bool,
    ram: [u8; 32 * 1024],
    vram: [u8; 16 * 1024],
    management_bits: [u8; 0x200],
    clock: u64,
    div_apu: u64,
    next_div_tick: u64,
    next_div_apu_tick: u64,
    input_actions: u8,
    input_directions: u8,
//    audio: Rc<GBCAudio>,
    verbose: bool,
    show_sprite_debug_panel: bool,
    turbo: bool,
}

struct MemoryMapping<'system> {
    cart: &'system mut dyn MemoryBanks,
    ram: &'system mut [u8],
    vram: &'system mut [u8],
    lcd: &'system mut Lcd,
    apu: &'system mut Apu,
    management_bits: &'system mut [u8],
    verbose: bool,
    dma_requested: bool,
}

impl<'a> fmt::Debug for MemoryMapping<'a> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "MemoryMapping {{ cart: {:?}, verbose: {}, .. }}", &self.cart, &self.verbose)
    }
}

impl MemoryMapping<'_> {
    fn recursive_translate(&self, addr: u16) -> MemoryAddress {
        let res = self.translate_address(addr);
        if res.segment == SEGMENT_CART {
            let mut res = self.cart.translate_address(res.address as u16);
            res.segment |= 0x80;
            res
        } else {
            res
        }
    }
}

impl MemoryBanks for MemoryMapping<'_> {
    fn reset(&mut self) {}
    fn translate_address(&self, addr: u16) -> MemoryAddress {
        if addr < 0x4000 {
            MemoryAddress {
                segment: SEGMENT_CART,
                address: addr as u32,
            }
        } else if addr < 0x8000 {
            MemoryAddress {
                segment: SEGMENT_CART,
                address: addr as u32,
            }
        } else if addr < 0xa000 {
            let offset = 0x2000 * (self.management_bits[VBK] as usize & 0b1);
            let addr = addr as usize - 0x8000 + offset;
            MemoryAddress {
                segment: SEGMENT_VRAM,
                address: addr as u32,
            }
        } else if addr < 0xc000 {
            MemoryAddress {
                segment: SEGMENT_CART,
                address: addr as u32,
            }
        } else if addr < 0xd000 {
            let addr = addr as usize - 0xc000;
            MemoryAddress::ram(addr as u32)
        } else if addr < 0xe000 {
            let nr = self.management_bits[SVBK] as usize & 0b111;
            let nr = if nr == 0 {
                1
            } else {
                nr
            };
            let addr = (addr as usize - 0xd000) + nr * 0x1000;
            MemoryAddress::ram(addr as u32)
        } else if addr < 0xfe00 {
            // aliases [c000,ddff]
            let addr = addr as usize - 0xe000;
            MemoryAddress::ram(addr as u32)
        } else if addr < 0xfea0 {
            MemoryAddress {
                segment: SEGMENT_OAM,
                address: (addr - 0xfe00) as u32,
            }
        } else {
            let addr = addr - 0xfe00;
            MemoryAddress::management(addr as u32)
        }
    }
    fn load(&self, addr: u16) -> u8 {
        let addr = self.translate_address(addr);
        match addr {
            MemoryAddress { segment: SEGMENT_CART, address } => {
                assert!(address < 0x10000);
                self.cart.load(address as u16)
            },
            MemoryAddress { segment: SEGMENT_VRAM, address } => {
                self.vram[address as usize]
            }
            MemoryAddress { segment: SEGMENT_RAM, address } => {
                self.ram[address as usize]
            }
            MemoryAddress { segment: SEGMENT_OAM, address } => {
                self.lcd.load(address as u16)
            },
            MemoryAddress { segment: SEGMENT_MANAGEMENT, address } => {
                if address < 0x0a0 {
                    // sprite attribute table
                    self.management_bits[address as usize]
                } else if address < 0x100 {
                    // "not usable"
                    self.management_bits[address as usize]
                } else if address < 0x180 {
                    let reg = address as usize;
                    let v = if (reg >= APU_MIN_REG && reg <= APU_MAX_REG) || reg == PCM12 || reg == PCM34 {
                        self.apu.load(reg)
                    } else if reg == JOYP {
                        let mut res = self.management_bits[reg];
//                        eprintln!("reading joyp: {:08b}", res);
                        res
                    } else if reg == VBK {
                        // "Reading from this register will return the number of the currently loaded VRAM
                        // bank in bit 0, and all other bits will be set to 1."
                        (self.management_bits[VBK] & 1) | 0b1111_1110
                    } else if reg == BGPI {
                        self.management_bits[reg]
                    } else if reg == BGPD {
                        let idx = self.management_bits[BGPI] & 0x3f;
                        self.lcd.background_palettes_data[idx as usize]
                    } else if reg == OBPI {
                        self.management_bits[reg]
                    } else if reg == OBPD {
                        let idx = self.management_bits[OBPI] & 0x3f;
                        self.lcd.object_palettes_data[idx as usize]
                    } else if reg == IF {
                        let v = self.management_bits[reg];
                        if self.verbose {
                            eprintln!("getting IF=${:02x}", v);
                        }
                        v
                    } else if reg == IE {
                        let v = self.management_bits[reg];
                        if self.verbose {
                            eprintln!("getting IE=${:02x}", v);
                        }
                        v
                    } else if reg == LY {
                        self.management_bits[reg]
                    } else if reg == LCDC {
                        if self.verbose {
                            eprintln!("getting LCDC=${:02x}", self.lcd.lcdc);
                        }
                        self.lcd.lcdc
                    } else if reg == BGP {
                        self.management_bits[reg]
                    } else if reg == OBP0 {
                        self.management_bits[reg]
                    } else if reg == OBP1 {
                        self.management_bits[reg]
                    } else if reg == BANK {
                        self.management_bits[reg]
                    } else if reg == IF {
                        self.management_bits[reg]
                    } else if reg == KEY0 {
                        self.management_bits[reg]
                    } else if reg == KEY1 {
                        self.management_bits[reg]
                    } else if reg == HDMA1 {
                        0
                    } else if reg == HDMA2 {
                        0
                    } else if reg == HDMA3 {
                        0
                    } else if reg == HDMA4 {
                        0
                    } else if reg == SVBK {
                        self.management_bits[reg]
                    } else if reg == SCY {
                        self.management_bits[reg]
                    } else if reg == SCX {
                        self.management_bits[reg]
                    } else if reg == LYC {
                        self.management_bits[reg]
                    } else if reg == DIV {
                        self.management_bits[reg]
                    } else if reg == TIMA {
                        self.management_bits[reg]
                    } else if reg == TMA {
                        self.management_bits[reg]
                    } else if reg == TAC {
                        self.management_bits[reg]
                    } else if reg == STAT {
                        self.management_bits[reg]
                    } else {
                        //panic!("unhandled load {:04x}", reg);
                        let v = self.management_bits[reg];
                        if self.verbose {
                            eprintln!("get ${:04x} (=${:02x})", address, v);
                        }
                        v
                    };
                    v
                } else if address < 0x1ff {
                    // "high ram (HRAM)"
                    self.management_bits[address as usize]
                } else {
                    // "interrupt enable register"
                    self.management_bits[address as usize]
                }
            }
            other => {
                unreachable!("impossible address: {other}");
            }
        }
    }

    fn store(&mut self, addr: u16, value: u8) {
        if addr < 0x4000 {
            self.cart.store(addr, value)
        } else if addr < 0x8000 {
            self.cart.store(addr, value)
        } else if addr < 0xa000 {
            let offset = 0x2000 * (self.management_bits[VBK] as usize & 0b1);
            self.vram[addr as usize - 0x8000 + offset] = value;
        } else if addr < 0xc000 {
            self.cart.store(addr, value)
        } else if addr < 0xd000 {
            self.ram[addr as usize - 0xc000] = value;
        } else if addr < 0xe000 {
            let nr = self.management_bits[SVBK] as usize & 0b111;
            let nr = if nr == 0 {
                1
            } else {
                nr
            };
            self.ram[(addr as usize - 0xd000) + nr * 0x1000] = value;
        } else if addr < 0xfe00 {
            self.ram[addr as usize - 0xe000] = value;
        } else if addr < 0xfea0 {
            // sprite attribute table
            self.lcd.store(addr as u16 - 0xfe00, value);
        } else if addr < 0xff00 {
            // "not usable"
            self.management_bits[addr as usize - 0xfe00] = value;
        } else if addr < 0xff80 {
            // "i/o registers"
            if self.verbose {
                eprintln!("set ${:04x}=${:02x}", addr, value);
            }
            let reg = addr as usize - 0xfe00;
            if (reg >= APU_MIN_REG && reg <= APU_MAX_REG) || reg == PCM12 || reg == PCM34 {
                self.apu.store(reg, value);
            } else if reg == JOYP {
                self.management_bits[reg] &= 0b1100_1111;
                self.management_bits[reg] |= (value & 0b0011_0000);
            } else if reg == KEY1 {
                self.management_bits[reg] |= value & 0b1;
            } else if reg == LY {
                // read-only register: discard the write
            } else if reg == VBK {
                self.management_bits[reg] = value & 0b01;
            } else if reg == BGPI {
                self.management_bits[reg] = value;
            } else if reg == BGPD {
                let idx = self.management_bits[BGPI] & 0x3f;
                self.lcd.background_palettes_data[idx as usize] = value;
                self.lcd.recompute_palette_cache();
                if self.management_bits[BGPI] & 0x80 != 0 {
                    self.management_bits[BGPI] = 0x80 | ((idx + 1) & 0x3f);
                }
            } else if reg == OBPI {
                self.management_bits[reg] = value;
            } else if reg == OBPD {
                let idx = self.management_bits[OBPI] & 0x3f;
                self.lcd.object_palettes_data[idx as usize] = value;
                self.lcd.recompute_palette_cache();
                if self.management_bits[OBPI] & 0x80 != 0 {
                    self.management_bits[OBPI] = 0x80 | ((idx + 1) & 0x3f);
                }
            } else if reg == OPRI {
                self.management_bits[reg] = value;
            } else if reg == SVBK {
                self.management_bits[reg] = value & 0b111;
            } else if reg == IE {
                if self.verbose {
                    eprintln!("setting IE=${:02x}", value);
                }
                self.management_bits[reg] = value;
            } else if reg == IF {
                if self.verbose {
                    eprintln!("setting IF=${:02x}", value);
                }
                self.management_bits[reg] = value & 0x1f;
            } else if reg == LCDC {
                if self.verbose {
                    eprintln!("setting LCDC=${:02x}", value);
                }
                self.lcd.set_lcdc(value);
            } else if reg == DMA {
                let source = value as u16 * 0x100;
//                eprintln!("doing DMA from {:04x})", source);
                for i in 0..0xa0 {
                    let b = self.load(source + i);
                    self.store(0xfe00 + i, b);
                }
            } else if reg == KEY0 {
                self.management_bits[reg] = value;
            } else if reg == HDMA1 {
                self.management_bits[reg] = value;
            } else if reg == HDMA2 {
                self.management_bits[reg] = value;
            } else if reg == HDMA3 {
                self.management_bits[reg] = value;
            } else if reg == HDMA4 {
                self.management_bits[reg] = value;
            } else if reg == HDMA5 {
                let source = (self.management_bits[HDMA1] as u16) << 8 | ((self.management_bits[HDMA2] as u16 & 0xf0));
                let dest = ((((self.management_bits[HDMA3] as u16) << 8) & 0x1fff) | 0x8000) | ((self.management_bits[HDMA4] as u16 & 0xf0));
                let size = self.management_bits[HDMA5] as u16;
                if size > 0x7f {
                    panic!("TODO: hblank dma");
                }
                let size = size * 0x10 + 0x10;
                for i in 0..size {
                    self.store(dest + i, self.load(source + i));
                }
                self.management_bits[reg] = 0;
            } else if reg == SCX {
//                eprintln!("SCX set to {:02x}", value);
                self.management_bits[reg] = value;
            } else if reg == TAC {
                if value & 0xf8 != 0 {
                    eprintln!("bogus TAC value: {:02x}", value);
                }
//                eprintln!("TAC set to {:02x}", value);
                self.management_bits[reg] = value;
            } else if reg == BGP {
                self.management_bits[reg] = value;
            } else if reg == WX {
                self.management_bits[reg] = value;
            } else if reg == WY {
                self.management_bits[reg] = value;
            } else if reg == OBP0 {
                self.management_bits[reg] = value & 0b1111_1100;
            } else if reg == OBP1 {
                self.management_bits[reg] = value & 0b1111_1100;
            } else if reg == BANK {
                self.management_bits[reg] = value;
            } else if reg == STAT {
                self.management_bits[reg] = value & 0b1111_0000;
            } else if reg == LYC {
//                eprintln!("setting LYC to {:02x}", value);
                self.management_bits[reg] = value;
            } else if reg == SCY {
                self.management_bits[reg] = value;
            } else if reg == SCX {
                self.management_bits[reg] = value;
            } else if reg == DIV {
                self.management_bits[reg] = value;
            } else if reg == TIMA {
                self.management_bits[reg] = value;
            } else if reg == TMA {
                self.management_bits[reg] = value;
            } else if reg == TAC {
                self.management_bits[reg] = value;
            } else if reg == SB {
                if self.verbose {
                    eprintln!("serial transfer unhandled");
                }
            } else if reg == SC {
                if self.verbose {
                    eprintln!("serial transfer unhandled");
                }
            } else if reg == RP {
                if self.verbose {
                    eprintln!("infrared port unhandled");
                }
            } else if reg == 0x17f || reg == 0x17e {
                // super mario world 6 golden coins writes to ff7f and ff7e. bug in the game?
                // anyway, writes are discarded.
            } else {
                        panic!("unhandled write {:04x}", reg);
                self.management_bits[reg] = value;
            }
        } else if addr < 0xffff {
            // "high ram (HRAM)"
            self.management_bits[addr as usize - 0xfe00] = value;
        } else {
            // "interrupt enable register"
            self.management_bits[addr as usize - 0xfe00] = value;
        }
    }
}

enum GBState {
    PreStart,
    RomExit,
    EmulationError,
}

// Joypad input
// Bit 7 - Not used
// Bit 6 - Not used
// Bit 5 - P15 Select Action buttons    (0=Select)
// Bit 4 - P14 Select Direction buttons (0=Select)
// Bit 3 - P13 Input: Down  or Start    (0=Pressed) (Read Only)
// Bit 2 - P12 Input: Up    or Select   (0=Pressed) (Read Only)
// Bit 1 - P11 Input: Left  or B        (0=Pressed) (Read Only)
// Bit 0 - P10 Input: Right or A        (0=Pressed) (Read Only)
const JOYP: usize = 0x100;
// SB: Serial transfer data. the next  byte of serial dat that will go out.
const SB: usize = 0x101;
// SC: Serial transfer control
const SC: usize = 0x102;
// timer divider
const DIV: usize = 0x104;
// timer counter
// incremented at frequency specified by TAC, overflow fires timer interrupt
const TIMA: usize = 0x105;
// timer modulo
// when TIMA overflows, it is reset to the value in this register
const TMA: usize = 0x106;
// timer control
// Bit 2 - Timer Enable
// Bits 1-0 - Input Clock Select
//   00: CPU Clock / 1024
//   01: CPU Clock / 16
//   10: CPU Clock / 64
//   11: CPU Clock / 256
const TAC: usize = 0x107;

// interrupts are laid out as
// Bit 0: VBlank   Interrupt Enable  (INT $40)
// Bit 1: LCD STAT Interrupt Enable  (INT $48)
// Bit 2: Timer    Interrupt Enable  (INT $50)
// Bit 3: Serial   Interrupt Enable  (INT $58)
// Bit 4: Joypad   Interrupt Enable  (INT $60)
//
// interrupt flag
// 1=enable
const IF: usize = 0x10f;
// interrupt enable
// 1=request

const APU_MIN_REG: usize = 0x110;
const APU_MAX_REG: usize = 0x13f;
// Channel 1 sweep
// Bit 6-4 - Sweep pace
// Bit 3   - Sweep increase/decrease
//            0: Addition
//            1: Subtraction
// Bit 2-0 - Sweep slope control (n: 0-7)
// see pan docs for a formula connecting these together
const NR10: usize = 0x110;
// Channel 1 length timer & duty cycle
const NR11: usize = 0x111;
// Channel 1 volume & envelope
const NR12: usize = 0x112;
// Channel 1 wavelength low [write-only]
const NR13: usize = 0x113;
// Channel 1 wavelength high & control
// wavelength: write-only
// sound length enable: rw
// trigger: write-only
const NR14: usize = 0x114;
// Sound channel 2: identical to channel 1,
const NR21: usize = 0x116;
const NR22: usize = 0x117;
const NR23: usize = 0x118;
const NR24: usize = 0x119;
// Sound channel 3: wave output
const NR30: usize = 0x11a;
const NR31: usize = 0x11b;
const NR32: usize = 0x11c;
const NR33: usize = 0x11d;
const NR34: usize = 0x11e;
// Sound channel 4
const NR41: usize = 0x120;
const NR42: usize = 0x121;
const NR43: usize = 0x122;
const NR44: usize = 0x123;
// Sound channel 3 wave ram
const WAVE_RAM_START: usize = 0x130;
const WAVE_RAM_END: usize = 0x13f;
// Master volume & VIN panning
// Bit 7   - Mix VIN into left output  (1=Enable)
// Bit 6-4 - Left output volume
// Bit 3   - Mix VIN into right output (1=Enable)
// Bit 2-0 - Right output volume
const NR50: usize = 0x124;
// Bit 7: Mix channel 4 into left output
// Bit 6: Mix channel 3 into left output
// Bit 5: Mix channel 2 into left output
// Bit 4: Mix channel 1 into left output
// Bit 3: Mix channel 4 into right output
// Bit 2: Mix channel 3 into right output
// Bit 1: Mix channel 2 into right output
// Bit 0: Mix channel 1 into right output
const NR51: usize = 0x125;
// Sound on/off
// Bit 7: All sound on/off (0: turn the API off) (RW)
// Bit 3: Channel 4 ON flag (read only)
// Bit 2: Channel 3 ON flag (read only)
// Bit 1: Channel 2 ON flag (read only)
// Bit 0: Channel 1 ON flag (read only)
const NR52: usize = 0x126;
// LCD control
// Bit 7 - LCD and PPU enable                   (0=Off, 1=On)
// Bit 6 - Window tile map area
// Bit 5 - Window enable
// Bit 4 - BG and Window tile data area
// Bit 3 - BG tile map area
// Bit 2 - OBJ size
// Bit 1 - OBJ enable
// Bit 0 - BG and Window enable/priority
const LCDC: usize = 0x140;
// LCD status
// Bit 6 - LYC=LY STAT Interrupt source         (1=Enable) (Read/Write)
// Bit 5 - Mode 2 OAM STAT Interrupt source     (1=Enable) (Read/Write)
// Bit 4 - Mode 1 VBlank STAT Interrupt source  (1=Enable) (Read/Write)
// Bit 3 - Mode 0 HBlank STAT Interrupt source  (1=Enable) (Read/Write)
// Bit 2 - LYC=LY Flag                          (0=Different, 1=Equal) (Read Only)
// Bit 1-0 - Mode Flag                          (Mode 0-3, see below) (Read Only)
//           0: HBlank
//           1: VBlank
//           2: Searching OAM
//           3: Transferring Data to LCD Controller
const STAT: usize = 0x141;
// Viewport Y position,
const SCY: usize = 0x142;
// Viewport X position,
const SCX: usize = 0x143;
// LCD Y coordinate
// current horizontal line, either about to be drawn, being drawn, or just been drawn.
// can be [0, 153], where [144, 153] indicates VBlank.
const LY: usize = 0x144;
// LY Compare
// raise STAT interrupt if LYC=LY flag in STAT is set and LYC == LY
const LYC: usize = 0x145;
// OAM DMA
const DMA: usize = 0x146;
// (DMG only) background palette
const BGP: usize = 0x147;
// OBP0, OBJ palette 0
const OBP0: usize = 0x148;
// OBP0, OBJ palette 1
const OBP1: usize = 0x149;

// upper/left positions of the window area
// window Y
const WY: usize = 0x14a;
// window X
const WX: usize = 0x14b;
// KEY0. device mode indicator? written by boot rom?
const KEY0: usize = 0x14c;
// Prepare speed switch (CGB mode only)
// see gbadev pan docs
const KEY1: usize = 0x14d;
// VBK: VRAM bank (CGB mode only)
// This register can be written to change VRAM banks. Only bit 0 matters, all other bits are
// ignored.
const VBK: usize = 0x14f;
// Disables boot ROM whet to non-zero
// // Disables boot ROM whet to non-zero
const BANK: usize = 0x150;
// VRAM DMA source high
const HDMA1: usize = 0x151;
// VRAM DMA source low
const HDMA2: usize = 0x152;
// VRAM DMA dest high
const HDMA3: usize = 0x153;
// VRAM DMA dest low
const HDMA4: usize = 0x154;
// VRAM DMA length/mode/start
const HDMA5: usize = 0x155;
// RP: Infrared communications port
const RP: usize = 0x156;
// Background palette index
// Bit 7   - Auto Increment (0 = disabled, 1 = increment after write)
// Bit 5-0 - Address ($00-$3F)
const BGPI: usize = 0x168;
// Background palette data
// Bit 0-4   - Red (00-1f)
// Bit 5-9   - Green (00-1f)
// Bit 10-14 - Blue (00-1f)
const BGPD: usize = 0x169;
// Object palette index
// Note that color 0 is always transparent.
// Bit 7   - Auto Increment (0 = disabled, 1 = increment after write)
// Bit 5-0 - Address ($00-$3F)
const OBPI: usize = 0x16a;
// Object palette data
// Bit 0-4   - Red (00-1f)
// Bit 5-9   - Green (00-1f)
// Bit 10-14 - Blue (00-1f)
const OBPD: usize = 0x16b;
// OPRI: Object priority mode
// 0 for OAM priority, 1 for coordinate priority. set by CGB bios??
const OPRI: usize = 0x16c;
// SVBK: WRAM bank (CGB mode only)
// In CGB Mode 32 KBytes internal RAM are available. This memory is divided into 8 banks of 4
// KBytes each. Bank 0 is always available in memory at C000-CFFF, Bank 1-7 can be selected into
// the address space at D000-DFFF.
const SVBK: usize = 0x170;
// readout of digital outputs 1 and 2
const PCM12: usize = 0x176;
// readout of digital outputs 3 and 4
const PCM34: usize = 0x177;
// Interrupt Enable
const IE: usize = 0x1ff;

fn dump_mem_region(mem_map: &dyn MemoryBanks, start: u16, words: u16, width: u16) {
    for i in 0..(words / 2) {
        if i % width == 0 {
            eprint!("    {:04x}:", start + (i * 2));
        }
        eprint!(" {:02x}{:02x}", mem_map.load((start + i * 2 + 1) as u16), mem_map.load((start + i * 2) as u16));
        if i % width == width - 1 {
            eprintln!("");
        }
    }
}

enum Input {
    Start,
    Select,
    A,
    B,
    BankToggleBackground,
    BankToggleOam,
    VerboseToggle,
    Left, Right, Up, Down,
    RenderSpriteDebugPanelToggle,
    Reset,
    Turbo,
}

impl GBC {
    fn new(boot_rom: GBCCart) -> Self {
        Self {
            frame_times: Vec::new(),
            cpu: Cpu::new(),
            lcd: Lcd::new(),
            sprite_debug_panel: Box::new([0; 4 * 182 * 42]),
            apu: Apu::new(),
            cart: GBCCart::empty(),
            audio_sink: None,
            boot_rom,
            in_boot: true,
            ram: [0u8; 32 * 1024],
            vram: [0u8; 16 * 1024],
            management_bits: [0u8; 0x200],
            clock: 0,
            div_apu: 0,
            next_div_tick: 256,
            next_div_apu_tick: 256,
            input_actions: 0,
            input_directions: 0,
            verbose: false,
            show_sprite_debug_panel: false,
            turbo: false,
        }
    }

    fn reset(&mut self) {
        self.cpu = Cpu::new();
        self.lcd = Lcd::new();
        self.sprite_debug_panel = Box::new([0; 4 * 182 * 42]);
        self.apu = Apu::new();
        self.cart.reset();
        // DO NOT clear the audio sink out
        // self.audio_sink = None;
        self.boot_rom.reset();
        self.in_boot = true;
        self.ram = [0u8; 32 * 1024];
        self.vram = [0u8; 16 * 1024];
        self.management_bits = [0u8; 0x200];
        self.clock = 0;
        self.div_apu = 0;
        self.next_div_tick = 256;
        self.next_div_apu_tick = 256;
        self.input_actions = 0;
        self.input_directions = 0;
        self.verbose = false;
        self.show_sprite_debug_panel = false;
        self.turbo = false;
    }

    fn clear_input(&mut self) {
        self.input_directions = 0;
        self.input_actions = 0;
    }

    fn do_input(&mut self, input: Input) {
        match input {
            Input::Start => {
                self.input_actions |= 0b0000_1000;
            },
            Input::Select => {
                self.input_actions |= 0b0000_0100;
            },
            Input::B => {
                self.input_actions |= 0b0000_0010;
            },
            Input::A => {
                self.input_actions |= 0b0000_0001;
            },
            Input::BankToggleBackground => {
                self.lcd.toggle_background_sprite_bank ^= true;
            }
            Input::BankToggleOam => {
                self.lcd.toggle_oam_sprite_bank ^= true;
            }
            Input::VerboseToggle => {
                self.cpu.verbose ^= true;
                self.verbose ^= true;
            }
            Input::Down => {
                self.input_directions |= 0b0000_1000;
            },
            Input::Up => {
                self.input_directions |= 0b0000_0100;
            },
            Input::Left => {
                self.input_directions |= 0b0000_0010;
            },
            Input::Right => {
                self.input_directions |= 0b0000_0001;
            },
            Input::RenderSpriteDebugPanelToggle => {
                self.show_sprite_debug_panel ^= true;
            }
            Input::Turbo => {
                self.turbo ^= true;
            }
            Input::Reset => {
                self.reset();
            }
        }
    }

    fn set_cart(&mut self, cart: GBCCart) {
        self.cart = cart;
    }

    fn set_audio(&mut self, sink: rodio::Sink) {
        self.audio_sink = Some(sink);
    }

//    #[inline(never)]
    fn advance_clock(&mut self, clocks: u64) {
        // practically speaking this will never overflow, but still..
        let new_clock = self.clock.wrapping_add(clocks);

        let div_overshoot = new_clock as i64 - self.next_div_tick as i64;
        let _div_ticks = if div_overshoot > 0 {
            // must advance div (now figure out by how much...)
            let div_amount = (div_overshoot as u64 + 255) / 256;
            self.management_bits[DIV] = self.management_bits[DIV].wrapping_add(div_amount as u8);
            self.next_div_tick = self.next_div_tick.wrapping_add(div_amount * 256);
            div_amount
        } else {
            0
        };

        let div_apu_overshoot = new_clock as i64 - self.next_div_apu_tick as i64;
        let _div_apu_ticks = if div_apu_overshoot > 0 {
            let div_tick_rate = if self.cpu.speed_mode != 0 {
                512
            } else {
                256
            };
            // must advance div (now figure out by how much...)
            let div_amount = (div_overshoot as u64 + div_tick_rate - 1) / div_tick_rate;
            self.div_apu = self.div_apu.wrapping_add(div_amount);
            self.next_div_apu_tick = self.next_div_apu_tick.wrapping_add(div_amount * div_tick_rate);
            div_amount
        } else {
            0
        };

        if self.management_bits[TAC] & 0b0100 != 0 {
//            let tac_div = [1024, 16, 64, 256][self.management_bits[TAC] as usize & 0b11];
//            let tima_increment = (new_clock / tac_div) - (self.clock / tac_div);
            let tac_scale = [10, 4, 6, 8][self.management_bits[TAC] as usize & 0b11];
            let tima_increment = (new_clock >> tac_scale) - (self.clock >> tac_scale);

            let tima = self.management_bits[TIMA] as u16;
            let new_tima = tima + tima_increment as u16;
            if new_tima > 0xff {
                self.management_bits[IF] |= 0b00100;
                // TODO: according to SameBoy it is four cycles to reload TIMA, during which period
                // TIMA should be 0
                self.management_bits[TIMA] = self.management_bits[TMA];
            } else {
                self.management_bits[TIMA] = new_tima as u8;
            }
        }

        let system_clocks = if self.cpu.speed_mode != 0 {
            // when in Double Speed Mode, the lcd clock is maintained as if it were normal mode;
            // however many clocks we run on the cpu and timers, we run half as many on the lcd
            // clock.
            clocks / 2
        } else {
            clocks
        };

        let (vblank_int, stat_int) = self.lcd.advance_clock(&self.vram, self.management_bits[STAT], self.management_bits[LYC], system_clocks, self.management_bits[SCX], self.management_bits[SCY], self.management_bits[WX], self.management_bits[WY]);
        if vblank_int {
            if self.show_sprite_debug_panel {
                self.lcd.render_sprite_debug(&self.vram, &mut self.sprite_debug_panel);
            }
//            eprintln!("fire vblank interrupt at clock {}", self.clock);
            self.management_bits[IF] |= 0b00001;
        }
        if stat_int {
//            eprintln!("firing STAT lyc={:02x}, ly={:02x}", self.management_bits[LYC], self.lcd.ly);
            self.management_bits[IF] |= 0b00010;
        }
        self.management_bits[STAT] &= 0b1111_1100;
        self.management_bits[STAT] |= self.lcd.mode;
        self.management_bits[LY] = self.lcd.ly;
        // for gameboy doctor
//        self.management_bits[LY] = 0x90;
        self.apu.advance_clock(self.audio_sink.as_ref().clone(), system_clocks, self.turbo);

        self.clock = new_clock;
    }

    fn run(&mut self) -> u64 {
        let mut res = self.management_bits[JOYP] & 0b0011_0000;
        if self.management_bits[JOYP] & 0b0001_0000 == 0 {
            res |= self.input_directions ^ 0b0000_1111;
        }
        if self.management_bits[JOYP] & 0b0010_0000 == 0 {
            res |= self.input_actions ^ 0b0000_1111;
        }
        self.management_bits[JOYP] = res;

        let mut mem_map = MemoryMapping {
            cart: if self.in_boot {
                self.boot_rom.mapper.as_mut()
            } else {
                self.cart.mapper.as_mut()
            },
            ram: &mut self.ram,
            vram: &mut self.vram,
            lcd: &mut self.lcd,
            apu: &mut self.apu,
            management_bits: &mut self.management_bits,
            verbose: self.verbose,
            dma_requested: false,
        };

        if self.verbose {
//        if true {
            let mut reader = BankReader::read_at(&mut mem_map, self.cpu.pc);
            let decoder = yaxpeax_sm83::InstDecoder::default();

            let instr = decoder.decode(&mut reader).unwrap();
            eprintln!("pc={:#04x} {}", self.cpu.pc, instr.decorate(&self.cpu, &mem_map));
            let translated = mem_map.translate_address(self.cpu.pc);
            if translated.segment == SEGMENT_CART {
                eprintln!("  cart addr: {}", mem_map.cart.translate_address(self.cpu.pc));
            } else {
                eprintln!("  addr: {}", translated);
            }
//            eprintln!("ram (first 512b): {:?}", &mem_map.ram[0..512]);
//            eprintln!("  {:?}", instr);
        }

        let pc_before = self.cpu.pc;
        let clocks = self.cpu.step(&mut mem_map);
        if !self.in_boot && false {
            eprintln!(
                "A:{:02X} F:{:02X} B:{:02X} C:{:02X} D:{:02X} E:{:02X} H:{:02X} L:{:02X} SP:{:04X} PC:{:04X} PCMEM:{:02X},{:02X},{:02X},{:02X}",
                self.cpu.af[1],
                self.cpu.af[0],
                self.cpu.bc[1],
                self.cpu.bc[0],
                self.cpu.de[1],
                self.cpu.de[0],
                self.cpu.hl[1],
                self.cpu.hl[0],
                self.cpu.sp,
                self.cpu.pc,
                mem_map.load(self.cpu.pc),
                mem_map.load(self.cpu.pc + 1),
                mem_map.load(self.cpu.pc + 2),
                mem_map.load(self.cpu.pc + 3),
            );
        }
        if self.cpu.sp >= 0xfe00 && self.cpu.sp < 0xff80 {
            panic!("nonsense sp: ${:04x}", self.cpu.sp);
        }
        /*
        if pc_before == self.cpu.pc {
            panic!("loop detected");
        }
        */
        /*
        if self.cpu.pc == 0x1c2 {
            eprintln!("pc: {}", mem_map.translate_address(self.cpu.pc));
            eprintln!("rom addr: {}", mem_map.cart.translate_address(self.cpu.pc));
            let mut reader = BankReader::read_at(&mut mem_map, self.cpu.pc);
            let decoder = yaxpeax_sm83::InstDecoder::default();

            let instr = decoder.decode(&mut reader).unwrap();
            eprintln!("pc={:#04x} {}", self.cpu.pc, instr.decorate(&self.cpu, &mem_map));
            self.cpu.step(&mut mem_map);
//            eprintln!("{:?}", &self.cpu);
//            eprintln!("emu breakpoint");
//            self.verbose = true;
//            self.cpu.verbose = true;
        }
        */

        if self.verbose {
            eprintln!("clock: {}", self.clock);
            eprint!("{:?}", &self.cpu);
            let stack_entries = std::cmp::min(std::cmp::max(32, 0x10000 - self.cpu.sp as u32), 64);
            let end = std::cmp::min(0x10000, self.cpu.sp as u32 + stack_entries);
            let start = end - stack_entries - 0x10;
            for i in 0..(stack_entries / 2) {
                if i % 8 == 0 {
                    eprint!("    {:04x}:", start + (i * 2));
                }
                eprint!(" {:02x}{:02x}", mem_map.load((start + i * 2 + 1) as u16), mem_map.load((start + i * 2) as u16));
                if i % 8 == 7 {
                    eprintln!("");
                }
            }
        }

        if self.in_boot {
            let boot_rom_disable = mem_map.load(0xff50);
            if boot_rom_disable != 0 {
                eprintln!("boot rom complete, switching to cart");
//                self.verbose = true;
//                self.cpu.verbose = true;
                self.in_boot = false;
            }
        }

        self.advance_clock(clocks as u64);

        clocks as u64
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
enum MemoryBankControllerType {
    MBC1,
    MBC2,
    MMM01,
    MBC3,
    MBC4,
    MBC5,
    None,
}

impl MemoryBankControllerType {
    fn make_mapper(&self, ram_style: RamStyle, rom_image: Box<[u8]>, mut save_file: Option<File>) -> Box<dyn MemoryBanks + Send> {
        match self {
            MemoryBankControllerType::MBC1 => {
                let mut ram = match ram_style {
                    RamStyle::None => Vec::new().into_boxed_slice(),
                    RamStyle::Flat2kb => vec![0u8; 2 * 1024].into_boxed_slice(),
                    RamStyle::Flat8kb => vec![0u8; 8 * 1024].into_boxed_slice(),
                    RamStyle::Banked4x8kb => vec![0u8; 32 * 1024].into_boxed_slice(),
                };

                if let Some(f) = save_file.as_mut() {
                    if f.metadata().expect("can stat").len() != 0 {
                        f.read_exact(&mut ram).expect("read succeeds");
                    } else {
                        f.set_len(ram.len() as u64).expect("can size save file");
                        eprintln!("formatting save to {}KiB", ram.len() / 1024);
                    }
                }

                Box::new(MBC1 {
                    ram_enable: 0u8,
                    rom_bank: 0u8,
                    ram_bank: 0u8,
                    bank_mode_select: 0u8,
                    rom: rom_image,
                    ram: ram,
                    save_file,
                })
            }
            MemoryBankControllerType::MBC3 => {
                let mut ram = match ram_style {
                    RamStyle::None => Vec::new().into_boxed_slice(),
                    RamStyle::Flat2kb => vec![0u8; 2 * 1024].into_boxed_slice(),
                    RamStyle::Flat8kb => vec![0u8; 8 * 1024].into_boxed_slice(),
                    RamStyle::Banked4x8kb => vec![0u8; 32 * 1024].into_boxed_slice(),
                };

                if let Some(f) = save_file.as_mut() {
                    if f.metadata().expect("can stat").len() != 0 {
                        f.read_exact(&mut ram).expect("read succeeds");
                    } else {
                        f.set_len(ram.len() as u64).expect("can size save file");
                        eprintln!("formatting save to {}KiB", ram.len() / 1024);
                    }
                }

                Box::new(MBC3 {
                    ram_enable: 0u8,
                    rom_bank: 0u8,
                    ram_bank: 0u8,
                    rom: rom_image,
                    ram: ram,
                    rtc: [0u8; 5],
                    save_file,
                })
            },
            MemoryBankControllerType::MBC5 => {
                let mut ram = match ram_style {
                    RamStyle::None => Vec::new().into_boxed_slice(),
                    RamStyle::Flat2kb => vec![0u8; 4 * 2 * 1024].into_boxed_slice(),
                    RamStyle::Flat8kb => vec![0u8; 4 * 8 * 1024].into_boxed_slice(),
                    RamStyle::Banked4x8kb => vec![0u8; 4 * 32 * 1024].into_boxed_slice(),
                };

                if let Some(f) = save_file.as_mut() {
                    if f.metadata().expect("can stat").len() != 0 {
                        f.read_exact(&mut ram).expect("read succeeds");
                    } else {
                        f.set_len(ram.len() as u64).expect("can size save file");
                        eprintln!("formatting save to {}KiB", ram.len() / 1024);
                    }
                }

                Box::new(MBC5 {
                    ram_enable: 0u8,
                    rom_bank: [0u8; 2],
                    ram_bank: 0u8,
                    rom: rom_image,
                    ram: ram,
                    save_file,
                })
            },
            MemoryBankControllerType::None => {
                Box::new(FlatMapper::new(rom_image))
            }
            other => {
                panic!("unsupported mbc type: {:?}", other)
            }
        }
    }
}

#[derive(Debug, Copy, Clone)]
struct CartridgeFeatures {
    mbc: MemoryBankControllerType,
    ram: bool,
    memory_layout: Option<RamStyle>,
    battery: bool,
    timer: bool,
    rumble: bool,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum RamStyle {
    None,
    Flat2kb,
    Flat8kb,
    Banked4x8kb,
}

impl Default for CartridgeFeatures {
    fn default() -> Self {
        Self {
            mbc: MemoryBankControllerType::None,
            ram: false,
            memory_layout: None,
            battery: false,
            timer: false,
            rumble: false,
        }
    }
}

// TODO: support cartridge type 0xfc, 0xfd, 0xfe, 0xff?
fn parse_features(features: u8) -> Option<CartridgeFeatures> {
    match features {
        0x00 => Some(CartridgeFeatures { mbc: MemoryBankControllerType::None, ..Default::default() }),
        0x01 => Some(CartridgeFeatures { mbc: MemoryBankControllerType::MBC1, ..Default::default() }),
        0x02 => Some(CartridgeFeatures { mbc: MemoryBankControllerType::MBC1, ram: true, ..Default::default() }),
        0x03 => Some(CartridgeFeatures { mbc: MemoryBankControllerType::MBC1, ram: true, battery: true, ..Default::default() }),

        0x05 => Some(CartridgeFeatures { mbc: MemoryBankControllerType::MBC2, ..Default::default() }),
        0x06 => Some(CartridgeFeatures { mbc: MemoryBankControllerType::MBC2, battery: true, ..Default::default() }),

        0x08 => Some(CartridgeFeatures { mbc: MemoryBankControllerType::None, ram: true, ..Default::default() }),
        0x09 => Some(CartridgeFeatures { mbc: MemoryBankControllerType::None, ram: true, battery: true, ..Default::default() }),

        0x0b => Some(CartridgeFeatures { mbc: MemoryBankControllerType::MMM01, ..Default::default() }),
        0x0c => Some(CartridgeFeatures { mbc: MemoryBankControllerType::MMM01, ram: true, ..Default::default() }),
        0x0d => Some(CartridgeFeatures { mbc: MemoryBankControllerType::MMM01, ram: true, battery: true, ..Default::default() }),

        0x0f => Some(CartridgeFeatures { mbc: MemoryBankControllerType::MBC3, timer: true, battery: true, ..Default::default() }),
        0x10 => Some(CartridgeFeatures { mbc: MemoryBankControllerType::MBC3, timer: true, ram: true, battery: true, ..Default::default() }),
        0x11 => Some(CartridgeFeatures { mbc: MemoryBankControllerType::MBC3, ..Default::default() }),
        0x12 => Some(CartridgeFeatures { mbc: MemoryBankControllerType::MBC3, ram: true, ..Default::default() }),
        0x13 => Some(CartridgeFeatures { mbc: MemoryBankControllerType::MBC3, ram: true, battery: true, ..Default::default() }),

        0x15 => Some(CartridgeFeatures { mbc: MemoryBankControllerType::MBC4, ..Default::default() }),
        0x16 => Some(CartridgeFeatures { mbc: MemoryBankControllerType::MBC4, ram: true, ..Default::default() }),
        0x17 => Some(CartridgeFeatures { mbc: MemoryBankControllerType::MBC4, ram: true, battery: true, ..Default::default() }),

        0x19 => Some(CartridgeFeatures { mbc: MemoryBankControllerType::MBC5, ..Default::default() }),
        0x1a => Some(CartridgeFeatures { mbc: MemoryBankControllerType::MBC5, ram: true, ..Default::default() }),
        0x1b => Some(CartridgeFeatures { mbc: MemoryBankControllerType::MBC5, ram: true, battery: true, ..Default::default() }),
        0x1c => Some(CartridgeFeatures { mbc: MemoryBankControllerType::MBC5, rumble: true, ..Default::default() }),
        0x1d => Some(CartridgeFeatures { mbc: MemoryBankControllerType::MBC5, rumble: true, ram: true, ..Default::default() }),
        0x1e => Some(CartridgeFeatures { mbc: MemoryBankControllerType::MBC5, rumble: true, ram: true, battery: true, ..Default::default() }),

        _ => None,
    }
}

struct GBCCart {
    features: CartridgeFeatures,
    mapper: Box<dyn MemoryBanks + Send>,
}

impl GBCCart {
    fn reset(&mut self) {
        self.mapper.reset();
    }
}

#[derive(Copy, Clone, PartialEq)]
struct MemoryAddress {
    segment: u8,
    address: u32,
}

impl fmt::Display for MemoryAddress {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "MemoryAddress {{ segment: {}, address: {:#04x} }}",
            self.segment,
            self.address
        )
    }
}

impl MemoryAddress {
    fn ram(address: u32) -> Self {
        MemoryAddress {
            segment: SEGMENT_RAM,
            address,
        }
    }
    fn rom(address: u32) -> Self {
        MemoryAddress {
            segment: SEGMENT_ROM,
            address,
        }
    }
    fn management(address: u32) -> Self {
        MemoryAddress {
            segment: SEGMENT_MANAGEMENT,
            address,
        }
    }
    fn segment_name(&self) -> &'static str {
        match self.segment {
            SEGMENT_ROM => "rom",
            SEGMENT_RAM => "ram",
            SEGMENT_MANAGEMENT => "management",
            SEGMENT_VRAM => "vram",
            SEGMENT_CART => "cart",
            SEGMENT_OAM => "oam",
            SEGMENT_RTC => "rtc",
            SEGMENT_CART_ROM => "cart:rom",
            SEGMENT_CART_RAM => "cart:ram",
            SEGMENT_CART_MANAGEMENT => "cart:management",
            SEGMENT_CART_VRAM => "cart:vram",
            SEGMENT_CART_CART => "cart:cart",
            SEGMENT_CART_OAM => "cart:oam",
            SEGMENT_CART_RTC => "cart:rtc",
            _other => "other",
        }
    }
}

const SEGMENT_ROM: u8 = 0;
const SEGMENT_RAM: u8 = 1;
const SEGMENT_MANAGEMENT: u8 = 2;
const SEGMENT_VRAM: u8 = 3;
const SEGMENT_CART: u8 = 4;
const SEGMENT_OAM: u8 = 5;
const SEGMENT_RTC: u8 = 6;
const SEGMENT_CART_ROM: u8 = 0x80;
const SEGMENT_CART_RAM: u8 = 0x81;
const SEGMENT_CART_MANAGEMENT: u8 = 0x82;
const SEGMENT_CART_VRAM: u8 = 0x83;
const SEGMENT_CART_CART: u8 = 0x84;
const SEGMENT_CART_OAM: u8 = 0x85;
const SEGMENT_CART_RTC: u8 = 0x86;

trait MemoryBanks: fmt::Debug {
    fn load(&self, addr: u16) -> u8;
    fn store(&mut self, addr: u16, value: u8);
    fn translate_address(&self, addr: u16) -> MemoryAddress;
    fn reset(&mut self);
}

#[derive(Debug)]
struct MBC1 {
    ram_enable: u8,
    rom_bank: u8,
    ram_bank: u8,
    bank_mode_select: u8,
    rom: Box<[u8]>,
    ram: Box<[u8]>,
    save_file: Option<File>,
}

impl MemoryBanks for MBC1 {
    fn reset(&mut self) {
        self.rom_bank = 0;
        self.ram_bank = 0;
        self.bank_mode_select = 0;
        for i in 0..self.ram.len() {
            self.ram[i] = 0;
        }
        if let Some(f) = self.save_file.as_mut() {
            f.seek(SeekFrom::Start(0 as u64)).expect("can seek appropriately");
            f.read_exact(&mut self.ram).expect("read succeeds");
        }
    }

    fn load(&self, addr: u16) -> u8 {
        let addr = self.translate_address(addr);
        match addr {
            MemoryAddress { segment: SEGMENT_ROM, address } => {
                self.rom[address as usize]
            }
            MemoryAddress { segment: SEGMENT_RAM, address } => {
                self.ram[address as usize]
            }
            other => {
                panic!("invalid address? {other}");
            }
        }
    }

    fn translate_address(&self, addr: u16) -> MemoryAddress {
        if addr <= 0x3fff {
            if self.bank_mode_select == 0 {
                MemoryAddress::rom(addr as u32)
            } else {
                let bank = if self.rom.len() <= 512 * 1024 {
                    0
                } else {
                    self.ram_bank as usize * 16
                };
                let addr = addr as usize + 0x4000 * bank;
                MemoryAddress::rom(addr as u32)
            }
        } else if addr <= 0x7fff {
            let bank_low = if self.rom_bank == 0 {
                1
            } else {
                self.rom_bank as usize
            };
            let bank_high = if self.rom.len() <= 512 * 1024 {
                0
            } else {
                self.ram_bank as usize * 16
            };
            let bank = bank_low + bank_high;
            let addr = addr as usize - 0x4000 + bank * 0x4000;
            MemoryAddress::rom(addr as u32)
        } else if addr < 0xa000 {
            eprintln!("bad cart access at {:#04x}", addr);
            MemoryAddress::rom(0)
        } else if addr < 0xc000 {
            let bank = if self.ram.len() <= 8 * 1024 {
                0
            } else {
                self.ram_bank as usize
            };
            let addr = addr as usize - 0xa000 + bank * 0x2000;
            MemoryAddress::ram(addr as u32)
        } else {
            eprintln!("bad cart access at {:#04x}", addr);
            MemoryAddress::rom(0)
        }
    }
    fn store(&mut self, addr: u16, value: u8) {
        if addr < 0xa000 {
            let reg_bits = addr >> 12;
            match reg_bits {
                0 | 1 => { self.ram_enable = value; },
                2 | 3 => {
//                    eprintln!("updating rom bank to {:#08x}", value);
                    self.rom_bank = value & 0b000_11111; },
                4 | 5 => {
//                    eprintln!("updating ram bank to {:#08x}", value);
                    self.ram_bank = value; },
                6 | 7 => { self.bank_mode_select = value & 1; },
                _ => { panic!("mbc1 bad mapper register bits ({:04b})", reg_bits) },
            }
        } else if addr < 0xc000 {
            if self.ram_enable & 0x0f != 0x0a {
                return;
            }
            let addr = self.translate_address(addr);
            debug_assert!(addr.segment == SEGMENT_RAM);
            self.ram[addr.address as usize] = value;
            if let Some(f) = self.save_file.as_mut() {
                f.seek(SeekFrom::Start(addr.address as u64)).expect("can seek appropriately");
                f.write_all(&[value]).expect("can store data");
                f.flush().expect("can flush");
            }
        } else {
            eprintln!("bad cart access at {:#04x}", addr);
        }
    }
}

#[derive(Debug)]
struct MBC3 {
    ram_enable: u8,
    rom_bank: u8,
    ram_bank: u8,
    rom: Box<[u8]>,
    ram: Box<[u8]>,
    rtc: [u8; 5],
    save_file: Option<File>,
}

impl MemoryBanks for MBC3 {
    fn reset(&mut self) {
        self.rom_bank = 0;
        self.ram_bank = 0;
        self.ram_enable = 0;
        for i in 0..self.ram.len() {
            self.ram[i] = 0;
        }
        if let Some(f) = self.save_file.as_mut() {
            f.seek(SeekFrom::Start(0 as u64)).expect("can seek appropriately");
            f.read_exact(&mut self.ram).expect("read succeeds");
        }
    }

    fn load(&self, addr: u16) -> u8 {
        let addr = self.translate_address(addr);
        match addr {
            MemoryAddress { segment: SEGMENT_ROM, address } => {
                self.rom[address as usize]
            }
            MemoryAddress { segment: SEGMENT_RAM, address } => {
                if self.ram_enable & 0x0f != 0x0a {
                    return 0;
                }
                self.ram[address as usize]
            }
            MemoryAddress { segment: SEGMENT_RTC, address } => {
//                eprintln!("read rtc word {}", address);
                if self.ram_enable & 0x0f != 0x0a {
                    return 0;
                }
                self.rtc[address as usize]
            }
            other => {
                panic!("invalid address? {other}");
            }
        }
    }

    fn translate_address(&self, addr: u16) -> MemoryAddress {
        if addr <= 0x3fff {
            MemoryAddress::rom(addr as u32)
        } else if addr <= 0x7fff {
            let bank = self.rom_bank as usize;
            let bank = if bank == 0 {
                1
            } else {
                bank
            };
            let addr = addr as usize - 0x4000 + bank * 0x4000;
            MemoryAddress::rom(addr as u32)
        } else if addr < 0xa000 {
            eprintln!("bad cart access at {:#04x}", addr);
            MemoryAddress::rom(0)
        } else if addr < 0xc000 {
            let bank = self.ram_bank as usize;
            if bank < 4 {
                let addr = addr as usize - 0xa000 + bank * 0x2000;
                MemoryAddress::ram(addr as u32)
            } else {
                MemoryAddress {
                    segment: SEGMENT_RTC,
                    address: bank as u32 - 0x08,
                }
            }
        } else {
            eprintln!("bad cart access at {:#04x}", addr);
            MemoryAddress::rom(0)
        }
    }
    fn store(&mut self, addr: u16, value: u8) {
        if addr < 0xa000 {
            let reg_bits = addr >> 12;
            match reg_bits {
                0 | 1 => { self.ram_enable = value; },
                2 | 3 => { self.rom_bank = if value == 0 { 1 } else { value }; },
                4 | 5 => {
                    if value <= 3 || (value >= 0x8 && value <= 0xc) {
                        self.ram_bank = value;
                    } else {
                        eprintln!("TODO: tried to select invalid ram bank, defaulting to bank 0");
                        self.ram_bank = 0;
                    }
                },
                _ => {
//                    eprintln!("TODO: latch clock data");
                    self.rtc = [59, 59, 23, 59, 0];
               //     panic!("latch clock data")
                },
            }
        } else if addr < 0xc000 {
            if self.ram_enable & 0x0f != 0x0a {
                return;
            }
            let addr = self.translate_address(addr);
            if addr.segment == SEGMENT_RAM {
                self.ram[addr.address as usize] = value;
                if let Some(f) = self.save_file.as_mut() {
                    f.seek(SeekFrom::Start(addr.address as u64)).expect("can seek appropriately");
                    f.write_all(&[value]).expect("can store data");
                    f.flush().expect("can flush");
                }
            } else {
                debug_assert!(addr.segment == SEGMENT_RTC);
                // TODO: accurate RTC semantics (respect halt bit, do something about writes with
                // halt bit clear, etc
                self.rtc[addr.address as usize] = value;
            }
        } else {
            eprintln!("bad cart access at {:#04x}", addr);
        }
    }
}

struct MBC5 {
    ram_enable: u8,
    rom_bank: [u8; 2],
    ram_bank: u8,
    rom: Box<[u8]>,
    ram: Box<[u8]>,
    save_file: Option<File>,
}

impl fmt::Debug for MBC5 {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "mbc5 state: ram_enable? {}, rom_bank: {:04x}, ram_bank: {:02x}", self.ram_enable, u16::from_le_bytes(self.rom_bank), self.ram_bank)
    }
}

impl MemoryBanks for MBC5 {
    fn reset(&mut self) {
        self.rom_bank = [0, 0];
        self.ram_bank = 0;
        self.ram_enable = 0;
        for i in 0..self.ram.len() {
            self.ram[i] = 0;
        }
        if let Some(f) = self.save_file.as_mut() {
            f.seek(SeekFrom::Start(0 as u64)).expect("can seek appropriately");
            f.read_exact(&mut self.ram).expect("read succeeds");
        }
    }

    fn load(&self, addr: u16) -> u8 {
        let addr = self.translate_address(addr);
        match addr {
            MemoryAddress { segment: SEGMENT_ROM, address } => {
                self.rom[address as usize]
            }
            MemoryAddress { segment: SEGMENT_RAM, address } => {
                self.ram[address as usize]
            }
            other => {
                panic!("invalid address? {other}");
            }
        }
    }

    fn translate_address(&self, addr: u16) -> MemoryAddress {
        if addr <= 0x3fff {
            MemoryAddress::rom(addr as u32)
        } else if addr <= 0x7fff {
            let bank = u16::from_le_bytes(self.rom_bank) as usize;
            let bank = bank & ((self.rom.len() / 0x4000) - 1);
            let addr = addr as usize - 0x4000 + bank * 0x4000;
            MemoryAddress::rom(addr as u32)
        } else if addr < 0xa000 {
            eprintln!("bad cart access at {:#04x}", addr);
            MemoryAddress::rom(0)
        } else if addr < 0xc000 {
            let bank = self.ram_bank as usize;
            let bank = bank & ((self.ram.len() / 0x2000) - 1);
            let addr = addr as usize - 0xa000 + bank * 0x2000;
            MemoryAddress::ram(addr as u32)
        } else {
            eprintln!("bad cart access at {:#04x}", addr);
            MemoryAddress::rom(0)
        }
    }
    fn store(&mut self, addr: u16, value: u8) {
        if addr < 0xa000 {
            let reg_bits = addr >> 12;
            match reg_bits {
                0 | 1 => { self.ram_enable = value; },
                2 => { self.rom_bank[0] = value; },
                3 => { self.rom_bank[1] = value; },
                _ => { self.ram_bank = value; },
            }
        } else if addr < 0xc000 {
            if self.ram_enable & 0x0f != 0x0a {
                return;
            }
            let addr = self.translate_address(addr);
            debug_assert!(addr.segment == SEGMENT_RAM);
            self.ram[addr.address as usize] = value;
            if let Some(f) = self.save_file.as_mut() {
                f.seek(SeekFrom::Start(addr.address as u64)).expect("can seek appropriately");
                f.write_all(&[value]).expect("can store data");
                f.flush().expect("can flush");
            }
        } else {
            eprintln!("bad cart access at {:#04x}", addr);
        }
    }
}

struct FlatMapper {
    rom: Box<[u8]>,
}

impl fmt::Debug for FlatMapper {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "FlatMapper {{ <{} bytes> }}", self.rom.len())
    }
}

impl FlatMapper {
    fn new(data: Box<[u8]>) -> Self {
        FlatMapper { rom: data }
    }
}

impl MemoryBanks for FlatMapper {
    fn reset(&mut self) {}
    fn load(&self, addr: u16) -> u8 {
        let addr = self.translate_address(addr);
        self.rom[addr.address as usize]
    }
    fn translate_address(&self, addr: u16) -> MemoryAddress {
        MemoryAddress::rom(addr as u32)
    }
    fn store(&mut self, _addr: u16, _value: u8) {}
}

impl GBCCart {
    fn empty() -> Self {
        Self::raw(Vec::new())
    }

    fn boot_rom(mut boot_rom: Vec<u8>, cart: &GBCCart) -> Self {
        while boot_rom.len() < 512 {
            boot_rom.push(0);
        }
        // copy in the bytes from $104 to $200
        for i in 0x104..0x200 {
            boot_rom[i] = cart.mapper.load(i as u16);
        }

        GBCCart::raw(boot_rom)
    }

    fn raw(data: Vec<u8>) -> Self {
        Self {
            features: CartridgeFeatures::default(),
            mapper: Box::new(FlatMapper::new(data.into_boxed_slice())),
        }
    }

    fn new(data: Vec<u8>, ram: Option<File>) -> Result<Self, String> {
        let features = parse_features(data[0x147]);

        let size = (32 * 1024) << data[0x148];
        eprintln!("ROM reports size {}", size);
        if data.len() != size {
            eprintln!("actual size did not match: {}", data.len());
            return Err("wrong rom size?".to_string());
        }

        let mut features = features.expect("features are known");

        let ram_style = match data[0x149] {
            0x00 => RamStyle::None,
            0x01 => RamStyle::Flat2kb,
            0x02 => RamStyle::Flat8kb,
            0x03 => RamStyle::Banked4x8kb,
            other => { panic!("unknown ram style {:#02x}", other); }
        };

        features.memory_layout = Some(ram_style);
        if features.memory_layout.is_some() && !(features.mbc == MemoryBankControllerType::MBC2 || features.ram) {
            eprintln!("rom reports memory when cartridge format reports no memory?");
        }

        let mapper = features.mbc.make_mapper(ram_style, data.into_boxed_slice(), ram);

        eprintln!("cartridge type {:#02x}, features: {:?}", mapper.load(0x147), features);

        Ok(Self { features, mapper })
    }
}

mod test {
    use super::*;

    fn with_test_mapping<U>(f: impl Fn(MemoryMapping) -> U) -> U {
        let mut rom_data = Vec::new();
        for i in 1..129 {
            rom_data.extend_from_slice(&[i; 1024]);
        }
        let mut rom = MBC5 {
            ram_enable: 0u8,
            rom_bank: [0u8; 2],
            ram_bank: 0u8,
            rom: rom_data.into_boxed_slice(),
            ram: vec![0u8; 32 * 1024].into_boxed_slice(),
            save_file: None,
        };
        let mut wram = [0u8; 32 * 1024];
        let mut vram = [0u8; 16 * 1024];
        let mut lcd = crate::Lcd::new();
        let mut apu = crate::Apu::new();
        let mut management_bits = [0u8; 0x200];

        let mut memory = MemoryMapping {
            cart: &mut rom,
            ram: &mut wram,
            vram: &mut vram,
            lcd: &mut lcd,
            apu: &mut apu,
            management_bits: &mut management_bits,
            verbose: false,
            dma_requested: false,
        };

        f(memory)
    }

    #[test]
    fn test_address_translation() {
        with_test_mapping(|mut memory| {
            memory.ram[0x000_000] = 0x00;
            assert_eq!(memory.load(0xc000), 0x00);
            memory.ram[0x000_000] = 0xaa;
            assert_eq!(memory.load(0xc000), 0xaa);

            memory.ram[0x000_001] = 0xff;
            memory.store(0xc001, 0xab);
            assert_eq!(memory.ram[0x000_001], 0xab);

            memory.ram[0x001_001] = 0xff;
            memory.store(0xd001, 0xab);
            assert_eq!(memory.ram[0x001_001], 0xab);

            memory.management_bits[SVBK] = 0x01;
            assert_eq!(memory.load(0xff70), 0x01);
            memory.store(0xff70, 0x03);
            assert_eq!(memory.management_bits[SVBK], 0x03);

            assert_ne!(memory.load(0xd001), 0xab);

            memory.ram[0x003_001] = 0xff;
            memory.store(0xd001, 0xab);
            assert_eq!(memory.ram[0x003_001], 0xab);
        })
    }

    #[test]
    fn test_rom_access() {
        with_test_mapping(|mut memory| {
            assert_eq!(memory.load(0x03ff), 1);
            assert_eq!(memory.load(0x0400), 2);
            assert_eq!(memory.load(0x3fff), 16);

            memory.store(0x2000, 1);
            assert_eq!(memory.load(0x4001), 17);

            memory.store(0x2000, 4);
            assert_eq!(memory.load(0x4001), 65);

            memory.store(0x2000, 0);
            assert_eq!(memory.load(0x4001), 1);
        })
    }
}
