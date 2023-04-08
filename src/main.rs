use clap::Parser;

use std::fmt;
use std::sync::Arc;
use std::sync::Mutex;
use std::fmt::Write;
use std::time::Duration;
use std::time::SystemTime;

use yaxpeax_arch::{Decoder, ReadError};

mod cpu;
use crate::cpu::Cpu;
use crate::cpu::DecorateExt;

#[derive(Parser)]
#[clap(about, version, author)]
struct Args {
    file: String
}

fn do_ui(gb_state: Arc<Mutex<GBC>>) {
    loop {
        let mut gb = gb_state.lock().unwrap();
        // paint screen
        let mut screen = String::new();

        // dump tile data as just some kind of guess...
        println!("!!!frame!!!");
        println!("ie: {}", gb.management_bits[IE]);
        println!("if: {}", gb.management_bits[IF]);
        println!("scy: {}", gb.management_bits[SCY]);
        println!("scx: {}", gb.management_bits[SCX]);
        println!("lcdc: {:08b}", gb.lcd.lcdc);
        println!("stat: {:08b}", gb.management_bits[STAT]);
        let vbk = gb.management_bits[VBK];
        let tilemap_base = gb.lcd.background_tile_base() as usize;
        println!("vbk: {:08b}", vbk);
        let vram = &gb.vram;
        for i in 0..18 {
            for j in 0..20 {
                print!(" {:02x}", vram[(i * 32 + j) + tilemap_base]);
            }
            println!("");
        }
        println!("and attrs:");
        for i in 0..18 {
            for j in 0..20 {
                print!(" {:02x}", vram[(i * 32 + j) + tilemap_base + 0x2000]);
            }
            println!("");
        }
        println!("");

        // eprintln!("{:?}", &vram[0..0x1800]);

        /*
        if gb.lcd.window_enable() {
            for i in 0..1024 {
                // .. try windows too
                let tile_base = gb.lcd.window_tile_base();
                if i % 32 == 0 {
                    eprintln!("");
                }
                eprint!("{:02x}", vram[tile_base as usize + i]);
            }
            eprintln!("");
        }
        */
        const PXMAP: [&'static str; 16] = [
            ".", "▖", "▗", "▄", "▘", "▌", "▚", "▙", "▝", "▞", "▐", "▟", "▀", "▛", "▜", "▓",
        ];

        for i in 0..72 {
            let i = i * 2;
            for j in 0..80 {
                let j = j * 2;
                let ul = (gb.lcd.display[((i + 0) * 160) + j + 0] > 0) as u8 * 4;
                let ur = (gb.lcd.display[((i + 0) * 160) + j + 1] > 0) as u8 * 8;
                let ll = (gb.lcd.display[((i + 1) * 160) + j + 0] > 0) as u8 * 1;
                let lr = (gb.lcd.display[((i + 1) * 160) + j + 1] > 0) as u8 * 2;
                let idx = ur | ul | lr | ll;
                write!(screen, "{}", PXMAP[idx as usize]);
            }
            write!(screen, "\n");
        }
        /*
        for i in 0..144 {
            for j in 0..160 {
                write!(screen, "{}", [".", "+", "*", "#"][gb.lcd.display[(i * 160 + j) as usize] as usize]);
            }
            write!(screen, "\n");
        }
        */
        println!("{}", screen);
        std::mem::drop(gb);
        std::thread::sleep(std::time::Duration::from_millis(33));
    }
}

fn main() {
    let args = Args::parse();

    let rom = std::fs::read(&args.file).unwrap();

    let cart = GBCCart::new(rom).unwrap();

    let bootrom = GBCCart::boot_rom(std::fs::read("cgb_boot.bin").unwrap(), &cart);

    let mut gb = GBC::new(bootrom);

    gb.set_cart(cart);

    let sins = Arc::new(Mutex::new(gb));

    let sins_ref = Arc::clone(&sins);

    std::thread::spawn(|| {
        do_ui(sins_ref)
    });

    let mut i = 0;
    loop {
        let frame_target = SystemTime::now() + Duration::from_millis(16);

        let mut gb = sins.lock().unwrap();
        let vblank_before = gb.management_bits[IF] & 1;

        gb.run();

        let vblank_after = gb.management_bits[IF] & 1;
        std::mem::drop(gb);

        let vblank_fired = vblank_before == 0 && vblank_after == 1;

        let now = SystemTime::now();

//        if (vblank_fired && now < frame_target) || (i % 655360 == 655300) {
//            std::thread::sleep(frame_target.duration_since(now).unwrap());
//            i = 0;
//        }
        if i % 4 == 0 {
            std::thread::sleep(Duration::from_micros(1));
        }
        i += 1;
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
    // when, in dots, we'll be at the next line. this is the end of the current line's HBlank.
    next_line: u64,
    next_draw_time: u64,
    oam_scan_items: Vec<OamItem>,
    oam: [u8; 0xa0],
    background_pixels: Vec<u8>,
    display: Box<[u8; 144 * 160]>
}

struct OamItem {
    selected_line: u8,
    x: u8,
    tile_index: u8,
    oam_flags: u8,
}

impl Lcd {
    const LINE_TIME: u64 = 376;
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
            next_line: Self::LINE_TIME,
            next_draw_time: Self::SCREEN_TIME,
            oam_scan_items: Vec::new(),
            oam: [0u8; 0xa0],
            background_pixels: Vec::new(),
            display: Box::new([0u8; 144 * 160]),
        }
    }

    fn tile_lookup_by_nr<'a>(&self, vram: &'a [u8], tile_nr: u16) -> &'a [u8] {
        // look up the tile number in vram bank 0
        // then attrs for that tile number in vram bank 1
        //
        // tile attrs can pick between vram banks 0 and 1 for tile data itself, so we need both to
        // find the right data.
        let tile_map_base = self.background_tile_base() as usize;
        let tile_id = vram[tile_map_base + tile_nr as usize];
        let tile_attrs = vram[tile_map_base + tile_nr as usize + 0x2000];

        let mut tile_data_addr = self.tile_addr_translate(tile_id);

        if tile_attrs & 0b0000_1000 != 0 {
            tile_data_addr += 0x2000;
        }

        &vram[tile_data_addr..][..16]
    }
    fn tile_addr_translate<'a>(&self, tile_id: u8) -> usize {
        let data_offset = if self.lcdc & 0b0001_0000 == 0 {
            let addr = (((tile_id as i8).wrapping_add(-0x80)) as u8 as u16 * 16);
            0x800 + addr as usize
        } else {
            tile_id as usize * 16
        };
        data_offset
    }

    fn background_tile_base(&self) -> u16 {
        if self.lcdc & 0b0000_1000 == 0 {
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
            eprintln!("TODO: allowed to set lcdc bits outside mode 1?");
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
        self.oam[address as usize] = value
    }

    // advance lcd state by `clocks` ticks, using `lcd_stat` to determine if we should generate an
    // interrupt.
    fn advance_clock(&mut self, vram: &[u8], lcd_stat: u8, lyc: u8, clocks: u64, scx: u8, scy: u8) -> (bool, bool) {
        // the number of dots (LCD clocks) to display one line. `HBlank` is whatever time is
        // necessary to meet this time, after completing mode 3.
        self.lcd_clock = self.lcd_clock.wrapping_add(clocks);
        if clocks > 40 {
            panic!("update Lcd::advance_clock to handle huge jumps");
        }
        let mut screen_time = self.lcd_clock - self.current_draw_start;

        if screen_time > Self::SCREEN_TIME {
            // ok, screen's done and we're resetting to mode 2 (exiting mode 1)
            eprintln!("screen done");
            self.current_draw_start += (screen_time / Self::SCREEN_TIME) * Self::SCREEN_TIME;
            self.ly = 0;
            self.current_line_start = self.current_draw_start;
            screen_time -= screen_time % Self::SCREEN_TIME;
        }

        let mut line_time = self.lcd_clock - self.current_line_start;

        if line_time > Self::LINE_TIME {
            // ok, line's done and we're resetting to mode 2 (exiting mode 0)
            // eprintln!("line {} done", self.ly);
            if self.ly < 144 {
                for px in 0..self.background_pixels.len() {
                    assert!(self.background_pixels.len() == 160);
                    self.display[self.ly as usize * 160 + px] = self.background_pixels[px];
                }
//                self.background_pixels.clear();
            }

            self.current_line_start += (line_time / Self::LINE_TIME) * Self::LINE_TIME;
            self.ly += 1;
            line_time -= line_time % Self::LINE_TIME;
        }

        let mut should_interrupt = false;

        // fire LYC=LY
        if self.ly == lyc && (lcd_stat & 0b0100_0000 != 0) {
            should_interrupt = true;
        }

        if self.ly >= 144 {
            // so ... does lcd_stat.vblank imply that vblank is suppressed entirely, or only that
            // STAT does not fire on vblank, but IF.vblank is still set?
            if self.mode != 1 && (lcd_stat & 0b0001_0000 != 0) {
                should_interrupt = true;
            }
            self.mode = 1;
            (true, should_interrupt)
        } else {
            if line_time < 80 {
                let prior_mode = self.mode;
                if self.mode != 2 && (lcd_stat & 0b0100_0000 != 0) {
                    should_interrupt = true;
                }
                self.mode = 2;

                if prior_mode != self.mode {
                    // do a full OAM scan up front. there might be benefits to driving the OAM reads in
                    // a more cycle-accurate manner, but i'm skimping on that for now.
                    self.oam_scan_items.clear();
                    for i in 0..40 {
                        let object_addr = 4 * i;
                        let y_end = self.oam[object_addr + 0];
                        let x_end = self.oam[object_addr + 1];

                        // TODO: respect LCDC.2 for double-height sprites
                        let sprite_height = 8;

                        let selected_line = self.ly as i16 - ((y_end as i16) - 16);
                        if selected_line < 0 || selected_line > sprite_height {
                            continue;
                        }

                        self.oam_scan_items.push(OamItem {
                            selected_line: selected_line as u8,
                            x: x_end,
                            tile_index: self.oam[object_addr + 2],
                            oam_flags: self.oam[object_addr + 3],
                        });
                    }
                    self.oam_scan_items.sort_by_key(|item| item.x);
                    while self.oam_scan_items.len() > 10 {
                        self.oam_scan_items.pop();
                    }
    //                eprintln!("{} sprites to draw", self.oam_scan_items.len());
                    self.background_pixels.clear();
                    let tile_base = self.background_tile_base();
//                    eprintln!("tile base: {:04x}", tile_base);
//                    eprintln!("tile y: {}.{}", self.ly / 8, self.ly % 8);
                    let background_y = scy + self.ly;
                    let background_x = scx + 0;
                    let tile_y = (background_y / 8) as u16;
                    let tile_yoffs = background_y as u16 % 8;
                    for i in 0..160 {
                        // NOTE: if the screen is scrolled such that x would overflow past the end
                        // of the tile map, x waps back around to the left. i think.
                        let line_x = i + background_x;
                        let tile_x = (line_x / 8) as u16;
                        let tile_nr = tile_y * 32 + tile_x;
                        let tile_data = self.tile_lookup_by_nr(vram, tile_nr);
                        let tile_xoffs = i % 8;
                        let tile_row_lo = tile_data[tile_yoffs as usize * 2];
                        let tile_row_hi = tile_data[tile_yoffs as usize * 2 + 1];
                        let px =
                            (((tile_row_hi >> (7 - tile_xoffs)) & 1) << 1) |
                            (((tile_row_lo >> (7 - tile_xoffs)) & 1) << 0);

                        self.background_pixels.push(px);
                    }
                    assert_eq!(self.background_pixels.len(), 160);
                }
            } else if line_time < 80 + 168 {
                // the exact timing here depends on how many OBJs were found. 168 is a minimum.
                self.mode = 3;
            } else if line_time < 80 + 168 + 208 {
                if self.mode != 0 && (lcd_stat & 0b0010_0000 != 0) {
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

struct GBCScreen {
    buf: Box<[u8; 1024]>,
    mode: u8,
    ly: u8,
}

struct GBC {
    cpu: Cpu,
    lcd: Lcd,
    boot_rom: GBCCart,
    cart: GBCCart,
    in_boot: bool,
    ram: [u8; 32 * 1024],
    vram: [u8; 16 *  1024],
    management_bits: [u8; 0x200],
    clock: u64,
    next_div_tick: u64,
//    audio: Rc<GBCAudio>,
    verbose: bool,
}

struct MemoryMapping<'system> {
    cart: &'system mut dyn MemoryBanks,
    ram: &'system mut [u8],
    vram: &'system mut [u8],
    lcd: &'system mut Lcd,
    management_bits: &'system mut [u8],
    verbose: bool,
    dma_requested: bool,
}

impl<'a> fmt::Debug for MemoryMapping<'a> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "MemoryMapping {{ cart: {:?}, verbose: {}, .. }}", &self.cart, &self.verbose)
    }
}

impl MemoryBanks for MemoryMapping<'_> {
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
            let addr = (addr as usize - 0xc000) + nr * 0x1000;
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
                    let v = if reg == VBK {
                        // "Reading from this register will return the number of the currently loaded VRAM
                        // bank in bit 0, and all other bits will be set to 1."
                        (self.management_bits[VBK] & 1) | 0b1111_1110
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
                    } else if reg == LCDC {
                        if self.verbose {
                            eprintln!("getting LCDC=${:02x}", self.lcd.lcdc);
                        }
                        self.lcd.lcdc
                    } else {
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
            self.ram[(addr as usize - 0xc000) + nr * 0x1000] = value;
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
            if reg == KEY1 {
                self.management_bits[reg] |= value & 0b1;
            } else if reg == LY {
                // read-only register: discard the write
            } else if reg == VBK {
                self.management_bits[reg] = value & 0b01;
            } else if reg == SVBK {
                self.management_bits[reg] = value & 0b111;
            } else if reg == IE {
                if self.verbose {
                    eprintln!("setting IE=${:02x}", value);
                }
                self.management_bits[reg] = value;
            } else if reg == LCDC {
                if self.verbose {
                    eprintln!("setting LCDC=${:02x}", value);
                }
                self.lcd.set_lcdc(value);
            } else if reg == DMA {
                let source = value as u16 * 0x100;
                for i in 0..0xa0 {
                    let b = self.load(source + i);
                    self.store(0xfe00 + i, b);
                }
            } else if reg == HDMA5 {
                self.dma_requested = true;
                self.management_bits[reg] = value;
            } else {
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
// timer divider
const DIV: usize = 0x104;
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
const IE: usize = 0x10f;
/*
 * 0x130 - 0x13f
 * Wave pattern RAM
 * CH3 plays from here
 */
const WAVE: usize = 0x130;
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

// Prepare speed switch (CGB mode only)
// see gbadev pan docs
const KEY1: usize = 0x14d;
// upper/left positions of the window area
// window Y
const WY: usize = 0x14a;
// window X
const WX: usize = 0x14b;
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
// SVBK: WRAM bank (CGB mode only)
// In CGB Mode 32 KBytes internal RAM are available. This memory is divided into 8 banks of 4
// KBytes each. Bank 0 is always available in memory at C000-CFFF, Bank 1-7 can be selected into
// the address space at D000-DFFF.
const SVBK: usize = 0x170;

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

impl GBC {
    fn new(boot_rom: GBCCart) -> Self {
        Self {
            cpu: Cpu::new(),
            lcd: Lcd::new(),
            cart: GBCCart::empty(),
            boot_rom,
            in_boot: true,
            ram: [0u8; 32 * 1024],
            vram: [0u8; 16 * 1024],
            management_bits: [0u8; 0x200],
            clock: 0,
            next_div_tick: 256,
            verbose: false,
        }
    }

    fn set_cart(&mut self, cart: GBCCart) {
        self.cart = cart;
    }

    fn advance_clock(&mut self, clocks: u64) {
        // TODO: support TAC/TMA/TIMA...

        // practically speaking this will never overflow, but still..
        let new_clock = self.clock.wrapping_add(clocks);

        let div_overshoot = new_clock as i64 - self.next_div_tick as i64;
        if div_overshoot > 0 {
            // must advance div (now figure out by how much...)
            let div_amount = (div_overshoot as u64 + 255) / 256;
            self.management_bits[DIV] = self.management_bits[DIV].wrapping_add(div_amount as u8);
            self.next_div_tick = self.next_div_tick.wrapping_add(div_amount * 256);
        }

        let lcd_clocks = if self.cpu.speed_mode != 0 {
            // when in Double Speed Mode, the lcd clock is maintained as if it were normal mode;
            // however many clocks we run on the cpu and timers, we run half as many on the lcd
            // clock.
            clocks / 2
        } else {
            clocks
        };

        let (vblank_int, stat_int) = self.lcd.advance_clock(&self.vram, self.management_bits[STAT], self.management_bits[LYC], lcd_clocks, self.management_bits[SCX], self.management_bits[SCY]);
        if vblank_int {
            self.management_bits[IF] |= 0b00001;
        }
        if stat_int {
            self.management_bits[IF] |= 0b00010;
        }
        self.management_bits[LY] = self.lcd.ly;

        self.clock = new_clock;
    }

    fn run(&mut self) {
        let mut mem_map = MemoryMapping {
            cart: if self.in_boot {
                self.boot_rom.mapper.as_mut()
            } else {
                self.cart.mapper.as_mut()
            },
            ram: &mut self.ram,
            vram: &mut self.vram,
            lcd: &mut self.lcd,
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
//            eprintln!("  {:?}", instr);
        }

        let pc_before = self.cpu.pc;
        let clocks = self.cpu.step(&mut mem_map);
        if mem_map.dma_requested {
            eprintln!("DMA REQUESTED");
            let source = (mem_map.management_bits[HDMA1] as u16) << 8 | (mem_map.management_bits[HDMA2] as u16);
            let dest = (mem_map.management_bits[HDMA3] as u16) << 8 | (mem_map.management_bits[HDMA4] as u16);
            let size = mem_map.management_bits[HDMA5] as u16;
            if size > 0x7f {
                panic!("TODO: hblank dma");
            }
            let size = size * 0x10 + 0x10;
            for i in 0..size {
                mem_map.store(dest + i, mem_map.load(source + i));
            }
        }
        if self.cpu.sp >= 0xfe00 && self.cpu.sp < 0xff80 {
            panic!("nonsense sp: ${:04x}", self.cpu.sp);
        }
        /*
        if pc_before == self.cpu.pc {
            panic!("loop detected");
        }
        */
        if self.cpu.pc == 0x1c2 {
            eprintln!("pc: {}", mem_map.translate_address(self.cpu.pc));
            eprintln!("rom addr: {}", mem_map.cart.translate_address(self.cpu.pc));
            let mut reader = BankReader::read_at(&mut mem_map, self.cpu.pc);
            let decoder = yaxpeax_sm83::InstDecoder::default();

            let instr = decoder.decode(&mut reader).unwrap();
            eprintln!("pc={:#04x} {}", self.cpu.pc, instr.decorate(&self.cpu, &mem_map));
            self.cpu.step(&mut mem_map);
            eprintln!("{:?}", &self.cpu);
            eprintln!("emu breakpoint");
//            self.verbose = true;
//            self.cpu.verbose = true;
        }

        if self.verbose {
            eprintln!("clock: {}", self.clock);
            eprint!("{:?}", &self.cpu);
            /*
            let stack_entries = std::cmp::min(std::cmp::max(32, 0x10000 - self.cpu.sp as u32), 64);
            let end = std::cmp::min(0x10000, self.cpu.sp as u32 + stack_entries);
            let start = end - stack_entries;
            for i in 0..(stack_entries / 2) {
                if i % 8 == 0 {
                    eprint!("    {:04x}:", start + (i * 2) / 8);
                }
                eprint!(" {:02x}{:02x}", mem_map.load((start + i * 2 + 1) as u16), mem_map.load((start + i * 2) as u16));
                if i % 8 == 7 {
                    eprintln!("");
                }
            }
            */
        }

        if self.in_boot {
            let boot_rom_disable = mem_map.load(0xff50);
            if boot_rom_disable != 0 {
                eprintln!("boot rom complete, switching to cart");
//                    self.verbose = true;
//                    self.cpu.verbose = true;
                self.in_boot = false;
            }
        }

        self.advance_clock(clocks as u64);
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
    fn make_mapper(&self, ram_style: RamStyle, rom_image: Box<[u8]>) -> Box<dyn MemoryBanks + Send> {
        match self {
            MemoryBankControllerType::MBC1 => {
                let ram = match ram_style {
                    RamStyle::None => Vec::new().into_boxed_slice(),
                    RamStyle::Flat2kb => vec![0u8; 2 * 1024].into_boxed_slice(),
                    RamStyle::Flat8kb => vec![0u8; 8 * 1024].into_boxed_slice(),
                    RamStyle::Banked4x8kb => vec![0u8; 32 * 1024].into_boxed_slice(),
                };

                Box::new(MBC3 {
                    ram_enable: 0u8,
                    rom_bank: [0u8; 2],
                    ram_bank: 0u8,
                    rom: rom_image,
                    ram: ram,
                })
            }
            MemoryBankControllerType::MBC3 => {
                let ram = match ram_style {
                    RamStyle::None => Vec::new().into_boxed_slice(),
                    RamStyle::Flat2kb => vec![0u8; 2 * 1024].into_boxed_slice(),
                    RamStyle::Flat8kb => vec![0u8; 8 * 1024].into_boxed_slice(),
                    RamStyle::Banked4x8kb => vec![0u8; 32 * 1024].into_boxed_slice(),
                };

                Box::new(MBC3 {
                    ram_enable: 0u8,
                    rom_bank: [0u8; 2],
                    ram_bank: 0u8,
                    rom: rom_image,
                    ram: ram,
                })
            },
            MemoryBankControllerType::MBC5 => {
                let ram = match ram_style {
                    RamStyle::None => Vec::new().into_boxed_slice(),
                    RamStyle::Flat2kb => vec![0u8; 4 * 2 * 1024].into_boxed_slice(),
                    RamStyle::Flat8kb => vec![0u8; 4 * 8 * 1024].into_boxed_slice(),
                    RamStyle::Banked4x8kb => vec![0u8; 4 * 32 * 1024].into_boxed_slice(),
                };

                Box::new(MBC5 {
                    ram_enable: 0u8,
                    rom_bank: [0u8; 2],
                    ram_bank: 0u8,
                    rom: rom_image,
                    ram: ram,
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
}

const SEGMENT_ROM: u8 = 0;
const SEGMENT_RAM: u8 = 1;
const SEGMENT_MANAGEMENT: u8 = 2;
const SEGMENT_VRAM: u8 = 3;
const SEGMENT_CART: u8 = 4;
const SEGMENT_OAM: u8 = 5;

trait MemoryBanks: fmt::Debug {
    fn load(&self, addr: u16) -> u8;
    fn store(&mut self, addr: u16, value: u8);
    fn translate_address(&self, addr: u16) -> MemoryAddress;
}

#[derive(Debug)]
struct MBC1 {
    ram_enable: u8,
    rom_bank: [u8; 2],
    ram_bank: u8,
    rom: Box<[u8]>,
    ram: Box<[u8]>,
}

impl MemoryBanks for MBC1 {
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
        } else if addr < 0x7fff {
            let bank = u16::from_le_bytes(self.rom_bank) as usize;
            let addr = addr as usize - 0x4000 + bank * 0x4000;
            MemoryAddress::rom(addr as u32)
        } else if addr < 0xa000 {
            eprintln!("bad cart access at {:#04x}", addr);
            MemoryAddress::rom(0)
        } else if addr < 0xc000 {
            let bank = self.ram_bank as usize;
            let addr = addr as usize - 0xa000 + bank * 0x2000;
            MemoryAddress::ram(addr as u32)
        } else {
            eprintln!("bad cart access at {:#04x}", addr);
            MemoryAddress::rom(0)
        }
    }
    fn store(&mut self, _addr: u16, _value: u8) {}
}

#[derive(Debug)]
struct MBC3 {
    ram_enable: u8,
    rom_bank: [u8; 2],
    ram_bank: u8,
    rom: Box<[u8]>,
    ram: Box<[u8]>,
}

impl MemoryBanks for MBC3 {
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
        } else if addr < 0x7fff {
            let bank = u16::from_le_bytes(self.rom_bank) as usize;
            let addr = addr as usize - 0x4000 + bank * 0x4000;
            MemoryAddress::rom(addr as u32)
        } else if addr < 0xa000 {
            eprintln!("bad cart access at {:#04x}", addr);
            MemoryAddress::rom(0)
        } else if addr < 0xc000 {
            let bank = self.ram_bank as usize;
            let addr = addr as usize - 0xa000 + bank * 0x2000;
            MemoryAddress::ram(addr as u32)
        } else {
            eprintln!("bad cart access at {:#04x}", addr);
            MemoryAddress::rom(0)
        }
    }
    fn store(&mut self, _addr: u16, _value: u8) {}
}

struct MBC5 {
    ram_enable: u8,
    rom_bank: [u8; 2],
    ram_bank: u8,
    rom: Box<[u8]>,
    ram: Box<[u8]>,
}

impl fmt::Debug for MBC5 {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "mbc5 state: ram_enable? {}, rom_bank: {:04x}, ram_bank: {:02x}", self.ram_enable, u16::from_le_bytes(self.rom_bank), self.ram_bank)
    }
}

impl MemoryBanks for MBC5 {
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
        } else if addr < 0x7fff {
            let bank = u16::from_le_bytes(self.rom_bank) as usize;
            let addr = addr as usize - 0x4000 + bank * 0x4000;
            MemoryAddress::rom(addr as u32)
        } else if addr < 0xa000 {
            eprintln!("bad cart access at {:#04x}", addr);
            MemoryAddress::rom(0)
        } else if addr < 0xc000 {
            let bank = self.ram_bank as usize;
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

    fn new(data: Vec<u8>) -> Result<Self, String> {
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

        let mapper = features.mbc.make_mapper(ram_style, data.into_boxed_slice());

        eprintln!("cartridge type {:#02x}, features: {:?}", mapper.load(0x147), features);

        Ok(Self { features, mapper })
    }
}
