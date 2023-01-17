use std::fmt;
use std::rc::Rc;
use clap::Parser;
use rand::RngCore;
use rand::rngs::OsRng;

use yaxpeax_arch::{AddressBase, Decoder, ReadError, ReaderBuilder, LengthedInstruction};
use yaxpeax_sm83::{Opcode, Operand};

#[derive(Parser)]
#[clap(about, version, author)]
struct Args {
    file: String
}

fn main() {
    let args = Args::parse();

    let rom = std::fs::read(&args.file).unwrap();

    let cart = GBCCart::new(rom).unwrap();

    let bootrom = GBCCart::boot_rom(std::fs::read("cgb_boot.bin").unwrap(), &cart);

    let mut gb = GBC::new(bootrom);

    gb.set_cart(cart);

    gb.run();
}

struct Lcd {
    // HBlank, VBlank, Searching OAM, Transferring Data to LCD Controller
    mode: u8,
    ly: u8,
    lcd_clock: u64,
    current_line_start: u64,
    current_draw_start: u64,
    // when, in dots, we'll be at the next line. this is the end of the current line's HBlank.
    next_line: u64,
    next_draw_time: u64,
}

impl Lcd {
    const LINE_TIME: u64 = 376;
    const VBLANK_TIME: u64 = 4560; // vblank is 10 scan lines
    const SCREEN_TIME: u64 = 154 * Self::LINE_TIME;

    fn new() -> Self {
        Self {
            mode: 1,
            ly: 0,
            lcd_clock: 0,
            current_line_start: 0,
            current_draw_start: 0,
            next_line: Self::LINE_TIME,
            next_draw_time: Self::SCREEN_TIME,
        }
    }

    // advance lcd state by `clocks` ticks, using `lcd_stat` to determine if we should generate an
    // interrupt.
    fn advance_clock(&mut self, lcd_stat: u8, lyc: u8, clocks: u64) -> (bool, bool) {
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
            eprintln!("line {} done", self.ly);
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
                if self.mode != 2 && (lcd_stat & 0b0100_0000 != 0) {
                    should_interrupt = true;
                }
                self.mode = 2;
                // do mode 2 work. if any? probably not.. we'll see how much time the OAM scan takes
                // later.
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

struct Cpu {
    af: [u8; 2],
    bc: [u8; 2],
    de: [u8; 2],
    hl: [u8; 2],
    sp: u16,
    pc: u16,
    speed_mode: u8, // 0=Normal, 1=Double
    // interrupt master enable
    ime: bool,
    verbose: bool,
}

impl fmt::Debug for Cpu {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        writeln!(f, "sm83 cpu (mode: {}):", ["normal", "double"][self.speed_mode as usize])?;
        writeln!(f, "        a f")?;
        writeln!(f, "  af: ${:02x}{:02x}", self.af[1], self.af[0])?;
        writeln!(f, "        b c")?;
        writeln!(f, "  bc: ${:02x}{:02x}", self.bc[1], self.bc[0])?;
        writeln!(f, "        d e")?;
        writeln!(f, "  de: ${:02x}{:02x}", self.de[1], self.de[0])?;
        writeln!(f, "        h l")?;
        writeln!(f, "  hl: ${:02x}{:02x}", self.hl[1], self.hl[0])?;
        writeln!(f, "  sp: ${:04x}", self.sp)?;
        if self.sp & 1 != 0 {
            writeln!(f, "    [!] misaligned")?;
        }
        writeln!(f, "  pc: ${:04x}", self.pc)?;
        Ok(())
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

trait OpWidth {
    fn width(&self) -> Option<u8>;
}
impl OpWidth for yaxpeax_sm83::Operand {
    fn width(&self) -> Option<u8> {
        match self {
            Operand::D8(_) |
            Operand::I8(_) |
            Operand::A |
            Operand::B |
            Operand::C |
            Operand::D |
            Operand::E |
            Operand::H |
            Operand::L => Some(8),
            Operand::AF |
            Operand::BC |
            Operand::DE |
            Operand::HL |
            Operand::SP |
            Operand::D16(_) => Some(16),
            Operand::Bit(_) |
            Operand::R8(_) |
            Operand::A16(_) |
            Operand::DerefHL |
            Operand::DerefBC |
            Operand::DerefDE |
            Operand::DerefDecHL |
            Operand::DerefIncHL |
            Operand::DerefHighC |
            Operand::DerefHighD8(_) |
            Operand::SPWithOffset(_) |
            Operand::CondC |
            Operand::CondNC |
            Operand::CondZ |
            Operand::CondNZ |
            Operand::Nothing => None
        }
    }
}

impl Cpu {
    fn new() -> Self {
        Self {
            af: [OsRng.next_u32() as u8, OsRng.next_u32() as u8],
            bc: [OsRng.next_u32() as u8, OsRng.next_u32() as u8],
            de: [OsRng.next_u32() as u8, OsRng.next_u32() as u8],
            hl: [OsRng.next_u32() as u8, OsRng.next_u32() as u8],
            sp: OsRng.next_u32() as u16,
            pc: 0,
            speed_mode: 0,
            ime: true,
            verbose: false,
        }
    }

    fn get_8b(&mut self, memory: &MemoryMapping, op: Operand) -> u8 {
        match op {
            Operand::A => { self.af[1] },
            Operand::B => { self.bc[1] },
            Operand::C => { self.bc[0] },
            Operand::D => { self.de[1] },
            Operand::E => { self.de[0] },
            Operand::H => { self.hl[1] },
            Operand::L => { self.hl[0] },
            Operand::AF => { panic!("invalid") },
            Operand::BC => { panic!("invalid") },
            Operand::DE => { panic!("invalid") },
            Operand::HL => { panic!("invalid") },
            Operand::SP => { panic!("invalid") },
            Operand::A16(addr) => {
                memory.load(addr)
            },
            Operand::DerefHighD8(offset) => {
                let addr = 0xff00 + offset as u16;
                memory.load(addr)
            },
            Operand::DerefIncHL => {
                let v = memory.load(u16::from_le_bytes(self.hl));
                self.hl = u16::from_le_bytes(self.hl).wrapping_add(1).to_le_bytes();
                v
            },
            Operand::DerefDecHL => {
                let v = memory.load(u16::from_le_bytes(self.hl));
                self.hl = u16::from_le_bytes(self.hl).wrapping_sub(1).to_le_bytes();
                v
            },
            Operand::DerefDE => {
                memory.load(u16::from_le_bytes(self.de))
            },
            Operand::DerefHL => {
                memory.load(u16::from_le_bytes(self.hl))
            },
            Operand::DerefHighC => {
                let addr = 0xff00 + self.bc[0] as u16;
                memory.load(addr)
            },
            Operand::D8(v) => v,
            Operand::D16(v) => { panic!("invalid") },
            other => { panic!("cannot get value from {:?}", other); }
        }
    }

    fn get_16b(&self, memory: &MemoryMapping, op: Operand) -> u16 {
        match op {
            Operand::A => { panic!("invalid") },
            Operand::B => { panic!("invalid") },
            Operand::C => { panic!("invalid") },
            Operand::D => { panic!("invalid") },
            Operand::E => { panic!("invalid") },
            Operand::H => { panic!("invalid") },
            Operand::L => { panic!("invalid") },
            Operand::AF => { u16::from_le_bytes(self.af) },
            Operand::BC => { u16::from_le_bytes(self.bc) },
            Operand::DE => { u16::from_le_bytes(self.de) },
            Operand::HL => { u16::from_le_bytes(self.hl) },
            Operand::SP => { self.sp },
            Operand::DerefHighD8(offset) => {
                panic!("invalid");
            },
            Operand::D8(v) => { panic!("invalid") },
            Operand::D16(v) => v,
            other => { panic!("cannot get value from {:?}", other); }
        }
    }

    fn set_8b(&mut self, memory: &mut MemoryMapping, op: Operand, value: u8) {
        match op {
            Operand::A => { self.af[1] = value; },
            Operand::B => { self.bc[1] = value; },
            Operand::C => { self.bc[0] = value; },
            Operand::D => { self.de[1] = value; },
            Operand::E => { self.de[0] = value; },
            Operand::H => { self.hl[1] = value; },
            Operand::L => { self.hl[0] = value; },
            Operand::AF => { panic!("invalid"); },
            Operand::BC => { panic!("invalid"); },
            Operand::DE => { panic!("invalid"); },
            Operand::HL => { panic!("invalid"); },
            Operand::SP => { panic!("invalid"); },
            Operand::A16(addr) => {
                memory.store(addr, value)
            },
            Operand::DerefHighD8(offset) => {
                let addr = 0xff00 + offset as u16;
                memory.store(addr, value)
            },
            Operand::DerefDecHL => {
                memory.store(u16::from_le_bytes(self.hl), value);
                self.hl = u16::from_le_bytes(self.hl).wrapping_sub(1).to_le_bytes();
            },
            Operand::DerefIncHL => {
                memory.store(u16::from_le_bytes(self.hl), value);
                self.hl = u16::from_le_bytes(self.hl).wrapping_add(1).to_le_bytes();
            },
            Operand::DerefDE => {
                memory.store(u16::from_le_bytes(self.de), value);
            },
            Operand::DerefHL => {
                memory.store(u16::from_le_bytes(self.hl), value);
            },
            Operand::DerefHighC => {
                let addr = 0xff00 + self.bc[0] as u16;
                memory.store(addr, value);
            },
            other => { panic!("cannot set value to {:?}", other); }
        }
    }

    fn set_16b(&mut self, memory: &mut MemoryMapping, op: Operand, value: u16) {
        match op {
            Operand::A => { panic!("invalid"); },
            Operand::B => { panic!("invalid"); },
            Operand::C => { panic!("invalid"); },
            Operand::D => { panic!("invalid"); },
            Operand::E => { panic!("invalid"); },
            Operand::H => { panic!("invalid"); },
            Operand::L => { panic!("invalid"); },
            Operand::AF => { self.af = value.to_le_bytes() },
            Operand::BC => { self.bc = value.to_le_bytes() },
            Operand::DE => { self.de = value.to_le_bytes() },
            Operand::HL => { self.hl = value.to_le_bytes() },
            Operand::SP => { self.sp = value; },
            Operand::DerefHighD8(offset) => {
                let bytes = value.to_le_bytes();
                let addr = 0xff00 + offset as u16;
                memory.store(addr, bytes[0]);
                memory.store(addr.wrapping_add(1), bytes[1]);
            },
            other => { panic!("cannot set value to {:?}", other); }
        }
    }

    fn push(&mut self, memory: &mut MemoryMapping, value: u16) {
        let bytes = value.to_le_bytes();
        self.sp = self.sp.wrapping_sub(2);
        memory.store(self.sp, bytes[0]);
        memory.store(self.sp.wrapping_add(1), bytes[1]);
    }

    fn pop(&mut self, memory: &mut MemoryMapping) -> u16 {
        let bytes = [memory.load(self.sp), memory.load(self.sp.wrapping_add(1))];
        self.sp = self.sp.wrapping_add(2);
        u16::from_le_bytes(bytes)
    }

    fn step(&mut self, memory: &mut MemoryMapping) -> u8 {
        let mut reader = BankReader::read_at(memory, self.pc);
        let decoder = yaxpeax_sm83::InstDecoder::default();

        let instr = decoder.decode(&mut reader).unwrap();

        self.pc += instr.len();

        match instr.opcode() {
            Opcode::CPL => {
                self.af[1] ^= 0xff;
                self.af[0] |= 0b0110_0000;
            }
            Opcode::DI => {
                self.ime = false;
            }
            Opcode::EI => {
                self.ime = true;
            }
            Opcode::STOP => {
                // if KEY1.0 is set, maybe switch clock speeds. otherwise, .. stop?
                let key1 = memory.load(0xff4d);
                if key1 & 0b01 != 0 {
                    self.speed_mode ^= 0b01;
                    if self.verbose {
                        eprintln!("switching speed mode to {}", self.speed_mode);
                    }
                    memory.management_bits[KEY1] &= 0b0111_1111;
                    memory.management_bits[KEY1] |= self.speed_mode << 7;
                } else {
                    eprintln!("stopping");
                    panic!("");
                }
            }
            Opcode::SWAP => {
                let value = self.get_8b(memory, instr.operands()[0]);
                let swapped = (value >> 4) | (value << 4);
                self.set_8b(memory, instr.operands()[0], swapped);
                self.af[0] = 0b0000_0000;
                if swapped == 0 {
                    self.af[0] |= 0b1000_0000;
                }
            }
            Opcode::AND => {
                let value = self.get_8b(memory, instr.operands()[0]);
                self.af[1] = self.af[1] & value;
                self.af[0] = 0b0000_0000;
                if self.af[1] == 0 {
                    self.af[0] |= 0b1000_0000;
                }
            }
            Opcode::XOR => {
                let value = self.get_8b(memory, instr.operands()[0]);
                self.af[1] = self.af[1] ^ value;
                self.af[0] = 0b0000_0000;
                if self.af[1] == 0 {
                    self.af[0] |= 0b1000_0000;
                }
            }
            Opcode::OR => {
                let value = self.get_8b(memory, instr.operands()[0]);
                self.af[1] = self.af[1] | value;
                self.af[0] = 0b0000_0000;
                if self.af[1] == 0 {
                    self.af[0] |= 0b1000_0000;
                }
            }
            Opcode::BIT => {
                let n = if let Operand::Bit(b) = instr.operands()[0] {
                    b
                } else {
                    panic!("[!] impossible `bit` instruction");
                };
                let res = self.get_8b(memory, instr.operands()[1]) & (1 << n);
                self.af[0] &= !0b1110_0000;
                self.af[0] |= 0b0010_0000;
                if res == 0 {
                    self.af[0] |= 0b1000_0000;
                }
            }
            Opcode::SET => {
                let n = if let Operand::Bit(b) = instr.operands()[0] {
                    b
                } else {
                    panic!("[!] impossible `set` instruction");
                };
                let res = self.get_8b(memory, instr.operands()[1]) | (1 << n);
                self.set_8b(memory, instr.operands()[1], res);
            }
            Opcode::RES => {
                let n = if let Operand::Bit(b) = instr.operands()[0] {
                    b
                } else {
                    panic!("[!] impossible `res` instruction");
                };
                let res = self.get_8b(memory, instr.operands()[1]) & !(1 << n);
                self.set_8b(memory, instr.operands()[1], res);
            }
            Opcode::SUB => {
                // all 8b operands
                // 8b add
                let a = self.af[1];
                let other = self.get_8b(memory, instr.operands()[0]);
                let (res, carry) = a.overflowing_sub(other);
                self.af[1] = res;
                self.af[0] &= !0b0111_0000;
                if carry {
                    self.af[0] |= 0b0001_0000;
                }
                if ((a as i8 & 0x0f) - (other as i8 & 0x0f)) as u8 > 0x0f {
                    self.af[0] |= 0b0010_0000;
                }
            }
            Opcode::ADD => {
                match instr.operands()[0] {
                    Operand::HL => {
                        // 16b add
                        let hl = u16::from_le_bytes(self.hl);
                        let other = self.get_16b(memory, instr.operands()[1]);
                        let (res, carry) = hl.overflowing_add(other);
                        self.hl = res.to_le_bytes();
                        self.af[0] &= !0b0111_0000;
                        if carry {
                            self.af[0] |= 0b0001_0000;
                        }
                        // half-carry is at bit 11 for hl?
                        if (hl & 0x0fff) + (other & 0x0fff) > 0x0fff {
                            self.af[0] |= 0b0010_0000;
                        }
                    }
                    Operand::SP => {
                        // 16b add
                        let sp = self.sp;
                        let other = match instr.operands()[1] {
                            Operand::I8(v) => v as i16 as u16,
                            other => { panic!("bad add operand {:?}", other); }
                        };
                        let (res, carry) = sp.overflowing_add(other);
                        self.sp = res;
                        self.af[0] &= !0b0111_0000;
                        if carry {
                            self.af[0] |= 0b0001_0000;
                        }
                        // half-carry is at bit 3 for hl?
                        if (sp & 0x000f) + (other & 0x000f) > 0x000f {
                            self.af[0] |= 0b0010_0000;
                        }
                    }
                    other => {
                        // 8b add
                        let a = self.af[1];
                        let other = self.get_8b(memory, other); // TODO: probably should be operands[0]?
                        let (res, carry) = a.overflowing_add(other);
                        self.af[1] = res;
                        self.af[0] &= !0b0111_0000;
                        if carry {
                            self.af[0] |= 0b0001_0000;
                        }
                        if (a & 0x0f) + (other & 0x0f) > 0x0f {
                            self.af[0] |= 0b0010_0000;
                        }
                    }
                }
            }
            Opcode::DEC => {
                match instr.operands()[0].width() {
                    Some(8) => {
                        let v = self.get_8b(memory, instr.operands()[0]);
                        if v & 0x0f == 0x0f {
                            self.af[0] |= 0b0010_0000;
                        } else {
                            self.af[0] &= !0b0010_0000;
                        }
                        self.af[0] |= 0b0100_0000;
                        if v == 0xff {
                            self.af[0] |= 0b1000_0000;
                        } else {
                            self.af[0] &= !0b1000_0000;
                        }
                        self.set_8b(memory, instr.operands()[0], v.wrapping_sub(1));
                    },
                    Some(16) => {
                        let v = self.get_16b(memory, instr.operands()[0]);
                        self.set_16b(memory, instr.operands()[0], v.wrapping_sub(1));
                    },
                    Some(other) => {
                        panic!("???");
                    }
                    None => {
                        let v = memory.load(u16::from_le_bytes(self.hl));
                        if v & 0x0f == 0x0f {
                            self.af[0] |= 0b0010_0000;
                        } else {
                            self.af[0] &= !0b0010_0000;
                        }
                        self.af[0] |= 0b0100_0000;
                        if v == 0xff {
                            self.af[0] |= 0b1000_0000;
                        } else {
                            self.af[0] &= !0b1000_0000;
                        }
                        memory.store(u16::from_le_bytes(self.hl), v.wrapping_sub(1));
                    }
                }
            }
            Opcode::INC => {
                match instr.operands()[0].width() {
                    Some(8) => {
                        let v = self.get_8b(memory, instr.operands()[0]);
                        if v & 0x0f == 0x0f {
                            self.af[0] |= 0b0010_0000;
                        } else {
                            self.af[0] &= !0b0010_0000;
                        }
                        self.af[0] |= 0b0100_0000;
                        if v == 0xff {
                            self.af[0] |= 0b1000_0000;
                        } else {
                            self.af[0] &= !0b1000_0000;
                        }
                        self.set_8b(memory, instr.operands()[0], v.wrapping_add(1));
                    },
                    Some(16) => {
                        let v = self.get_16b(memory, instr.operands()[0]);
                        self.set_16b(memory, instr.operands()[0], v.wrapping_add(1));
                    },
                    Some(other) => {
                        panic!("???");
                    }
                    None => {
                        let v = memory.load(u16::from_le_bytes(self.hl));
                        if v & 0x0f == 0x0f {
                            self.af[0] |= 0b0010_0000;
                        } else {
                            self.af[0] &= !0b0010_0000;
                        }
                        self.af[0] |= 0b0100_0000;
                        if v == 0xff {
                            self.af[0] |= 0b1000_0000;
                        } else {
                            self.af[0] &= !0b1000_0000;
                        }
                        memory.store(u16::from_le_bytes(self.hl), v.wrapping_add(1));
                    }
                }
            }
            Opcode::LD => {
                let operands = instr.operands();
                let widths = (operands[0].width(), operands[1].width());
                match widths {
                    (Some(8), None) |
                    (None, Some(8)) |
                    (Some(8), Some(8)) => {
                        let loaded = self.get_8b(memory, operands[1]);
                        self.set_8b(memory, operands[0], loaded);
                    },
                    (Some(16), None) |
                    (None, Some(16)) |
                    (Some(16), Some(16)) => {
                        let loaded = self.get_16b(memory, operands[1]);
                        self.set_16b(memory, operands[0], loaded);
                        if let [Operand::D16(_), Operand::SP] = operands {
                            // for some reason this is four more clocks than bytes+mem access would
                            // predict.
                            return 20;
                        }
                    },
                    others => {
                        panic!("inconsistent ld widths: {:?}", others);
                    }
                }
            }
            Opcode::LDH => {
                let loaded = self.get_8b(memory, instr.operands()[1]);
                self.set_8b(memory, instr.operands()[0], loaded);
            }
            Opcode::JP => {
                let operands = instr.operands();
                if operands[1] == Operand::Nothing {
                    // unconditional
                    let dest = self.get_16b(memory, operands[0]);
                    self.pc = dest;
                    if operands[0] == Operand::HL {
                        return 4;
                    } else {
                        // jp <d16>
                        return 16;
                    }
                } else {
                    let br = match operands[0] {
                        Operand::CondNZ => {
                            self.af[0] & 0b1000_0000 == 0
                        },
                        Operand::CondZ => {
                            self.af[0] & 0b1000_0000 != 0
                        },
                        Operand::CondNC => {
                            self.af[0] & 0b0001_0000 == 0
                        },
                        Operand::CondC => {
                            self.af[0] & 0b0001_0000 != 0
                        },
                        other => {
                            panic!("[!] impossible conditional jp");
                        }
                    };

                    if br {
                        if let Operand::D16(addr) = operands[1] {
                            self.pc = addr;
                            return 16;
                        } else {
                            panic!("[!] op? {:?}", operands[1]);
                        }
                    } else {
                        return 12;
                    }
                }
            }
            Opcode::JR => {
                let operands = instr.operands();
                if operands[1] == Operand::Nothing {
                    // unconditional
                    if let Operand::R8(r) = operands[0] {
                        self.pc = self.pc.wrapping_add(r as i16 as u16);
                        return 12;
                    } else {
                        panic!("[!] op? {:?}", operands[0]);
                    }
                } else {
                    let br = match operands[0] {
                        Operand::CondNZ => {
                            self.af[0] & 0b1000_0000 == 0
                        },
                        Operand::CondZ => {
                            self.af[0] & 0b1000_0000 != 0
                        },
                        Operand::CondNC => {
                            self.af[0] & 0b0001_0000 == 0
                        },
                        Operand::CondC => {
                            self.af[0] & 0b0001_0000 != 0
                        },
                        other => {
                            panic!("[!] impossible conditional branch");
                        }
                    };

                    if br {
                        if let Operand::R8(r) = operands[1] {
                            self.pc = self.pc.wrapping_add(r as i16 as u16);
                            return 12;
                        } else {
                            panic!("[!] op? {:?}", operands[1]);
                        }
                    } else {
                        return 8;
                    }
                }
            }
            Opcode::SLA => {
                let v = self.get_8b(memory, instr.operands()[0]);
                self.af[0] = 0b0000_0000;
                if v & 0x80 != 0 {
                    self.af[0] |= 0b0001_0000;
                }
                let shifted = v << 1;
                if shifted == 0 {
                    self.af[0] |= 0b1000_0000;
                }
                self.set_8b(memory, instr.operands()[0], shifted);
            }
            Opcode::SRA => {
                let v = self.get_8b(memory, instr.operands()[0]);
                self.af[0] = 0b0000_0000;
                if v & 0x01 != 0 {
                    self.af[0] |= 0b0001_0000;
                }
                let shifted = v >> 1;
                if shifted == 0 {
                    self.af[0] |= 0b1000_0000;
                }
                self.set_8b(memory, instr.operands()[0], shifted);
            }
            Opcode::SRL => {
                let v = self.get_8b(memory, instr.operands()[0]);
                self.af[0] = 0b0000_0000;
                if v & 0x01 != 0 {
                    self.af[0] |= 0b0001_0000;
                }
                let shifted = ((v as i8) >> 1) as u8;
                if shifted == 0 {
                    self.af[0] |= 0b1000_0000;
                }
                self.set_8b(memory, instr.operands()[0], shifted);
            }
            Opcode::RL => {
                let v = self.get_8b(memory, instr.operands()[0]);
                let c = (self.af[0] >> 4) & 0b0001;
                self.af[0] = 0b0000_0000;
                if v & 0x80 != 0 {
                    self.af[0] |= 0b0001_0000;
                }
                let rotated = (v << 1) | c;
                if rotated == 0 {
                    self.af[0] |= 0b1000_0000;
                }
                self.set_8b(memory, instr.operands()[0], rotated);
            }
            Opcode::RR => {
                let v = self.get_8b(memory, instr.operands()[0]);
                let c = (self.af[0] >> 4) & 0b0001;
                self.af[0] = 0b0000_0000;
                if v & 0x01 != 0 {
                    self.af[0] |= 0b0001_0000;
                }
                let rotated = (v >> 1) | (c << 7);
                if rotated == 0 {
                    self.af[0] |= 0b1000_0000;
                }
                self.set_8b(memory, instr.operands()[0], rotated);
            }
            Opcode::RLC => {
                let v = self.get_8b(memory, instr.operands()[0]);
                let mut c = 0;
                self.af[0] = 0b0000_0000;
                if v & 0x80 != 0 {
                    c = 0x01;
                    self.af[0] |= 0b0001_0000;
                }
                let rotated = (v << 1) | c;
                if rotated == 0 {
                    self.af[0] |= 0b1000_0000;
                }
                self.set_8b(memory, instr.operands()[0], rotated);
            }
            Opcode::RRC => {
                let v = self.get_8b(memory, instr.operands()[0]);
                let mut c = 0;
                self.af[0] = 0b0000_0000;
                if v & 0x01 != 0 {
                    c = 0x80;
                    self.af[0] |= 0b0001_0000;
                }
                let rotated = (v >> 1) | c;
                if rotated == 0 {
                    self.af[0] |= 0b1000_0000;
                }
                self.set_8b(memory, instr.operands()[0], rotated);
            }
            Opcode::RLA => {
                let v = self.af[1];
                let c = (self.af[0] >> 4) & 0b0001;
                self.af[0] = 0b0000_0000;
                if v & 0x80 != 0 {
                    self.af[0] |= 0b0001_0000;
                }
                let rotated = (v << 1) | c;
                if rotated == 0 {
                    self.af[0] |= 0b1000_0000;
                }
                self.af[1] = rotated;
            }
            Opcode::RRA => {
                let v = self.af[1];
                let c = (self.af[0] >> 4) & 0b0001;
                self.af[0] = 0b0000_0000;
                if v & 0x01 != 0 {
                    self.af[0] |= 0b0001_0000;
                }
                let rotated = (v >> 1) | (c << 7);
                if rotated == 0 {
                    self.af[0] |= 0b1000_0000;
                }
                self.af[1] = rotated;
            }
            Opcode::RLCA => {
                let v = self.af[1];
                let mut c = 0;
                self.af[0] = 0b0000_0000;
                if v & 0x80 != 0 {
                    c = 0x01;
                    self.af[0] |= 0b0001_0000;
                }
                let rotated = (v << 1) | c;
                if rotated == 0 {
                    self.af[0] |= 0b1000_0000;
                }
                self.af[1] = rotated;
            }
            Opcode::RRCA => {
                let v = self.af[1];
                let mut c = 0;
                self.af[0] = 0b0000_0000;
                if v & 0x01 != 0 {
                    c = 0x80;
                    self.af[0] |= 0b0001_0000;
                }
                let rotated = (v >> 1) | c;
                if rotated == 0 {
                    self.af[0] |= 0b1000_0000;
                }
                self.af[1] = rotated;
            }
            Opcode::CP => {
                let a = self.af[1];
                let other = self.get_8b(memory, instr.operands()[0]);

                self.af[0] = 0b0100_0000;

                let res = a.wrapping_sub(other);
                if res == 0 {
                    // set Z, clear C
                    self.af[0] |= 0b1000_0000;
                } else if res >= 0x80 {
                    // clear Z, set C
                    self.af[0] |= 0b0001_0000;
                } else {
                    // clear Z, clear C
                }

                let res = (a & 0xf).wrapping_sub(other & 0xf);
                if res >= 0x80 {
                    // set H
                    self.af[0] |= 0b0010_0000;
                }
            }
            Opcode::RETI => {
                // unconditional
                self.pc = self.pop(memory);
                self.ime = true;
                return 16;
            }
            Opcode::RET => {
                let operands = instr.operands();
                if operands[1] == Operand::Nothing {
                    // unconditional
                    self.pc = self.pop(memory);
                    return 16;
                } else {
                    let ret = match operands[0] {
                        Operand::CondNZ => {
                            self.af[0] & 0b1000_0000 == 0
                        },
                        Operand::CondZ => {
                            self.af[0] & 0b1000_0000 != 0
                        },
                        Operand::CondNC => {
                            self.af[0] & 0b0001_0000 == 0
                        },
                        Operand::CondC => {
                            self.af[0] & 0b0001_0000 != 0
                        },
                        other => {
                            panic!("[!] impossible conditional ret: {}", other);
                        }
                    };

                    if ret {
                        self.pc = self.pop(memory);
                        return 20;
                    } else {
                        return 8;
                    }
                }
            }
            Opcode::POP => {
                let v = self.pop(memory);
                self.set_16b(memory, instr.operands()[0], v);
            }
            Opcode::PUSH => {
                let v = self.get_16b(memory, instr.operands()[0]);
                self.push(memory, v);
            }
            Opcode::NOP => {
            }
            Opcode::CALL => {
                let operands = instr.operands();
                if operands[1] == Operand::Nothing {
                    // unconditional
                    let dest = self.get_16b(memory, operands[0]);
                    self.push(memory, self.pc);
                    self.pc = dest;
                    return 24;
                } else {
                    let call = match operands[0] {
                        Operand::CondNZ => {
                            self.af[0] & 0b1000_0000 == 0
                        },
                        Operand::CondZ => {
                            self.af[0] & 0b1000_0000 != 0
                        },
                        Operand::CondNC => {
                            self.af[0] & 0b0001_0000 == 0
                        },
                        Operand::CondC => {
                            self.af[0] & 0b0001_0000 != 0
                        },
                        other => {
                            panic!("[!] impossible conditional call: {}", other);
                        }
                    };

                    if call {
                        let dest = self.get_16b(memory, operands[1]);
                        self.push(memory, self.pc);
                        self.pc = dest;
                        return 24;
                    } else {
                        return 12;
                    }
                }
            }
            other => {
                panic!("[!] unhandled instruction");
            }
        }

        // baseline execution time: four clocks per byte of instruction..
        let mut clocks = 0u16.wrapping_offset(instr.len()).to_linear() as u8 * 4;

        // if an operand accesses memory, that's an extra 4 clocks...
        match instr.operands() {
            [Operand::DerefHL, _] |
            [_, Operand::DerefHL] |
            [Operand::DerefBC, _] |
            [_, Operand::DerefBC] |
            [Operand::DerefDE, _] |
            [_, Operand::DerefDE] |
            [Operand::DerefDecHL, _] |
            [_, Operand::DerefDecHL] |
            [Operand::DerefIncHL, _] |
            [_, Operand::DerefIncHL] |
            [Operand::DerefHighC, _] |
            [_, Operand::DerefHighC] |
            [Operand::DerefHighD8(_), _] |
            [_, Operand::DerefHighD8(_)] => {
                clocks += 4;
            }
            _ => {}
        }

        // inc/dec/rotates read and write to memory, so they pay 4 clock cost twice....
        match instr.opcode() {
            Opcode::INC | Opcode::DEC |
            Opcode::RLC | Opcode::RL | Opcode::RRC | Opcode::RR |
            Opcode::SLA | Opcode::SWAP | Opcode::SRA | Opcode::SRL |
            Opcode::SET | Opcode::RES => {
                if instr.operands()[0] == Operand::DerefHL {
                    clocks += 4;
                }
            }
            _ => {}
        }

        clocks
    }
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
//    screen: Rc<GBCScreen>,
//    audio: Rc<GBCAudio>,
    verbose: bool,
}

struct MemoryMapping<'system> {
    cart: &'system mut dyn MemoryBanks,
    ram: &'system mut [u8],
    vram: &'system mut [u8],
    management_bits: &'system mut [u8],
    verbose: bool,
}

impl MemoryBanks for MemoryMapping<'_> {
    fn load(&self, addr: u16) -> u8 {
        if addr < 0x4000 {
            if self.verbose && addr >= 0x104 && addr < 0x144 {
                eprintln!("loading from ${:04x} (${:02x})", addr, self.cart.load(addr));
            }
            self.cart.load(addr)
        } else if addr < 0x8000 {
            self.cart.load(addr)
        } else if addr < 0xa000 {
            let offset = 0x2000 * (self.management_bits[VBK] as usize & 0b1);
            self.vram[addr as usize - 0x8000 + offset]
        } else if addr < 0xc000 {
            self.cart.load(addr)
        } else if addr < 0xd000 {
            self.ram[addr as usize - 0xc000]
        } else if addr < 0xe000 {
            let nr = self.management_bits[SVBK] as usize & 0b11;
            let nr = if nr == 0 {
                1
            } else {
                nr
            };
            self.ram[(addr as usize - 0xc000) + nr * 0x1000]
        } else if addr < 0xfe00 {
            // aliases [c000,ddff]
            self.ram[addr as usize - 0xe000]
        } else if addr < 0xfea0 {
            // sprite attribute table
            self.management_bits[addr as usize - 0xfe00]
        } else if addr < 0xff00 {
            // "not usable"
            self.management_bits[addr as usize - 0xfe00]
        } else if addr < 0xff80 {
            // "i/o ports"
            let reg = addr as usize - 0xfe00;
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
            } else {
                let v = self.management_bits[reg];
                if self.verbose {
                    eprintln!("get ${:04x} (=${:02x})", addr, v);
                }
                v
            };
            v
        } else if addr < 0xffff {
            // "high ram (HRAM)"
            self.management_bits[addr as usize - 0xfe00]
        } else {
            // "interrupt enable register"
            self.management_bits[addr as usize - 0xfe00]
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
            let nr = self.management_bits[SVBK] as usize & 0b11;
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
            self.management_bits[addr as usize - 0xfe00] = value;
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
                self.management_bits[reg] = value & 0b11;
            } else if reg == KEY1 {
            } else if reg == IE {
                if self.verbose {
                    eprintln!("setting IE=${:02x}", value);
                }
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
// SVBK: WRAM bank (CGB mode only)
// In CGB Mode 32 KBytes internal RAM are available. This memory is divided into 8 banks of 4
// KBytes each. Bank 0 is always available in memory at C000-CFFF, Bank 1-7 can be selected into
// the address space at D000-DFFF.
const SVBK: usize = 0x170;

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

        let div_overshoot = (new_clock - self.next_div_tick) as i64;
        if div_overshoot > 0 {
            // must advance div (now figure out by how much...)
            let div_amount = (div_overshoot as u64 + 255) / 256;
            self.management_bits[DIV] = self.management_bits[DIV].wrapping_add(div_amount as u8);
            self.next_div_tick = self.next_div_tick.wrapping_add((div_amount * 256));
        }

        let lcd_clocks = if self.cpu.speed_mode != 0 {
            // when in Double Speed Mode, the lcd clock is maintained as if it were normal mode;
            // however many clocks we run on the cpu and timers, we run half as many on the lcd
            // clock.
            clocks / 2
        } else {
            clocks
        };
        let (vblank_int, stat_int) = self.lcd.advance_clock(self.management_bits[STAT], self.management_bits[LYC], lcd_clocks);
        if vblank_int {
            self.management_bits[IF] |= 0b00001;
        }
        if stat_int {
            self.management_bits[IF] |= 0b00010;
        }
        self.management_bits[LY] = self.lcd.ly;

        self.clock = new_clock;
    }

    fn run(&mut self) -> GBState {
        loop {
            let mut mem_map = MemoryMapping {
                cart: if self.in_boot {
                    self.boot_rom.mapper.as_mut()
                } else {
                    self.cart.mapper.as_mut()
                },
                ram: &mut self.ram,
                vram: &mut self.vram,
                management_bits: &mut self.management_bits,
                verbose: self.verbose,
            };

            if self.verbose {
                let mut reader = BankReader::read_at(&mut mem_map, self.cpu.pc);
                let decoder = yaxpeax_sm83::InstDecoder::default();

                let instr = decoder.decode(&mut reader).unwrap();
                eprintln!("pc={:#04x} {}", self.cpu.pc, instr);
                eprintln!("  {:?}", instr);
            }

            let clocks = self.cpu.step(&mut mem_map);

            if self.verbose {
                eprintln!("clock: {}", self.clock);
                eprint!("{:?}", &self.cpu);
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
            }

            if self.in_boot {
                let boot_rom_disable = mem_map.load(0xff50);
                if boot_rom_disable != 0 {
                    eprintln!("boot rom complete, switching to cart");
                    self.verbose = true;
                    self.cpu.verbose = true;
                    self.in_boot = false;
                }
            }

            self.advance_clock(clocks as u64);
        }
        GBState::EmulationError
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
    fn make_mapper(&self, ram_style: RamStyle, rom_image: Box<[u8]>) -> Box<dyn MemoryBanks> {
        match self {
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
    mapper: Box<dyn MemoryBanks>,
}

trait MemoryBanks {
    fn load(&self, addr: u16) -> u8;
    fn store(&mut self, addr: u16, value: u8);
}

struct MBC3 {
    ram_enable: u8,
    rom_bank: [u8; 2],
    ram_bank: u8,
    rom: Box<[u8]>,
    ram: Box<[u8]>,
}

impl MemoryBanks for MBC3 {
    fn load(&self, addr: u16) -> u8 {
        if addr <= 0x3fff {
            self.rom[addr as usize]
        } else if addr < 0x7fff {
            let bank = u16::from_le_bytes(self.rom_bank) as usize;
            self.rom[addr as usize - 0x4000 + bank * 0x4000]
        } else if addr < 0xa000 {
            eprintln!("bad cart access at {:#04x}", addr);
            0
        } else if addr < 0xc000 {
            let bank = self.ram_bank as usize;
            self.ram[addr as usize - 0xa000 + bank * 0x2000]
        } else {
            eprintln!("bad cart access at {:#04x}", addr);
            0
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

impl MemoryBanks for MBC5 {
    fn load(&self, addr: u16) -> u8 {
        if addr <= 0x3fff {
            self.rom[addr as usize]
        } else if addr < 0x7fff {
            let bank = u16::from_le_bytes(self.rom_bank) as usize;
            self.rom[addr as usize - 0x4000 + bank * 0x4000]
        } else if addr < 0xa000 {
            eprintln!("bad cart access at {:#04x}", addr);
            0
        } else if addr < 0xc000 {
            let bank = self.ram_bank as usize;
            self.ram[addr as usize - 0xa000 + bank * 0x2000]
        } else {
            eprintln!("bad cart access at {:#04x}", addr);
            0
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
            let bank = self.ram_bank as usize;
            self.rom[addr as usize - 0xa000 + bank * 0x2000] = value;
        } else {
            eprintln!("bad cart access at {:#04x}", addr);
        }
    }
}

struct FlatMapper {
    rom: Box<[u8]>,
}

impl FlatMapper {
    fn new(data: Box<[u8]>) -> Self {
        FlatMapper { rom: data }
    }
}

impl MemoryBanks for FlatMapper {
    fn load(&self, addr: u16) -> u8 {
        self.rom[addr as usize]
    }
    fn store(&mut self, _addr: u16, _value: u8) {}
}

impl GBCCart {
    fn empty() -> Self {
        Self::raw(Vec::new())
    }

    fn boot_rom(mut boot_rom: Vec<u8>, cart: &GBCCart) -> Self {
        // copy in the bytes from $104 to $144
        for i in 0x104..0x144 {
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
