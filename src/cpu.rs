use std::fmt;

use rand::RngCore;
use rand::rngs::OsRng;

use yaxpeax_arch::{Arch, U8Reader};
use yaxpeax_sm83::{BitOp, RotOp, Op8bAOp, Reg8b, Reg16b, DerefReg};
use yaxpeax_sm83::SM83;

use crate::{MemoryBanks, MemoryMapping};
use crate::KEY1;

#[derive(Clone, PartialEq)]
pub(crate) struct Cpu {
    af: [u8; 2],
    bc: [u8; 2],
    de: [u8; 2],
    hl: [u8; 2],
    pub sp: u16,
    pub pc: u16,
    pub speed_mode: u8, // 0=Normal, 1=Double
    // interrupt master enable
    pub ime: bool,
    pub verbose: bool,
}

pub(crate) struct ExecutionEnvironment<'env, 'storage: 'env> {
    pub cpu: &'env mut Cpu,
    pub storage: &'env mut MemoryMapping<'storage>,
    pub clocks: u16,
}

impl<T: yaxpeax_arch::Reader<<SM83 as Arch>::Address, <SM83 as Arch>::Word>> yaxpeax_sm83::DecodeHandler<T> for ExecutionEnvironment<'_, '_> {
    fn on_word_read(&mut self, word: u8) {
        self.cpu.pc = self.cpu.pc.wrapping_add(1);
        self.clocks += 4;
    }

    fn on_ld_8b_imm(&mut self, op: Reg8b, imm: u8) -> Result<(), <SM83 as Arch>::DecodeError> {
        match op {
            Reg8b::A => { self.cpu.af[1] = imm; }
            Reg8b::B => { self.cpu.bc[1] = imm; }
            Reg8b::C => { self.cpu.bc[0] = imm; }
            Reg8b::D => { self.cpu.de[1] = imm; }
            Reg8b::E => { self.cpu.de[0] = imm; }
            Reg8b::H => { self.cpu.hl[1] = imm; }
            Reg8b::L => { self.cpu.hl[0] = imm; }
            Reg8b::DerefHL => { self.storage.store(u16::from_le_bytes(self.cpu.hl), imm) }
        };
        Ok(())
    }

    fn on_inc_8b(&mut self, op: Reg8b) -> Result<(), <SM83 as Arch>::DecodeError> {
        fn do_reg_inc(reg: &mut u8, f: &mut u8) {
            let v = *reg;
            if v & 0x0f == 0x0f {
                *f |= 0b0010_0000;
            } else {
                *f &= !0b0010_0000;
            }
            *f |= 0b0100_0000;
            if v == 0xff {
                *f |= 0b1000_0000;
            } else {
                *f &= !0b1000_0000;
            }
            let v = v.wrapping_add(1);
            *reg = v;
        }
        match op {
            Reg8b::A => {
                let (l, r) = self.cpu.af.split_at_mut(1);
                do_reg_inc(&mut r[0], &mut l[0]);
            }
            Reg8b::B => { do_reg_inc(&mut self.cpu.bc[1], &mut self.cpu.af[0]); }
            Reg8b::C => { do_reg_inc(&mut self.cpu.bc[0], &mut self.cpu.af[0]); }
            Reg8b::D => { do_reg_inc(&mut self.cpu.de[1], &mut self.cpu.af[0]); }
            Reg8b::E => { do_reg_inc(&mut self.cpu.de[0], &mut self.cpu.af[0]); }
            Reg8b::H => { do_reg_inc(&mut self.cpu.hl[1], &mut self.cpu.af[0]); }
            Reg8b::L => { do_reg_inc(&mut self.cpu.hl[0], &mut self.cpu.af[0]); }
            Reg8b::DerefHL => {
                let hl = u16::from_le_bytes(self.cpu.hl);
                let mut v = self.storage.load(hl);
                do_reg_inc(&mut v, &mut self.cpu.af[0]);
                self.storage.store(hl, v);
            }
        };
        Ok(())
    }

    fn on_dec_8b(&mut self, op: Reg8b) -> Result<(), <SM83 as Arch>::DecodeError> {
        fn do_reg_dec(reg: &mut u8, f: &mut u8) {
            let v = *reg;
            if v & 0x0f == 0x00 {
                *f |= 0b0010_0000;
            } else {
                *f &= !0b0010_0000;
            }
            *f |= 0b0100_0000;
            if v == 0x01 {
                *f |= 0b1000_0000;
            } else {
                *f &= !0b1000_0000;
            }
            let v = v.wrapping_sub(1);
            *reg = v;
        }
        match op {
            Reg8b::A => {
                let (l, r) = self.cpu.af.split_at_mut(1);
                do_reg_dec(&mut r[0], &mut l[0]);
            }
            Reg8b::B => { do_reg_dec(&mut self.cpu.bc[1], &mut self.cpu.af[0]); }
            Reg8b::C => { do_reg_dec(&mut self.cpu.bc[0], &mut self.cpu.af[0]); }
            Reg8b::D => { do_reg_dec(&mut self.cpu.de[1], &mut self.cpu.af[0]); }
            Reg8b::E => { do_reg_dec(&mut self.cpu.de[0], &mut self.cpu.af[0]); }
            Reg8b::H => { do_reg_dec(&mut self.cpu.hl[1], &mut self.cpu.af[0]); }
            Reg8b::L => { do_reg_dec(&mut self.cpu.hl[0], &mut self.cpu.af[0]); }
            Reg8b::DerefHL => {
                let hl = u16::from_le_bytes(self.cpu.hl);
                let mut v = self.storage.load(hl);
                do_reg_dec(&mut v, &mut self.cpu.af[0]);
                self.storage.store(hl, v);
            }
        };
        Ok(())
    }

    fn on_stop(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> {
        // if KEY1.0 is set, maybe switch clock speeds. otherwise, .. stop?
        let key1 = self.storage.load(0xff4d);
        if key1 & 0b01 != 0 {
            self.cpu.speed_mode ^= 0b01;
            if self.cpu.verbose {
                eprintln!("switching speed mode to {}", self.cpu.speed_mode);
            }
            self.storage.management_bits[KEY1] &= 0b0111_1111;
            self.storage.management_bits[KEY1] |= self.cpu.speed_mode << 7;
        } else {
            eprintln!("stopping");
            panic!("");
        }
        Ok(())
    }

    fn on_cpl(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> {
        self.cpu.af[1] ^= 0xff;
        self.cpu.af[0] |= 0b0110_0000;
        Ok(())
    }

    fn on_di(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> {
        self.cpu.ime = false;
        Ok(())
    }

    fn on_ei(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> {
        self.cpu.ime = false;
        Ok(())
    }
    fn on_ld_r_r(&mut self, dest: Reg8b, src: Reg8b) -> Result<(), <SM83 as Arch>::DecodeError> {
        let v = match src {
            Reg8b::A => { self.cpu.af[1] }
            Reg8b::B => { self.cpu.bc[1] }
            Reg8b::C => { self.cpu.bc[0] }
            Reg8b::D => { self.cpu.de[1] }
            Reg8b::E => { self.cpu.de[0] }
            Reg8b::H => { self.cpu.hl[1] }
            Reg8b::L => { self.cpu.hl[0] }
            Reg8b::DerefHL => { self.storage.load(u16::from_le_bytes(self.cpu.hl)) }
        };
        match dest {
            Reg8b::A => { self.cpu.af[1] = v; }
            Reg8b::B => { self.cpu.bc[1] = v; }
            Reg8b::C => { self.cpu.bc[0] = v; }
            Reg8b::D => { self.cpu.de[1] = v; }
            Reg8b::E => { self.cpu.de[0] = v; }
            Reg8b::H => { self.cpu.hl[1] = v; }
            Reg8b::L => { self.cpu.hl[0] = v; }
            Reg8b::DerefHL => { self.storage.store(u16::from_le_bytes(self.cpu.hl), v) }
        };
        Ok(())
    }
    fn on_ld_8b_mem_a(&mut self, reg: DerefReg) -> Result<(), <SM83 as Arch>::DecodeError> {
        match reg {
            DerefReg::DerefBC => {
                self.storage.store(u16::from_le_bytes(self.cpu.bc), self.cpu.af[1]);
            },
            DerefReg::DerefDE => {
                self.storage.store(u16::from_le_bytes(self.cpu.de), self.cpu.af[1]);
            },
            DerefReg::DerefIncHL => {
                self.storage.store(u16::from_le_bytes(self.cpu.hl), self.cpu.af[1]);
                self.cpu.hl = u16::from_le_bytes(self.cpu.hl).wrapping_add(1).to_le_bytes();
            },
            DerefReg::DerefDecHL => {
                self.storage.store(u16::from_le_bytes(self.cpu.hl), self.cpu.af[1]);
                self.cpu.hl = u16::from_le_bytes(self.cpu.hl).wrapping_sub(1).to_le_bytes();
            },
        }
        Ok(())
    }
    fn on_ld_8b_a_mem(&mut self, reg: DerefReg) -> Result<(), <SM83 as Arch>::DecodeError> {
        let v = match reg {
            DerefReg::DerefBC => {
                self.storage.load(u16::from_le_bytes(self.cpu.bc))
            },
            DerefReg::DerefDE => {
                self.storage.load(u16::from_le_bytes(self.cpu.de))
            },
            DerefReg::DerefIncHL => {
                let v = self.storage.load(u16::from_le_bytes(self.cpu.hl));
                self.cpu.hl = u16::from_le_bytes(self.cpu.hl).wrapping_add(1).to_le_bytes();
                v
            },
            DerefReg::DerefDecHL => {
                let v = self.storage.load(u16::from_le_bytes(self.cpu.hl));
                self.cpu.hl = u16::from_le_bytes(self.cpu.hl).wrapping_sub(1).to_le_bytes();
                v
            },
        };
        self.cpu.af[1] = v;
        Ok(())
    }
    fn on_ld_a16_sp(&mut self, addr: u16) -> Result<(), <SM83 as Arch>::DecodeError> {
        let bytes = self.cpu.sp.to_le_bytes();
        self.storage.store(addr, bytes[0]);
        self.storage.store(addr.wrapping_add(1), bytes[1]);
        Ok(())
    }
    fn on_ld_rr_d16(&mut self, op: Reg16b, v: u16) -> Result<(), <SM83 as Arch>::DecodeError> {
        match op {
            Reg16b::BC => { self.cpu.bc = v.to_le_bytes(); },
            Reg16b::DE => { self.cpu.de = v.to_le_bytes(); },
            Reg16b::HL => { self.cpu.hl = v.to_le_bytes(); },
            Reg16b::SP => { self.cpu.sp = v; },
        };
        Ok(())
    }
    fn on_ld_sp_hl(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> {
        self.cpu.sp = u16::from_le_bytes(self.cpu.hl);
        Ok(())
    }
    fn on_ld_a_8b_deref_addr(&mut self, addr: u16) -> Result<(), <SM83 as Arch>::DecodeError> {
        self.cpu.af[1] = self.storage.load(addr);
        Ok(())
    }
    fn on_ld_8b_deref_addr_a(&mut self, addr: u16) -> Result<(), <SM83 as Arch>::DecodeError> {
        self.storage.store(addr, self.cpu.af[1]);
        Ok(())
    }
    fn on_ld_hl_deref_sp_offset(&mut self, ofs: i8) -> Result<(), <SM83 as Arch>::DecodeError> {
        let addr = self.cpu.sp.wrapping_add(ofs as i16 as u16);
        let low = self.storage.load(addr);
        let high = self.storage.load(addr.wrapping_add(1));
        self.cpu.hl = [low, high];
        Ok(())
    }
    fn on_ldh_a_deref_high_8b(&mut self, ofs: u8) -> Result<(), <SM83 as Arch>::DecodeError> {
        self.cpu.af[1] = self.storage.load(0xff00 + ofs as u16);
        Ok(())
    }
    fn on_ldh_deref_high_8b_a(&mut self, ofs: u8) -> Result<(), <SM83 as Arch>::DecodeError> {
        self.storage.store(0xff00 + ofs as u16, self.cpu.af[1]);
        Ok(())
    }
    fn on_ldh_deref_high_c_a(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> {
        self.storage.store(0xff00 + self.cpu.bc[0] as u16, self.cpu.af[1]);
        Ok(())
    }
    fn on_ldh_a_deref_high_c(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> {
        self.cpu.af[1] = self.storage.load(0xff00 + self.cpu.bc[0] as u16);
        Ok(())
    }
    fn on_add_16b_hl_rr(&mut self, op: Reg16b) -> Result<(), <SM83 as Arch>::DecodeError> {
        let hl = u16::from_le_bytes(self.cpu.hl);
        let other = match op {
            Reg16b::BC => { u16::from_le_bytes(self.cpu.bc) },
            Reg16b::DE => { u16::from_le_bytes(self.cpu.de) },
            Reg16b::HL => { u16::from_le_bytes(self.cpu.hl) },
            Reg16b::SP => { self.cpu.sp },
        };
        let (res, carry) = hl.overflowing_add(other);
        self.cpu.hl = res.to_le_bytes();
        self.cpu.af[0] &= !0b0111_0000;
        if carry {
            self.cpu.af[0] |= 0b0001_0000;
        }
        // half-carry is at bit 11 for hl?
        if (hl & 0x0fff) + (other & 0x0fff) > 0x0fff {
            self.cpu.af[0] |= 0b0010_0000;
        }
        Ok(())
    }
    fn on_add_sp_i8(&mut self, imm: i8) -> Result<(), <SM83 as Arch>::DecodeError> {
        let sp = self.cpu.sp;
        let other = imm as i16 as u16;
        let (res, carry) = sp.overflowing_add(other);
        self.cpu.sp = res;
        self.cpu.af[0] &= !0b0111_0000;
        // TODO: is this for a carry out of 7, or out of 15...
        if carry {
            self.cpu.af[0] |= 0b0001_0000;
        }
        // half-carry is at bit 3 for sp? source:
        // https://stackoverflow.com/questions/57958631/game-boy-half-carry-flag-and-16-bit-instructions-especially-opcode-0xe8
        if (sp & 0x000f) + (other & 0x000f) > 0x000f {
            self.cpu.af[0] |= 0b0010_0000;
        }
        Ok(())
    }
    fn on_inc_16b_rr(&mut self, op0: Reg16b) -> Result<(), <SM83 as Arch>::DecodeError> {
        match op0 {
            Reg16b::BC => { self.cpu.bc = u16::from_le_bytes(self.cpu.bc).wrapping_add(1).to_le_bytes(); },
            Reg16b::DE => { self.cpu.de = u16::from_le_bytes(self.cpu.de).wrapping_add(1).to_le_bytes(); },
            Reg16b::HL => { self.cpu.hl = u16::from_le_bytes(self.cpu.hl).wrapping_add(1).to_le_bytes(); },
            Reg16b::SP => { self.cpu.sp = self.cpu.sp.wrapping_add(1); },
        }
        Ok(())
    }
    fn on_dec_16b_rr(&mut self, op0: Reg16b) -> Result<(), <SM83 as Arch>::DecodeError> {
        match op0 {
            Reg16b::BC => { self.cpu.bc = u16::from_le_bytes(self.cpu.bc).wrapping_sub(1).to_le_bytes(); },
            Reg16b::DE => { self.cpu.de = u16::from_le_bytes(self.cpu.de).wrapping_sub(1).to_le_bytes(); },
            Reg16b::HL => { self.cpu.hl = u16::from_le_bytes(self.cpu.hl).wrapping_sub(1).to_le_bytes(); },
            Reg16b::SP => { self.cpu.sp = self.cpu.sp.wrapping_sub(1); },
        }
        Ok(())
    }
    fn on_jr_unconditional(&mut self, rel: i8) -> Result<(), <SM83 as Arch>::DecodeError> {
        self.cpu.pc = self.cpu.pc.wrapping_add(rel as i16 as u16);
        Ok(())
    }
    fn on_jr_nz(&mut self, rel: i8) -> Result<(), <SM83 as Arch>::DecodeError> {
        if (self.cpu.af[0] & 0b1000_0000) == 0 {
            <Self as yaxpeax_sm83::DecodeHandler::<T>>::on_jr_unconditional(self, rel)?;
        }
        Ok(())
    }
    fn on_jr_z(&mut self, rel: i8) -> Result<(), <SM83 as Arch>::DecodeError> {
        if (self.cpu.af[0] & 0b1000_0000) != 0 {
            <Self as yaxpeax_sm83::DecodeHandler::<T>>::on_jr_unconditional(self, rel)?;
        }
        Ok(())
    }
    fn on_jr_nc(&mut self, rel: i8) -> Result<(), <SM83 as Arch>::DecodeError> {
        if (self.cpu.af[0] & 0b0001_0000) == 0 {
            <Self as yaxpeax_sm83::DecodeHandler::<T>>::on_jr_unconditional(self, rel)?;
        }
        Ok(())
    }
    fn on_jr_c(&mut self, rel: i8) -> Result<(), <SM83 as Arch>::DecodeError> {
        if (self.cpu.af[0] & 0b0001_0000) != 0 {
            <Self as yaxpeax_sm83::DecodeHandler::<T>>::on_jr_unconditional(self, rel)?;
        }
        Ok(())
    }
    fn on_jp_unconditional(&mut self, addr: u16) -> Result<(), <SM83 as Arch>::DecodeError> {
        self.cpu.pc = addr;
        Ok(())
    }
    fn on_jp_nz(&mut self, addr: u16) -> Result<(), <SM83 as Arch>::DecodeError> {
        if (self.cpu.af[0] & 0b1000_0000) == 0 {
            self.cpu.pc = addr;
        }
        Ok(())
    }
    fn on_jp_z(&mut self, addr: u16) -> Result<(), <SM83 as Arch>::DecodeError> {
        if (self.cpu.af[0] & 0b1000_0000) != 0 {
            self.cpu.pc = addr;
        }
        Ok(())
    }
    fn on_jp_nc(&mut self, addr: u16) -> Result<(), <SM83 as Arch>::DecodeError> {
        if (self.cpu.af[0] & 0b0001_0000) == 0 {
            self.cpu.pc = addr;
        }
        Ok(())
    }
    fn on_jp_c(&mut self, addr: u16) -> Result<(), <SM83 as Arch>::DecodeError> {
        if (self.cpu.af[0] & 0b0001_0000) != 0 {
            self.cpu.pc = addr;
        }
        Ok(())
    }
    fn on_jp_hl(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> {
        self.cpu.pc = u16::from_le_bytes(self.cpu.hl);
        Ok(())
    }
    fn on_call_unconditional(&mut self, addr: u16) -> Result<(), <SM83 as Arch>::DecodeError> {
        self.cpu.push(self.storage, self.cpu.pc);
        self.cpu.pc = addr;
        Ok(())
    }
    fn on_call_nz(&mut self, addr: u16) -> Result<(), <SM83 as Arch>::DecodeError> {
        if (self.cpu.af[0] & 0b1000_0000) == 0 {
            <Self as yaxpeax_sm83::DecodeHandler::<T>>::on_call_unconditional(self, addr)?;
        }
        Ok(())
    }
    fn on_call_z(&mut self, addr: u16) -> Result<(), <SM83 as Arch>::DecodeError> {
        if (self.cpu.af[0] & 0b1000_0000) != 0 {
            <Self as yaxpeax_sm83::DecodeHandler::<T>>::on_call_unconditional(self, addr)?;
        }
        Ok(())
    }
    fn on_call_nc(&mut self, addr: u16) -> Result<(), <SM83 as Arch>::DecodeError> {
        if (self.cpu.af[0] & 0b0001_0000) == 0 {
            <Self as yaxpeax_sm83::DecodeHandler::<T>>::on_call_unconditional(self, addr)?;
        }
        Ok(())
    }
    fn on_call_c(&mut self, addr: u16) -> Result<(), <SM83 as Arch>::DecodeError> {
        if (self.cpu.af[0] & 0b0001_0000) != 0 {
            <Self as yaxpeax_sm83::DecodeHandler::<T>>::on_call_unconditional(self, addr)?;
        }
        Ok(())
    }
    fn on_bit_op(&mut self, op: BitOp, bit: u8, operand: Reg8b) -> Result<(), <SM83 as Arch>::DecodeError> {
        match op {
            BitOp::BIT => {
                let v = match operand {
                    Reg8b::B => { self.cpu.bc[1] },
                    Reg8b::C => { self.cpu.bc[0] },
                    Reg8b::D => { self.cpu.de[1] },
                    Reg8b::E => { self.cpu.de[0] },
                    Reg8b::H => { self.cpu.hl[1] },
                    Reg8b::L => { self.cpu.hl[0] },
                    Reg8b::DerefHL => { self.storage.load(u16::from_le_bytes(self.cpu.hl)) },
                    Reg8b::A => { self.cpu.af[1] },
                };
                let res = v & (1 << bit);
                self.cpu.af[0] &= !0b1110_0000;
                self.cpu.af[0] |= 0b0010_0000;
                if res == 0 {
                    self.cpu.af[0] |= 0b1000_0000;
                }
            },
            BitOp::RES => {
                match operand {
                    Reg8b::B => { self.cpu.bc[1] &= !(1 << bit) },
                    Reg8b::C => { self.cpu.bc[0] &= !(1 << bit) },
                    Reg8b::D => { self.cpu.de[1] &= !(1 << bit) },
                    Reg8b::E => { self.cpu.de[0] &= !(1 << bit) },
                    Reg8b::H => { self.cpu.hl[1] &= !(1 << bit) },
                    Reg8b::L => { self.cpu.hl[0] &= !(1 << bit) },
                    Reg8b::DerefHL => {
                        let addr = u16::from_le_bytes(self.cpu.hl);
                        let v = self.storage.load(addr);
                        self.storage.store(addr, v & !(1 << bit));
                    },
                    Reg8b::A => { self.cpu.af[1] &= !(1 << bit) },
                };
            }
            BitOp::SET => {
                match operand {
                    Reg8b::B => { self.cpu.bc[1] |= 1 << bit },
                    Reg8b::C => { self.cpu.bc[0] |= 1 << bit },
                    Reg8b::D => { self.cpu.de[1] |= 1 << bit },
                    Reg8b::E => { self.cpu.de[0] |= 1 << bit },
                    Reg8b::H => { self.cpu.hl[1] |= 1 << bit },
                    Reg8b::L => { self.cpu.hl[0] |= 1 << bit },
                    Reg8b::DerefHL => {
                        let addr = u16::from_le_bytes(self.cpu.hl);
                        let v = self.storage.load(addr);
                        self.storage.store(addr, v | (1 << bit));
                    },
                    Reg8b::A => { self.cpu.af[1] |= 1 << bit },
                };
            }
        }
        Ok(())
    }
    fn on_op_8b_a_r(&mut self, op: Op8bAOp, operand: Reg8b) -> Result<(), <SM83 as Arch>::DecodeError> {
        let left = self.cpu.af[1];
        let right = match operand {
            Reg8b::B => { self.cpu.bc[1] },
            Reg8b::C => { self.cpu.bc[0] },
            Reg8b::D => { self.cpu.de[1] },
            Reg8b::E => { self.cpu.de[0] },
            Reg8b::H => { self.cpu.hl[1] },
            Reg8b::L => { self.cpu.hl[0] },
            Reg8b::DerefHL => { self.storage.load(u16::from_le_bytes(self.cpu.hl)) }
            Reg8b::A => { self.cpu.af[1] },
        };
        match op {
            Op8bAOp::ADD => {
                let (res, carry) = left.overflowing_add(right);
                let h = (left & 0xf) + (right & 0xf) > 0xf;
                self.cpu.af[0] = 0b0000_0000;
                if carry { self.cpu.af[0] |= 0b0001_0000; }
                if h { self.cpu.af[0] |= 0b0010_0000; }
                if res == 0 { self.cpu.af[0] |= 0b1000_0000; }
                self.cpu.af[1] = res;
            },
            Op8bAOp::ADC => {
                panic!("adc");
            },
            Op8bAOp::SUB => {
                let (res, carry) = left.overflowing_sub(right);
                let h = (left & 0xf).wrapping_sub(right & 0xf) > 0xf;
                self.cpu.af[0] = 0b0100_0000;
                if carry { self.cpu.af[0] |= 0b0001_0000; }
                if h { self.cpu.af[0] |= 0b0010_0000; }
                if res == 0 { self.cpu.af[0] |= 0b1000_0000; }
                self.cpu.af[1] = res;
            },
            Op8bAOp::SBC => {
                panic!("sbc");
            },
            Op8bAOp::AND => {
                self.cpu.af[1] = left & right;
                self.cpu.af[0] = 0b0010_0000;
                if self.cpu.af[1] == 0 {
                    self.cpu.af[0] |= 0b1000_0000;
                }
            },
            Op8bAOp::XOR => {
                self.cpu.af[1] = left ^ right;
                self.cpu.af[0] = 0b0000_0000;
                if self.cpu.af[1] == 0 {
                    self.cpu.af[0] |= 0b1000_0000;
                }
            },
            Op8bAOp::OR => {
                self.cpu.af[1] = left | right;
                self.cpu.af[0] = 0b0000_0000;
                if self.cpu.af[1] == 0 {
                    self.cpu.af[0] |= 0b1000_0000;
                }
            },
            Op8bAOp::CP => {
                let (res, carry) = left.overflowing_sub(right);
                let h = (left & 0xf).wrapping_sub(right & 0xf) > 0xf;
                self.cpu.af[0] = 0b0100_0000;
                if carry { self.cpu.af[0] |= 0b0001_0000; }
                if h { self.cpu.af[0] |= 0b0010_0000; }
                if res == 0 { self.cpu.af[0] |= 0b1000_0000; }
            },
        };
        Ok(())
    }
    fn on_op_8b_a_d8(&mut self, op: Op8bAOp, operand: u8) -> Result<(), <SM83 as Arch>::DecodeError> {
        let left = self.cpu.af[1];
        let right = operand;
        match op {
            Op8bAOp::ADD => {
                let (res, carry) = left.overflowing_add(right);
                let h = (left & 0xf) + (right & 0xf) > 0xf;
                self.cpu.af[0] = 0b0000_0000;
                if carry { self.cpu.af[0] |= 0b0001_0000; }
                if h { self.cpu.af[0] |= 0b0010_0000; }
                if res == 0 { self.cpu.af[0] |= 0b1000_0000; }
                self.cpu.af[1] = res;
            },
            Op8bAOp::ADC => {
                panic!("adc");
            },
            Op8bAOp::SUB => {
                let (res, carry) = left.overflowing_sub(right);
                let h = (left & 0xf).wrapping_sub(right & 0xf) > 0xf;
                self.cpu.af[0] = 0b0100_0000;
                if carry { self.cpu.af[0] |= 0b0001_0000; }
                if h { self.cpu.af[0] |= 0b0010_0000; }
                if res == 0 { self.cpu.af[0] |= 0b1000_0000; }
                self.cpu.af[1] = res;
            },
            Op8bAOp::SBC => {
                panic!("sbc");
            },
            Op8bAOp::AND => {
                self.cpu.af[1] = left & right;
                self.cpu.af[0] = 0b0010_0000;
                if self.cpu.af[1] == 0 {
                    self.cpu.af[0] |= 0b1000_0000;
                }
            },
            Op8bAOp::XOR => {
                self.cpu.af[1] = left ^ right;
                self.cpu.af[0] = 0b0000_0000;
                if self.cpu.af[1] == 0 {
                    self.cpu.af[0] |= 0b1000_0000;
                }
            },
            Op8bAOp::OR => {
                self.cpu.af[1] = left | right;
                self.cpu.af[0] = 0b0000_0000;
                if self.cpu.af[1] == 0 {
                    self.cpu.af[0] |= 0b1000_0000;
                }
            },
            Op8bAOp::CP => {
                let (res, carry) = left.overflowing_sub(right);
                let h = (left & 0xf).wrapping_sub(right & 0xf) > 0xf;
                self.cpu.af[0] = 0b0100_0000;
                if carry { self.cpu.af[0] |= 0b0001_0000; }
                if h { self.cpu.af[0] |= 0b0010_0000; }
                if res == 0 { self.cpu.af[0] |= 0b1000_0000; }
            },
        };
        Ok(())
    }
    fn on_rotate_8b_r(&mut self, op: RotOp, operand: Reg8b) -> Result<(), <SM83 as Arch>::DecodeError> {
        let v = match operand {
            Reg8b::B => { self.cpu.bc[1] },
            Reg8b::C => { self.cpu.bc[0] },
            Reg8b::D => { self.cpu.de[1] },
            Reg8b::E => { self.cpu.de[0] },
            Reg8b::H => { self.cpu.hl[1] },
            Reg8b::L => { self.cpu.hl[0] },
            Reg8b::DerefHL => { self.storage.load(u16::from_le_bytes(self.cpu.hl)) },
            Reg8b::A => { self.cpu.af[1] },
        };
        let res = match op {
            RotOp::RL => {
                let c = (self.cpu.af[0] >> 4) & 0b0001;
                self.cpu.af[0] = 0b0000_0000;
                if v & 0x80 != 0 {
                    self.cpu.af[0] |= 0b0001_0000;
                }
                let rotated = (v << 1) | c;
                if rotated == 0 {
                    self.cpu.af[0] |= 0b1000_0000;
                }
                rotated
            }
            RotOp::RR => {
                let c = (self.cpu.af[0] >> 4) & 0b0001;
                self.cpu.af[0] = 0b0000_0000;
                if v & 0x01 != 0 {
                    self.cpu.af[0] |= 0b0001_0000;
                }
                let rotated = (v >> 1) | (c << 7);
                if rotated == 0 {
                    self.cpu.af[0] |= 0b1000_0000;
                }
                rotated
            }
            RotOp::RLC => {
                let mut c = 0;
                self.cpu.af[0] = 0b0000_0000;
                if v & 0x80 != 0 {
                    c = 0x01;
                    self.cpu.af[0] |= 0b0001_0000;
                }
                let rotated = (v << 1) | c;
                if rotated == 0 {
                    self.cpu.af[0] |= 0b1000_0000;
                }
                rotated
            }
            RotOp::RRC => {
                let mut c = 0;
                self.cpu.af[0] = 0b0000_0000;
                if v & 0x01 != 0 {
                    c = 0x80;
                    self.cpu.af[0] |= 0b0001_0000;
                }
                let rotated = (v >> 1) | c;
                if rotated == 0 {
                    self.cpu.af[0] |= 0b1000_0000;
                }
                rotated
            }
            RotOp::SLA => {
                self.cpu.af[0] = 0b0000_0000;
                if v & 0x80 != 0 {
                    self.cpu.af[0] |= 0b0001_0000;
                }
                let shifted = v << 1;
                if shifted == 0 {
                    self.cpu.af[0] |= 0b1000_0000;
                }
                shifted
            }
            RotOp::SRA => {
                self.cpu.af[0] = 0b0000_0000;
                if v & 0x01 != 0 {
                    self.cpu.af[0] |= 0b0001_0000;
                }
                let shifted = v >> 1;
                if shifted == 0 {
                    self.cpu.af[0] |= 0b1000_0000;
                }
                shifted
            }
            RotOp::SRL => {
                self.cpu.af[0] = 0b0000_0000;
                if v & 0x01 != 0 {
                    self.cpu.af[0] |= 0b0001_0000;
                }
                let shifted = ((v as i8) >> 1) as u8;
                if shifted == 0 {
                    self.cpu.af[0] |= 0b1000_0000;
                }
                shifted
            }
            RotOp::SWAP => {
                let swapped = (v >> 4) | (v << 4);
                self.cpu.af[0] = 0b0000_0000;
                if swapped == 0 {
                    self.cpu.af[0] |= 0b1000_0000;
                }
                swapped
            }
        };
        match operand {
            Reg8b::B => { self.cpu.bc[1] = res; }
            Reg8b::C => { self.cpu.bc[0] = res; }
            Reg8b::D => { self.cpu.hl[1] = res; }
            Reg8b::E => { self.cpu.hl[0] = res; }
            Reg8b::H => { self.cpu.hl[1] = res; }
            Reg8b::L => { self.cpu.hl[0] = res; }
            Reg8b::DerefHL => { self.storage.store(u16::from_le_bytes(self.cpu.hl), res) }
            Reg8b::A => { self.cpu.af[1] = res; }
        };
        Ok(())
    }
    fn on_push_af(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> {
        self.cpu.push(self.storage, u16::from_le_bytes(self.cpu.af));
        Ok(())
    }
    fn on_pop_af(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> {
        self.cpu.af = self.cpu.pop(self.storage).to_le_bytes();
        Ok(())
    }
    fn on_push_bc(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> {
        self.cpu.push(self.storage, u16::from_le_bytes(self.cpu.bc));
        Ok(())
    }
    fn on_pop_bc(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> {
        self.cpu.bc = self.cpu.pop(self.storage).to_le_bytes();
        Ok(())
    }
    fn on_push_de(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> {
        self.cpu.push(self.storage, u16::from_le_bytes(self.cpu.de));
        Ok(())
    }
    fn on_pop_de(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> {
        self.cpu.de = self.cpu.pop(self.storage).to_le_bytes();
        Ok(())
    }
    fn on_push_hl(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> {
        self.cpu.push(self.storage, u16::from_le_bytes(self.cpu.hl));
        Ok(())
    }
    fn on_pop_hl(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> {
        self.cpu.hl = self.cpu.pop(self.storage).to_le_bytes();
        Ok(())
    }
    fn on_nop(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_rlca(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> {
        let v = self.cpu.af[1];
        let mut c = 0;
        self.cpu.af[0] = 0b0000_0000;
        if v & 0x80 != 0 {
            c = 0x01;
            self.cpu.af[0] |= 0b0001_0000;
        }
        let rotated = (v << 1) | c;
        if rotated == 0 {
            self.cpu.af[0] |= 0b1000_0000;
        }
        self.cpu.af[1] = rotated;
        Ok(())
    }
    fn on_rrca(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> {
        let v = self.cpu.af[1];
        let mut c = 0;
        self.cpu.af[0] = 0b0000_0000;
        if v & 0x01 != 0 {
            c = 0x80;
            self.cpu.af[0] |= 0b0001_0000;
        }
        let rotated = (v >> 1) | c;
        if rotated == 0 {
            self.cpu.af[0] |= 0b1000_0000;
        }
        self.cpu.af[1] = rotated;
        Ok(())
    }
    fn on_rla(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> {
        let v = self.cpu.af[1];
        let c = (self.cpu.af[0] >> 4) & 0b0001;
        self.cpu.af[0] = 0b0000_0000;
        if v & 0x80 != 0 {
            self.cpu.af[0] |= 0b0001_0000;
        }
        let rotated = (v << 1) | c;
        if rotated == 0 {
            self.cpu.af[0] |= 0b1000_0000;
        }
        self.cpu.af[1] = rotated;
        Ok(())
    }
    fn on_rra(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> {
        let v = self.cpu.af[1];
        let c = (self.cpu.af[0] >> 4) & 0b0001;
        self.cpu.af[0] = 0b0000_0000;
        if v & 0x01 != 0 {
            self.cpu.af[0] |= 0b0001_0000;
        }
        let rotated = (v >> 1) | (c << 7);
        if rotated == 0 {
            self.cpu.af[0] |= 0b1000_0000;
        }
        self.cpu.af[1] = rotated;
        Ok(())
    }
    fn on_daa(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> { todo!() }
    fn on_scf(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> { todo!() }
    fn on_ccf(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> { todo!() }
    fn on_ret_unconditional(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> {
        self.cpu.pc = self.cpu.pop(self.storage);
        Ok(())
    }
    fn on_ret_nz(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> {
        if (self.cpu.af[0] & 0b1000_0000) == 0 {
            <Self as yaxpeax_sm83::DecodeHandler::<T>>::on_ret_unconditional(self)?;
        }
        Ok(())
    }
    fn on_ret_z(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> {
        if (self.cpu.af[0] & 0b1000_0000) != 0 {
            <Self as yaxpeax_sm83::DecodeHandler::<T>>::on_ret_unconditional(self)?;
        }
        Ok(())
    }
    fn on_ret_nc(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> {
        if (self.cpu.af[0] & 0b0001_0000) == 0 {
            <Self as yaxpeax_sm83::DecodeHandler::<T>>::on_ret_unconditional(self)?;
        }
        Ok(())
    }
    fn on_ret_c(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> {
        if (self.cpu.af[0] & 0b0001_0000) != 0 {
            <Self as yaxpeax_sm83::DecodeHandler::<T>>::on_ret_unconditional(self)?;
        }
        Ok(())
    }
    fn on_rst(&mut self, imm: u8) -> Result<(), <SM83 as Arch>::DecodeError> { todo!() }
    fn on_reti(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> {
        self.cpu.pc = self.cpu.pop(self.storage);
        self.cpu.ime = true;
//        return 16;
        Ok(())
    }
}

impl fmt::Debug for Cpu {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        writeln!(f, "sm83 cpu (mode: {}):", ["normal", "double"][self.speed_mode as usize])?;
        write!(f, "        a f       b c")?;
        write!(f,  "       d e       h l")?;
        writeln!(f, "")?;
        write!(f, "  af: ${:02x}{:02x} bc: ${:02x}{:02x}", self.af[1], self.af[0], self.bc[1], self.bc[0])?;
        write!(f,  " de: ${:02x}{:02x} hl: ${:02x}{:02x}", self.de[1], self.de[0], self.hl[1], self.hl[0])?;
        write!(f,  " sp: ${:04x} pc: ${:04x}", self.sp, self.pc)?;
        writeln!(f, "")?;
        if self.sp & 1 != 0 {
            writeln!(f, "    [!] sp misaligned")?;
        }
        Ok(())
    }
}

impl Cpu {
    pub fn new() -> Self {
        Self {
            af: [OsRng.next_u32() as u8, OsRng.next_u32() as u8],
            bc: [OsRng.next_u32() as u8, OsRng.next_u32() as u8],
            de: [OsRng.next_u32() as u8, OsRng.next_u32() as u8],
            hl: [OsRng.next_u32() as u8, OsRng.next_u32() as u8],
            sp: OsRng.next_u32() as u16,
            pc: 0,
            speed_mode: 0,
            ime: true,
            verbose: true,
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

    pub fn step<'storage: 'env, 'env>(&'env mut self, memory: &'env mut MemoryMapping<'storage>) -> u16 {
        let mut buf = [
            memory.load(self.pc),
            memory.load(self.pc + 1),
            memory.load(self.pc + 2),
            memory.load(self.pc + 3),
        ];
        let mut reader = U8Reader::new(&buf);
        let decoder = yaxpeax_sm83::InstDecoder::default();
        let mut env = ExecutionEnvironment { cpu: self, storage: memory, clocks: 0 };

        yaxpeax_sm83::decode_inst(&decoder, &mut env, &mut reader).unwrap();

        return env.clocks;

                /*
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
        */
    }
}


mod test {
    use crate::{Cpu, FlatMapper, MemoryMapping};

    fn execute_test(cpu: &mut Cpu, program: &[u8]) {
        let mut rom = program.to_vec();
        while rom.len() < 4 {
            rom.push(0u8);
        }

        let mut rom = FlatMapper::new(rom.into_boxed_slice());
        let mut memory = MemoryMapping {
            cart: &mut rom,
            ram: &mut [],
            vram: &mut [],
            management_bits: &mut [],
            verbose: false,
        };
        cpu.step(&mut memory);
    }

    mod ld {
        use crate::Cpu;
        use crate::FlatMapper;
        use crate::MemoryMapping;

        #[test]
        fn test_jp_jcc() {
            let mut cpu = Cpu::new();
            cpu.af[0] = 0;
            let mut result = cpu.clone();
            result.pc += 3;
            super::execute_test(&mut cpu, &[0x20, 0x01]);

            assert_eq!(cpu, result);

            let mut cpu = Cpu::new();
            cpu.af[0] = 0xff;
            let mut result = cpu.clone();
            result.pc += 2;
            super::execute_test(&mut cpu, &[0x20, 0x01]);

            assert_eq!(cpu, result);

            let mut cpu = Cpu::new();
            cpu.af[0] = 0;
            let mut result = cpu.clone();
            result.pc += 2;
            super::execute_test(&mut cpu, &[0x20, 0x00]);

            assert_eq!(cpu, result);

            let mut cpu = Cpu::new();
            cpu.af[0] = 0xff;
            let mut result = cpu.clone();
            result.pc += 2;
            super::execute_test(&mut cpu, &[0x20, 0x00]);

            assert_eq!(cpu, result);
        }

        #[test]
        fn test_ld_imm() {
            let mut cpu = Cpu::new();
            let mut result = cpu.clone();
            result.af[1] = 0x01;
            result.pc += 2;
            super::execute_test(&mut cpu, &[0x3e, 0x01]);

            assert_eq!(cpu, result);

            let mut cpu = Cpu::new();
            let mut result = cpu.clone();
            result.bc = [0x00, 0x04];
            result.pc += 3;
            super::execute_test(&mut cpu, &[0x01, 0x00, 0x04]);

            assert_eq!(cpu, result);

            let mut cpu = Cpu::new();
            let mut result = cpu.clone();
            result.hl = [0x98, 0x00];
            result.pc += 3;
            super::execute_test(&mut cpu, &[0x21, 0x98, 0x00]);

            assert_eq!(cpu, result);
        }
    }
}