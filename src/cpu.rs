use std::fmt;
use std::collections::VecDeque;

use rand::RngCore;
use rand::rngs::OsRng;

use yaxpeax_arch::{Arch, U8Reader};
use yaxpeax_sm83::{BitOp, RotOp, Op8bAOp, Reg8b, Reg16b, DerefReg};
use yaxpeax_sm83::SM83;

use crate::{MemoryBanks, MemoryMapping};
use crate::KEY1;

const TRACE_DEPTH: usize = 40;

#[derive(PartialEq, Clone, Copy)]
pub struct BranchAddrs {
    from: u16,
    to: u16,
    from_linear: crate::MemoryAddress,
    to_linear: crate::MemoryAddress,
}

impl fmt::Debug for BranchAddrs {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{:#x} ({}:{:#x}) -> {:#x} ({}:{:#x})",
            self.from,
            self.from_linear.segment_name(),
            self.from_linear.address,
            self.to,
            self.to_linear.segment_name(),
            self.to_linear.address,
        )
    }
}

#[derive(Debug, PartialEq, Clone)]
pub enum BranchTrace {
    Call { addrs: BranchAddrs },
    Branch { addrs: BranchAddrs, count: u32 },
    Ret { addrs: BranchAddrs },
    Reti { addrs: BranchAddrs },
    Int { addrs: BranchAddrs },
}

#[derive(Debug, PartialEq, Clone)]
pub enum CallKind {
    Call,
    Int { iflags: u8 },
}

#[derive(Debug, PartialEq, Clone)]
pub struct CallRecord {
    addrs: BranchAddrs,
    call_kind: CallKind,
}

impl CallRecord {
    fn call(addrs: BranchAddrs) -> Self {
        CallRecord { addrs, call_kind: CallKind::Call }
    }

    fn int(addrs: BranchAddrs, iflags: u8) -> Self {
        CallRecord { addrs, call_kind: CallKind::Int { iflags } }
    }
}

#[derive(Clone, PartialEq)]
pub(crate) struct Cpu {
    pub af: [u8; 2],
    pub bc: [u8; 2],
    pub de: [u8; 2],
    pub hl: [u8; 2],
    pub sp: u16,
    pub pc: u16,
    pub speed_mode: u8, // 0=Normal, 1=Double
    // interrupt master enable
    pub ime: bool,
    pub halted: bool,
    pub verbose: bool,
    pub branch_trace: VecDeque<BranchTrace>,
    pub call_stack: Vec<CallRecord>,
}

pub(crate) struct ExecutionEnvironment<'env, 'storage: 'env> {
    pub cpu: &'env mut Cpu,
    pub storage: &'env mut MemoryMapping<'storage>,
    pub clocks: u16,
}

impl ExecutionEnvironment<'_, '_> {
    fn print_branch_trace(&self) {
        for record in self.cpu.branch_trace.iter() {
            eprintln!("  {:?}", record);
        }
    }
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
            *f &= !0b0100_0000;
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
//                eprintln!("switching speed mode to {}", self.cpu.speed_mode);
            }
            self.storage.state.management_bits[KEY1] &= 0b0111_1111;
            self.storage.state.management_bits[KEY1] |= self.cpu.speed_mode << 7;
        } else {
            eprintln!("stopping");
            panic!("");
        }
        Ok(())
    }

    fn on_halt(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> {
        self.cpu.halted = true;
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
        self.cpu.ime = true;
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
    fn on_ld_hl_sp_offset(&mut self, ofs: i8) -> Result<(), <SM83 as Arch>::DecodeError> {
        let l = self.cpu.sp;
        let r = ofs as i16 as u16;
        let res = l.wrapping_add(r);
        self.cpu.hl = res.to_le_bytes();
        self.cpu.af[0] = 0x00;
        if (l & 0x0f) + (r & 0x0f) >= 0x10 {
            self.cpu.flag_h_set(true);
        }
        if (l & 0xff) + (r & 0xff) >= 0x100 {
            self.cpu.flag_c_set(true);
        }
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
    #[inline(always)]
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
        let res = sp.wrapping_add(other);
        let carry = (sp as u8 as u16) + (imm as u8 as u16) >= 0x100;
        self.cpu.sp = res;
        self.cpu.af[0] = 0x00;
        // TODO: is this for a carry out of 7, or out of 15...
        // gameboy doctor says carry out of 7
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
    #[inline(always)]
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
        let target = self.cpu.pc.wrapping_add(rel as i16 as u16);
        self.cpu.trace_branch(&self.storage, self.cpu.pc, target);
        self.cpu.pc = target;
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
        self.cpu.trace_branch(&self.storage, self.cpu.pc, addr);
        self.cpu.pc = addr;
        Ok(())
    }
    fn on_jp_nz(&mut self, addr: u16) -> Result<(), <SM83 as Arch>::DecodeError> {
        if (self.cpu.af[0] & 0b1000_0000) == 0 {
            self.cpu.trace_branch(&self.storage, self.cpu.pc, addr);
            self.cpu.pc = addr;
        }
        Ok(())
    }
    fn on_jp_z(&mut self, addr: u16) -> Result<(), <SM83 as Arch>::DecodeError> {
        if (self.cpu.af[0] & 0b1000_0000) != 0 {
            self.cpu.trace_branch(&self.storage, self.cpu.pc, addr);
            self.cpu.pc = addr;
        }
        Ok(())
    }
    fn on_jp_nc(&mut self, addr: u16) -> Result<(), <SM83 as Arch>::DecodeError> {
        if (self.cpu.af[0] & 0b0001_0000) == 0 {
            self.cpu.trace_branch(&self.storage, self.cpu.pc, addr);
            self.cpu.pc = addr;
        }
        Ok(())
    }
    fn on_jp_c(&mut self, addr: u16) -> Result<(), <SM83 as Arch>::DecodeError> {
        if (self.cpu.af[0] & 0b0001_0000) != 0 {
            self.cpu.trace_branch(&self.storage, self.cpu.pc, addr);
            self.cpu.pc = addr;
        }
        Ok(())
    }
    fn on_jp_hl(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> {
        let target = u16::from_le_bytes(self.cpu.hl);
        self.cpu.trace_branch(&self.storage, self.cpu.pc, target);
        self.cpu.pc = target;
        Ok(())
    }
    fn on_call_unconditional(&mut self, addr: u16) -> Result<(), <SM83 as Arch>::DecodeError> {
        self.cpu.push(self.storage, self.cpu.pc);
        self.cpu.trace_call(&self.storage, self.cpu.pc, addr);
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
                self.cpu.flag_n_set(false);
                self.cpu.flag_h_set(true);
                self.cpu.flag_z_set(res == 0);
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
                let c_in = self.cpu.flag_c() as u8;
                let (res, carry) = left.overflowing_add(right);
                let (res, c2) = res.overflowing_add(c_in);
                let c_out = carry || c2;
                let h = (left & 0xf) + (right & 0xf) + c_in > 0xf;
                self.cpu.af[0] = 0b0000_0000;
                if c_out { self.cpu.af[0] |= 0b0001_0000; }
                if h { self.cpu.af[0] |= 0b0010_0000; }
                if res == 0 { self.cpu.af[0] |= 0b1000_0000; }
                self.cpu.af[1] = res;
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
                let c_in = self.cpu.flag_c() as u8;
                let (res, carry) = left.overflowing_sub(right);
                let (res, c2) = res.overflowing_sub(c_in);
                let c_out = carry || c2;
                let h = (left & 0xf).wrapping_sub(right & 0xf).wrapping_sub(c_in) > 0xf;
                self.cpu.af[0] = 0b0100_0000;
                if c_out { self.cpu.af[0] |= 0b0001_0000; }
                if h { self.cpu.af[0] |= 0b0010_0000; }
                if res == 0 { self.cpu.af[0] |= 0b1000_0000; }
                self.cpu.af[1] = res;
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
                let c_in = self.cpu.flag_c() as u8;
                let (res, carry) = left.overflowing_add(right);
                let (res, c2) = res.overflowing_add(c_in);
                let c_out = carry || c2;
                let h = (left & 0xf) + (right & 0xf) + c_in > 0xf;
                self.cpu.af[0] = 0b0000_0000;
                if c_out { self.cpu.af[0] |= 0b0001_0000; }
                if h { self.cpu.af[0] |= 0b0010_0000; }
                if res == 0 { self.cpu.af[0] |= 0b1000_0000; }
                self.cpu.af[1] = res;
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
                let c_in = self.cpu.flag_c() as u8;
                let (res, carry) = left.overflowing_sub(right);
                let (res, c2) = res.overflowing_sub(c_in);
                let c_out = carry || c2;
                let h = (left & 0xf).wrapping_sub(right & 0xf).wrapping_sub(c_in) > 0xf;
                self.cpu.af[0] = 0b0100_0000;
                if c_out { self.cpu.af[0] |= 0b0001_0000; }
                if h { self.cpu.af[0] |= 0b0010_0000; }
                if res == 0 { self.cpu.af[0] |= 0b1000_0000; }
                self.cpu.af[1] = res;
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
                self.cpu.flag_z_set(rotated == 0);
                rotated
            }
            RotOp::RR => {
                let c = (self.cpu.af[0] >> 4) & 0b0001;
                self.cpu.af[0] = 0b0000_0000;
                if v & 0x01 != 0 {
                    self.cpu.af[0] |= 0b0001_0000;
                }
                let rotated = (v >> 1) | (c << 7);
                self.cpu.flag_z_set(rotated == 0);
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
                self.cpu.flag_z_set(rotated == 0);
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
                self.cpu.flag_z_set(rotated == 0);
                rotated
            }
            RotOp::SLA => {
                self.cpu.af[0] = 0b0000_0000;
                if v & 0x80 != 0 {
                    self.cpu.af[0] |= 0b0001_0000;
                }
                let shifted = v << 1;
                self.cpu.flag_z_set(shifted == 0);
                shifted
            }
            RotOp::SRA => {
                self.cpu.af[0] = 0b0000_0000;
                if v & 0x01 != 0 {
                    self.cpu.af[0] |= 0b0001_0000;
                }
                let shifted = ((v as i8) >> 1) as u8;
                self.cpu.flag_z_set(shifted == 0);
                shifted
            }
            RotOp::SRL => {
                self.cpu.af[0] = 0b0000_0000;
                if v & 0x01 != 0 {
                    self.cpu.af[0] |= 0b0001_0000;
                }
                let shifted = v >> 1;
                self.cpu.flag_z_set(shifted == 0);
                shifted
            }
            RotOp::SWAP => {
                let swapped = (v >> 4) | (v << 4);
                self.cpu.flags_clear();
                self.cpu.flag_z_set(swapped == 0);
                swapped
            }
        };
        match operand {
            Reg8b::B => { self.cpu.bc[1] = res; }
            Reg8b::C => { self.cpu.bc[0] = res; }
            Reg8b::D => { self.cpu.de[1] = res; }
            Reg8b::E => { self.cpu.de[0] = res; }
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
        self.cpu.af[0] &= 0xf0;
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
        self.cpu.flags_clear();
        self.cpu.flag_c_set(v & 0x80 != 0);
        if v & 0x80 != 0 {
            c = 0x01;
        }
        let rotated = (v << 1) | c;
        self.cpu.af[1] = rotated;
        Ok(())
    }
    fn on_rrca(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> {
        let v = self.cpu.af[1];
        let mut c = 0;
        self.cpu.flags_clear();
        self.cpu.flag_c_set(v & 0x01 != 0);
        if v & 0x01 != 0 {
            c = 0x80;
        }
        let rotated = (v >> 1) | c;
        self.cpu.af[1] = rotated;
        Ok(())
    }
    fn on_rla(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> {
        let v = self.cpu.af[1];
        let c = self.cpu.flag_c() as u8;
        self.cpu.flags_clear();
        self.cpu.flag_c_set(v & 0x80 != 0);
        let rotated = (v << 1) | c;
        self.cpu.af[1] = rotated;
        Ok(())
    }
    fn on_rra(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> {
        let v = self.cpu.af[1];
        let c_out = self.cpu.af[1] & 1;
        let c_in = self.cpu.flag_c() as u8;
        self.cpu.flags_clear();
        self.cpu.flag_c_set(c_out != 0);
        let rotated = (v >> 1) | (c_in << 7);
        /*
        // allegedly rra never sets zero
        if rotated == 0 {
            self.cpu.af[0] |= 0b1000_0000;
        }
        */
        self.cpu.af[1] = rotated;
        Ok(())
    }
    fn on_daa(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> {
        let amt: u8 = if !self.cpu.flag_n() {
            let adjust_low = self.cpu.flag_h() || (self.cpu.af[1] & 0x0f) > 9;
            let adjust_high = self.cpu.flag_c() || (self.cpu.af[1] > 0x99);

            let mut adjust: u8 = 0;

            if adjust_low {
                adjust += 0x06;
            }

            if adjust_high {
                adjust += 0x60;
            }

            adjust
        } else {
            let mut adjust: u8 = 0;

            if self.cpu.flag_h() {
                adjust = adjust.wrapping_sub(0x06);
            }

            if self.cpu.flag_c() {
                adjust = adjust.wrapping_sub(0x60);
            }

            adjust
        };

        let (res, carry) = self.cpu.af[1].overflowing_add(amt);
        self.cpu.af[1] = res;
        self.cpu.flag_h_set(false);
        self.cpu.flag_c_set(self.cpu.flag_c() || (!self.cpu.flag_n() && carry));
        self.cpu.flag_z_set(self.cpu.af[1] == 0);
        Ok(())
    }
    fn on_scf(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> {
        self.cpu.af[0] &= 0b1000_0000;
        self.cpu.af[0] |= 0b0001_0000;
        Ok(())
    }
    fn on_ccf(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> {
        self.cpu.af[0] &= 0b1001_0000;
        self.cpu.af[0] ^= 0b0001_0000;
        Ok(())
    }
    fn on_ret_unconditional(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> {
        let target = self.cpu.pop(self.storage);
        self.cpu.trace_ret(&self.storage, self.cpu.pc, target);
        self.cpu.pc = target;
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
    fn on_rst(&mut self, imm: u8) -> Result<(), <SM83 as Arch>::DecodeError> {
        if imm == 0x38 {
            self.print_branch_trace();
            panic!("probably bug. do you really mean to run 0xff?");
        }
        self.cpu.push(self.storage, self.cpu.pc);
        self.cpu.pc = imm as u16;
        Ok(())
    }
    fn on_reti(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> {
        let target = self.cpu.pop(self.storage);
        self.cpu.trace_reti(&self.storage, self.cpu.pc, target);
        self.cpu.pc = target;
        self.cpu.ime = true;
        self.clocks += 16;
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
            halted: false,
            branch_trace: VecDeque::new(),
            call_stack: Vec::new(),
        }
    }

    fn trace_int<'storage: 'env, 'env>(&mut self, storage: &'env MemoryMapping<'storage>, from: u16, to: u16, int_bit: u8) {
        #[cfg(not(feature="branch-trace"))]
        return;

        let from_linear = storage.recursive_translate(from);
        let to_linear = storage.recursive_translate(to);

        let addrs = BranchAddrs {
            from,
            to,
            from_linear,
            to_linear,
        };


        self.branch_trace.push_front(BranchTrace::Int { addrs });
        self.call_stack.push(CallRecord::int(addrs, int_bit));
    }

    fn trace_reti<'storage: 'env, 'env>(&mut self, storage: &'env MemoryMapping<'storage>, from: u16, to: u16) {
        #[cfg(not(feature="branch-trace"))]
        return;

        if self.branch_trace.len() > TRACE_DEPTH {
            self.branch_trace.pop_back();
        }
        let from_linear = storage.recursive_translate(from);
        let to_linear = storage.recursive_translate(to);

        let prev_rec = self.call_stack.pop();
        match prev_rec {
            Some(CallRecord { addrs, call_kind: CallKind::Int { iflags }}) => {
                if addrs.from_linear != to_linear {
                    eprintln!("matched interrupt (if: {:#08x}) at {}:{:#x} but returning to {}:{:#x}",
                        iflags,
                        addrs.from_linear.segment_name(),
                        addrs.from_linear.address,
                        to_linear.segment_name(),
                        to_linear.address,
                    );
                    eprintln!("call stack as traced is ... {:?}", self.call_stack.as_slice());
                }
            },
            Some(CallRecord { addrs, call_kind }) => {
                eprintln!("reti (returning to {}:{:#x}) paired with non-interrupt at {}:{:#x}",
                    addrs.from_linear.segment_name(),
                    addrs.from_linear.address,
                    to_linear.segment_name(),
                    to_linear.address,
                );
                eprintln!("call stack as traced is ... {:?}", self.call_stack.as_slice());
            }
            None => {
                eprintln!("reti, but not matched with an interrupt");
            }
        }

        self.branch_trace.push_front(BranchTrace::Reti {
            addrs: BranchAddrs {
                from,
                to,
                from_linear,
                to_linear,
            }
        });
    }

    fn trace_ret<'storage: 'env, 'env>(&mut self, storage: &'env MemoryMapping<'storage>, from: u16, to: u16) {
        #[cfg(not(feature="branch-trace"))]
        return;

        if self.branch_trace.len() > TRACE_DEPTH {
            self.branch_trace.pop_back();
        }
        let from_linear = storage.recursive_translate(from);
        let to_linear = storage.recursive_translate(to);

        let prev_rec = self.call_stack.pop();
        match prev_rec {
            Some(rec) => {
                if rec.addrs.from_linear != to_linear {
                    eprintln!("matched call at {}:{:#?} but returning to {}:{:#?}",
                        rec.addrs.from_linear.segment_name(),
                        rec.addrs.from_linear.address,
                        to_linear.segment_name(),
                        to_linear.address,
                    );
                }
            },
            None => {
                eprintln!("reti, but not matched with an interrupt");
            }
        }

        self.branch_trace.push_front(BranchTrace::Ret {
            addrs: BranchAddrs {
                from,
                to,
                from_linear,
                to_linear,
            }
        });
    }

    fn trace_call<'storage: 'env, 'env>(&mut self, storage: &'env MemoryMapping<'storage>, from: u16, to: u16) {
        #[cfg(not(feature="branch-trace"))]
        return;

        let from_linear = storage.recursive_translate(from);
        let to_linear = storage.recursive_translate(to);

        let addrs = BranchAddrs {
            from,
            to,
            from_linear,
            to_linear,
        };

        if self.branch_trace.len() > TRACE_DEPTH {
            self.branch_trace.pop_back();
        }

        self.branch_trace.push_front(BranchTrace::Call { addrs });
        self.call_stack.push(CallRecord::call(addrs));
    }

    fn trace_branch<'storage: 'env, 'env>(&mut self, storage: &'env MemoryMapping<'storage>, from: u16, to: u16) {
        #[cfg(not(feature="branch-trace"))]
        return;

        let from_linear = storage.recursive_translate(from);
        let to_linear = storage.recursive_translate(to);

        let traced_addrs = BranchAddrs {
            from,
            to,
            from_linear,
            to_linear,
        };

        if let Some(front) = self.branch_trace.front_mut() {
            if let BranchTrace::Branch { addrs, count } = front {
                if *addrs == traced_addrs {
                    *count += 1;
                    return;
                }
            }
        }

        if self.branch_trace.len() > 10 {
            self.branch_trace.pop_back();
        }

        self.branch_trace.push_front(BranchTrace::Branch { addrs: traced_addrs, count: 1 });
    }

    fn flags_clear(&mut self) {
        self.af[0] = 0b0000_0000;
    }

    fn flag_z_set(&mut self, v: bool) {
        if v {
            self.af[0] |= 0b1000_0000;
        } else {
            self.af[0] &= !0b1000_0000;
        }
    }

    fn flag_n_set(&mut self, v: bool) {
        if v {
            self.af[0] |= 0b0100_0000;
        } else {
            self.af[0] &= !0b0100_0000;
        }
    }

    fn flag_h_set(&mut self, v: bool) {
        if v {
            self.af[0] |= 0b0010_0000;
        } else {
            self.af[0] &= !0b0010_0000;
        }
    }

    fn flag_c_set(&mut self, v: bool) {
        if v {
            self.af[0] |= 0b0001_0000;
        } else {
            self.af[0] &= !0b0001_0000;
        }
    }

    fn flag_h(&self) -> bool {
        self.af[0] & 0b0010_0000 != 0
    }

    fn flag_c(&self) -> bool {
        self.af[0] & 0b0001_0000 != 0
    }

    fn flag_n(&self) -> bool {
        self.af[0] & 0b0100_0000 != 0
    }

    fn flag_z(&self) -> bool {
        self.af[0] & 0b1000_0000 != 0
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

//    #[inline(never)]
    pub fn step<'storage: 'env, 'env>(&'env mut self, memory: &'env mut MemoryMapping<'storage>) -> u16 {
        if self.ime {
            // otherwise, interrupts are enabled, and if none have fired we will remain in
            // low-power mode (advancing by one clock as the rest of the machine still has to
            // operate).
            let interrupts = memory.state.management_bits[crate::IF as usize] & memory.state.management_bits[crate::IE as usize] & 0x1f;
            if interrupts != 0 {
                // have an interrupt, update state appropriately and wake from halt
                self.halted = false;

                // interrupt entry itself is essentially a fused `call $isr; di`
                self.push(memory, self.pc);
                let interrupt_nr = interrupts.trailing_zeros();
                assert!(interrupt_nr < 4, "bogus interrupt? {}", interrupt_nr);
//                eprintln!("interrupt {}", interrupt_nr);

                // unset the bit for the interrupt we're about to service
                memory.state.management_bits[crate::IF as usize] ^= 1 << interrupt_nr;

                let interrupt_addr = 0x40 + (8 * interrupt_nr) as u16;
                self.trace_int(memory, self.pc, interrupt_addr, 1 << interrupt_nr);
//                eprintln!("interrupt fired {} to {:#04x}", interrupt_nr, interrupt_addr);
                self.pc = interrupt_addr;
                // and finally, enter the ISR with interrupts disabled again.
                self.ime = false;
            }
        }

        if self.halted {
            // if the cpu is halted and interrupts are disabled, the last instruction we executed
            // was, itself, `halt`. wake from halt and advance one clock. (programmers: do not do
            // this).
            //
            // otherwise, interrupts are enabled and we're just still halted. the rest of the
            // machine is still running, so advance a clock.
            if !self.ime {
                self.halted = false;
            }
            return 4;
        }

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
    }
}

// TODO: CB3F does not work?

mod test {
    use crate::{Cpu, FlatMapper, MemoryMapping};

    fn execute_test(cpu: &mut Cpu, program: &[u8]) {
        let mut rom = program.to_vec();
        while rom.len() < 4 {
            rom.push(0u8);
        }

        let mut rom = FlatMapper::new(rom.into_boxed_slice());
        let mut lcd = crate::Lcd::new();
        let mut apu = crate::Apu::new();
        let mut memory = MemoryMapping {
            cart: &mut rom,
            state: &mut crate::GBCState {
                ram: [0; 32768],
                vram: [0; 16384],
                lcd: lcd,
                apu: apu,
                management_bits: [0u8; 512],
            },
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
        fn test_ld_special() {
            let mut cpu = Cpu::new();
            cpu.sp = 0x1234;
            let mut result = cpu.clone();
            result.hl = 0x1284u16.to_le_bytes();
            result.pc += 2;
            result.af[0] = 0x00;
            super::execute_test(&mut cpu, &[0xf8, 0x50]);

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

        #[test]
        fn test_rotate() {
            let mut cpu = Cpu::new();
            cpu.bc[0] = 0x01;
            cpu.af[0] = 0x00;
            let mut result = cpu.clone();
            result.bc[0] = 0x00;
            result.af[0] = 0x90;
            result.pc += 2;
            super::execute_test(&mut cpu, &[0xcb, 0x19]);

            assert_eq!(cpu, result);

            let mut cpu = Cpu::new();
            cpu.de[1] = 0x01;
            cpu.af[0] = 0x00;
            let mut result = cpu.clone();
            result.de[1] = 0x00;
            result.af[0] = 0x90;
            result.pc += 2;
            super::execute_test(&mut cpu, &[0xcb, 0x1a]);

            assert_eq!(cpu, result);

            let test_pairs: &[(u8, (u8, u8))] = &[
                (0x01, (0x00, 0x10)),
                (0x02, (0x01, 0x00)),
                (0x04, (0x02, 0x00)),
                (0x08, (0x04, 0x00)),
                (0x10, (0x08, 0x00)),
                (0x20, (0x10, 0x00)),
                (0x40, (0x20, 0x00)),
                (0x80, (0x40, 0x00)),
            ];

            for (init, (res_a, res_f)) in test_pairs.iter() {
                let mut cpu = Cpu::new();
                cpu.af[1] = *init;
                cpu.af[0] = 0x00;
                let mut result = cpu.clone();
                result.af[1] = *res_a;
                result.af[0] = *res_f;
                result.pc += 1;
                super::execute_test(&mut cpu, &[0x1f]);

                assert_eq!(cpu, result);
            }

            let mut cpu = Cpu::new();
            cpu.af[1] = 0x00;
            cpu.af[0] = 0x10;
            let mut result = cpu.clone();
            result.af[1] = 0x80;
            result.af[0] = 0x00;
            result.pc += 1;
            super::execute_test(&mut cpu, &[0x1f]);

            assert_eq!(cpu, result);
        }

        #[test]
        fn test_srl() {
            let mut cpu = Cpu::new();
            cpu.af[0] = 0x00;
            cpu.af[1] = 0x02;
            let mut result = cpu.clone();
            result.af[0] = 0x00;
            result.af[1] = 0x01;
            result.pc += 2;
            super::execute_test(&mut cpu, &[0xcb, 0x3f]);

            assert_eq!(cpu, result);

            let mut cpu = Cpu::new();
            cpu.af[0] = 0x00;
            cpu.af[1] = 0x80;
            let mut result = cpu.clone();
            result.af[0] = 0x00;
            result.af[1] = 0x40;
            result.pc += 2;
            super::execute_test(&mut cpu, &[0xcb, 0x3f]);

            assert_eq!(cpu, result);
        }

        #[test]
        fn test_sra() {
            let mut cpu = Cpu::new();
            cpu.af[0] = 0x00;
            cpu.af[1] = 0x02;
            let mut result = cpu.clone();
            result.af[0] = 0x00;
            result.af[1] = 0x01;
            result.pc += 2;
            super::execute_test(&mut cpu, &[0xcb, 0x2f]);

            assert_eq!(cpu, result);

            let mut cpu = Cpu::new();
            cpu.af[0] = 0x00;
            cpu.af[1] = 0x80;
            let mut result = cpu.clone();
            result.af[0] = 0x00;
            result.af[1] = 0xc0;
            result.pc += 2;
            super::execute_test(&mut cpu, &[0xcb, 0x2f]);

            assert_eq!(cpu, result);
        }

        #[test]
        fn test_adc() {
            let mut cpu = Cpu::new();
            cpu.af[0] = 0x10;
            cpu.af[1] = 0xfe;
            let mut result = cpu.clone();
            result.af[1] = 0x00;
            result.flag_z_set(true);
            result.flag_h_set(true);
            result.flag_c_set(true);
            result.pc += 2;
            super::execute_test(&mut cpu, &[0xce, 0x01]);

            assert_eq!(cpu, result);

            let mut cpu = Cpu::new();
            cpu.af[0] = 0x10;
            cpu.af[1] = 0x10;
            cpu.hl[1] = 0x00;
            let mut result = cpu.clone();
            result.af[0] = 0x00;
            result.af[1] = 0x11;
            result.hl[1] = 0x00;
            result.pc += 1;
            super::execute_test(&mut cpu, &[0x8c]);

            assert_eq!(cpu, result);

            let mut cpu = Cpu::new();
            cpu.af[0] = 0x00;
            cpu.af[1] = 0x10;
            cpu.hl[1] = 0x00;
            let mut result = cpu.clone();
            result.af[0] = 0x00;
            result.af[1] = 0x10;
            result.hl[1] = 0x00;
            result.pc += 1;
            super::execute_test(&mut cpu, &[0x8c]);

            assert_eq!(cpu, result);

            let mut cpu = Cpu::new();
            cpu.af[0] = 0x00;
            cpu.af[1] = 0x10;
            cpu.hl[1] = 0x11;
            let mut result = cpu.clone();
            result.af[0] = 0x00;
            result.af[1] = 0x21;
            result.hl[1] = 0x11;
            result.pc += 1;
            super::execute_test(&mut cpu, &[0x8c]);

            assert_eq!(cpu, result);

            let mut cpu = Cpu::new();
            cpu.af[0] = 0x10;
            cpu.af[1] = 0xf0;
            cpu.hl[1] = 0x0f;
            let mut result = cpu.clone();
            result.af[0] = 0xb0;
            result.af[1] = 0x00;
            result.hl[1] = 0x0f;
            result.pc += 1;
            super::execute_test(&mut cpu, &[0x8c]);

            assert_eq!(cpu, result);
        }
    }
}


pub(crate) struct DecoratedInstruction<'instr, 'data> {
    cpu: &'instr Cpu,
    memory: &'instr MemoryMapping<'data>,
    inst: &'instr yaxpeax_sm83::Instruction,
}

pub(crate) trait DecorateExt {
    fn decorate<'instr, 'data>(&'instr self, cpu: &'instr Cpu, memory: &'instr MemoryMapping<'data>) -> DecoratedInstruction<'instr, 'data>;
}

impl DecorateExt for yaxpeax_sm83::Instruction {
    fn decorate<'instr, 'data>(&'instr self, cpu: &'instr Cpu, memory: &'instr MemoryMapping<'data>) -> DecoratedInstruction<'instr, 'data> {
        DecoratedInstruction {
            cpu,
            memory,
            inst: self
        }
    }
}

enum DecoratedOperand {
    Register(&'static str),
    Deref(String, Option<String>),
    Imm(String),
    Rel(String, String),
    Cond(&'static str),
}

impl fmt::Display for DecoratedOperand {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            DecoratedOperand::Register(r) => {
                f.write_str(r)
            }
            DecoratedOperand::Deref(addr, addr_name) => {
                f.write_str("[")?;
                f.write_str(&addr)?;
                if let Some(addr_name) = addr_name {
                    f.write_str(" (")?;
                    f.write_str(addr_name)?;
                    f.write_str(")")?;
                }
                f.write_str("]")?;
                Ok(())
            }
            DecoratedOperand::Imm(i) => {
                f.write_str(i)
            }
            DecoratedOperand::Rel(addr, dest) => {
                write!(f, "{} ({})", addr, dest)
            }
            DecoratedOperand::Cond(cond) => {
                f.write_str(cond)
            }
        }
    }
}

fn addr_name_lookup(addr: u16) -> Option<&'static str> {
    match addr {
        0xff00 => Some("JOYP"),
        0xff04 => Some("DIV"),
        0xff0f => Some("IF"),
        0xff0f => Some("IE"),
        0xff30 => Some("WAVE"),
        0xff40 => Some("LCDC"),
        0xff41 => Some("STAT"),
        0xff42 => Some("SCY"),
        0xff43 => Some("SCX"),
        0xff44 => Some("LY"),
        0xff45 => Some("LYC"),
        0xff46 => Some("DMA"),
        0xff47 => Some("BGP"),
        0xff4a => Some("WY"),
        0xff4b => Some("WX"),
        0xff4d => Some("KEY1"),
        0xff4f => Some("VBK"),
        0xff50 => Some("BANK"),
        0xff51 => Some("HDMA1"),
        0xff52 => Some("HDMA2"),
        0xff53 => Some("HDMA3"),
        0xff54 => Some("HDMA4"),
        0xff55 => Some("HDMA5"),
        0xff70 => Some("SVBK"),
        _ => None,
    }
}

use yaxpeax_sm83::Operand;
impl<'instr, 'data> DecoratedInstruction<'instr, 'data> {
    fn decorate_operand(&self, operand: &Operand) -> DecoratedOperand {
        match operand {
            Operand::A => DecoratedOperand::Register("a"),
            Operand::B => DecoratedOperand::Register("b"),
            Operand::C => DecoratedOperand::Register("c"),
            Operand::D => DecoratedOperand::Register("d"),
            Operand::E => DecoratedOperand::Register("e"),
            Operand::H => DecoratedOperand::Register("h"),
            Operand::L => DecoratedOperand::Register("l"),
            Operand::AF => DecoratedOperand::Register("af"),
            Operand::BC => DecoratedOperand::Register("bc"),
            Operand::DE => DecoratedOperand::Register("de"),
            Operand::HL => DecoratedOperand::Register("hl"),
            Operand::SP => DecoratedOperand::Register("sp"),
            Operand::DerefHL => {
                let v = u16::from_le_bytes(self.cpu.hl);
                let name = addr_name_lookup(v).map(|x| x.to_owned());
                let addr_string = format!("${:04x}", v);
                let text = if let Some(name) = name {
                    format!("{} ({})", addr_string, name)
                } else {
                    addr_string
                };
                DecoratedOperand::Deref("hl".to_string(), Some(text))
            },
            Operand::DerefBC => {
                let v = u16::from_le_bytes(self.cpu.bc);
                let name = addr_name_lookup(v).map(|x| x.to_owned());
                let addr_string = format!("${:04x}", v);
                let text = if let Some(name) = name {
                    format!("{} ({})", addr_string, name)
                } else {
                    addr_string
                };
                DecoratedOperand::Deref("bc".to_string(), Some(text))
            },
            Operand::DerefDE => {
                let v = u16::from_le_bytes(self.cpu.de);
                let name = addr_name_lookup(v).map(|x| x.to_owned());
                let addr_string = format!("${:04x}", v);
                let text = if let Some(name) = name {
                    format!("{} ({})", addr_string, name)
                } else {
                    addr_string
                };
                DecoratedOperand::Deref("de".to_string(), Some(text))
            },
            Operand::DerefDecHL => {
                let v = u16::from_le_bytes(self.cpu.hl);
                let name = addr_name_lookup(v).map(|x| x.to_owned());
                let addr_string = format!("${:04x}", v);
                let text = if let Some(name) = name {
                    format!("{} ({})", addr_string, name)
                } else {
                    addr_string
                };
                DecoratedOperand::Deref("hl-".to_string(), Some(text))
            },
            Operand::DerefIncHL => {
                let v = u16::from_le_bytes(self.cpu.hl);
                let name = addr_name_lookup(v).map(|x| x.to_owned());
                let addr_string = format!("${:04x}", v);
                let text = if let Some(name) = name {
                    format!("{} ({})", addr_string, name)
                } else {
                    addr_string
                };
                DecoratedOperand::Deref("hl+".to_string(), Some(text))
            },
            Operand::DerefHighC => DecoratedOperand::Deref("$ff00 + c".to_string(), None),
            Operand::DerefHighD8(offs) => {
                DecoratedOperand::Deref(
                    format!("$ff00 + ${:02x}", offs),
                    addr_name_lookup(0xff00 + (*offs as u16)).map(|x| x.to_owned()),
                )
            },
            Operand::SPWithOffset(imm) => {
                let amt = if *imm == -128 {
                    "sp - $80".to_string()
                } else if *imm >= 0 {
                    format!("sp + ${:02x}", imm)
                } else {
                    format!("sp - ${:02x}", -imm)
                };
                DecoratedOperand::Deref(amt, None)
            }
            Operand::Bit(imm) => {
                DecoratedOperand::Imm(format!("{}", imm))
            },
            Operand::D8(imm) => {
                DecoratedOperand::Imm(format!("${:02x}", imm))
            },
            Operand::D16(imm) => {
                DecoratedOperand::Imm(format!("${:04x}", imm))
            },
            Operand::I8(imm) => {
                let amt = if *imm == -128 {
                    format!("-0x80")
                } else if *imm >= 0 {
                    format!("${:02x}", imm)
                } else {
                    format!("-${:02x}", -imm)
                };
                DecoratedOperand::Imm(amt)
            }
            Operand::R8(imm) => {
                let amt = if *imm == -128 {
                    format!("$-0x80")
                } else if *imm >= 0 {
                    format!("$+${:02x}", imm)
                } else {
                    format!("$-${:02x}", -imm)
                };
                let dest = (self.cpu.pc + self.inst.length() as u16).wrapping_add(*imm as i16 as u16);
                DecoratedOperand::Rel(amt, format!("${:04x}", dest))
            }
            Operand::A16(addr) => {
                let name = addr_name_lookup(*addr).map(|x| x.to_owned());
                DecoratedOperand::Deref(
                    format!("${:4x}", addr),
                    name,
                )
            },

            Operand::CondC => DecoratedOperand::Cond("C"),
            Operand::CondNC => DecoratedOperand::Cond("nC"),
            Operand::CondZ => DecoratedOperand::Cond("Z"),
            Operand::CondNZ => DecoratedOperand::Cond("nZ"),

            Operand::Nothing => {
                unreachable!("operand::nothing");
            }
        }
    }
}

impl<'instr, 'data> fmt::Display for DecoratedInstruction<'instr, 'data> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{}", self.inst.opcode())?;

        let ops = self.inst.operands();
        if ops[0] != Operand::Nothing {
            f.write_str(" ")?;
        } else {
            return Ok(());
        }
        write!(f, "{}", self.decorate_operand(&ops[0]))?;
        if ops[1] != Operand::Nothing {
            f.write_str(", ")?;
        } else {
            return Ok(());
        }
        write!(f, "{}", self.decorate_operand(&ops[1]))?;

        Ok(())
    }
}
