use std::fmt::Write;
use std::sync::{Arc, Mutex};

use crate::{STAT, SCX, SCY, IF, IE, GBC};

pub(crate) fn do_ui(gb_state: Arc<Mutex<GBC>>) {
    loop {
        let mut gb = gb_state.lock().unwrap();
        // paint screen
        let mut screen = String::new();

        // dump tile data as just some kind of guess...
        println!("!!!frame!!!");
        println!("ie: {}", gb.state.management_bits[IE]);
        println!("if: {}", gb.state.management_bits[IF]);
        println!("scy: {}", gb.state.management_bits[SCY]);
        println!("scx: {}", gb.state.management_bits[SCX]);
        println!("lcdc: {:08b}", gb.state.lcd.lcdc);
        println!("stat: {:08b}", gb.state.management_bits[STAT]);
        /*
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
        */

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
        /*
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
        */
        for i in 0..144 {
            for j in 0..160 {
                write!(screen, "{}", [".", "+", "*", "#"][gb.state.lcd.display[(i * 160 + j) as usize] as usize]);
            }
            write!(screen, "\n");
        }
        println!("{}", screen);
        std::mem::drop(gb);
        std::thread::sleep(std::time::Duration::from_millis(16));
    }
}
