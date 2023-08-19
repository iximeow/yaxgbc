use std::fmt::Write;
use std::sync::{Arc, Mutex};

use crate::{STAT, SCX, SCY, IF, IE, GBC, LCDC, LY, LYC};

use std::time::SystemTime;

use egui_miniquad;
use egui;
use egui::Vec2;
use miniquad::{Buffer, BufferLayout, BufferType, Bindings, Texture, Pipeline, Shader, VertexAttribute, VertexFormat};

struct GBCPainter {
//    bindings: Bindings,
    vertex_buffer: miniquad::Buffer,
    index_buffer: miniquad::Buffer,
    pipeline: Pipeline,
    gb_state: Arc<Mutex<GBC>>,
    egui: egui_miniquad::EguiMq,
    fps_tracker: Vec<SystemTime>,
}

impl miniquad::EventHandler for GBCPainter {
    fn update(&mut self, _ctx: &mut miniquad::Context) {}

    fn draw(&mut self, ctx: &mut miniquad::Context) {
        ctx.clear(Some((1.0, 1.0, 1.0, 1.0)), None, None);
        ctx.begin_default_pass(miniquad::PassAction::clear_color(1.0, 0.0, 0.0, 1.0));
        ctx.begin_default_pass(Default::default());
        ctx.end_render_pass();
//        ctx.commit_frame();

        let dpi_scale = ctx.dpi_scale();

        let mut gb = self.gb_state.lock().unwrap();

        self.egui.run(ctx, |_miniquad_ctx, egui_ctx| {
            egui::Window::new("aaa").show(egui_ctx, |ui| {
                if ui.button("quit").clicked() {
                    std::process::exit(0);
                }
                ui.label(format!("fps: {}", self.fps_tracker.len()));
                ui.label(format!("scy: {}", gb.management_bits[SCY]));
                ui.label(format!("scx: {}", gb.management_bits[SCX]));
                ui.label(format!("ly: {}", gb.management_bits[LY]));
                ui.label(format!("lyc: {}", gb.management_bits[LYC]));
                ui.label(format!("lcdc: {}", gb.management_bits[LCDC]));
                ui.label(format!("ie: {}", gb.management_bits[IE]));
                ui.label(format!("if: {}", gb.management_bits[IF]));
                ui.label(format!("pc: {:04x}", gb.cpu.pc));
            });
            gb.clear_input();
            if egui_ctx.input(|i| i.key_down(egui::Key::Q)) {
                gb.do_input(crate::Input::Start);
            }
            if egui_ctx.input(|i| i.key_down(egui::Key::E)) {
                gb.do_input(crate::Input::Select);
            }
            if egui_ctx.input(|i| i.key_down(egui::Key::Z)) {
                gb.do_input(crate::Input::A);
            }
            if egui_ctx.input(|i| i.key_down(egui::Key::X)) {
                gb.do_input(crate::Input::B);
            }
            if egui_ctx.input(|i| i.key_down(egui::Key::ArrowRight)) {
                gb.do_input(crate::Input::Right);
            }
            if egui_ctx.input(|i| i.key_down(egui::Key::ArrowLeft)) {
                gb.do_input(crate::Input::Left);
            }
            if egui_ctx.input(|i| i.key_down(egui::Key::ArrowUp)) {
                gb.do_input(crate::Input::Up);
            }
            if egui_ctx.input(|i| i.key_down(egui::Key::ArrowDown)) {
                gb.do_input(crate::Input::Down);
            }
        });
        let mut pixels: Vec<u8> = Vec::new();
        for y in 0..SCREEN_HEIGHT {
            for x in 0..SCREEN_WIDTH {
                pixels.extend_from_slice(&[
                    ((gb.lcd.display[((y * 160) + x) as usize] >>  0) as u8),
                    ((gb.lcd.display[((y * 160) + x) as usize] >>  8) as u8),
                    ((gb.lcd.display[((y * 160) + x) as usize] >> 16) as u8),
                    0xff
                ]);
            }
        }

        std::mem::drop(gb);

        let texture = Texture::from_rgba8(ctx, SCREEN_WIDTH, SCREEN_HEIGHT, &pixels);


        let bindings = Bindings {
            vertex_buffers: vec![self.vertex_buffer],
            index_buffer: self.index_buffer,
            images: vec![texture],
        };

        ctx.begin_default_pass(Default::default());

        ctx.apply_pipeline(&self.pipeline);

        ctx.apply_bindings(&bindings);
        ctx.apply_uniforms(&shader::Uniforms {
            offset: (0.0, 0.0)
        });
        ctx.draw(0, 6, 1);
        ctx.end_render_pass();

        self.egui.draw(ctx);

        ctx.commit_frame();
        let now = SystemTime::now();
        let tracker = std::mem::replace(&mut self.fps_tracker, Vec::new());
        let tracker = tracker.into_iter().filter(|x| now.duration_since(*x).unwrap() < std::time::Duration::from_millis(1000)).collect();
        self.fps_tracker = tracker;
        self.fps_tracker.push(now);
    }

    fn mouse_motion_event(&mut self, _: &mut miniquad::Context, x: f32, y: f32) {
        self.egui.mouse_motion_event(x, y);
    }

    fn mouse_wheel_event(&mut self, _: &mut miniquad::Context, dx: f32, dy: f32) {
        self.egui.mouse_wheel_event(dx, dy);
    }

    fn mouse_button_down_event(
        &mut self,
        ctx: &mut miniquad::Context,
        mb: miniquad::MouseButton,
        x: f32,
        y: f32,
    ) {
        self.egui.mouse_button_down_event(ctx, mb, x, y);
    }

    fn mouse_button_up_event(
        &mut self,
        ctx: &mut miniquad::Context,
        mb: miniquad::MouseButton,
        x: f32,
        y: f32,
    ) {
        self.egui.mouse_button_up_event(ctx, mb, x, y);
    }

    fn char_event(
        &mut self,
        _ctx: &mut miniquad::Context,
        character: char,
        _keymods: miniquad::KeyMods,
        _repeat: bool,
    ) {
        self.egui.char_event(character);
    }

    fn key_down_event(
        &mut self,
        ctx: &mut miniquad::Context,
        keycode: miniquad::KeyCode,
        keymods: miniquad::KeyMods,
        _repeat: bool,
    ) {
        self.egui.key_down_event(ctx, keycode, keymods);
    }

    fn key_up_event(&mut self, _ctx: &mut miniquad::Context, keycode: miniquad::KeyCode, keymods: miniquad::KeyMods) {
        self.egui.key_up_event(keycode, keymods);
    }
}

const SCREEN_HEIGHT: u16 = 144;
const SCREEN_WIDTH: u16 = 160;

pub(crate) fn do_ui(gb_state: Arc<Mutex<GBC>>) {
    let conf = miniquad::conf::Conf {
//        high_dpi: true,
        window_height: SCREEN_HEIGHT as i32 * 6,
        window_width: SCREEN_WIDTH as i32 * 6,
        platform: miniquad::conf::Platform {
            ..Default::default()
        },
        ..Default::default()
    };

    miniquad::start(miniquad::conf::Conf::default(), |mut ctx| {
        #[repr(C)]
        struct Vec2 {
            x: f32,
            y: f32,
        }

        #[repr(C)]
        struct Vertex {
            pos: Vec2,
            uv: Vec2,
        }

//        let lcd_center_x = 800 / 4;
//        let lcd_center_y = 600 / 4;
        let dx = SCREEN_WIDTH as f32 * 2.0 / 800.0;
        let dy = SCREEN_HEIGHT as f32 * 2.0 / 800.0;

        let vertices: [Vertex; 4] = [
            Vertex { pos: Vec2 { x: -dx, y: -dy }, uv: Vec2 { x: 0.0, y: 1.0 } },
            Vertex { pos: Vec2 { x:  dx, y: -dy }, uv: Vec2 { x: 1.0, y: 1.0 } },
            Vertex { pos: Vec2 { x:  dx, y:  dy }, uv: Vec2 { x: 1.0, y: 0.0 } },
            Vertex { pos: Vec2 { x: -dx, y:  dy }, uv: Vec2 { x: 0.0, y: 0.0 } },
        ];

        /*
        let vertices: [Vertex; 4] = [
            Vertex { pos: Vec2 { x: -0.5, y: -0.5 }, uv: Vec2 { x: 0.0, y: 0.0 } },
            Vertex { pos: Vec2 { x:  0.5, y: -0.5 }, uv: Vec2 { x: 1.0, y: 0.0 } },
            Vertex { pos: Vec2 { x:  0.5, y:  0.5 }, uv: Vec2 { x: 1.0, y: 1.0 } },
            Vertex { pos: Vec2 { x: -0.5, y:  0.5 }, uv: Vec2 { x: 0.0, y: 1.0 } },
        ];
        */

        let vertex_buffer = Buffer::immutable(ctx,
            BufferType::VertexBuffer,
            &vertices,
        );

        let indices: [u16; 6] = [0, 1, 2, 0, 2, 3];

        let index_buffer = Buffer::immutable(ctx,
            BufferType::IndexBuffer,
            &indices,
        );

        let shader = Shader::new(
            ctx,
            shader::VERTEX,
            shader::FRAGMENT,
            shader::meta()
        ).unwrap();

        let pipeline = Pipeline::new(
            ctx,
            &[BufferLayout::default()],
            &[
                VertexAttribute::new("pos", VertexFormat::Float2),
                VertexAttribute::new("uv", VertexFormat::Float2),
            ],
            shader,
        );

        Box::new(GBCPainter {
//            bindings,
            vertex_buffer,
            index_buffer,
            pipeline,
            gb_state,
            egui: egui_miniquad::EguiMq::new(ctx),
            fps_tracker: Vec::new(),
        })
    });
    /*
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
                write!(screen, "{}", [".", "+", "*", "#"][gb.lcd.display[(i * 160 + j) as usize] as usize]);
            }
            write!(screen, "\n");
        }
        println!("{}", screen);
        std::mem::drop(gb);
        std::thread::sleep(std::time::Duration::from_millis(16));
    }
*/
}
mod shader {
    use miniquad::*;

    pub const VERTEX: &str = r#"#version 100
    attribute vec2 pos;
    attribute vec2 uv;

    uniform vec2 offset;

    varying lowp vec2 texcoord;

    void main() {
        gl_Position = vec4(pos + offset, 0, 1);
        texcoord = uv;
    }"#;

    pub const FRAGMENT: &str = r#"#version 100
    varying lowp vec2 texcoord;

    uniform sampler2D tex;

    void main() {
        gl_FragColor = texture2D(tex, texcoord);
    }"#;

    pub fn meta() -> ShaderMeta {
        ShaderMeta {
            images: vec!["tex".to_string()],
            uniforms: UniformBlockLayout {
                uniforms: vec![UniformDesc::new("offset", UniformType::Float2)],
            },
        }
    }

    #[repr(C)]
    pub struct Uniforms {
        pub offset: (f32, f32),
    }
}
