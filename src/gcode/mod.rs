use crate::settings::Settings;
use std::fs::{File, OpenOptions};
use std::io::Write;

pub struct GcodeWriter {
    file_buffer: File
}

impl GcodeWriter {
    pub fn new(gcode_file: &str) -> Self {
        let file_buffer = OpenOptions::new()
            .write(true)
            .create(true)
            .truncate(true)
            .open(gcode_file)
            .unwrap();
        Self {
            file_buffer: file_buffer
        }
    }

    pub fn write_gcode(&mut self, gcode: &str) -> () {
        let _ = writeln!(self.file_buffer, "{}", gcode);
    } 

    pub fn write_header(&mut self, settings: &Settings) -> () {
        let _ = writeln!(
            self.file_buffer, "; Generated with {} {}", 
            env!("CARGO_PKG_NAME"), env!("CARGO_PKG_VERSION")
        );
        self.write_gcode(format!("; {} settings:", env!("CARGO_PKG_NAME")).as_str());
        self.write_gcode(&settings.to_gcode_comment());
        self.write_gcode("G21; units in milimeters"); // TODO
        self.write_gcode("G90; absolute positioning"); // TODO
        self.write_gcode("M82; extruder set to absolute mode\n;"); // TODO
        // TODO need to specialize this to type and what not
        self.write_gcode("M104 S200; heating nozzle to 200C without waiting");
        self.write_gcode("M140 S60; heating bed to 60C without waiting\n;");
        self.write_home_all();
        self.write_gcode("M109 S200 ; wait for nozzle to reach 200C"); // TODO
        self.write_gcode("M190 S60 ; wait for bed to reach 60C"); // TODO
        self.write_gcode("G92 E0; zero the extruder");
    }

    pub fn write_home_all(&mut self) -> () {
        self.write_gcode("G28 ; home all axes");
    }

    pub fn write_layer_change(
        &mut self, 
        n: u32, z: f32, 
        feed_axis: char, feed_rate: f32
    ) -> () {
        self.write_gcode(format!(";\n; Layer {}\n;", n).as_str());
        self.write_gcode(
            format!(
                "G1 Z{} {}{} ; layer change",
                z, feed_axis, feed_rate
            ).as_str()
        );
    }
    pub fn write_perimeter(
        &mut self,
        x: f32, y: f32, s:f32,
        extrusion: f32
    ) -> () {
        self.write_gcode(
            format!(
                "G1 X{} Y{} E{} ; Move while extruding",
                x, y, s
            ).as_str()
        );
    }
}