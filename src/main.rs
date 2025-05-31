#[cfg(feature = "arcwelder")]
use arcwelderlib_sys;

use clap::{Parser, Subcommand};
use slicey::{
    geometry::STLMesh,
    settings::Settings,
    slicer::{
        DLPSlicer,
        FFFSlicer,
        Slicer
    }
};

/// CLIP parser
#[derive(Debug, Parser)]
#[command(version, about, long_about = None)]
struct CLIArgs {
    #[command(subcommand)]
    command: Option<Commands>
}

/// Subcommands
#[derive(Clone, Debug, Subcommand)]
enum Commands {
    #[command(about = "DLP")]
    DLP {
        #[arg(short, long, value_delimiter = ',')]
        stl_file: String,
        #[arg(long)]
        settings_file: String,
        #[arg(short, long)]
        image_folder: String
    },
    #[command(about = "DIW/FFF/SLA")]
    FFF {
        /// Path to gcode to write file to
        #[arg(short, long)]
        gcode_file: String,
        /// Paths to stl file/files
        #[arg(short, long, value_delimiter = ',')]
        stl_files: Vec<String>,
        /// Path to settings file
        #[arg(long)]
        settings_file: String,
        /// Whether or not to use arcwelder. Requires cargo build --features arcwelder
        #[arg(long)]
        arcwelder: bool
    }
}

fn main() {
    let command = CLIArgs::parse();

    match command.command {
        Some(x) => match x {
            Commands::DLP { stl_file, settings_file, image_folder } => {
                let settings = Settings::new(&settings_file);
                let mut stl_mesh = STLMesh::new(stl_file);
                stl_mesh.center_xy();
                let mut slicer = DLPSlicer::new(settings, stl_mesh);
                // slicer.center(slicer.settings, slicer.stl_mesh);
                let _ = slicer.slice(&image_folder);
            },
            Commands::FFF { gcode_file, stl_files, settings_file, arcwelder } => {

                println!("STL file      = {:?}", stl_files);
                let settings = Settings::new(&settings_file);
                let mut stl_meshes: Vec<STLMesh> = stl_files
                    .iter()
                    .map(|x| STLMesh::new(x.clone()))
                    .collect();
                
                let z_offset = -1.0 * stl_meshes[0].bounding_box().z_min;
                let _ = stl_meshes[0].translate(20.0,20.0,z_offset);
                let _ = stl_meshes[0].scale(10.0, 10.0, 10.0);

                let slicer = FFFSlicer::new(settings, stl_meshes);
                let _ = slicer.slice(&gcode_file);

                // arcwelder stuff
                let arcwelder_enabled = cfg!(feature = "arcwelder");
                if arcwelder_enabled {
                    println!("ArcWelder feature is enabled.");
                } else {
                    println!("ArcWelder feature is NOT enabled.");
                    if arcwelder {
                        panic!("You requested arcwelder but the feature is not enabled!");
                    }
                }

                #[cfg(feature = "arcwelder")]
                if arcwelder {
                    arcwelderlib_sys::arcwelder(&gcode_file, &gcode_file)
                }
            }
        },
        None => panic!("Need a subcommand!")
    }
}
