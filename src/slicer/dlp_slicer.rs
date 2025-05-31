use crate::geometry::STLMesh;
use crate::settings::Settings;
use crate::slicer::Slicer;
use nalgebra;


pub struct DLPSlicer {
    settings: Settings,
    stl_mesh: STLMesh
}

impl DLPSlicer {
    pub fn new(
        settings: Settings, 
        stl_mesh: STLMesh
    ) -> Self {
        Self {
            settings: settings,
            stl_mesh: stl_mesh
        }
    }

    fn layer_image(&self, grid: &Vec::<[f32; 2]>, z: f32) {
        let mut image = Vec::<bool>::new();
        for (n, point) in grid.into_iter().enumerate() {
            println!("Checking point {}/{}", n, grid.len());
            let p = nalgebra::Point3::<f32>::new(point[0], point[1], z);
            let _ = image.push(self.stl_mesh.is_inside(&p));
        }
    }

    fn planar_grid(&self) -> Vec::<[f32; 2]> {
        let settings = self.settings.xy_resolution.clone().unwrap();
        let n_x = settings.x_pixels.try_into().unwrap();
        let n_y = settings.y_pixels.try_into().unwrap();
        let d_x = settings.x_pixel_resolution;
        let d_y = settings.y_pixel_resolution;
        let mut grid = Vec::<[f32; 2]>::new();
        
        for i in 0..n_x {
            for j in 0..n_y {
                let x = (i as f32 + 0.5) * d_x;
                let y = (j as f32 + 0.5) * d_y;
                let p = [x, y];
                grid.push(p);
            }
        }
        grid
    }
}

impl Slicer for DLPSlicer {
    fn slice(&self, image_folder: &str) {
        println!("Slicing stl file {:?}", self.stl_mesh.file_name());
        println!("{}", self.settings);
        println!("Generating layer heights");
        let zs = self.layer_heights(&self.settings, &self.stl_mesh);
        println!("Total number of layers        = {}", zs.len());
        println!("Generating planar grid");
        let grid = self.planar_grid();
        println!("Total voxels in planar grid   = {}", grid.len());
        println!("Total number of STL triangles = {}", self.stl_mesh.triangles().len());
        println!("Total number of voxel queries = {} million", zs.len() * grid.len() * self.stl_mesh.triangles().len() / 1000000);
        
        // loop over layers
        for (n, z) in zs.into_iter().enumerate() {
            println!("Generating slice for layer {}", n);
            // let grid = self.planar_grid(z);
            let _ = self.layer_image(&grid, z);
        }
    }
}
