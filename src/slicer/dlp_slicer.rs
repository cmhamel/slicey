use crate::geometry::{Triangle, STLMesh};
use crate::settings::Settings;
use crate::slicer::Slicer;
use bvh::aabb::Bounded;
use bvh::bounding_hierarchy::BoundingHierarchy;
use bvh::bvh::Bvh;
use bvh::ray::Ray;
use image;
use nalgebra;
use rayon::iter::{IntoParallelIterator, ParallelIterator};
use std::path::Path;


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

    fn layer_image(
        &self, 
        direction: &nalgebra::Vector3::<f32>,
        grid: &Vec::<[f32; 2]>, 
        // grid: &nalgebra::DMatrix::<[f32; 2]>,
        hits: &Vec<&Triangle>, 
        z: f32,
        file_name: &str
    ) {
        let settings = self.settings.xy_resolution.clone().unwrap();
        let n_x = settings.x_pixels.try_into().unwrap();
        let n_y = settings.y_pixels.try_into().unwrap();

        // let mut flat_image = Vec::<u8>::new();
        // for point in grid.into_par_iter() {
        //     let p = nalgebra::Point3::<f32>::new(point[0], point[1], z);
        //     let _ = flat_image.push(self.stl_mesh.is_inside(direction, hits, &p));
        // }
        // // println!("Image = {:?}", flat_image.clone().into_iter().filter(|b| *b).count());

        let mut img = image::GrayImage::new(n_x, n_y);
        let pixels: Vec<_> = grid
            // .into_par_iter()
            .into_iter()
            .map(|p| {
                let temp = nalgebra::Point3::<f32>::new(p[0], p[1], z);
                self.stl_mesh.is_inside(direction, hits, &temp)
            })
            .collect();
        // // for i in 0..n_x {
        // //     for j in 0..n_y {
        // //         let k = n_x * i + j;
        // //         // image[i, j] = flat_image[k];
        // //         let pixel = image::Luma([flat_image[k as usize] as u8 / 255]);
        // //         image.put_pixel(i, j, pixel);
        // //     }
        // // }


        // for j in 0..n_y {
        //     for i in 0..n_x {
        for i in 0..n_x {
            for j in 0..n_y {
                let k = (j * n_x + i) as usize;
                let value = (pixels[k] as i32 * 255).clamp(0, 255) as u8;
                img.put_pixel(i as u32, j as u32, image::Luma([value]));
            }
        }
        img.save_with_format(file_name, image::ImageFormat::Bmp)
            .expect(format!("Failed to write image to file {}", file_name).as_str());
    }

    fn planar_grid(&self) -> Vec::<[f32; 2]> {
        let settings = self.settings.xy_resolution.clone().unwrap();
        let n_x = settings.x_pixels.try_into().unwrap();
        let n_y = settings.y_pixels.try_into().unwrap();
        let d_x = settings.x_pixel_resolution;
        let d_y = settings.y_pixel_resolution;
        let mut grid = Vec::<[f32; 2]>::new();
        
        // for j in 0..n_y {
        //     for i in 0..n_x {
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
        println!("BoundingBox = {:?}", self.stl_mesh.bounding_box());
        let xy_res = self.settings.xy_resolution.clone().unwrap();
        println!(
            "Print area bounds = {} x {}", 
            xy_res.x_pixels as f32 * 
            xy_res.x_pixel_resolution,
            xy_res.y_pixels as f32 * 
            xy_res.y_pixel_resolution
        );
        println!("{}", self.settings);
        println!("Generating layer heights");
        let zs = self.layer_heights(&self.settings, &self.stl_mesh);
        println!("Total number of layers        = {}", zs.len());
        println!("Generating planar grid");
        let grid = self.planar_grid();
        println!("Total voxels in planar grid   = {}", grid.len());
        println!("Total number of STL triangles = {}", self.stl_mesh.triangles().len());
        println!("Total number of voxel queries = {} million", zs.len() * grid.len() * self.stl_mesh.triangles().len() / 1000000);

        // some setup 
        let x = (xy_res.x_pixels as f32 * xy_res.x_pixel_resolution) / 2.;
        let y = (xy_res.y_pixels as f32 * xy_res.y_pixel_resolution) / 2.;
        let bb = self.stl_mesh.bounding_box();
        let x = (bb.x_max - bb.x_min) / 2.;
        let y = (bb.y_max - bb.y_min) / 2.;
        // let x = 0.;
        // let y = 0.;
        let origin = nalgebra::Point3::<f32>::new(x, y, 0.2);
        let direction = nalgebra::Vector3::<f32>::new(1.0, 0.0, 0.0);
        let ray = Ray::new(origin, direction);

        let base_file_name = Path::new(self.stl_mesh.file_name())
            .file_stem()
            .unwrap_or_default()
            .to_string_lossy();

        let mut tris = self.stl_mesh.triangles().clone();
        let bvh = Bvh::build_par(&mut tris);
        let hits = bvh.traverse(&ray, &tris);

        // loop over layers
        let mut z_query: f32 = 0.0;
        for (n, z) in zs.into_iter().enumerate() {
            println!("Generating slice for layer {}", n);
            let file_name = format!("{}_{}.bmp", base_file_name, format!("{:06}", n));

            println!("z = {}", z);

            // attemp to filter tris to subset of 
            // tris "near" the bounds of the layer
            // let mut tris: Vec<_> = self.stl_mesh
            //     .triangles()
            //     .into_iter()
            //     .filter(|tri| {
            //         let z_min = tri.aabb().min.z;
            //         let z_max = tri.aabb().max.z;
            //         let epsilon = 2. * z;
            //         z_query + epsilon >= z_min && z_query - epsilon <= z_max
            //         // z_query - epsilon >= z_min && z_query + epsilon <= z_max
            //     })
            //     .into_iter()
            //     .enumerate()
            //     .map(|(n, tri)| Triangle::new(n, *tri.vertices()))
            //     .collect();

            // let bvh = Bvh::build_par(&mut tris);
            // let hits = bvh.traverse(&ray, &tris);
            println!("Number of tris before filter {}", self.stl_mesh.triangles().len());
            println!("Number of tris after filter {}", tris.len());
            println!("Got {} hits", hits.len());
            let _ = self.layer_image(&direction, &grid, &hits, z, file_name.as_str());
            // update layer height
            z_query = z_query + z;
        }
    }
}
