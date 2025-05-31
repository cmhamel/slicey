use bvh::aabb::{Aabb, Bounded};
use bvh::bounding_hierarchy::BHShape;
use nalgebra;
use rayon::iter::{IntoParallelIterator, ParallelIterator};
use std::env;
use std::fs::OpenOptions;
use std::ops::Index;
use std::path;
use stl_io::{self, IndexedTriangle};

type Point = nalgebra::Point3::<f32>;
type Vertex = Point;

pub type Edge = [Vertex; 2];
type Face = IndexedTriangle;

pub type Edges = Vec<Edge>;
type Faces = Vec<Face>;
type Triangles = Vec<Triangle>;
type Vertices = Vec<Vertex>;

/// BVH stuff
#[derive(Clone, Debug)]
pub struct Triangle {
    id: usize,
    vertices: [Vertex; 3]
}

impl Triangle {
    pub fn new(id: usize, vertices: [Vertex; 3]) -> Self {
        Triangle {
            id: id,
            vertices: vertices
        }
    }

    pub fn intersect(
        &self, 
        origin: &nalgebra::Point3::<f32>,
        dir: &nalgebra::Vector3::<f32>
    ) -> bool {
        let epsilon = 1e-6;
        let v0 = self.vertices[0];
        let v1 = self.vertices[1];
        let v2 = self.vertices[2];
        let edge1 = v1 - v0;
        let edge2 = v2 - v0;
        let h = dir.cross(&edge2);
        let a = edge1.dot(&h);
        if a.abs() < epsilon {
            return false;
        }
        let f = 1.0 / a;
        let s = origin - v0;
        let u = f * s.dot(&h);
        if u < 0.0 || u > 1.0 {
            return false;
        }
        let q = s.cross(&edge1);
        let v = f * dir.dot(&q);
        if v < 0.0 || u + v > 1.0 {
            return false;
        }
        let t = f * edge2.dot(&q);
        t > epsilon
    }

    pub fn vertices(&self) -> &[Vertex; 3] {
        &self.vertices
    }
}

impl Bounded<f32, 3> for Triangle {
    fn aabb(&self) -> Aabb<f32, 3> {
        let epsilon = 1e-4;
        let min = self.vertices
            .iter()
            .fold(self.vertices[0], |min, v| min.inf(v)) - nalgebra::Vector3::<f32>::new(epsilon, epsilon, epsilon);
        let max = self.vertices
            .iter()
            .fold(self.vertices[0], |max, v| max.sup(v)) + nalgebra::Vector3::<f32>::new(epsilon, epsilon, epsilon);
        let aabb = Aabb::with_bounds(min, max);
        aabb
    }
}

impl BHShape<f32, 3> for Triangle {
    fn set_bh_node_index(&mut self, id: usize) { self.id = id }
    fn bh_node_index(&self) -> usize { self.id }
}

impl Index<usize> for Triangle {
    type Output = Vertex;

    fn index(&self, index: usize) -> &Self::Output {
        &self.vertices[index]
    }
}

///
#[derive(Debug)]
pub struct BoundingBox {
    pub x_min: f32,
    pub y_min: f32,
    pub z_min: f32,
    pub x_max: f32,
    pub y_max: f32,
    pub z_max: f32
}

///
#[derive(Clone, Debug)]
pub struct STLMesh {
    faces: Faces,
    file_name: String,
    triangles: Triangles,
    vertices: Vertices,
}

impl STLMesh {
    pub fn new(file_name: String) -> Self {
        let mut file = OpenOptions::new()
            .read(true)
            .open(&file_name)
            // .unwrap();
            .expect(
                format!(
                    "Failed to open STL file. Path is {:?} and current dir is {:?}", 
                    path::Path::new(&file_name),
                    env::current_dir().unwrap()
                ).as_str()
            );
        let stl = stl_io::read_stl(&mut file).unwrap();

        let vertices: Vec<_> = stl.vertices
            .iter()
            .map(|x| Point::new(x[0], x[1], x[2]))
            .collect();
        let triangles = STLMesh::_triangles(&stl.faces, &vertices);
        let triangles: Vec<_> = triangles
            .iter()
            .enumerate()
            .map(|(i, tri)| Triangle::new(i, *tri))
            .collect();
        STLMesh {
            faces: stl.faces,
            file_name: file_name,
            triangles: triangles,
            vertices: vertices
        }
    }

    pub fn bounding_box(&self) -> BoundingBox {
        let mut x_min = self.vertices[0];
        let mut x_max = self.vertices[1];

        for v in &self.vertices {
            x_min[0] = x_min[0].min(v[0]);
            x_min[1] = x_min[1].min(v[1]);
            x_min[2] = x_min[2].min(v[2]);

            x_max[0] = x_max[0].max(v[0]);
            x_max[1] = x_max[1].max(v[1]);
            x_max[2] = x_max[2].max(v[2]);
        }

        BoundingBox {
            x_min: x_min[0], 
            y_min: x_min[1],
            z_min: x_min[2],
            x_max: x_max[0],
            y_max: x_max[1],
            z_max: x_max[2]
        }
    }

    pub fn faces(&self) -> &Vec<IndexedTriangle> {
        &self.faces
    }

    pub fn file_name(&self) -> &String {
        &self.file_name
    }

    pub fn home_z(&mut self) -> () {
        let bb = self.bounding_box();
        self.translate(0., 0., -bb.z_min);
    }

    pub fn is_inside(&self, direction: &nalgebra::Vector3::<f32>, hits: &Vec<&Triangle>, point: &Point) -> u8 {
        // let mut count = 0;
        // for tri in hits {
        //     if tri.intersect(point, &direction) {
        //         count = count + 1
        //     }
        // }
        let count: i32 = hits
            // .into_par_iter()
            .into_iter()
            .map(|tri| if tri.intersect(point, &direction) { 1 } else { 0 })
            .sum();
        (count % 2 == 1) as u8
    }

    pub fn center_xy(&mut self) {
        let bb = self.bounding_box();
        let x = (bb.x_max - bb.x_min) / 2.;
        let y = (bb.y_max - bb.y_min) / 2.;
        let z = 0.;
        self.translate(x, y, z);
    }

    pub fn scale(&mut self, x: f32, y: f32, z: f32) {
        self.vertices = self.vertices
            .iter_mut()
            .map(|a| Vertex::new(
                x * a[0], y * a[1], z * a[2]
            ))
            .collect::<Vertices>();
        let triangles = STLMesh::_triangles(&self.faces, &self.vertices);
        self.triangles = triangles
            .iter()
            .enumerate()
            .map(|(i, tri)| Triangle::new(i, *tri))
            .collect();
    }

    pub fn translate(&mut self, x: f32, y: f32, z: f32) {
        self.vertices = self.vertices
            .iter_mut()
            .map(|a| Vertex::new(
                a[0] + x, a[1] + y, a[2] + z
            ))
            .collect::<Vertices>();
        let triangles = STLMesh::_triangles(&self.faces, &self.vertices);
        self.triangles = triangles
            .iter()
            .enumerate()
            .map(|(i, tri)| Triangle::new(i, *tri))
            .collect();
    }

    /// helper method for generating triangles
    /// should only really be called once for each STL
    /// unless of course you scale, translate, etc.
    pub fn _triangles(faces: &Faces, vertices: &Vertices) -> Vec<[Vertex; 3]> {
        let tris: Vec<_> = faces
            .iter()
            .map(|x| [
                vertices[x.vertices[0]], 
                vertices[x.vertices[1]], 
                vertices[x.vertices[2]]
            ])
            .collect();
        tris
    } 

    pub fn triangles(&self) -> &Triangles {
        &self.triangles
    }

    pub fn vertices(&self) -> &Vertices {
        &self.vertices
    }
}
