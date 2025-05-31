use crate::gcode::GcodeWriter;
use crate::geometry::STLMesh;
use crate::settings::Settings;
use crate::slicer::Slicer;

pub struct FFFSlicer {
    settings: Settings,
    stl_meshes: Vec<STLMesh>
}

impl FFFSlicer {
    pub fn new(
        settings: Settings, 
        stl_meshes: Vec<STLMesh>
    ) -> Self {
        Self {
            settings: settings,
            stl_meshes: stl_meshes
        }
    }

    pub fn perimeters(&self, stl: &STLMesh, gcode_writer: &mut GcodeWriter, z: f32) -> () {
        let tris = stl.triangles();
        

        // counter to track the index of the triangles that intersect with each z layer
        let mut counter = 0;
        // temprorary vector that stores the intersecting triangle indices
        let mut vec = vec![];
        // vector containing the intersection coordinates between the 
        // triangles and the plane z_height
        let mut ab_coords = vec![];

        // iterate through all the triangles in the stl
        for tri in tris.iter() {

            // break down vertices for easier handling (for me)
            let vert_1 = tri[0];
            let vert_2 = tri[1];
            let vert_3 = tri[2];

            // if any of the vertices are above AND any are below, 
            if vert_1[2] < z || vert_2[2] < z || vert_3[2] < z {
                if vert_1[2] > z || vert_2[2] > z || vert_3[2] > z {
                    
                    // append the index of intersecting triangles
                    vec.push(counter);

                    // Must sort the vertices to be able to calculate
                    // the line segment created by the interception of
                    // the z_height and the triangles that bound it.
                    // Assign signed integers to represent whether a
                    // vertex of a triangle is above (+1), below (-1),
                    // or on (0) the z_height plane.
                    //
                    // add all z vertices to vector
                    let z_vec = [vert_1[2], vert_2[2], vert_3[2]];
                    // empty vector for sorting
                    let mut z_sort = vec![];
                    for z_s in z_vec {
                        if z_s - z < 0.0 {
                            z_sort.push(-1);
                        } else if z_s - z > 0.0 {
                            z_sort.push(1);
                        } else {
                            z_sort.push(0);
                        }
                    }

                    // Test the relationship of the first two vertices,
                    // with the goal of arranging the vertices to be
                    // able to calculate the z intercept line segment.
                    // There are three triangles possible: two vertices 
                    // below z_height and one above, two vertices above
                    // z_height and one below, and one vertex coincident
                    // with z_height and one above and one below. In 
                    // the first two cases, the single vertex will go into
                    // the first index (0). In the last case, either 
                    // vertex not coincident with z_height will go into
                    // the first index (0). 
                    //
                    // Test the first two vertices by adding them. 
                    // There are five possible values for sum_z0_z1:
                    // 1) -2 the first and second vertices are below z_height
                    // 2) -1 the first and second vertices are coincident and below z_height
                    // 3)  0 the first and second vertices are below and above z_height
                    // 4) +1 the first and second vertices are coincident and above z_height
                    // 5) +2 the first and second vertices are above z_height
                    // For cases 2, 3, and 4, further logic is required to 
                    // deal with which vertex is coincident or if the third
                    // vertex is above or below z_height. 
                    let sum_z0_z1 = z_sort[0] + z_sort[1];
                    let sum_z0_z2 = z_sort[0] + z_sort[2];

                    // empty vector to store the properly sorted vertices
                    let mut sorted_z = vec![]; 

                    // are the first and second vertex on the same side?
                    if sum_z0_z1 > 1 || sum_z0_z1 < -1 {
                        // then the third vertex goes into index 0
                        sorted_z = vec![2, 0, 1];
                    
                    // is the first or second vertex coincident with z_height?
                    } else if sum_z0_z1 > 0 || sum_z0_z1 < 0 {
                        // is the first vertex coincident with z_height?
                        if z_sort[0] == 0 {
                            // any other vertex goes in index 0
                            sorted_z = vec![1, 0, 2];

                        // then the second vertex is coincident with z_height
                        } else {
                            // original order acceptable
                            sorted_z = vec![0, 1, 2];
                        }
                        
                    // are the first and second vertices on opposite sides?
                    } else if sum_z0_z1 == 0 {
                        // is the third vertex coincident with z_height?
                        if z_sort[2] == 0 {
                            // order is fine
                            sorted_z = vec![0, 1, 2];

                        // is the third element the same as the first?
                        } else if sum_z0_z2 > 0 || sum_z0_z2 < 0 {
                            // the second element goes into index 0
                            sorted_z = vec![1, 0, 2];
                        // the first element is different than the second and third
                        } else {
                            // the order is acceptable
                            sorted_z = vec![0, 1, 2];
                        }
                            
                    }

                    
                    // The z_height intersection will be composed of two points,
                    // a and b. Calculate intersection points (xa, ya) and 
                    // (xb, yb), and push them to a vector ab_coords.
                    let z_factor_a = (z - tri[sorted_z[0]][2])/(tri[sorted_z[1]][2] - tri[sorted_z[0]][2]);
                    let z_factor_b = (z - tri[sorted_z[0]][2])/(tri[sorted_z[2]][2] - tri[sorted_z[0]][2]);
                    let xa = (z_factor_a * (tri[sorted_z[1]][0] - tri[sorted_z[0]][0])) + tri[sorted_z[0]][0];
                    let ya = (z_factor_a * (tri[sorted_z[1]][1] - tri[sorted_z[0]][1])) + tri[sorted_z[0]][1];
                    let xb = (z_factor_b * (tri[sorted_z[2]][0] - tri[sorted_z[0]][0])) + tri[sorted_z[0]][0];
                    let yb = (z_factor_b * (tri[sorted_z[2]][1] - tri[sorted_z[0]][1])) + tri[sorted_z[0]][1];

                    let a_coord = [xa, ya];
                    let b_coord = [xb, yb];
                    ab_coords.push([a_coord, b_coord]);
                }
            }
            // increase counter for next triangle
            counter += 1;
            
        } // end looping through all triangles for a layer at z_height
        
        println!("Global Layer height; {:?}", z);
        println!("Number of triangles in Layer: {:?}", vec.len());
        
        // perimeter array of the sorted line segments
        let mut perimeter_sort = Vec::new();
        // Clone the vector of all line segments, we 
        // will be removing all used segments
        let mut ab_coords = Vec::from(ab_coords.clone());
        // push the first line segment to the sorted
        // list, the exact place to start will need 
        // to be adjusted in the future.
        if ab_coords.len() > 0 {
            perimeter_sort.push(ab_coords[0]);           
            // remove the first line segment
            ab_coords.swap_remove(0);
            // create a counter variable
            let mut num_available_segs = ab_coords.len();

            // cycle through all line segments in ab_coords; if x_a and y_a of
            // a line segment matches the x_b and y_b of the last segment in 
            // perimeter_sort, then push that line segment to perimeter_sort. 
            //
            // TODO This doesn't delete the used segment.

            // Search radius for next line segment.
            let mut search_radius = 1.0e-3;

            // while the number of available line segments is 
            // above some number, keep building the connected
            // lines
            while num_available_segs > 0 {
                // index counter, for the triangle in ab_coords
                let mut index_count = 0;
                // vector to save the matcing indices. This 
                // needs to be a vector because there is a 
                // chance the search radius is too big and
                // finds more than one viable coordinate.
                let mut index_match = vec![];
                // does the order of the vector of coordinats 
                // need to switched to properly follow the  
                // perimeter
                let mut vertex_switch = vec![];
                // for each line segment in the layer, 
                // compare the beginning (a) of a segment
                // to the end of thelast line segment (b)
                // in perimeter_sort vector. 
                // 
                // TODO THIS IS WRONG, it needs to look at 
                // both points in each line segment, since
                // the order of a and b is arbitrary. Oof.
                for segs in ab_coords.iter() {
                    // println!("Segments: {:?}", ab_coords);
                    // println!("Last Segment: {:?}",perimeter_sort.last().expect("Perimeter is empty"));
                    
                    // find the distance between the last coordinate in perimeter_sort
                    // and the current line segment, vertex a
                    let x_diff_a = (segs[0][0] - perimeter_sort.last().expect("Perimeter is empty")[1][0]).abs();
                    let y_diff_a = (segs[0][1] - perimeter_sort.last().expect("Perimeter is empty")[1][1]).abs();
                    // ... same as above, but with vertex b
                    let x_diff_b = (segs[1][0] - perimeter_sort.last().expect("Perimeter is empty")[1][0]).abs();
                    let y_diff_b = (segs[1][1] - perimeter_sort.last().expect("Perimeter is empty")[1][1]).abs();
                    
                    // square the difference values
                    let x_diff_a_square = x_diff_a.powi(2);
                    let y_diff_a_square = y_diff_a.powi(2);
                    let x_diff_b_square = x_diff_b.powi(2);
                    let y_diff_b_square = y_diff_b.powi(2);
                    // sum the square differences
                    let diff_a_square_sum = x_diff_a_square + y_diff_a_square;
                    let diff_b_square_sum = x_diff_b_square + y_diff_b_square;
                    // calculate the euclidean distance between
                    // the a and b verices and the endpoint of
                    // the perimeter line vector.
                    let euclidean_a = diff_a_square_sum.sqrt();
                    let euclidean_b = diff_b_square_sum.sqrt();
                    
                    // let mut x_diff_bits: u64 = x_diff.to_bits();
                    // let mut y_diff_bits: u64 = y_diff.to_bits();
                    // x_diff_bits = x_diff_bits * x_diff_bits as u64;
                    // y_diff_bits = y_diff_bits * y_diff_bits as u64;
                    // let diff_square_sum = x_diff_bits + y_diff_bits;
                    // let mut euclidean = f32::from_bits(diff_square_sum);
                    
                    // Only use the closer of the two euclidean values
                    //
                    // TODO Need to decide which euclidean is smaller
                    // and pass those values to the search.

                    // check which vertex is closer to the last point 
                    // in the perimeter. Assign the respective
                    // euclidean value and a 0 or 1 for which is 
                    // closer (this is for rearranging outside this
                    // loop).
                    let which_vertex: f32;
                    let smaller_euclidean: f32;
                    if euclidean_a > euclidean_b {
                        smaller_euclidean = euclidean_b;
                        which_vertex = 1.0;
                    } else {
                        smaller_euclidean = euclidean_a;
                        which_vertex = 0.0;
                    }

                    // let smaller_euclidean = euclidean_a;
                    let euclidean = smaller_euclidean;
                    if euclidean <= search_radius  {
                        // if y_diff <= search_radius {
                        // perimeter_sort.push(*segs);
                        index_match.push(index_count);
                        vertex_switch.push(which_vertex);
                        // }
                    }
                    index_count += 1;
                }

                // The search radius may result in one, more than one, or 
                // no matches found. If one is found, we consider it a true match
                // and pass it through to the ordered perimeter_sort vector. If
                // more than one is found, shrink the search radius, and repeat
                // the search. If none are found, grow the search radus, and
                // repeat the search. (Hopefully).
                //
                if index_match.len() == 1 {
                    perimeter_sort.push(ab_coords[index_match[0]]);
                    num_available_segs -= 1;
                    let remove = *index_match.last().expect("Nothing in the perimeter");
                    ab_coords.swap_remove(remove);
                    search_radius = 1e-3;
                    gcode_writer.write_perimeter(perimeter_sort.last().unwrap()[1][0], perimeter_sort.last().unwrap()[1][1], 555.0, 1200.0);
                } else if index_match.len() > 1 {
                    search_radius = search_radius * 0.9;
                } else {
                    search_radius = search_radius * 1.1;
                }
                
            }            
            println!("----------------------------------");
        }
        
        // Sort the ab_coords per layer to produce the perimeter
        // ouline paths. 
        // Must remove used coord pairs from available list in
        // order to check if multiple print "islands" occure per
        // layer.
    }
}

impl Slicer for FFFSlicer {
    fn slice(&self, gcode_file: &str) {
        let mut gcode_writer = GcodeWriter::new(gcode_file);
        gcode_writer.write_header(&self.settings);

        for stl in &self.stl_meshes {
            println!("Slicing stl file {:?}", stl.file_name());
            println!("{}", self.settings);
            println!("Generating layer heights");
            let zs = self.layer_heights(&self.settings, &stl);
            let mut z_height: f32 = 0.0;
            // TODO must shift the stl up in the z direction, no negatives
            // let z_offset = -1.0 * -stl.bounding_box().z_min;
            // let _ = stl.translate(0.0,0.0,z_offset);
            for (n, z) in zs.iter().enumerate() {
                
                let positioning = 0;

                if positioning == 0 {
                    z_height = z_height + *z;
                } else {
                    z_height = *z;
                }
                
                 // height of print plane
                gcode_writer.write_layer_change(
                    n.try_into().unwrap(), z_height, 'F', 1200. // TODO
                );
                // TODO skirt/brim
                println!("Generating perimeters for layer {}", n);
                println!("Layer heigh: {:?}",*z);
                let _ = self.perimeters(&stl, &mut gcode_writer, z_height);
                // TODO infill
            }
        }
    }
}
