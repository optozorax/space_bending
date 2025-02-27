use std::ops::{Add, Mul, Sub};
use wasm_bindgen::prelude::*;

#[allow(non_camel_case_types)]
pub type fxx = f64;

// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, Debug)]
pub struct Vector3 {
    pub x: fxx,
    pub y: fxx,
    pub z: fxx,
}

impl Vector3 {
    pub fn new(x: fxx, y: fxx, z: fxx) -> Self {
        Self { x, y, z }
    }

    pub fn length(&self) -> fxx {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    pub fn normalize(&self) -> Self {
        let len = self.length();
        if len == 0.0 {
            return Self::new(0.0, 0.0, 0.0);
        }
        Self::new(self.x / len, self.y / len, self.z / len)
    }

    pub fn dot(&self, other: &Self) -> fxx {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    pub fn cross(&self, other: &Self) -> Self {
        Self::new(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x,
        )
    }
}

impl Add for Vector3 {
    type Output = Self;

    fn add(self, other: Self) -> Self {
        Self::new(self.x + other.x, self.y + other.y, self.z + other.z)
    }
}

impl Sub for Vector3 {
    type Output = Self;

    fn sub(self, other: Self) -> Self {
        Self::new(self.x - other.x, self.y - other.y, self.z - other.z)
    }
}

impl Mul<fxx> for Vector3 {
    type Output = Self;

    fn mul(self, scalar: fxx) -> Self {
        Self::new(self.x * scalar, self.y * scalar, self.z * scalar)
    }
}

// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------

#[derive(Clone, Debug)]
pub struct Particle {
    pub position: Vector3,
    pub velocity: Vector3,
    pub force: Vector3,
}

impl Particle {
    pub fn new(x: fxx, y: fxx, z: fxx) -> Self {
        Self {
            position: Vector3::new(x, y, z),
            velocity: Vector3::new(0.0, 0.0, 0.0),
            force: Vector3::new(0.0, 0.0, 0.0),
        }
    }
}

#[derive(Clone, Debug)]
pub struct EdgeSpring {
    pub i: usize,
    pub j: usize,
    pub rest_length: fxx,
    pub edge: bool,
}

impl EdgeSpring {
    pub fn new(i: usize, j: usize, rest_length: fxx, edge: bool) -> Self {
        Self {
            i,
            j,
            rest_length,
            edge,
        }
    }
}

#[derive(Clone, Debug)]
pub struct DihedralSpring {
    pub i1: usize, // First edge point
    pub i2: usize, // Second edge point
    pub i3: usize, // First wing point
    pub i4: usize, // Second wing point
    pub force: f64,
    pub edge: bool,
}

impl DihedralSpring {
    pub fn new(i1: usize, i2: usize, i3: usize, i4: usize, force: f64, edge: bool) -> Self {
        Self {
            i1,
            i2,
            i3,
            i4,
            force,
            edge,
        }
    }
}

#[derive(Clone, Debug)]
pub struct Triangle {
    pub indices: [usize; 3],
}

impl Triangle {
    pub fn new(i: usize, j: usize, k: usize) -> Self {
        Self { indices: [i, j, k] }
    }
}

// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------

fn dihedral_spring(p1: &Vector3, p2: &Vector3, p3: &Vector3, p4: &Vector3, k: fxx) -> [Vector3; 4] {
    // 1. Hinge axis and midpoint
    let hinge = *p2 - *p1;
    let axis = hinge.normalize();
    let mid = (*p1 + *p2) * 0.5;

    // 2. Compute wing vectors and project
    let v3 = *p3 - mid;
    let v4 = *p4 - mid;
    let v3_proj = v3 - (axis * v3.dot(&axis));
    let v4_proj = v4 - (axis * v4.dot(&axis));

    // Avoid division by zero
    if v3_proj.length() < 1e-6 || v4_proj.length() < 1e-6 {
        return [
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(0.0, 0.0, 0.0),
        ];
    }

    // 3. Measure misalignment
    let misalignment = v3_proj + v4_proj;

    // 4. Compute forces on wings
    let compute_wing_force = |v_proj: Vector3| -> Vector3 {
        let parallel = v_proj * (v_proj.dot(&misalignment) / v_proj.length().powi(2));
        (misalignment - parallel) * -k
    };

    let f3 = compute_wing_force(v3_proj);
    let f4 = compute_wing_force(v4_proj);

    // 5. Forces on hinge points
    let hinge_force = (f3 + f4) * -0.5;

    [hinge_force, hinge_force, f3, f4]
}

fn calc_forces(
    particles: &mut Vec<Particle>,
    edge_springs: &Vec<EdgeSpring>,
    dihedral_springs: &Vec<DihedralSpring>,
    edge_k: fxx,
    dihedral_k: fxx,
    damping: fxx,
    edge_coef: fxx,
    gravity_coef: fxx,
    vertex_graph_distance: &[Vec<fxx>],
) {
    // Reset forces
    for particle in particles.iter_mut() {
        particle.force = Vector3::new(0.0, 0.0, 0.0);
    }

    // Apply edge spring forces
    for spring in edge_springs {
        let p1 = &particles[spring.i];
        let p2 = &particles[spring.j];

        let delta = p2.position - p1.position;
        let current_length = delta.length();

        if current_length == 0.0 {
            continue;
        }

        let direction = delta * (1.0 / current_length);

        // Calculate relative velocity
        let relative_velocity = (p2.velocity - p1.velocity).dot(&direction);

        // Spring force with damping
        let spring_force = edge_k * (current_length - spring.rest_length).min(0.1);
        let damping_force = damping * relative_velocity;
        let mut total_force = spring_force + damping_force;

        if spring.edge {
            total_force *= edge_coef;
        }

        let force_vector = direction * total_force;

        // Apply forces
        particles[spring.i].force = particles[spring.i].force + force_vector;
        particles[spring.j].force = particles[spring.j].force - force_vector;
    }

    // Apply dihedral spring forces
    for spring in dihedral_springs {
        let p1 = particles[spring.i1].position;
        let p2 = particles[spring.i2].position;
        let p3 = particles[spring.i3].position;
        let p4 = particles[spring.i4].position;

        let forces = dihedral_spring(&p1, &p2, &p3, &p4, dihedral_k);

        let mul = spring.force;

        particles[spring.i1].force = particles[spring.i1].force + (forces[0] * mul);
        particles[spring.i2].force = particles[spring.i2].force + (forces[1] * mul);
        particles[spring.i3].force = particles[spring.i3].force + (forces[2] * mul);
        particles[spring.i4].force = particles[spring.i4].force + (forces[3] * mul);
    }

    // Apply gravity forces
    if gravity_coef > 0.0001 {
        for i in 0..particles.len() {
            for j in 0..i {
                let dir = particles[i].position - particles[j].position;
                let dir_len = dir.length();
                let graph_dist = vertex_graph_distance[i][j] / (2.0 as fxx).sqrt();
                if graph_dist < 0.2 {
                    continue;
                }
                let dir = dir.normalize()
                    * (1. / (dir_len * dir_len) * gravity_coef
                        / (particles.len() * particles.len()) as fxx)
                    * (graph_dist * graph_dist);

                particles[i].force = particles[i].force + dir;
                particles[j].force = particles[j].force - dir;
            }
        }
    }
}

fn all_pairs_shortest_paths(edges: &[EdgeSpring]) -> Vec<Vec<fxx>> {
    // Find the maximum vertex index to determine the graph size
    let max_vertex = edges.iter().map(|s| s.i.max(s.j)).max().unwrap_or(0);

    let n = max_vertex + 1; // Number of vertices

    // Initialize distance matrix with infinity
    let mut dist = vec![vec![fxx::INFINITY; n]; n];

    // Distance from a vertex to itself is 0
    for i in 0..n {
        dist[i][i] = 0.0;
    }

    // Set initial distances based on the edges
    for s in edges {
        dist[s.i][s.j] = s.rest_length;
        dist[s.j][s.i] = s.rest_length;
    }

    // Floyd-Warshall algorithm
    for k in 0..n {
        for i in 0..n {
            for j in 0..n {
                if dist[i][k] != fxx::INFINITY && dist[k][j] != fxx::INFINITY {
                    dist[i][j] = dist[i][j].min(dist[i][k] + dist[k][j]);
                }
            }
        }
    }

    dist
}

// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------

#[derive(Clone, Debug)]
pub struct Mesh {
    particles: Vec<Particle>,
    edge_springs: Vec<EdgeSpring>,
    dihedral_springs: Vec<DihedralSpring>,
    triangles: Vec<Triangle>,
    sizex: usize,
    sizey: usize,

    // Simulation constants
    dt: fxx,
    edge_spring_constant: fxx,
    dihedral_spring_constant: fxx,
    damping_coefficient: fxx,
    global_damping: fxx,
    edge_coef: fxx,
    gravity_coef: fxx,

    // Buffer for triangle data
    triangle_buffer: Vec<f32>,
    uv_buffer: Vec<f32>,

    vertex_graph_distance: Vec<Vec<fxx>>,
}

impl Mesh {
    pub fn new() -> Self {
        Self {
            particles: Vec::new(),
            edge_springs: Vec::new(),
            dihedral_springs: Vec::new(),
            triangles: Vec::new(),
            sizex: 0,
            sizey: 0,

            dt: 0.1,
            edge_spring_constant: 10.0,
            dihedral_spring_constant: 10.0,
            damping_coefficient: 0.5,
            global_damping: 0.5,
            edge_coef: 1.0,
            gravity_coef: 0.0,

            triangle_buffer: Vec::new(),
            uv_buffer: Vec::new(),
            vertex_graph_distance: Vec::new(),
        }
    }

    pub fn init(&mut self, sizex: usize, sizey: usize) {
        // Clear existing data
        self.particles.clear();
        self.edge_springs.clear();
        self.dihedral_springs.clear();
        self.triangles.clear();
        self.sizex = sizex;
        self.sizey = sizey;

        let sizea = sizex.min(sizey);

        // Create particles in a grid
        for i in 0..sizex {
            for j in 0..sizey {
                let x = i as fxx / (sizea - 1) as fxx;
                let y = j as fxx / (sizea - 1) as fxx;
                let z = 0.0;
                self.particles.push(Particle::new(x, y, z));
            }
        }

        // Helper function to get index from grid coordinates
        // let get_index = |mut i: i32, mut j: i32| {
        let get_index = |i, j| {
            // if i < 0 { i = sizex + i; }
            // if i >= sizex
            i * sizey + j
        };

        let get_index_inv = |i, j| get_index(j, i);

        // Create edge springs and triangles
        for i in 0..sizex - 1 {
            for j in 0..sizey - 1 {
                let idx00 = get_index(i, j);
                let idx01 = get_index(i, j + 1);
                let idx10 = get_index(i + 1, j);
                let idx11 = get_index(i + 1, j + 1);

                // Get positions
                let p00 = self.particles[idx00].position;
                let p01 = self.particles[idx01].position;
                let p10 = self.particles[idx10].position;
                let p11 = self.particles[idx11].position;

                // Create horizontal edge springs
                let rest_length_h = (p01 - p00).length();
                self.edge_springs
                    .push(EdgeSpring::new(idx00, idx01, rest_length_h, false));

                // Create vertical edge springs
                let rest_length_v = (p10 - p00).length();
                self.edge_springs
                    .push(EdgeSpring::new(idx00, idx10, rest_length_v, false));

                // Bottom edge springs (for last row/column)
                if i == sizex - 2 {
                    let rest_length_bottom = (p11 - p10).length();
                    self.edge_springs.push(EdgeSpring::new(
                        idx10,
                        idx11,
                        rest_length_bottom,
                        false,
                    ));
                }

                if j == sizey - 2 {
                    let rest_length_right = (p11 - p01).length();
                    self.edge_springs
                        .push(EdgeSpring::new(idx01, idx11, rest_length_right, false));
                }

                // Create diagonal springs (for stability)
                let rest_length_d1 = (p11 - p00).length();
                self.edge_springs
                    .push(EdgeSpring::new(idx00, idx11, rest_length_d1, false));

                let rest_length_d2 = (p10 - p01).length();
                self.edge_springs
                    .push(EdgeSpring::new(idx01, idx10, rest_length_d2, false));

                // Create triangles
                self.triangles.push(Triangle::new(idx00, idx01, idx10));
                self.triangles.push(Triangle::new(idx01, idx11, idx10));
            }
        }

        // Fill uv buffer
        {
            // Size the buffer correctly (3 vertices * 2 UV coords per triangle)
            let buffer_size = self.triangles.len() * 6;
            if self.uv_buffer.len() != buffer_size {
                self.uv_buffer.resize(buffer_size, 0.0);
            }

            // Fill the buffer with UV coordinates based on particle positions
            let mut index = 0;
            for triangle in &self.triangles {
                for &idx in &triangle.indices {
                    let particle = &self.particles[idx];
                    // Use x and y as u and v (they're already in 0..1 range)
                    self.uv_buffer[index] = particle.position.x as f32 / ((sizex - 1) as f32 / (sizea - 1) as f32);
                    self.uv_buffer[index + 1] = particle.position.y as f32 / ((sizey - 1) as f32 / (sizea - 1) as f32);
                    index += 2;
                }
            }
        }

        let spring_dist = 1;
        for i in spring_dist..sizex - 1 - spring_dist {
            for j in 0..sizey - 2 {
                self.dihedral_springs.push(DihedralSpring::new(
                    get_index(i, j),
                    get_index(i, j + 1),
                    get_index(i - spring_dist, j),
                    get_index(i + spring_dist, j + 1),
                    1.0,
                    false,
                ));
                self.dihedral_springs.push(DihedralSpring::new(
                    get_index(i, j),
                    get_index(i, j + 1),
                    get_index(i - spring_dist, j+1),
                    get_index(i + spring_dist, j),
                    1.0,
                    false,
                ));
            }
        }
        for i in 0..sizex - 2 {
            for j in spring_dist..sizey - 1 - spring_dist {
                self.dihedral_springs.push(DihedralSpring::new(
                    get_index(i, j),
                    get_index(i + 1, j),
                    get_index(i, j - spring_dist),
                    get_index(i + 1, j + spring_dist),
                    1.0,
                    false,
                ));
                self.dihedral_springs.push(DihedralSpring::new(
                    get_index(i, j),
                    get_index(i + 1, j),
                    get_index(i+1, j - spring_dist),
                    get_index(i, j + spring_dist),
                    1.0,
                    false,
                ));
            }
        }
        for i in 0..sizex - 2 {
            for j in 0..sizey - 2 {
                self.dihedral_springs.push(DihedralSpring::new(
                    get_index(i+1, j),
                    get_index(i, j+1),
                    get_index(i, j),
                    get_index(i+1, j+1),
                    1.0,
                    false,
                ));
                self.dihedral_springs.push(DihedralSpring::new(
                    get_index(i, j),
                    get_index(i+1, j+1),
                    get_index(i, j+1),
                    get_index(i+1, j),
                    1.0,
                    false,
                ));
            }
        }

        let spring_dist = 2;
        for i in spring_dist..sizex - 1 - spring_dist {
            for j in 0..sizey - 2 {
                self.dihedral_springs.push(DihedralSpring::new(
                    get_index(i, j),
                    get_index(i, j + 1),
                    get_index(i - spring_dist, j),
                    get_index(i + spring_dist, j + 1),
                    0.25,
                    false,
                ));
            }
        }
        for i in 0..sizex - 2 {
            for j in spring_dist..sizey - 1 - spring_dist {
                self.dihedral_springs.push(DihedralSpring::new(
                    get_index(i, j),
                    get_index(i + 1, j),
                    get_index(i, j - spring_dist),
                    get_index(i + 1, j + spring_dist),
                    0.25,
                    false,
                ));
            }
        }

        // Modify Z coordinates of first row
        let mut rng: xorshift::Xorshift128 =
            xorshift::SeedableRng::from_seed([42u64, 137u64].as_slice());
        for particle in &mut self.particles {
            use xorshift::Rng;
            particle.position.z += rng.gen_range(-0.001, 0.001);
        }

        // Add special springs
        if false {
            let blue_x = (sizea as fxx * 0.2) as usize;
            let orange_x = (sizea as fxx * 0.8) as usize;
            let start_y = (sizea as fxx * 0.4) as usize;
            let end_y = (sizea as fxx * 0.6) as usize;

            for i in start_y..=end_y {
                self.edge_springs.push(EdgeSpring::new(
                    get_index(blue_x, i),
                    // get_index(orange_x, end_y + start_y - i),
                    get_index(orange_x, i),
                    0.0,
                    true,
                ));
            }

            for i in start_y..end_y {
                self.dihedral_springs.push(DihedralSpring::new(
                    get_index(blue_x, i),
                    get_index(blue_x, i + 1),
                    get_index(blue_x + 1, i + 1),
                    get_index(orange_x - 1, i),
                    1.0,
                    true,
                ));
                self.dihedral_springs.push(DihedralSpring::new(
                    get_index(blue_x, i),
                    get_index(blue_x, i + 1),
                    get_index(blue_x + 2, i + 1),
                    get_index(orange_x - 2, i),
                    0.25,
                    true,
                ));
                self.dihedral_springs.push(DihedralSpring::new(
                    get_index(blue_x+1, i),
                    get_index(blue_x+1, i + 1),
                    get_index(blue_x + 3, i + 1),
                    get_index(orange_x - 1, i),
                    0.25,
                    true,
                ));

                self.dihedral_springs.push(DihedralSpring::new(
                    get_index(blue_x, i),
                    get_index(blue_x, i + 1),
                    get_index(blue_x - 1, i + 1),
                    get_index(orange_x + 1, i),
                    1.0,
                    true,
                ));
                self.dihedral_springs.push(DihedralSpring::new(
                    get_index(blue_x, i),
                    get_index(blue_x, i + 1),
                    get_index(blue_x - 2, i + 1),
                    get_index(orange_x + 2, i),
                    0.25,
                    true,
                ));
                self.dihedral_springs.push(DihedralSpring::new(
                    get_index(blue_x-1, i),
                    get_index(blue_x-1, i + 1),
                    get_index(blue_x - 3, i + 1),
                    get_index(orange_x + 1, i),
                    0.25,
                    true,
                ));
            }
        }

        if true {
            for j in 0..sizey {
                // cylinder
                self.edge_springs.push(EdgeSpring::new(
                    get_index(0, j),
                    get_index(sizex - 1, j),
                    0.0,
                    true,
                ));
            }
            for i in 0..sizex {
                // cylinder + this = torus
                self.edge_springs.push(EdgeSpring::new(
                    get_index(i, 0),
                    get_index(i, sizey-1),
                    0.0,
                    true
                ));
            }

            // for j in 0..sizey {
            //     // mobius strip
            //     self.edge_springs.push(EdgeSpring::new(
            //         get_index(0, j),
            //         get_index(sizex - 1, sizey - 1 - j),
            //         0.0,
            //         true,
            //     ));
            // }

            for j in 0..sizey - 1 {
                // cylinder
                self.dihedral_springs.push(DihedralSpring::new(
                    get_index(0, j),
                    get_index(0, j + 1),
                    get_index(1, j + 1),
                    get_index(sizex - 2, j),
                    1.0,
                    true,
                ));
                self.dihedral_springs.push(DihedralSpring::new(
                    get_index(1, j),
                    get_index(1, j + 1),
                    get_index(3, j + 1),
                    get_index(sizex - 2, j),
                    0.25,
                    true,
                ));
                self.dihedral_springs.push(DihedralSpring::new(
                    get_index(0, j),
                    get_index(0, j + 1),
                    get_index(2, j + 1),
                    get_index(sizex - 3, j),
                    0.25,
                    true,
                ));
                self.dihedral_springs.push(DihedralSpring::new(
                    get_index(sizex - 2, j),
                    get_index(sizex - 2, j + 1),
                    get_index(1, j + 1),
                    get_index(sizex - 4, j),
                    0.25,
                    true,
                ));
            }
            for i in 0..sizex - 1 {
                // cylinder + this = torus
                self.dihedral_springs.push(DihedralSpring::new(
                    get_index(i, 0),
                    get_index(i+1, 0),
                    get_index(i+1, 1),
                    get_index(i, sizey-2),
                    1.0,
                    true,
                ));
                self.dihedral_springs.push(DihedralSpring::new(
                    get_index_inv(1, i),
                    get_index_inv(1, i + 1),
                    get_index_inv(3, i + 1),
                    get_index_inv(sizey - 2, i),
                    0.25,
                    true,
                ));
                self.dihedral_springs.push(DihedralSpring::new(
                    get_index_inv(0, i),
                    get_index_inv(0, i + 1),
                    get_index_inv(2, i + 1),
                    get_index_inv(sizey - 3, i),
                    0.25,
                    true,
                ));
                self.dihedral_springs.push(DihedralSpring::new(
                    get_index_inv(sizey - 2, i),
                    get_index_inv(sizey - 2, i + 1),
                    get_index_inv(1, i + 1),
                    get_index_inv(sizey - 4, i),
                    0.25,
                    true,
                ));
            }

            // for i in 0..sizey - 1 {
            //     // mobius strip
            //     self.dihedral_springs.push(DihedralSpring::new(
            //         get_index(0, sizey - 1 - i),
            //         get_index(0, sizey - 1 - (i + 1)),
            //         get_index(1, sizey - 1 - (i + 1)),
            //         get_index(sizex - 1 - 1, sizey - 1 - i),
            //         1.0,
            //         true,
            //     ));
            //     self.dihedral_springs.push(DihedralSpring::new(
            //         get_index(0, sizey - 1 - i),
            //         get_index(0, sizey - 1 - (i + 1)),
            //         get_index(2, sizey - 1 - (i + 1)),
            //         get_index(sizex - 1 - 2, sizey - 1 - i),
            //         0.25,
            //         true,
            //     ));
            // }
        }

        // Update vertex graph distances
        // self.vertex_graph_distance = all_pairs_shortest_paths(&self.edge_springs);

        // Initialize the triangle buffer
        self.update_triangle_buffer();
    }

    // Get triangle data for rendering
    fn update_triangle_buffer(&mut self) {
        // Calculate center of mesh
        let mut avg = Vector3::new(0.0, 0.0, 0.0);
        for particle in &self.particles {
            avg = avg + particle.position;
        }
        avg = avg * (1.0 / self.particles.len() as fxx);

        // Size the buffer correctly (3 vertices * 3 coordinates per triangle)
        let buffer_size = self.triangles.len() * 9;
        if self.triangle_buffer.len() != buffer_size {
            self.triangle_buffer.resize(buffer_size, 0.0);
        }

        // Fill the buffer with triangle vertex positions
        let mut index = 0;
        for triangle in &self.triangles {
            for &idx in &triangle.indices {
                let pos = self.particles[idx].position - avg;
                self.triangle_buffer[index] = pos.x as f32;
                self.triangle_buffer[index + 1] = pos.y as f32;
                self.triangle_buffer[index + 2] = pos.z as f32;
                index += 3;
            }
        }
    }

    // Step the simulation forward in time
    pub fn step(&mut self) {
        self.integrate_rk4();
        self.update_triangle_buffer();
    }

    // Runge-Kutta 4 integration
    fn integrate_rk4(&mut self) {
        // Store original state
        let original_states: Vec<(Vector3, Vector3)> = self
            .particles
            .iter()
            .map(|p| (p.position, p.velocity))
            .collect();

        // Arrays for the 4 evaluations
        let mut k1 =
            vec![(Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 0.0)); self.particles.len()];
        let mut k2 =
            vec![(Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 0.0)); self.particles.len()];
        let mut k3 =
            vec![(Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 0.0)); self.particles.len()];
        let mut k4 =
            vec![(Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 0.0)); self.particles.len()];

        // STEP 1: First evaluation (k1) at the current state
        self.evaluate_derivatives(&mut k1);

        // STEP 2: Second evaluation (k2) at t + dt/2 using k1
        self.apply_derivatives_half_step(&k1, &original_states);
        self.evaluate_derivatives(&mut k2);

        // STEP 3: Third evaluation (k3) at t + dt/2 using k2
        self.restore_original_state(&original_states);
        self.apply_derivatives_half_step(&k2, &original_states);
        self.evaluate_derivatives(&mut k3);

        // STEP 4: Fourth evaluation (k4) at t + dt using k3
        self.restore_original_state(&original_states);
        self.apply_derivatives_full_step(&k3, &original_states);
        self.evaluate_derivatives(&mut k4);

        // Restore to original state before final update
        self.restore_original_state(&original_states);

        // STEP 5: Combine the derivatives with RK4 weights
        for i in 0..self.particles.len() {
            let p = &mut self.particles[i];

            // Update position
            let position_change = k1[i].0 * 1.0 + k2[i].0 * 2.0 + k3[i].0 * 2.0 + k4[i].0 * 1.0;

            // Update velocity
            let velocity_change = k1[i].1 * 1.0 + k2[i].1 * 2.0 + k3[i].1 * 2.0 + k4[i].1 * 1.0;

            p.position = p.position + position_change * (self.dt / 6.0);
            p.velocity = p.velocity + velocity_change * (self.dt / 6.0);
        }
    }

    // Evaluate derivatives at current state
    fn evaluate_derivatives(&mut self, derivatives: &mut Vec<(Vector3, Vector3)>) {
        // Calculate forces
        calc_forces(
            &mut self.particles,
            &self.edge_springs,
            &self.dihedral_springs,
            self.edge_spring_constant,
            self.dihedral_spring_constant,
            self.damping_coefficient,
            self.edge_coef,
            self.gravity_coef,
            &self.vertex_graph_distance,
        );

        // Store derivatives
        for (i, p) in self.particles.iter().enumerate() {
            // Apply global damping
            let damped_force = p.force - (p.velocity * self.global_damping);

            derivatives[i] = (p.velocity, damped_force);
        }
    }

    // Apply half-step changes
    fn apply_derivatives_half_step(
        &mut self,
        derivatives: &Vec<(Vector3, Vector3)>,
        original_states: &Vec<(Vector3, Vector3)>,
    ) {
        let half_dt = self.dt * 0.5;

        for i in 0..self.particles.len() {
            let p = &mut self.particles[i];
            let original = &original_states[i];

            p.position = original.0 + derivatives[i].0 * half_dt;
            p.velocity = original.1 + derivatives[i].1 * half_dt;
        }
    }

    // Apply full-step changes
    fn apply_derivatives_full_step(
        &mut self,
        derivatives: &Vec<(Vector3, Vector3)>,
        original_states: &Vec<(Vector3, Vector3)>,
    ) {
        for i in 0..self.particles.len() {
            let p = &mut self.particles[i];
            let original = &original_states[i];

            p.position = original.0 + derivatives[i].0 * self.dt;
            p.velocity = original.1 + derivatives[i].1 * self.dt;
        }
    }

    // Restore original state
    fn restore_original_state(&mut self, original_states: &Vec<(Vector3, Vector3)>) {
        for i in 0..self.particles.len() {
            let p = &mut self.particles[i];
            let original = &original_states[i];

            p.position = original.0;
            p.velocity = original.1;
        }
    }

    // Get constants
    pub fn get_constants(&self) -> (fxx, fxx, fxx, fxx, fxx, fxx, fxx) {
        (
            self.dt,
            self.edge_spring_constant,
            self.dihedral_spring_constant,
            self.damping_coefficient,
            self.global_damping,
            self.edge_coef,
            self.gravity_coef,
        )
    }

    // Set constant
    pub fn set_constant(&mut self, name: &str, value: f32) {
        match name {
            "dt" => self.dt = value as fxx,
            "edgeSpringConstant" => self.edge_spring_constant = value as fxx,
            "dihedralSpringConstant" => self.dihedral_spring_constant = value as fxx,
            "dampingCoefficient" => self.damping_coefficient = value as fxx,
            "globalDamping" => self.global_damping = value as fxx,
            "edgeCoef" => self.edge_coef = value as fxx,
            "gravityCoef" => self.gravity_coef = value as fxx,
            _ => {}
        }
    }

    // Get triangle buffer pointer and size
    pub fn get_triangle_buffer_ptr(&self) -> *const f32 {
        self.triangle_buffer.as_ptr()
    }

    pub fn get_uv_buffer_ptr(&mut self) -> *const f32 {
        self.uv_buffer.as_ptr()
    }

    pub fn get_triangle_count(&self) -> usize {
        self.triangles.len()
    }
}

// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------

#[wasm_bindgen]
pub struct MeshHandle {
    mesh: Mesh,
}

#[wasm_bindgen]
impl MeshHandle {
    #[wasm_bindgen(constructor)]
    pub fn new(sizex: usize, sizey: usize) -> Self {
        let mut mesh = Mesh::new();
        mesh.init(sizex, sizey);
        mesh.step();

        Self { mesh }
    }

    pub fn step(&mut self) {
        self.mesh.step();
        self.mesh.update_triangle_buffer();
    }

    pub fn get_triangle_buffer(&self) -> *const f32 {
        self.mesh.get_triangle_buffer_ptr()
    }

    pub fn get_uv_buffer(&mut self) -> *const f32 {
        self.mesh.get_uv_buffer_ptr()
    }

    pub fn get_triangle_count(&self) -> usize {
        self.mesh.get_triangle_count()
    }

    pub fn set_constant(&mut self, name: &str, value: f32) {
        self.mesh.set_constant(name, value);
    }
}

// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test() {
        let mut mesh = Mesh::new();
        mesh.init(10, 20);
        mesh.step();
        mesh.step();
        mesh.step();
        mesh.step();
        mesh.get_constants();
    }
}
