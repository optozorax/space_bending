use std::ops::{Add, Mul, Sub};
use wasm_bindgen::prelude::*;

// ============= Vector3 Implementation =============
#[derive(Clone, Copy, Debug)]
pub struct Vector3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vector3 {
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    pub fn length(&self) -> f32 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    pub fn normalize(&self) -> Self {
        let len = self.length();
        if len == 0.0 {
            return Self::new(0.0, 0.0, 0.0);
        }
        Self::new(self.x / len, self.y / len, self.z / len)
    }

    pub fn dot(&self, other: &Self) -> f32 {
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

impl Mul<f32> for Vector3 {
    type Output = Self;

    fn mul(self, scalar: f32) -> Self {
        Self::new(self.x * scalar, self.y * scalar, self.z * scalar)
    }
}

// ============= Particle, Spring, and Triangle Structs =============
pub struct Particle {
    pub position: Vector3,
    pub velocity: Vector3,
    pub force: Vector3,
}

impl Particle {
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self {
            position: Vector3::new(x, y, z),
            velocity: Vector3::new(0.0, 0.0, 0.0),
            force: Vector3::new(0.0, 0.0, 0.0),
        }
    }
}

pub struct EdgeSpring {
    pub i: usize,
    pub j: usize,
    pub rest_length: f32,
    pub edge: bool,
}

impl EdgeSpring {
    pub fn new(i: usize, j: usize, rest_length: f32, edge: bool) -> Self {
        Self {
            i,
            j,
            rest_length,
            edge,
        }
    }
}

pub struct DihedralSpring {
    pub i1: usize,
    pub i2: usize,
    pub i3: usize,
    pub i4: usize,
    pub edge: bool,
}

impl DihedralSpring {
    pub fn new(i1: usize, i2: usize, i3: usize, i4: usize, edge: bool) -> Self {
        Self {
            i1,
            i2,
            i3,
            i4,
            edge,
        }
    }
}

pub struct Triangle {
    pub indices: [usize; 3],
}

impl Triangle {
    pub fn new(i: usize, j: usize, k: usize) -> Self {
        Self { indices: [i, j, k] }
    }
}

// ============= Dihedral Spring Function =============
fn dihedral_spring(p1: &Vector3, p2: &Vector3, p3: &Vector3, p4: &Vector3, k: f32) -> [Vector3; 4] {
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

// ============= Force Calculation =============
fn calc_forces(
    particles: &mut Vec<Particle>,
    edge_springs: &Vec<EdgeSpring>,
    dihedral_springs: &Vec<DihedralSpring>,
    edge_k: f32,
    dihedral_k: f32,
    damping: f32,
    edge_coef: f32,
    gravity_coef: f32,
    vertex_graph_distance: &[Vec<f32>],
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

        // let mul = if spring.edge { edge_coef } else { 1.0 };
        let mul = 1.0;

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
                let graph_dist = vertex_graph_distance[i][j] / 2.0_f32.sqrt();
                if graph_dist < 0.2 {
                    continue;
                }
                let dir = dir.normalize()
                    * (1. / (dir_len * dir_len) * gravity_coef
                        / (particles.len() * particles.len()) as f32)
                    * (graph_dist * graph_dist);

                particles[i].force = particles[i].force + dir;
                particles[j].force = particles[j].force - dir;
            }
        }
    }
}

fn all_pairs_shortest_paths(edges: &[EdgeSpring]) -> Vec<Vec<f32>> {
    // Find the maximum vertex index to determine the graph size
    let max_vertex = edges.iter().map(|s| s.i.max(s.j)).max().unwrap_or(0);

    let n = max_vertex + 1; // Number of vertices

    // Initialize distance matrix with infinity
    let mut dist = vec![vec![f32::INFINITY; n]; n];

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
                if dist[i][k] != f32::INFINITY && dist[k][j] != f32::INFINITY {
                    dist[i][j] = dist[i][j].min(dist[i][k] + dist[k][j]);
                }
            }
        }
    }

    dist
}

// ============= Main Mesh Implementation =============
pub struct Mesh {
    particles: Vec<Particle>,
    edge_springs: Vec<EdgeSpring>,
    dihedral_springs: Vec<DihedralSpring>,
    triangles: Vec<Triangle>,
    size: usize,

    // Simulation constants
    dt: f32,
    edge_spring_constant: f32,
    dihedral_spring_constant: f32,
    damping_coefficient: f32,
    global_damping: f32,
    edge_coef: f32,
    gravity_coef: f32,

    // Buffer for triangle data
    triangle_buffer: Vec<f32>,
    uv_buffer: Vec<f32>,

    vertex_graph_distance: Vec<Vec<f32>>,

    rng: xorshift::Xorshift128,
}

impl Mesh {
    pub fn new() -> Self {
        Self {
            particles: Vec::new(),
            edge_springs: Vec::new(),
            dihedral_springs: Vec::new(),
            triangles: Vec::new(),
            size: 0,

            dt: 0.1,
            edge_spring_constant: 10.0,
            dihedral_spring_constant: 10.0,
            damping_coefficient: 0.5,
            global_damping: 0.5,
            edge_coef: 1.0,
            gravity_coef: 1.0,

            triangle_buffer: Vec::new(),
            uv_buffer: Vec::new(),
            vertex_graph_distance: Vec::new(),

            rng: xorshift::SeedableRng::from_seed([42u64, 137u64].as_slice()),
        }
    }

    pub fn init(&mut self, size: usize) {
        // Clear existing data
        self.particles.clear();
        self.edge_springs.clear();
        self.dihedral_springs.clear();
        self.triangles.clear();
        self.size = size;

        let vertical_size = 1.;

        // Create particles in a grid
        for i in 0..size {
            for j in 0..size {
                let x = i as f32 / (size - 1) as f32;
                let y = j as f32 / (size - 1) as f32 * vertical_size;
                let z = 0.0;
                self.particles.push(Particle::new(x, y, z));
            }
        }

        // Helper function to get index from grid coordinates
        let get_index = |i, j| i * size + j;

        // Create edge springs and triangles
        for i in 0..size - 1 {
            for j in 0..size - 1 {
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
                if i == size - 2 {
                    let rest_length_bottom = (p11 - p10).length();
                    self.edge_springs.push(EdgeSpring::new(
                        idx10,
                        idx11,
                        rest_length_bottom,
                        false,
                    ));
                }

                if j == size - 2 {
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
                    self.uv_buffer[index] = particle.position.x;
                    self.uv_buffer[index + 1] = particle.position.y / vertical_size;
                    index += 2;
                }
            }
        }

        // Create dihedral springs
        let mut edge_to_triangles: std::collections::HashMap<String, Vec<(usize, usize)>> =
            std::collections::HashMap::new();

        // Helper function to create edge key
        let get_edge_key = |i: usize, j: usize| -> String {
            if i < j {
                format!("{}-{}", i, j)
            } else {
                format!("{}-{}", j, i)
            }
        };

        // Register triangle edges
        for t in 0..self.triangles.len() {
            let triangle = &self.triangles[t];

            for e in 0..3 {
                let i = triangle.indices[e];
                let j = triangle.indices[(e + 1) % 3];
                let edge_key = get_edge_key(i, j);

                let k = triangle.indices[(e + 2) % 3];

                edge_to_triangles
                    .entry(edge_key)
                    .or_insert_with(Vec::new)
                    .push((t, k));
            }
        }

        // Create dihedral springs
        for (edge_key, entries) in edge_to_triangles {
            if entries.len() == 2 {
                let edge_parts: Vec<&str> = edge_key.split('-').collect();
                let i: usize = edge_parts[0].parse().unwrap();
                let j: usize = edge_parts[1].parse().unwrap();

                self.dihedral_springs.push(DihedralSpring::new(
                    i,
                    j,
                    entries[0].1,
                    entries[1].1,
                    false,
                ));
            }
        }

        // Modify Z coordinates of first row
        for particle in &mut self.particles {
            use xorshift::Rng;
            particle.position.z += self.rng.gen_range(-0.001, 0.001);
        }

        // Add special springs
        if true {
            let blue_x = (size as f32 * 0.2) as usize;
            let orange_x = (size as f32 * 0.8) as usize;
            let start_y = (size as f32 * 0.4) as usize;
            let end_y = (size as f32 * 0.6) as usize;

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
                    true,
                ));

                self.dihedral_springs.push(DihedralSpring::new(
                    get_index(blue_x, i),
                    get_index(blue_x, i + 1),
                    get_index(blue_x - 1, i + 1),
                    get_index(orange_x + 1, i),
                    true,
                ));
            }
        }

        if false {
            for i in 0..size {
                // cylinder
                self.edge_springs.push(EdgeSpring::new(
                    get_index(0, i),
                    get_index(size - 1, i),
                    0.0,
                    true,
                ));

                // cylinder + this = torus
                // self.edge_springs.push(EdgeSpring::new(
                //     get_index(i, 0),
                //     get_index(i, size-1),
                //     0.0,
                //     true
                // ));

                // mobius strip
                // self.edge_springs.push(EdgeSpring::new(
                //     get_index(0, i),
                //     get_index(size-1, size-1 - i),
                //     0.0,
                //     true
                // ));
            }

            for i in 0..size - 1 {
                // cylinder
                self.dihedral_springs.push(DihedralSpring::new(
                    get_index(0, i),
                    get_index(0, i + 1),
                    get_index(1, i + 1),
                    get_index(size - 2, i),
                    true,
                ));

                // cylinder + this = torus
                // self.dihedral_springs.push(DihedralSpring::new(
                //     get_index(i, 0),
                //     get_index(i+1, 0),
                //     get_index(i+1, 1),
                //     get_index(i, size-2),
                //     true
                // ));

                // mobius strip
                // self.dihedral_springs.push(DihedralSpring::new(
                //     get_index(0, size-1 - i),
                //     get_index(0, size-1 - (i+1)),
                //     get_index(1, size-1 - (i+1)),
                //     get_index(size-2, size-1 - i),
                //     true
                // ));
            }
        }

        // Update vertex graph distances
        self.vertex_graph_distance = all_pairs_shortest_paths(&self.edge_springs);

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
        avg = avg * (1.0 / self.particles.len() as f32);

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
                self.triangle_buffer[index] = pos.x;
                self.triangle_buffer[index + 1] = pos.y;
                self.triangle_buffer[index + 2] = pos.z;
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
    pub fn get_constants(&self) -> (f32, f32, f32, f32, f32, f32, f32) {
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
            "dt" => self.dt = value,
            "edgeSpringConstant" => self.edge_spring_constant = value,
            "dihedralSpringConstant" => self.dihedral_spring_constant = value,
            "dampingCoefficient" => self.damping_coefficient = value,
            "globalDamping" => self.global_damping = value,
            "edgeCoef" => self.edge_coef = value,
            "gravityCoef" => self.gravity_coef = value,
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

// ============= WASM Interface =============
#[wasm_bindgen]
pub struct MeshHandle {
    mesh: Mesh,
}

#[wasm_bindgen]
impl MeshHandle {
    #[wasm_bindgen(constructor)]
    pub fn new(size: usize) -> Self {
        let mut mesh = Mesh::new();
        mesh.init(size);
        mesh.step();
        Self { mesh }
    }

    pub fn step(&mut self) {
        self.mesh.step()
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test() {
        let mut mesh = Mesh::new();
        mesh.init(10);
        mesh.step();
        mesh.step();
        mesh.step();
        mesh.step();
        mesh.get_constants();
    }
}
