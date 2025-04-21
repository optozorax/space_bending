use std::collections::BTreeSet;
use std::collections::VecDeque;
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
}

impl EdgeSpring {
    pub fn new(i: usize, j: usize, rest_length: fxx) -> Self {
        Self { i, j, rest_length }
    }
}

#[derive(Clone, Debug)]
pub struct DihedralSpring {
    pub i1: usize, // First edge point
    pub i2: usize, // Second edge point
    pub i3: usize, // First wing point
    pub i4: usize, // Second wing point
    pub force: fxx,
}

impl DihedralSpring {
    pub fn new_full(i1: usize, i2: usize, i3: usize, i4: usize, force: fxx) -> Self {
        Self {
            i1,
            i2,
            i3,
            i4,
            force,
        }
    }

    pub fn new(kind: usize, i1: usize, i2: usize, i3: usize, i4: usize) -> Self {
        Self::new_full(i1, i2, i3, i4, 1.0 / 4.0_f64.powi(kind as i32 - 1))
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
        let spring_force = edge_k * (current_length - spring.rest_length);
        let damping_force = damping * relative_velocity;
        let total_force = spring_force + damping_force;

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
}

// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------

struct SpaceParticle {
    left: Option<usize>,
    right: Option<usize>,
    down: Option<usize>,
    up: Option<usize>,

    uv: BTreeSet<(usize, usize)>,

    disabled: bool,
}

struct SpaceGraph {
    graph: Vec<SpaceParticle>,
    sizex: usize,
    sizey: usize,
}

impl SpaceGraph {
    fn new(sizex: usize, sizey: usize) -> Self {
        let mut result = Self {
            graph: Default::default(),
            sizex,
            sizey,
        };
        for i in 0..sizex {
            for j in 0..sizey {
                result.graph.push(SpaceParticle {
                    left: if i == 0 {
                        None
                    } else {
                        Some(result.get_index(i - 1, j))
                    },
                    right: if i == sizex - 1 {
                        None
                    } else {
                        Some(result.get_index(i + 1, j))
                    },
                    down: if j == 0 {
                        None
                    } else {
                        Some(result.get_index(i, j - 1))
                    },
                    up: if j == sizey - 1 {
                        None
                    } else {
                        Some(result.get_index(i, j + 1))
                    },
                    uv: vec![(i, j)].into_iter().collect(),
                    disabled: false,
                });
            }
        }
        result
    }

    fn get_index(&self, i: usize, j: usize) -> usize {
        i * self.sizey + j
    }

    fn make_portal1_scene(&mut self, identify_points: &mut Vec<(usize, usize)>) {
        let blue_x = (self.sizex as f32 * 0.2) as usize;
        let orange_x = (self.sizex as f32 * 0.8) as usize;
        let start_y = (self.sizey as f32 * 0.4) as usize;
        let end_y = (self.sizey as f32 * 0.6) as usize;

        let sizey = self.sizey;
        let get_index = |i, j| i * sizey + j;

        // -------------------------------------------------------------------

        // Disconnect singularity points from up and down

        self.graph[get_index(blue_x, start_y)].down = None;
        self.graph[get_index(blue_x, start_y - 1)].up = None;

        self.graph[get_index(blue_x, end_y)].up = None;
        self.graph[get_index(blue_x, end_y + 1)].down = None;

        self.graph[get_index(orange_x, start_y)].down = None;
        self.graph[get_index(orange_x, start_y - 1)].up = None;

        self.graph[get_index(orange_x, end_y)].up = None;
        self.graph[get_index(orange_x, end_y + 1)].down = None;

        identify_points.push((get_index(blue_x, start_y), get_index(orange_x, start_y)));
        identify_points.push((get_index(blue_x, end_y), get_index(orange_x, end_y)));

        // -------------------------------------------------------------------

        // Connect the main vertical surface

        for j in start_y..=end_y {
            let m1 = get_index(blue_x, j);
            let r1 = get_index(blue_x + 1, j);
            let m2 = get_index(orange_x, j);
            let r2 = get_index(orange_x + 1, j);

            self.graph[m1].right = Some(r2);
            self.graph[r2].left = Some(m1);

            self.graph[m2].right = Some(r1);
            self.graph[r1].left = Some(m2);

            let new_uv = self.graph[m2].uv.clone();
            self.graph[m1].uv.extend(new_uv);

            let new_uv = self.graph[m1].uv.clone();
            self.graph[m2].uv.extend(new_uv);
        }
    }

    fn make_negative_portal_scene(&mut self, identify_points: &mut Vec<(usize, usize)>) {
        let blue_x = (self.sizex as f32 * 0.2) as usize;
        let orange_x = (self.sizex as f32 * 0.8) as usize;
        let start_y = (self.sizey as f32 * (0.6)) as usize;
        let end_y = (self.sizey as f32 * (0.6+(1.-0.2))) as usize;

        let sizey = self.sizey;
        let get_index = |i, j| (i % self.sizex) * sizey + (j % self.sizey);

        // -------------------------------------------------------------------

        // Disconnect singularity points from up and down

        self.graph[get_index(blue_x, start_y)].down = None;
        self.graph[get_index(blue_x, start_y - 1)].up = None;

        self.graph[get_index(blue_x, end_y)].up = None;
        self.graph[get_index(blue_x, end_y + 1)].down = None;

        self.graph[get_index(orange_x, start_y)].down = None;
        self.graph[get_index(orange_x, start_y - 1)].up = None;

        self.graph[get_index(orange_x, end_y)].up = None;
        self.graph[get_index(orange_x, end_y + 1)].down = None;

        identify_points.push((get_index(blue_x, start_y), get_index(orange_x, start_y)));
        identify_points.push((get_index(blue_x, end_y), get_index(orange_x, end_y)));

        // -------------------------------------------------------------------

        // Connect the main vertical surface

        for j in start_y..=end_y {
            let m1 = get_index(blue_x, j);
            let r1 = get_index(blue_x + 1, j);
            let m2 = get_index(orange_x, j);
            let r2 = get_index(orange_x + 1, j);

            self.graph[m1].right = Some(r2);
            self.graph[r2].left = Some(m1);

            self.graph[m2].right = Some(r1);
            self.graph[r1].left = Some(m2);

            let new_uv = self.graph[m2].uv.clone();
            self.graph[m1].uv.extend(new_uv);

            let new_uv = self.graph[m1].uv.clone();
            self.graph[m2].uv.extend(new_uv);
        }
    }

    fn make_portal2_scene(&mut self, identify_points: &mut Vec<(usize, usize)>) {
        let mut blue_x = (self.sizex as f32 * 0.2) as usize;
        let mut orange_x = (self.sizex as f32 * 0.8) as usize;
        let mut start_y = (self.sizey as f32 * 0.4) as usize;
        let mut end_y = (self.sizey as f32 * 0.6) as usize;

        let sizey = self.sizey;
        let get_index = |i, j| i * sizey + j; // I'm not using method because of stupid borrowing issues, why rust can't handle that

        // -------------------------------------------------------------------

        // Disconnect singularity points from up and down

        self.graph[get_index(blue_x, start_y)].down = None;
        self.graph[get_index(blue_x, start_y - 1)].up = None;

        self.graph[get_index(blue_x, end_y)].up = None;
        self.graph[get_index(blue_x, end_y + 1)].down = None;

        self.graph[get_index(orange_x, start_y)].down = None;
        self.graph[get_index(orange_x, start_y - 1)].up = None;

        self.graph[get_index(orange_x, end_y)].up = None;
        self.graph[get_index(orange_x, end_y + 1)].down = None;

        identify_points.push((get_index(blue_x, start_y), get_index(orange_x, start_y)));
        identify_points.push((get_index(blue_x, end_y), get_index(orange_x, end_y)));

        // -------------------------------------------------------------------

        // Connect upper and lower portal surface

        for _ in 0..2 {
            self.graph[get_index(blue_x, start_y)].up = Some(get_index(orange_x, start_y + 1));
            self.graph[get_index(orange_x, start_y + 1)].down = Some(get_index(blue_x, start_y));

            self.graph[get_index(orange_x, start_y)].up = Some(get_index(blue_x, start_y + 1));
            self.graph[get_index(blue_x, start_y + 1)].down = Some(get_index(orange_x, start_y));

            let new_uv = self.graph[get_index(blue_x, start_y)].uv.clone();
            self.graph[get_index(orange_x, start_y)].uv.extend(new_uv);

            let new_uv = self.graph[get_index(orange_x, start_y)].uv.clone();
            self.graph[get_index(blue_x, start_y)].uv.extend(new_uv);

            // ---------------------------------------------------------------

            self.graph[get_index(blue_x, end_y)].down = Some(get_index(orange_x, end_y - 1));
            self.graph[get_index(orange_x, end_y - 1)].up = Some(get_index(blue_x, end_y));

            self.graph[get_index(orange_x, end_y)].down = Some(get_index(blue_x, end_y - 1));
            self.graph[get_index(blue_x, end_y - 1)].up = Some(get_index(orange_x, end_y));

            let new_uv = self.graph[get_index(blue_x, end_y)].uv.clone();
            self.graph[get_index(orange_x, end_y)].uv.extend(new_uv);

            let new_uv = self.graph[get_index(orange_x, end_y)].uv.clone();
            self.graph[get_index(blue_x, end_y)].uv.extend(new_uv);

            blue_x -= 1;
            orange_x -= 1;
        }
        blue_x += 2;
        orange_x += 2;

        // -------------------------------------------------------------------

        // Connect singularity points from left to right (this is ok) in lower part

        let m1 = get_index(blue_x, start_y);
        let r1 = get_index(blue_x + 1, start_y);
        let m2 = get_index(orange_x, start_y);
        let r2 = get_index(orange_x + 1, start_y);

        self.graph[m1].right = Some(r2);
        self.graph[r2].left = Some(m1);

        self.graph[m2].right = Some(r1);
        self.graph[r1].left = Some(m2);

        let new_uv = self.graph[m2].uv.clone();
        self.graph[m1].uv.extend(new_uv);

        let new_uv = self.graph[m1].uv.clone();
        self.graph[m2].uv.extend(new_uv);

        let m1 = get_index(blue_x, end_y);
        let r1 = get_index(blue_x + 1, end_y);
        let m2 = get_index(orange_x, end_y);
        let r2 = get_index(orange_x + 1, end_y);

        self.graph[m1].right = Some(r2);
        self.graph[r2].left = Some(m1);

        self.graph[m2].right = Some(r1);
        self.graph[r1].left = Some(m2);

        let new_uv = self.graph[m2].uv.clone();
        self.graph[m1].uv.extend(new_uv);

        let new_uv = self.graph[m1].uv.clone();
        self.graph[m2].uv.extend(new_uv);

        // -------------------------------------------------------------------

        blue_x -= 2;
        orange_x -= 2;

        // -------------------------------------------------------------------

        // Add uv coordinates to singularity points

        let m1 = get_index(blue_x, start_y);
        let m2 = get_index(orange_x, start_y);
        let new_uv = self.graph[m2].uv.clone();
        self.graph[m1].uv.extend(new_uv);

        let new_uv = self.graph[m1].uv.clone();
        self.graph[m2].uv.extend(new_uv);

        let m1 = get_index(blue_x, end_y);
        let m2 = get_index(orange_x, end_y);
        let new_uv = self.graph[m2].uv.clone();
        self.graph[m1].uv.extend(new_uv);

        let new_uv = self.graph[m1].uv.clone();
        self.graph[m2].uv.extend(new_uv);

        // -------------------------------------------------------------------

        // Connect the main vertical surface

        start_y += 1;
        end_y -= 1;
        for j in start_y..=end_y {
            let m1 = get_index(blue_x, j);
            let r1 = get_index(blue_x + 1, j);
            let m2 = get_index(orange_x, j);
            let r2 = get_index(orange_x + 1, j);

            self.graph[m1].right = Some(r2);
            self.graph[r2].left = Some(m1);

            self.graph[m2].right = Some(r1);
            self.graph[r1].left = Some(m2);

            let new_uv = self.graph[m2].uv.clone();
            self.graph[m1].uv.extend(new_uv);

            let new_uv = self.graph[m1].uv.clone();
            self.graph[m2].uv.extend(new_uv);
        }
    }
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

    // Buffer for triangle data
    triangle_buffer: Vec<f32>,
    uv_buffer: Vec<f32>,

    identify_points: Vec<(usize, usize)>,
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

            triangle_buffer: Vec::new(),
            uv_buffer: Vec::new(),
            identify_points: Vec::new(),
        }
    }

    pub fn init(&mut self, sizex: usize, sizey: usize, scene: &str) {
        // Clear existing data
        self.particles.clear();
        self.edge_springs.clear();
        self.dihedral_springs.clear();
        self.triangles.clear();
        self.sizex = sizex;
        self.sizey = sizey;

        let sizea = sizex.min(sizey);

        let regular_len = 1. / sizea as fxx;
        let diagonal_len = regular_len * (2.0 as fxx).sqrt();

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
        let get_index = |i, j| i * sizey + j;
        let get_index_inv = |i, j| get_index(j, i);
        // let get_coords = |idx: usize| (idx / sizey, idx % sizey);

        let mut space_graph = SpaceGraph::new(sizex, sizey);

        if scene == "cylinder" || scene == "torus" {
            for j in 0..sizey {
                space_graph.graph[get_index_inv(j, 0)].left = Some(get_index_inv(j, sizex - 2));
                space_graph.graph[get_index_inv(j, sizex - 2)].right = Some(get_index_inv(j, 0));

                let new_uv = space_graph.graph[get_index_inv(j, sizex - 1)].uv.clone();
                space_graph.graph[get_index_inv(j, 0)].uv.extend(new_uv);
                space_graph.graph[get_index_inv(j, sizex - 1)].left = None;
            }
        }
        if scene == "torus" {
            for i in 0..sizex {
                space_graph.graph[get_index(i, 0)].down = Some(get_index(i, sizey - 2));
                space_graph.graph[get_index(i, sizey - 2)].up = Some(get_index(i, 0));

                let new_uv = space_graph.graph[get_index(i, sizey - 1)].uv.clone();
                space_graph.graph[get_index(i, 0)].uv.extend(new_uv);
                space_graph.graph[get_index(i, sizey - 1)].down = None;
            }
        }
        if scene == "genus_2" {
            let ratiox = 3.;
            let ratioy = 3.;
            let sizexr = (sizex as f32 / ratiox) as usize;
            let sizeyr = (sizey as f32 / ratioy) as usize;
            // A
            for i in 0..sizexr {
                space_graph.graph[get_index(i, 0)].down = Some(get_index(i, sizey - 2));
                space_graph.graph[get_index(i, sizey - 2)].up = Some(get_index(i, 0));

                let new_uv = space_graph.graph[get_index(i, sizey - 1)].uv.clone();
                space_graph.graph[get_index(i, 0)].uv.extend(new_uv);
                space_graph.graph[get_index(i, sizey - 1)].down = None;
            }
            // B
            for i in sizexr..sizex {
                space_graph.graph[get_index(i, 0)].down = Some(get_index(i, sizeyr - 2));
                space_graph.graph[get_index(i, sizeyr - 2)].up = Some(get_index(i, 0));

                let new_uv = space_graph.graph[get_index(i, sizeyr - 1)].uv.clone();
                space_graph.graph[get_index(i, 0)].uv.extend(new_uv);
                space_graph.graph[get_index(i, sizeyr - 1)].down = None;
            }
            // C
            for j in 0..sizeyr {
                space_graph.graph[get_index_inv(j, 0)].left = Some(get_index_inv(j, sizex - 2));
                space_graph.graph[get_index_inv(j, sizex - 2)].right = Some(get_index_inv(j, 0));

                let new_uv = space_graph.graph[get_index_inv(j, sizex - 1)].uv.clone();
                space_graph.graph[get_index_inv(j, 0)].uv.extend(new_uv);
                space_graph.graph[get_index_inv(j, sizex - 1)].left = None;
            }
            // D
            for j in sizeyr..sizey {
                space_graph.graph[get_index_inv(j, 0)].left = Some(get_index_inv(j, sizexr - 2));
                space_graph.graph[get_index_inv(j, sizexr - 2)].right = Some(get_index_inv(j, 0));

                let new_uv = space_graph.graph[get_index_inv(j, sizexr - 1)].uv.clone();
                space_graph.graph[get_index_inv(j, 0)].uv.extend(new_uv);
                space_graph.graph[get_index_inv(j, sizexr - 1)].left = None;
            }

            for i in sizexr..sizex {
                for j in sizeyr..sizey {
                    space_graph.graph[get_index(i, j)].disabled = true;
                    space_graph.graph[get_index(i, j)].up = None;
                    space_graph.graph[get_index(i, j)].down = None;
                    space_graph.graph[get_index(i, j)].left = None;
                    space_graph.graph[get_index(i, j)].right = None;
                }
            }
        }
        if scene == "mobius_strip" {
            for i in 0..sizex {
                space_graph.graph[get_index(i, 0)].down = Some(get_index(sizex - 1 - i, sizey - 2));
                space_graph.graph[get_index(sizex - 1 - i, sizey - 2)].up = Some(get_index(i, 0));

                let new_uv = space_graph.graph[get_index(sizex - 1 - i, sizey - 1)]
                    .uv
                    .clone();
                space_graph.graph[get_index(i, 0)].uv.extend(new_uv);
                space_graph.graph[get_index(i, sizey - 1)].down = None;
            }
        }
        if scene == "portal1" {
            space_graph.make_portal1_scene(&mut self.identify_points);
        }
        if scene == "portal2" {
            space_graph.make_portal2_scene(&mut self.identify_points);
        }
        if scene == "negative_portal" {
            space_graph.make_negative_portal_scene(&mut self.identify_points);
        }

        macro_rules! trying {
            ($($body:stmt);* $(;)?) => {
                (|| -> Option<()> {
                    $($body)*
                    Some(())
                })();
            };
        }

        macro_rules! trying_with_else {
            // two–arm version: {body} else {on_none}
            ({ $($body:tt)* } else { $($on_none:tt)* }) => {{
                let __res = (|| -> Option<()> {
                    $($body)*
                    Some(())
                })();

                if __res.is_none() {
                    (|| -> Option<()> {
                        $($on_none)*
                        Some(())
                    })();
                }
            }};
        }

        for idx in 0..self.particles.len() {
            macro_rules! process_move {
                ($temp:expr, R) => {
                    space_graph.graph[$temp].right?
                };
                ($temp:expr, U) => {
                    space_graph.graph[$temp].up?
                };
                ($temp:expr, D) => {
                    space_graph.graph[$temp].down?
                };
                ($temp:expr, L) => {
                    space_graph.graph[$temp].left?
                };
            }

            macro_rules! mv {
                () => { idx };
                ($($moves:tt)*) => {{
                    let mut temp = idx;
                    $(
                        temp = process_move!(temp, $moves);
                    )*
                    temp
                }};
            }

            macro_rules! edge_spring {
                ($($data:tt)*) => {
                    trying! { self.edge_springs.push(EdgeSpring::new($($data)*)); }
                };
            }

            macro_rules! dihedral_spring {
                ($($data:tt)*) => {
                    trying! { self.dihedral_springs.push(DihedralSpring::new($($data)*)); }
                };
            }

            // Add springs
            edge_spring!(idx, mv!(R), regular_len);
            edge_spring!(idx, mv!(U), regular_len);
            edge_spring!(idx, mv!(R U), diagonal_len);
            edge_spring!(mv!(R), mv!(U), diagonal_len);

            // Add dihedral springs of distance 1
            dihedral_spring!(1, idx, mv!(U), mv!(R), mv!(U L));
            dihedral_spring!(1, idx, mv!(U), mv!(L), mv!(U R));
            dihedral_spring!(1, idx, mv!(R), mv!(D), mv!(R U));
            dihedral_spring!(1, idx, mv!(R), mv!(U), mv!(R D));
            dihedral_spring!(1, idx, mv!(R U), mv!(R), mv!(U));
            dihedral_spring!(1, mv!(R), mv!(U), idx, mv!(R U));

            // Add dihedral springs of distance 2
            dihedral_spring!(2, idx, mv!(U), mv!(R R), mv!(U L L));
            dihedral_spring!(2, idx, mv!(U), mv!(L L), mv!(U R R));
            dihedral_spring!(2, idx, mv!(R), mv!(D D), mv!(R U U));
            dihedral_spring!(2, idx, mv!(R), mv!(U U), mv!(R D D));
            dihedral_spring!(2, idx, mv!(R U), mv!(R R D), mv!(U U L));
            dihedral_spring!(2, mv!(R), mv!(U), mv!(L D), mv!(R U R U));

            // Add dihedral springs of distance 3
            dihedral_spring!(3, idx, mv!(U), mv!(R R R), mv!(U L L L));
            dihedral_spring!(3, idx, mv!(U), mv!(L L L), mv!(U R R R));
            dihedral_spring!(3, idx, mv!(R), mv!(D D D), mv!(R U U U));
            dihedral_spring!(3, idx, mv!(R), mv!(U U U), mv!(R D D D));
            dihedral_spring!(3, idx, mv!(R U), mv!(R R R D D), mv!(U U U L L));
            dihedral_spring!(3, mv!(R), mv!(U), mv!(L D L D), mv!(R U R U R U));

            // // Add dihedral springs of distance 4
            // dihedral_spring!(4, idx, mv!(U), mv!(R R R R), mv!(U L L L L));
            // dihedral_spring!(4, idx, mv!(U), mv!(L L L L), mv!(U R R R R));
            // dihedral_spring!(4, idx, mv!(R), mv!(D D D D), mv!(R U U U U));
            // dihedral_spring!(4, idx, mv!(R), mv!(U U U U), mv!(R D D D D));
            // dihedral_spring!(4, idx, mv!(R U), mv!(R R R R D D D), mv!(U U U U L L L));
            // dihedral_spring!(4, mv!(R), mv!(U), mv!(L D L D L D), mv!(R U R U R U R U));

            // // Add dihedral springs of distance 5
            // dihedral_spring!(5, idx, mv!(U), mv!(R R R R R), mv!(U L L L L L));
            // dihedral_spring!(5, idx, mv!(U), mv!(L L L L L), mv!(U R R R R R));
            // dihedral_spring!(5, idx, mv!(R), mv!(D D D D D), mv!(R U U U U U));
            // dihedral_spring!(5, idx, mv!(R), mv!(U U U U U), mv!(R D D D D D));
            // dihedral_spring!(5, idx, mv!(R U), mv!(R R R R R D D D D), mv!(U U U U U L L L L));
            // dihedral_spring!(5, mv!(R), mv!(U), mv!(L D L D L D L D), mv!(R U R U R U R U R U));

            // // Add dihedral springs of distance 6
            // dihedral_spring!(6, idx, mv!(U), mv!(R R R R R R), mv!(U L L L L L L));
            // dihedral_spring!(6, idx, mv!(U), mv!(L L L L L L), mv!(U R R R R R R));
            // dihedral_spring!(6, idx, mv!(R), mv!(D D D D D D), mv!(R U U U U U U));
            // dihedral_spring!(6, idx, mv!(R), mv!(U U U U U U), mv!(R D D D D D D));
            // dihedral_spring!(6, idx, mv!(R U), mv!(R R R R R R D D D D D), mv!(U U U U U U L L L L L));
            // dihedral_spring!(6, mv!(R), mv!(U), mv!(L D L D L D L D L D), mv!(R U R U R U R U R U R U));

            // Add triangles for drawing
            trying_with_else! {{
                if idx == mv!(U R D L) && mv!(U R D L) == mv!(R U L D) {
                    self.triangles.push(Triangle::new(idx, mv!(R), mv!(U)));
                } else {
                    // eprintln!("TR_RU, idx: {:?}, URDL: {:?}, RULD: {:?}", get_coords(idx), get_coords(mv!(U R D L)), get_coords(mv!(R U L D)));
                    // eprintln!("U: {:?}, UR: {:?}, URD: {:?}, URDL: {:?}", get_coords(mv!(U)), get_coords(mv!(U R)), get_coords(mv!(U R D)), get_coords(mv!(U R D L)));
                }
            } else {
                if space_graph.graph[idx].up == None {
                    self.triangles.push(Triangle::new(idx, mv!(R), mv!(R U L)));
                } else {
                    self.triangles.push(Triangle::new(idx, mv!(R), mv!(U)));
                }
            }}
            trying_with_else! {{
                if idx == mv!(L D R U) && mv!(L D R U) == mv!(D L U R) {
                    self.triangles.push(Triangle::new(idx, mv!(L), mv!(D)));
                } else {
                    // eprintln!("TR_LD, idx: {:?}, LDRU: {:?}, DLUR: {:?}", get_coords(idx), get_coords(mv!(L D R U)), get_coords(mv!(D L U R)));
                    // eprintln!("L: {:?}, LD: {:?}, LDR: {:?}, LDRU: {:?}", get_coords(mv!(L)), get_coords(mv!(L D)), get_coords(mv!(L D R)), get_coords(mv!(L D R U)));
                }
            } else {
                if space_graph.graph[idx].down == None {
                    self.triangles.push(Triangle::new(idx, mv!(L), mv!(L D R)));
                } else {
                    self.triangles.push(Triangle::new(idx, mv!(L), mv!(D)));
                }
            }}
        }

        self.edge_springs.retain(|s| !space_graph.graph[s.i].disabled && !space_graph.graph[s.j].disabled);
        self.dihedral_springs.retain(|s| 
            !space_graph.graph[s.i1].disabled && 
            !space_graph.graph[s.i2].disabled &&
            !space_graph.graph[s.i3].disabled &&
            !space_graph.graph[s.i4].disabled
        );
        self.triangles.retain(|t|
            !space_graph.graph[t.indices[0]].disabled && 
            !space_graph.graph[t.indices[1]].disabled && 
            !space_graph.graph[t.indices[2]].disabled
        );

        // Fill uv buffer
        {
            // Size the buffer correctly (3 vertices * 2 UV coords per triangle)
            let buffer_size = self.triangles.len() * 6;
            if self.uv_buffer.len() != buffer_size {
                self.uv_buffer.resize(buffer_size, 0.0);
            }

            // Fill the buffer with UV coordinates based on particle positions
            let mut index = 0;
            let mut push = |uv: &(usize, usize)| {
                self.uv_buffer[index] = uv.0 as f32 / (sizex - 1) as f32;
                self.uv_buffer[index + 1] = uv.1 as f32 / (sizey - 1) as f32;
                index += 2;
            };
            for triangle in self.triangles.iter() {
                let mut found = false;
                'top: for uv1 in &space_graph.graph[triangle.indices[0]].uv {
                    for uv2 in &space_graph.graph[triangle.indices[1]].uv {
                        for uv3 in &space_graph.graph[triangle.indices[2]].uv {
                            // first type of triangle
                            if uv1.0 + 1 == uv2.0
                                && uv1.1 == uv2.1
                                && uv1.1 + 1 == uv3.1
                                && uv1.0 == uv3.0
                            {
                                push(uv1);
                                push(uv2);
                                push(uv3);
                                found = true;
                                break 'top;
                            }

                            // second type of triangle
                            if uv2.0 + 1 == uv1.0
                                && uv2.1 == uv1.1
                                && uv3.1 + 1 == uv1.1
                                && uv3.0 == uv1.0
                            {
                                push(uv1);
                                push(uv2);
                                push(uv3);
                                found = true;
                                break 'top;
                            }
                        }
                    }
                }
                if !found {
                    push(&(0, 0));
                    push(&(0, 0));
                    push(&(0, 0));

                    // dbg!(get_coords(triangle.indices[0]));
                    // dbg!(get_coords(triangle.indices[1]));
                    // dbg!(get_coords(triangle.indices[2]));
                    // dbg!(triangle, &space_graph.graph[triangle.indices[0]].uv, &space_graph.graph[triangle.indices[1]].uv, &space_graph.graph[triangle.indices[2]].uv);
                    // dbg!("-------------------------------------------------");
                    // panic!();
                }
            }
        }

        // Modify Z coordinates so that simulation does not stuck in 2D
        for particle in &mut self.particles {
            let x = particle.position.x - sizex as fxx / (sizea - 1) as fxx / 2.;
            let y = particle.position.y - sizey as fxx / (sizea - 1) as fxx / 2.;
            particle.position.z -= (x * x + y * y).sqrt() * 0.05;
        }

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
                let mut pos = self.particles[idx].position - avg;

                for (first, second) in &self.identify_points {
                    let avg2 = (self.particles[*first].position + self.particles[*second].position) * 0.5;
                    if idx == *first || idx == *second {
                        pos = avg2 - avg;
                    }
                }

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
    pub fn get_constants(&self) -> (fxx, fxx, fxx, fxx, fxx) {
        (
            self.dt,
            self.edge_spring_constant,
            self.dihedral_spring_constant,
            self.damping_coefficient,
            self.global_damping,
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
    pub fn new(sizex: usize, sizey: usize, scene: &str) -> Self {
        let mut mesh = Mesh::new();
        mesh.init(sizex, sizey, scene);
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

    impl SpaceGraph {
        /// Compares if this SpaceGraph is structurally equal to another SpaceGraph
        pub fn is_equal_to(&self, other: &SpaceGraph) -> bool {
            use std::collections::{HashMap, VecDeque};

            // Check if dimensions match
            if self.sizex != other.sizex || self.sizey != other.sizey {
                return false;
            }

            // Handle empty graphs
            if self.graph.is_empty() && other.graph.is_empty() {
                return true;
            }
            if self.graph.is_empty() || other.graph.is_empty() {
                return false;
            }

            // Initialize BFS queue starting with both graphs' node 0
            let mut queue = VecDeque::new();
            queue.push_back((0, 0)); // (self_index, other_index)

            // Keep track of visited nodes and mappings between graph indices
            let mut visited = HashSet::new();
            let mut index_mapping = HashMap::new(); // Maps self indices to other indices

            while let Some((self_idx, other_idx)) = queue.pop_front() {
                // Skip if we've already processed this pair
                if !visited.insert((self_idx, other_idx)) {
                    continue;
                }

                // Store the mapping
                index_mapping.insert(self_idx, other_idx);

                // Get the particles to compare
                let self_particle = &self.graph[self_idx];
                let other_particle = &other.graph[other_idx];

                // Check directional connections
                if !self.check_direction_match(self_particle.left, other_particle.left)
                    || !self.check_direction_match(self_particle.right, other_particle.right)
                    || !self.check_direction_match(self_particle.down, other_particle.down)
                    || !self.check_direction_match(self_particle.up, other_particle.up)
                {
                    return false;
                }

                // Add connected nodes to the queue
                self.add_to_queue_if_connected(&mut queue, self_particle.left, other_particle.left);
                self.add_to_queue_if_connected(&mut queue, self_particle.right, other_particle.right);
                self.add_to_queue_if_connected(&mut queue, self_particle.down, other_particle.down);
                self.add_to_queue_if_connected(&mut queue, self_particle.up, other_particle.up);
            }

            // Ensure all nodes were visited (graph is fully connected)
            index_mapping.len() == self.graph.len() && index_mapping.len() == other.graph.len()
        }

        /// Helper to check if direction connections match in pattern
        fn check_direction_match(&self, self_dir: Option<usize>, other_dir: Option<usize>) -> bool {
            // Both should either have a connection or not have a connection
            self_dir.is_some() == other_dir.is_some()
        }

        /// Helper to add connected nodes to the queue
        fn add_to_queue_if_connected(
            &self,
            queue: &mut VecDeque<(usize, usize)>,
            self_dir: Option<usize>,
            other_dir: Option<usize>,
        ) {
            if let (Some(self_next), Some(other_next)) = (self_dir, other_dir) {
                queue.push_back((self_next, other_next));
            }
        }
    }

    #[test]
    fn basic() {
        let mut mesh = Mesh::new();
        mesh.init(50, 50, "portal2");
        mesh.step();
        mesh.step();
        mesh.step();
        mesh.step();
        mesh.get_constants();
        // panic!();
    }

    #[test]
    fn same_graph() {
        let mut graph1 = SpaceGraph::new(50, 50);
        let mut identify_points = Vec::new();
        graph1.make_portal1_scene(&mut identify_points);

        let mut graph2 = SpaceGraph::new(50, 50);
        graph2.make_portal2_scene(&mut identify_points);

        assert!(graph1.is_equal_to(&graph2));
    }
}
