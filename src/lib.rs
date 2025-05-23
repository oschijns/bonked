#![cfg_attr(not(feature = "std"), no_std)]

// cannot use 2D and 3D features at the same time
#[cfg(all(feature = "2d", feature = "3d"))]
compile_error!("The '2d' & '3d' features cannot be used at the same time.");

// cannot use f32 and f64 features at the same time
#[cfg(all(feature = "parry-f32", feature = "parry-f64"))]
compile_error!("The 'parry-f32' & 'parry-f64' features cannot be used at the same time.");

// cannot use u32 and u64 features at the same time
#[cfg(all(feature = "mask-u32", feature = "mask-u64"))]
compile_error!("The 'mask-u32' & 'mask-u64' features cannot be used at the same time.");

/// Define the physics objects
pub mod object;

/// Define the world
pub mod world;

/// Use alloc crate for no_std support
extern crate alloc;

// pick the parry variant based on feature flags

#[cfg(all(feature = "2d", feature = "parry-f32"))]
pub extern crate parry2d as parry;

#[cfg(all(feature = "2d", feature = "parry-f64"))]
pub extern crate parry2d_f64 as parry;

#[cfg(all(feature = "3d", feature = "parry-f32"))]
pub extern crate parry3d as parry;

#[cfg(all(feature = "3d", feature = "parry-f64"))]
pub extern crate parry3d_f64 as parry;

// pick the mask size based on feature flags

#[cfg(feature = "mask-u32")]
pub type Mask = u32;

#[cfg(feature = "mask-u64")]
pub type Mask = u64;

// pick hashset and hashmap based on feature flags

#[cfg(feature = "std")]
pub mod collections {
    pub use std::collections::{HashMap, HashSet};
}

#[cfg(not(feature = "std"))]
pub mod collections {
    pub use hashbrown::{HashMap, HashSet};
}
