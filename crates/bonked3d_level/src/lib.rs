#![cfg_attr(not(feature = "std"), no_std)]

/// Level data layout
pub mod level;

/// Convert mesh into a level component
pub mod mesh;

/// Use alloc crate for no_std support
extern crate alloc;

#[cfg(feature = "parry-f32")]
pub extern crate parry3d as parry;

#[cfg(feature = "parry-f64")]
pub extern crate parry3d_f64 as parry;
