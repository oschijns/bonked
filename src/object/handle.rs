//! Handle for accessing bodies in the world

use super::{
    kinematic_body::KinematicBody, static_body::StaticBody, trigger_area::TriggerArea, Object,
};
use alloc::sync::Arc;
use parry::partitioning::IndexedData;

/// Handle to a body in the physics world
#[repr(u8)]
#[derive(Clone, Copy, Eq, Default, Hash)]
pub enum Handle {
    /// Invalid handle
    #[default]
    Invalid = 0,

    /// Handle for a trigger zone
    Trigger(*const TriggerArea),

    /// Handle for a fixed body
    Static(*const StaticBody),

    /// Handle for a kinematic body
    Kinematic(*mut KinematicBody),
}

/// Create an handle for a trigger zone
impl From<Arc<TriggerArea>> for Handle {
    fn from(area: Arc<TriggerArea>) -> Self {
        Self::Trigger(Arc::as_ptr(&area))
    }
}

/// Create an handle for a fixed body
impl From<Arc<StaticBody>> for Handle {
    fn from(body: Arc<StaticBody>) -> Self {
        Self::Static(Arc::as_ptr(&body))
    }
}

/// Create an handle for a kinematic body
impl From<Arc<KinematicBody>> for Handle {
    fn from(body: Arc<KinematicBody>) -> Self {
        Self::Kinematic(Arc::as_ptr(&body) as *mut KinematicBody)
    }
}

impl Handle {
    pub fn as_dyn(&self) -> &dyn Object {
        match self {
            Self::Trigger(area) => unsafe { area.as_ref() }.unwrap(),
            Self::Static(body) => unsafe { body.as_ref() }.unwrap(),
            Self::Kinematic(body) => unsafe { body.as_ref() }.unwrap(),
            _ => panic!("Cannot convert invalid handle into dyn Object"),
        }
    }

    #[inline]
    pub fn discriminant(&self) -> u8 {
        unsafe { *<*const _>::from(self).cast::<u8>() }
    }
}

/// Allow the handle to be used as an index in a QBVH
impl IndexedData for Handle {
    /// Create an invalid handle
    #[inline]
    fn default() -> Self {
        Self::Invalid
    }

    /// Get the index of the handle
    #[inline]
    fn index(&self) -> usize {
        match self {
            Self::Invalid => 0,
            Self::Trigger(area) => *area as usize,
            Self::Static(body) => *body as usize,
            Self::Kinematic(body) => *body as usize,
        }
    }
}

/// Perform equality tests based on pointer values
impl PartialEq for Handle {
    fn eq(&self, other: &Self) -> bool {
        let index0 = self.index();
        let index1 = other.index();
        index0 == index1
    }
}
