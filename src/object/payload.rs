//! Arbitrary payload for physics objects

use alloc::sync::{Arc, Weak};
use core::any::Any;
use spin::RwLock;

/// No payload value
pub const NO_PAYLOAD: usize = usize::MAX;

/// Optionnal shared pointer with lock over an arbitrary payload
pub type OptPayload = Option<Arc<RwLock<dyn Payload>>>;

/// Arbitrary payload that can be stored in a physics object
pub trait Payload {
    /// Identify the type of payload
    /// usize::MAX is reserved for the absence of payload
    fn payload_type(&self) -> usize;

    /// Cast the payload into the appropriate type
    fn as_any(&self) -> &dyn Any;

    /// Cast the payload into the appropriate type
    fn as_any_mut(&mut self) -> &mut dyn Any;
}

/// Payload stored
#[derive(Default, Clone)]
pub enum SharedPayload {
    /// No payload defined
    #[default]
    None,

    /// Payload is owned by the physics object
    Owned(Arc<RwLock<dyn Payload>>),

    /// Payload is stored as a weak pointer to avoid cyclic reference counting
    Weak(Weak<RwLock<dyn Payload>>),
}

#[derive(Default, Clone)]
pub enum WeakPayload {
    /// No payload defined
    #[default]
    None,

    /// Payload is stored as a weak pointer to avoid cyclic reference counting
    Some(Weak<RwLock<dyn Payload>>),
}

impl SharedPayload {
    /// Wrap the payload into an shared pointer
    #[inline]
    pub fn new<P>(payload: P) -> Self
    where
        P: Payload + 'static,
    {
        Self::Owned(Arc::new(RwLock::new(payload)))
    }

    /// Get the type of payload stored
    pub fn payload_type(&self) -> usize {
        match self {
            Self::None => NO_PAYLOAD,
            Self::Owned(ptr) => ptr.read().payload_type(),
            Self::Weak(ptr) => ptr
                .upgrade()
                .map_or(NO_PAYLOAD, |ptr| ptr.read().payload_type()),
        }
    }

    /// Get the payload
    pub fn get(&self) -> OptPayload {
        match self {
            Self::None => None,
            Self::Owned(ptr) => Some(ptr.clone()),
            Self::Weak(ptr) => ptr.upgrade(),
        }
    }
}

impl WeakPayload {
    /// New weak payload
    #[inline]
    pub fn new(ptr: Weak<RwLock<dyn Payload>>) -> Self {
        Self::Some(ptr)
    }

    /// Get the type of payload stored
    pub fn payload_type(&self) -> usize {
        match self {
            Self::None => NO_PAYLOAD,
            Self::Some(ptr) => ptr
                .upgrade()
                .map_or(NO_PAYLOAD, |ptr| ptr.read().payload_type()),
        }
    }

    /// Get the payload
    pub fn get(&self) -> OptPayload {
        match self {
            Self::None => None,
            Self::Some(ptr) => ptr.upgrade(),
        }
    }
}

impl From<SharedPayload> for WeakPayload {
    fn from(value: SharedPayload) -> Self {
        match value {
            SharedPayload::None => Self::None,
            SharedPayload::Owned(ptr) => Self::Some(Arc::downgrade(&ptr)),
            SharedPayload::Weak(ptr) => Self::Some(ptr.clone()),
        }
    }
}
