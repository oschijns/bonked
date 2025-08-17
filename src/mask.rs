//! Define collision masks

// pick the mask size based on feature flags

#[cfg(feature = "mask-u32")]
pub type Mask = u32;

#[cfg(feature = "mask-u64")]
pub type Mask = u64;

/// Mask where all bits are set to 1
pub const MASK_ALL: LayerFilter = LayerFilter::new(Mask::MAX, Mask::MAX);

/// Pair a layer with a filter
#[derive(Debug, Clone, Copy)]
pub struct LayerFilter {
    /// The layer(s) the object exists in
    pub(crate) layer: Mask,

    /// The layer(s) the object will interact with
    pub(crate) filter: Mask,
}

impl LayerFilter {
    /// Create a layer filter
    #[inline]
    pub const fn new(layer: Mask, filter: Mask) -> Self {
        Self { layer, filter }
    }

    /// Check if the two objects can interact with each other
    #[inline]
    pub fn can_interact(self, other: Self) -> bool {
        ((self.layer & other.filter) != 0) && ((self.filter & other.layer) != 0)
    }
}
