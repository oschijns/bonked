//! Encoding and decoding strategy for the mesh type

use crate::level::{Index, LevelPart, List, Real, Vector};
use alloc::vec::Vec;
use bincode::{
    Decode, Encode,
    de::Decoder,
    enc::Encoder,
    error::{DecodeError, EncodeError},
};
use num_traits::ConstZero;

bitfield::bitfield! {
    /// Specify if vertices have some extra data (normal, color, UV)
    struct Mask(u8);
    impl Debug;

    /// Normals are defined
    use_normals, set_normals_use: 0;

    /// Colors are defined
    use_colors, set_colors_use: 1;

    /// UV coordinates are defined
    use_uvs, set_uvs_use: 2;
}

impl Encode for LevelPart {
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        // check the number of vertices do not exceed the supported limit
        let count = self.positions.0.len();
        if count > Index::MAX as usize {
            return Err(EncodeError::Other(
                "Mesh contains too many vertices to be encoded",
            ));
        }

        // compose the mask of extra data available
        let mut mask = Mask(0);

        // check each list of extra vertex data and compose the bitmap
        macro_rules! check {
            ($data:ident, $setter:ident, $error:literal) => {
                if let Some(list) = &self.$data {
                    mask.$setter(true);
                    if list.len() != count {
                        return Err(EncodeError::Other($error));
                    }
                }
            };
        }
        check!(normals, set_normals_use, "Mismatched normals count");
        check!(colors, set_colors_use, "Mismatched colors count");
        check!(uvs, set_uvs_use, "Mismatched UV coordinates count");

        // encode available vertices extra data
        mask.encode(encoder)?;

        // encode the vertices coordinates
        self.positions.encode(encoder)?;

        // encode all the extra vertices data
        macro_rules! encode {
            ($data:ident) => {
                if let Some(list) = &self.$data {
                    encode_list(list, encoder)?;
                }
            };
        }
        encode!(normals);
        encode!(colors);
        encode!(uvs);

        // encode the indicies
        self.indices.encode(encoder)?;
        self.hull_indices.encode(encoder)?;

        Ok(())
    }
}

impl<Ctx> Decode<Ctx> for LevelPart {
    fn decode<D: Decoder<Context = Ctx>>(decoder: &mut D) -> Result<Self, DecodeError> {
        let mask = Mask(Decode::decode(decoder)?);

        // read the vertices coordinates
        let positions: List<Vector<Real, 3>> = Decode::decode(decoder)?;
        let size = positions.0.len();

        // decode the extra data
        macro_rules! decode {
            ($getter:ident) => {
                if mask.$getter() {
                    Some(decode_list(size, decoder)?)
                } else {
                    None
                }
            };
        }
        let normals = decode!(use_normals);
        let colors = decode!(use_colors);
        let uvs = decode!(use_uvs);

        let indices = Decode::decode(decoder)?;
        let hull_indices = Decode::decode(decoder)?;

        Ok(Self {
            positions,
            normals,
            colors,
            uvs,
            indices,
            hull_indices,
        })
    }
}

impl<T: Encode> Encode for List<T> {
    #[inline]
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        (self.0.len() as Index).encode(encoder)?;
        encode_list(&self.0, encoder)
    }
}

impl<T: Decode<Ctx>, Ctx> Decode<Ctx> for List<T> {
    #[inline]
    fn decode<D: Decoder<Context = Ctx>>(decoder: &mut D) -> Result<Self, DecodeError> {
        let size: u16 = Decode::decode(decoder)?;
        Ok(Self(decode_list(size as usize, decoder)?))
    }
}

/// Encode the list without its size
fn encode_list<I: Encode, E: Encoder>(list: &[I], encoder: &mut E) -> Result<(), EncodeError> {
    for item in list {
        item.encode(encoder)?;
    }
    Ok(())
}

/// Decode the list knowing in advance its size
fn decode_list<I: Decode<Ctx>, D: Decoder<Context = Ctx>, Ctx>(
    size: usize,
    decoder: &mut D,
) -> Result<Vec<I>, DecodeError> {
    let mut list = Vec::with_capacity(size);
    for _ in 0..size {
        list.push(Decode::decode(decoder)?);
    }
    Ok(list)
}

impl<N, const DIM: usize> Encode for Vector<N, DIM>
where
    N: Encode,
{
    #[inline]
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        for n in &self.0 {
            n.encode(encoder)?;
        }
        Ok(())
    }
}

impl<N, const DIM: usize, Ctx> Decode<Ctx> for Vector<N, DIM>
where
    N: ConstZero + Decode<Ctx>,
{
    #[inline]
    fn decode<D: Decoder<Context = Ctx>>(decoder: &mut D) -> Result<Self, DecodeError> {
        let mut a = [N::ZERO; DIM];
        for i in a.iter_mut().take(DIM) {
            *i = Decode::decode(decoder)?;
        }
        Ok(Self(a))
    }
}

impl Encode for Mask {
    #[inline]
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        self.0.encode(encoder)
    }
}

impl<Ctx> Decode<Ctx> for Mask {
    #[inline]
    fn decode<D: Decoder<Context = Ctx>>(decoder: &mut D) -> Result<Self, DecodeError> {
        Ok(Self(Decode::decode(decoder)?))
    }
}
