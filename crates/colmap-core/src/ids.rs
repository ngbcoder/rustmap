//! Strongly typed ID newtypes.

use serde::{Deserialize, Serialize};
use std::fmt;

macro_rules! id_newtype {
    ($name:ident, $inner:ty, $doc:expr) => {
        #[doc = $doc]
        #[derive(
            Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize,
        )]
        #[serde(transparent)]
        pub struct $name(pub $inner);

        impl $name {
            /// Wrap a raw value.
            pub const fn new(v: $inner) -> Self {
                Self(v)
            }
            /// Underlying raw value.
            pub const fn get(self) -> $inner {
                self.0
            }
        }

        impl fmt::Display for $name {
            fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                write!(f, "{}", self.0)
            }
        }

        impl From<$inner> for $name {
            fn from(v: $inner) -> Self {
                Self(v)
            }
        }
        impl From<$name> for $inner {
            fn from(v: $name) -> Self {
                v.0
            }
        }
    };
}

id_newtype!(
    CameraId,
    u32,
    "Identifier for a camera (matches COLMAP `camera_id`)."
);
id_newtype!(ImageId, u32, "Identifier for an image.");
id_newtype!(Point3DId, u64, "Identifier for a 3D point.");
id_newtype!(Point2DIdx, u32, "Index into an image's 2D point list.");
id_newtype!(RigId, u32, "Identifier for a rig (stub).");
id_newtype!(SensorId, u32, "Identifier for a sensor (stub).");

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trip_serde() {
        let id = CameraId::new(42);
        let s = serde_json::to_string(&id).unwrap();
        assert_eq!(s, "42");
        let back: CameraId = serde_json::from_str(&s).unwrap();
        assert_eq!(back, id);
    }

    #[test]
    fn ordering() {
        assert!(CameraId::new(1) < CameraId::new(2));
    }
}
