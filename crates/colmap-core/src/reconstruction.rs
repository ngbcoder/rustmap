//! Reconstruction data model.

use crate::ids::{CameraId, ImageId, Point2DIdx, Point3DId};
use crate::types::Point3;
use nalgebra::Vector3;
use std::collections::BTreeMap;

/// A 2D point observation in an image.
#[derive(Clone, Debug)]
pub struct Point2D {
    pub xy: [f64; 2],
    pub point3d_id: Option<Point3DId>,
}

/// A 3D point with its track.
#[derive(Clone, Debug)]
pub struct Point3D {
    pub xyz: Point3,
    pub color: [u8; 3],
    pub error: f64,
    /// Track: list of (image_id, point2d_idx).
    pub track: Vec<(ImageId, Point2DIdx)>,
}

/// An image in a reconstruction.
#[derive(Clone, Debug)]
pub struct Image {
    pub image_id: ImageId,
    pub name: String,
    pub camera_id: CameraId,
    pub pose: crate::geometry::Rigid3,
    pub points2d: Vec<Point2D>,
}

/// A reconstruction (arena-style).
#[derive(Clone, Debug, Default)]
pub struct Reconstruction {
    pub cameras: BTreeMap<CameraId, crate::camera::Camera>,
    pub images: BTreeMap<ImageId, Image>,
    pub points3d: BTreeMap<Point3DId, Point3D>,
}

impl Reconstruction {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn add_camera(&mut self, id: CameraId, cam: crate::camera::Camera) {
        self.cameras.insert(id, cam);
    }

    pub fn add_image(&mut self, img: Image) {
        self.images.insert(img.image_id, img);
    }

    pub fn add_point3d(&mut self, id: Point3DId, pt: Point3D) {
        self.points3d.insert(id, pt);
    }

    pub fn num_reg_images(&self) -> usize {
        self.images.len()
    }
    pub fn num_points3d(&self) -> usize {
        self.points3d.len()
    }

    /// Validate internal invariants.
    pub fn validate(&self) -> Result<(), String> {
        // Every image's camera exists.
        for (iid, img) in &self.images {
            if !self.cameras.contains_key(&img.camera_id) {
                return Err(format!(
                    "image {} references missing camera {}",
                    iid, img.camera_id
                ));
            }
            // point3d references valid
            for (idx, p2) in img.points2d.iter().enumerate() {
                if let Some(pid) = p2.point3d_id {
                    match self.points3d.get(&pid) {
                        None => {
                            return Err(format!(
                                "image {} pt {} references missing point3d {}",
                                iid, idx, pid
                            ))
                        }
                        Some(pt) => {
                            if !pt
                                .track
                                .iter()
                                .any(|(iid2, idx2)| *iid2 == *iid && idx2.get() == idx as u32)
                            {
                                return Err(format!(
                                    "point3d {} track missing obs ({} ,{})",
                                    pid, iid, idx
                                ));
                            }
                        }
                    }
                }
            }
        }
        // Every point3d track obs valid
        for (pid, pt) in &self.points3d {
            for (iid, idx) in &pt.track {
                match self.images.get(iid) {
                    None => {
                        return Err(format!("point3d {} references missing image {}", pid, iid))
                    }
                    Some(img) => {
                        if idx.get() as usize >= img.points2d.len() {
                            return Err(format!("point3d {} idx {} out of range", pid, idx));
                        }
                        if img.points2d[idx.get() as usize].point3d_id != Some(*pid) {
                            return Err(format!("point3d {} obs mismatch at image {}", pid, iid));
                        }
                    }
                }
            }
        }
        Ok(())
    }
}

// Helper to update observation: set point2d's point3d id
impl Image {
    pub fn set_point3d_for_point2d(&mut self, idx: usize, pid: Option<Point3DId>) {
        if idx < self.points2d.len() {
            self.points2d[idx].point3d_id = pid;
        }
    }
}

// suppress unused
#[allow(dead_code)]
fn _use_vec3() -> Vector3<f64> {
    Vector3::zeros()
}
#[allow(dead_code)]
fn _use_point3() -> Point3 {
    Point3::zeros()
}
