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

    /// Link an image observation to a 3D point, maintaining the track.
    pub fn add_observation(&mut self, image_id: ImageId, idx: usize, point3d_id: Point3DId) {
        debug_assert!(self.images.contains_key(&image_id));
        debug_assert!(self.points3d.contains_key(&point3d_id));
        if let Some(img) = self.images.get_mut(&image_id) {
            if idx < img.points2d.len() {
                img.points2d[idx].point3d_id = Some(point3d_id);
            }
        }
        if let Some(pt) = self.points3d.get_mut(&point3d_id) {
            if !pt
                .track
                .iter()
                .any(|(i, x)| *i == image_id && x.get() == idx as u32)
            {
                pt.track.push((image_id, Point2DIdx::new(idx as u32)));
            }
        }
    }

    /// Remove the link between an image observation and its 3D point.
    pub fn remove_observation(&mut self, image_id: ImageId, idx: usize) {
        let pid = self.images.get(&image_id).and_then(|img| {
            if idx < img.points2d.len() {
                img.points2d[idx].point3d_id
            } else {
                None
            }
        });
        if let Some(img) = self.images.get_mut(&image_id) {
            if idx < img.points2d.len() {
                img.points2d[idx].point3d_id = None;
            }
        }
        if let Some(pid) = pid {
            if let Some(pt) = self.points3d.get_mut(&pid) {
                pt.track
                    .retain(|(i, x)| !(*i == image_id && x.get() == idx as u32));
                if pt.track.is_empty() {
                    self.points3d.remove(&pid);
                }
            }
        }
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::camera::{Camera, CameraModelKind};
    use crate::ids::{CameraId, ImageId, Point3DId};

    #[test]
    fn track_maintenance() {
        let mut rec = Reconstruction::new();
        let cam = Camera::new(CameraModelKind::Pinhole, vec![1.0, 1.0, 0.0, 0.0], 10, 10).unwrap();
        rec.add_camera(CameraId::new(1), cam);
        let mut img = Image {
            image_id: ImageId::new(1),
            name: "a.jpg".into(),
            camera_id: CameraId::new(1),
            pose: crate::geometry::Rigid3::identity(),
            points2d: vec![Point2D {
                xy: [0.0, 0.0],
                point3d_id: None,
            }],
        };
        rec.add_image(img);
        rec.add_point3d(
            Point3DId::new(1),
            Point3D {
                xyz: nalgebra::Vector3::zeros(),
                color: [0, 0, 0],
                error: 0.0,
                track: Vec::new(),
            },
        );
        rec.add_observation(ImageId::new(1), 0, Point3DId::new(1));
        assert!(rec.validate().is_ok());
        // point track contains obs
        assert_eq!(rec.points3d.get(&Point3DId::new(1)).unwrap().track.len(), 1);
        rec.remove_observation(ImageId::new(1), 0);
        // point removed since track empty
        assert!(!rec.points3d.contains_key(&Point3DId::new(1)));
        assert!(rec.validate().is_ok());
    }
}
