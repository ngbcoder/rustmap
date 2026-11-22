# Todo

Tracking document for `colmap-rs` (Rust port of COLMAP). Parity target: installed COLMAP **4.3.0.dev0** (commit 137f2168, no GPU). Updated as work progresses. Milestones are intentionally granular.

Legend: `[ ]` pending, `[~]` in progress, `[x]` done, `[-]` skipped/not applicable.

---

## M0 — Repository & tooling

- [x] M0.1 Create GitHub repository `ngbcoder/rustmap` (public)
- [x] M0.2 Initialize local git repo on `main`
- [x] M0.3 Add `.gitignore` (Rust + IDE + build artifacts)
- [x] M0.4 Add `rust-toolchain.toml` (pin stable, components rustfmt/clippy)
- [x] M0.5 Add `Cargo.toml` workspace root
- [x] M0.6 Add CI workflow (build, test, clippy, fmt) on Linux/macOS/Windows
- [x] M0.7 Add `LICENSE` (BSD-3-Clause, preserving upstream notice)
- [x] M0.8 Add `README.md` with build instructions and attribution
- [ ] M0.9 Configure `cargo deny` / dependency policy
- [x] M0.10 First commit & push

## M1 — Phase 0: Foundations (core + io)

### M1.1 `colmap-core` crate skeleton
- [x] M1.1.1 Create crate `crates/colmap-core`
- [x] M1.1.2 `Cargo.toml` with deps (nalgebra, thiserror, serde)
- [x] M1.1.3 Crate-level `#![forbid(unsafe_code)]`
- [x] M1.1.4 Module layout: `ids`, `types`, `camera`, `geometry`, `error`, `reconstruction`
- [ ] M1.1.5 README/module docs

### M1.2 Typed IDs
- [x] M1.2.1 `CameraId` newtype
- [x] M1.2.2 `ImageId`
- [x] M1.2.3 `Point3DId`
- [x] M1.2.4 `Point2DIdx`
- [x] M1.2.5 `RigId`, `SensorId` (stubs)
- [x] M1.2.6 Display/Debug/Hash/Eq/Ord impls
- [x] M1.2.7 Serde impls (transparent)
- [x] M1.2.8 Unit tests for ID round-trip

### M1.3 Scalar/geometry primitives
- [x] M1.3.1 `Rigid3` (rotation quaternion + translation) with inverse/compose
- [x] M1.3.2 `Sim3` (scale + rotation + translation)
- [ ] M1.3.3 `Pose` aliasing and conversions
- [x] M1.3.4 Vector/point type aliases (Point2, Point3)
- [x] M1.3.5 Angle helpers, degree/radian conversions
- [x] M1.3.6 Unit tests (composition associativity, inverse)

### M1.4 Camera models
- [ ] M1.4.1 `CameraModel` trait: `project`, `unproject`, `jacobian`, `params_info`, `name` (enum methods used instead)
- [x] M1.4.2 `Camera` struct (model enum, params Vec<f64>, width, height)
- [x] M1.4.3 `SIMPLE_PINHOLE`
- [x] M1.4.4 `PINHOLE`
- [x] M1.4.5 `SIMPLE_RADIAL`
- [x] M1.4.6 `RADIAL`
- [x] M1.4.7 `OPENCV`
- [x] M1.4.8 `OPENCV_FISHEYE`
- [x] M1.4.9 `FULL_OPENCV`
- [x] M1.4.10 `SIMPLE_RADIAL_FISHEYE`
- [x] M1.4.11 `RADIAL_FISHEYE`
- [x] M1.4.12 `THIN_PRISM_FISHEYE`
- [x] M1.4.13 Model enum discriminants matching COLMAP numeric IDs
- [ ] M1.4.14 `FromStr`/`Display` for model names (from_name exists; Display not)
- [x] M1.4.15 Project/unproject round-trip tests
- [ ] M1.4.16 Jacobian finite-difference validation tests

### M1.5 Core data model
- [x] M1.5.1 `Point2D` (xy, optional `Point3DId`)
- [x] M1.5.2 `Point3D` (xyz, color, error, track list)
- [x] M1.5.3 `Image` (name, camera id, pose, points2d)
- [x] M1.5.4 `Reconstruction` (maps + methods)
- [x] M1.5.5 Invariant methods: add/remove observation, merge/split track (partial)
- [x] M1.5.6 `validate()` consistency check
- [ ] M1.5.7 `debug_assert!` invariants
- [ ] M1.5.8 Unit tests for track maintenance

### M1.6 Error handling
- [x] M1.6.1 `ColmapError` enum (thiserror)
- [x] M1.6.2 Result aliases
- [x] M1.6.3 No panics on degenerate geometry

### M1.7 `colmap-io` crate skeleton
- [x] M1.7.1 Create crate `crates/colmap-io`
- [x] M1.7.2 Deps (rusqlite bundled, byteorder, bytemuck, serde, colmap-core)
- [x] M1.7.3 Module layout: `database`, `model`, `ply`, `error`

### M1.8 Database schema (SQLite)
- [x] M1.8.1 Open/create database, version check (create_tables)
- [x] M1.8.2 `cameras` table read/write
- [x] M1.8.3 `images` table read/write
- [x] M1.8.4 `keypoints` table read/write (blob format)
- [x] M1.8.5 `descriptors` table read/write (blob format)
- [x] M1.8.6 `matches` table read/write
- [x] M1.8.7 `two_view_geometries` table read/write
- [ ] M1.8.8 `pose_priors` table read/write
- [ ] M1.8.9 `rigs`/`frames` tables (stubs)
- [ ] M1.8.10 Schema version validation
- [ ] M1.8.11 Unit tests: create db, insert, read back

### M1.9 Model files (text + binary)
- [x] M1.9.1 Binary reader/writer for cameras
- [x] M1.9.2 Binary reader/writer for images
- [x] M1.9.3 Binary reader/writer for points3D
- [x] M1.9.4 Text reader/writer for cameras
- [x] M1.9.5 Text reader/writer for images
- [x] M1.9.6 Text reader/writer for points3D
- [ ] M1.9.7 Rig definitions (stubs)
- [x] M1.9.8 Round-trip tests: binary read/write
- [x] M1.9.9 Round-trip tests: text semantic compare
- [x] M1.9.10 Cross-compat test: read upstream model, write, diff (validated via colmap model_converter + committed golden fixture)

### M1.10 PLY export
- [x] M1.10.1 Write PLY point cloud
- [ ] M1.10.2 Unit test

### M1.11 Golden-data harness
- [x] M1.11.1 Add `tests/` golden fixtures directory
- [x] M1.11.2 Generate a tiny upstream model with installed COLMAP (model_converter validated)
- [x] M1.11.3 Regression test loading it (golden.rs)

### M1.12 Phase 0 exit
- [x] M1.12.1 `cargo build` clean
- [x] M1.12.2 `cargo test` all pass
- [x] M1.12.3 `cargo clippy` clean
- [x] M1.12.4 `cargo fmt` clean
- [x] M1.12.5 Commit & push

## M2 — Phase 1: Geometry

### M2.1 RANSAC framework
- [x] M2.1.1 Generic `Estimator` trait
- [ ] M2.1.2 `SupportMeasurer` (inlier count + residual sum)
- [ ] M2.1.3 MSAC variant
- [ ] M2.1.4 LO-RANSAC local optimization
- [ ] M2.1.5 Adaptive iteration count
- [x] M2.1.6 Seeded RNG (rand_pcg) threading
- [ ] M2.1.7 Unit tests: known inlier sets

### M2.2 Minimal solvers
- [ ] M2.2.1 5-point essential
- [ ] M2.2.2 7-point fundamental
- [ ] M2.2.3 8-point fundamental
- [ ] M2.2.4 4-point homography
- [ ] M2.2.5 P3P absolute pose
- [ ] M2.2.6 EPnP
- [x] M2.2.7 DLT triangulation
- [ ] M2.2.8 Relative-pose decomposition + cheirality
- [ ] M2.2.9 Unit tests per solver

### M2.3 Triangulation
- [x] M2.3.1 Midpoint / DLT triangulation
- [ ] M2.3.2 Track triangulation from multiple views
- [ ] M2.3.3 Reprojection error computation
- [ ] M2.3.4 Unit tests

### M2.4 PnP
- [ ] M2.4.1 PnP RANSAC wrapper
- [ ] M2.4.2 Pose refinement (nonlinear)
- [ ] M2.4.3 Unit tests

### M2.5 Two-view geometry
- [ ] M2.5.1 Classifier (calibrated/uncalibrated/planar/panoramic/watermark)
- [ ] M2.5.2 Database output parity
- [ ] M2.5.3 Tests on pairs

### M2.6 `colmap-geometry` crate
- [x] M2.6.1 Create crate
- [x] M2.6.2 Wire into workspace
- [ ] M2.6.3 Benchmarks

## M3 — Phase 2: Optimization (BA)

### M3.1 Problem representation
- [ ] M3.1.1 Parameter blocks (cameras, intrinsics, points)
- [ ] M3.1.2 Constant/variable masks
- [ ] M3.1.3 Manifold handling for rotations (quaternion/Lie)
- [ ] M3.1.4 Gauge fixing

### M3.2 Residuals & Jacobians
- [ ] M3.2.1 Reprojection residual per observation
- [ ] M3.2.2 Analytic Jacobians per camera model
- [ ] M3.2.3 Finite-difference validation tests
- [ ] M3.2.4 Robust losses (Huber, Cauchy, soft-L1, Triggs)

### M3.3 LM solver
- [ ] M3.3.1 Levenberg-Marquardt with trust region
- [ ] M3.3.2 Schur complement elimination of points
- [ ] M3.3.3 Sparse Cholesky via faer
- [ ] M3.3.4 Preconditioned CG option
- [ ] M3.3.5 Local BA window
- [ ] M3.3.6 Global BA
- [ ] M3.3.7 Pose-prior BA
- [ ] M3.3.8 Intrinsics refinement toggles
- [ ] M3.3.9 Parallel residual/Jacobian (rayon)
- [ ] M3.3.10 Ceres FFI stopgap (feature flag) — decision pending

### M3.4 Validation
- [ ] M3.4.1 Exported BA fixtures from upstream
- [ ] M3.4.2 Cost/parameter comparison tests
- [ ] M3.4.3 Benchmarks vs Ceres

## M4 — Phase 3: Features & matching

### M4.1 `colmap-features`
- [ ] M4.1.1 Crate skeleton
- [ ] M4.1.2 SIFT: Gaussian scale space
- [ ] M4.1.3 DoG extrema
- [ ] M4.1.4 Sub-pixel refinement
- [ ] M4.1.5 Orientation assignment
- [ ] M4.1.6 128-D descriptors
- [ ] M4.1.7 RootSIFT
- [ ] M4.1.8 Upright variant
- [ ] M4.1.9 u8 quantization output
- [ ] M4.1.10 EXIF focal length extraction
- [ ] M4.1.11 Max image size downscaling
- [ ] M4.1.12 single_camera / per_folder handling
- [ ] M4.1.13 `FeatureExtractor` trait
- [ ] M4.1.14 CPU rayon backend
- [ ] M4.1.15 wgpu kernels (later)

### M4.2 `colmap-matching`
- [ ] M4.2.1 Crate skeleton
- [ ] M4.2.2 `PairGenerator` trait
- [ ] M4.2.3 Exhaustive pair generator
- [ ] M4.2.4 Sequential (overlap + loop detection)
- [ ] M4.2.5 Vocab-tree retrieval
- [ ] M4.2.6 Spatial (GPS)
- [ ] M4.2.7 Transitive
- [ ] M4.2.8 Custom pair list
- [ ] M4.2.9 `Matcher` trait
- [ ] M4.2.10 Brute-force + ratio test + cross-check
- [ ] M4.2.11 Guided matching
- [ ] M4.2.12 Vocab tree port (load .bin)
- [ ] M4.2.13 Write matches/two_view_geometries tables

## M5 — Phase 4: Incremental mapper

- [ ] M5.1 `colmap-sfm` crate skeleton
- [ ] M5.2 Correspondence graph
- [ ] M5.3 Initial pair selection
- [ ] M5.4 Triangulate initial structure + global BA
- [ ] M5.5 Next-best-image selection
- [ ] M5.6 Absolute pose via RANSAC+PnP
- [ ] M5.7 Pose refinement
- [ ] M5.8 Triangulate new points
- [ ] M5.9 Track complete/merge
- [ ] M5.10 Local BA
- [ ] M5.11 Filtering (reproj error, angle, count)
- [ ] M5.12 Periodic global BA (adaptive)
- [ ] M5.13 Retriangulation & re-registration
- [ ] M5.14 Multi-model splitting
- [ ] M5.15 Final BA + filtering + export
- [ ] M5.16 Observer callbacks (progress/events)
- [ ] M5.17 Golden dataset acceptance tests

## M6 — Phase 5: GPU acceleration

- [ ] M6.1 `colmap-gpu` crate skeleton
- [ ] M6.2 wgpu context + buffers
- [ ] M6.3 SIFT pyramid/DoG kernels
- [ ] M6.4 Matching tiled kernel
- [ ] M6.5 CPU reference parity tests
- [ ] M6.6 Optional CUDA backend (feature)

## M7 — Phase 6: Dense

- [ ] M7.1 `colmap-mvs` crate skeleton
- [ ] M7.2 PatchMatch stereo (CPU reference)
- [ ] M7.3 PatchMatch (wgpu)
- [ ] M7.4 Stereo fusion
- [ ] M7.5 `colmap-mesh` crate (Poisson wrapper)
- [ ] M7.6 Undistortion + export layouts

## M8 — Phase 7: Global mapper & polish

- [ ] M8.1 Rotation averaging (robust L1 + IRLS)
- [ ] M8.2 Global positioning (GLOMAP-style)
- [ ] M8.3 Global BA + filtering
- [ ] M8.4 CLI (`colmap-rs` with subcommands + aliases)
- [ ] M8.5 TOML project file support
- [ ] M8.6 Rerun visualization (optional)
- [ ] M8.7 `egui` viewer (phase 2 GUI)
- [ ] M8.8 PyO3 bindings
- [ ] M8.9 Docs & public 0.1 release

## M9 — Cross-cutting

- [ ] M9.1 Property tests (proptest) across crates
- [ ] M9.2 Fuzz targets for db/model readers
- [ ] M9.3 Benchmarks (criterion) tracked in CI
- [ ] M9.4 `cargo miri` on safe crates
- [ ] M9.5 Determinism under fixed seed

---

## Notes / decisions

- Parity target: COLMAP 4.3.0.dev0 (installed) — overrides earlier 3.11 pick.
- BA stopgap: Ceres FFI allowed (behind feature flag).
- Dense GPU: wgpu first; optional CUDA later.
- API: stable semver from 0.1 (be careful with public API).
- First slice: Phase 0 (M1) only.
- Validation data: installed COLMAP available at `/home/hoid/lp/colmap/build/src/colmap/exe/colmap`.
