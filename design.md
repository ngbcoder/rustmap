# COLMAP Rust Port: Design Doc

**Working name:** `colmap-rs`\
**Status:** Draft v0.1\
**Upstream:** COLMAP (BSD-3-Clause), C++ / CUDA / Qt / Ceres

## 1. Summary

COLMAP is the de facto open-source structure-from-motion (SfM) and multi-view stereo (MVS) pipeline. This document proposes a port to Rust that keeps COLMAP's algorithms and file formats but replaces its C++ core, SQLite/Ceres/FLANN/Qt dependency stack and CUDA-only dense path with a memory-safe, easily built, cross-platform Rust workspace.

The port is **incremental and interoperable**: every stage reads and writes COLMAP-compatible databases and sparse/dense model folders, so users can mix `colmap-rs` and upstream COLMAP stage by stage while the port matures.

## 2. Goals and non-goals

### Goals

- Functional parity for the sparse pipeline: feature extraction, matching, geometric verification, incremental mapping, bundle adjustment.
- Drop-in compatibility with the COLMAP database schema (`database.db`) and model formats (`cameras`, `images`, `points3D` in text and binary).
- A clean library API (crates) first, CLI second, GUI last.
- Single `cargo build` with no system dependencies for the CPU path.
- Deterministic results under a fixed seed, within floating-point tolerance of upstream.
- Vendor-neutral GPU acceleration (wgpu) rather than CUDA-only.

### Non-goals (initially)

- Bit-exact reproduction of upstream outputs.
- Reimplementing the Qt GUI one-to-one.
- Python bindings parity with pycolmap (planned later via PyO3).
- New research algorithms. Port first, innovate second.

## 3. Upstream architecture recap

| Stage | Upstream implementation | Notes |
| --- | --- | --- |
| Feature extraction | SIFT (CPU via VLFeat-derived code, GPU via SiftGPU) | RootSIFT, domain-size pooling, up to \~8k features |
| Feature matching | Exhaustive, sequential, vocab-tree, spatial, transitive, custom | Brute-force and FLANN, GPU matcher |
| Geometric verification | RANSAC for E, F, H; two-view geometry classification | Config types: calibrated, uncalibrated, planar, panoramic, watermark |
| Mapper | Incremental SfM: init pair, PnP registration, triangulation, local/global BA, filtering | Ceres for BA |
| Global mapper | Rotation averaging, global positioning (GLOMAP-style) | Newer upstream releases |
| Dense | PatchMatch stereo (CUDA), stereo fusion, Poisson / Delaunay meshing | CUDA-only dense path |
| Storage | SQLite database, binary/text model files, PLY output |  |
| UI | Qt GUI, CLI |  |
| Math | Eigen, Ceres, PoissonRecon, FLANN, LSD |  |

## 4. Rust ecosystem mapping

| Concern | C++ dependency | Rust choice | Risk |
| --- | --- | --- | --- |
| Linear algebra | Eigen | `nalgebra` (core types), `faer` for large dense/sparse factorizations | Low |
| Sparse solvers (BA Schur) | Ceres + SuiteSparse | Own LM + Schur complement solver on `faer` sparse Cholesky; evaluate `tiny-solver` as a baseline | **High** |
| Auto-diff | Ceres jets | Analytic Jacobians per camera model (fast, testable); optional dual-number fallback | Medium |
| Database | SQLite | `rusqlite` (bundled) | Low |
| Images | FreeImage / OpenImageIO | `image`, `kamadak-exif` for EXIF | Low |
| Parallelism | OpenMP | `rayon` | Low |
| Nearest neighbor | FLANN | `kiddo` / `hnsw` crates; brute-force on GPU | Medium |
| GPU compute | CUDA | `wgpu` + WGSL compute; optional `cudarc` backend for dense | **High** |
| Meshing | PoissonRecon, CGAL (Delaunay) | Start by shelling out / FFI to PoissonRecon; later native port | Medium |
| CLI | custom | `clap` | Low |
| GUI | Qt | `egui` + `wgpu` (or `rerun` integration for visualization) | Medium |
| Serialization | custom binary | `byteorder`, `bytemuck` | Low |
| Logging | glog | `tracing` | Low |

The two highest risks are **bundle adjustment** (Ceres has no mature Rust equivalent) and **GPU dense stereo** (PatchMatch is CUDA-tuned). Both are scheduled early in the plan to retire risk quickly.

## 5. Workspace layout

```
colmap-rs/
├── crates/
│   ├── colmap-core        # scalar types, IDs, camera models, Sim3/Rigid3, geometry primitives
│   ├── colmap-io          # database (SQLite), model read/write (txt/bin), PLY
│   ├── colmap-features    # SIFT extraction, descriptors, RootSIFT
│   ├── colmap-matching    # matchers, retrieval (vocab tree), pair generators
│   ├── colmap-geometry    # RANSAC, 5pt/7pt/8pt, homography, PnP, triangulation
│   ├── colmap-optim       # LM solver, Schur BA, robust losses, gauge handling
│   ├── colmap-sfm         # incremental mapper, global mapper, track management
│   ├── colmap-mvs         # patch match, fusion, depth/normal maps
│   ├── colmap-mesh        # Poisson / Delaunay wrappers
│   ├── colmap-gpu         # wgpu context, kernels, buffers
│   ├── colmap-cli         # `colmap-rs` binary
│   └── colmap-gui         # egui viewer (later)
├── tests/                 # golden-data regression suite
├── benches/               # criterion benchmarks
└── docs/
```

Layering rule: crates only depend downward (`core` ← `io`/`geometry` ← `features`/`matching`/`optim` ← `sfm`/`mvs` ← `cli`/`gui`). No crate depends on the CLI or GUI.

## 6. Core design decisions

### 6.1 Types and IDs

- Strongly typed newtype IDs: `CameraId`, `ImageId`, `Point3DId`, `Point2DIdx`, replacing raw `uint32_t`/`uint64_t`. Same numeric values on disk.
- Precision: `f64` for geometry and optimization, `f32` for descriptors, image data and GPU buffers.
- Camera models as an enum with a trait (`CameraModel`) providing `project`, `unproject`, `jacobian`, `params_info`. Models to cover: `SIMPLE_PINHOLE`, `PINHOLE`, `SIMPLE_RADIAL`, `RADIAL`, `OPENCV`, `OPENCV_FISHEYE`, `FULL_OPENCV`, `SIMPLE_RADIAL_FISHEYE`, `RADIAL_FISHEYE`, `THIN_PRISM_FISHEYE`, plus rig/sensor extensions later.

### 6.2 The reconstruction data model

Upstream uses mutable shared state (`Reconstruction`, `CorrespondenceGraph`, `Track`) with cross-references by ID. In Rust:

- Arena-style storage: `Vec`/`HashMap` keyed by typed IDs; relationships stored as IDs, never references or `Rc<RefCell>`.
- `Reconstruction` owns `cameras`, `images`, `points3d`; `Image` stores `points2d: Vec<Point2D>` with an optional `Point3DId`.
- `CorrespondenceGraph` is immutable once built (shared via `Arc`), while the mapper keeps its own mutable observation-tracking state. This removes a whole class of aliasing bugs.
- Track updates (merge, split, delete observation) go through methods that maintain invariants, with `debug_assert!` consistency checks and a `validate()` used in tests.

### 6.3 Concurrency

- `rayon` for data parallelism (feature extraction per image, matching per pair, triangulation per image, filtering).
- A bounded work queue with a dedicated I/O thread for image decoding, to overlap disk and compute.
- No global mutable state. Configuration passed as plain option structs implementing `serde::{Serialize, Deserialize}`, so options load from TOML/JSON and map 1:1 to CLI flags.

### 6.4 Error handling

- Library crates use `thiserror` enums; CLI uses `anyhow`.
- Degenerate geometry returns `Option`/`Result`, never panics. Panics are reserved for violated internal invariants.
- `#![forbid(unsafe_code)]` in all crates except `colmap-gpu` and explicit FFI shims, which are audited.

## 7. Component designs

### 7.1 Feature extraction (`colmap-features`)

- Implement SIFT natively: Gaussian scale-space, DoG extrema, sub-pixel refinement, orientation assignment, 128-D descriptors, optional RootSIFT (L1-normalize, square root) and upright variants.
- Optional **affine-shape / DSP-SIFT** as follow-ups.
- Output: `Keypoints { x, y, scale, orientation }` and `Descriptors` as `u8`-quantized rows, matching COLMAP's database representation.
- Backends: CPU (rayon) first; wgpu compute kernels for the Gaussian pyramid and DoG as a later optimization.
- Image handling: EXIF focal length extraction, max image size downscaling, camera model/prior intrinsics per folder (matching `single_camera` and `single_camera_per_folder`).
- Pluggable `FeatureExtractor` trait so learned features (SuperPoint, ALIKED) can be added via `ort` (ONNX Runtime) later without touching the pipeline.

### 7.2 Matching (`colmap-matching`)

- `PairGenerator` trait with implementations: exhaustive, sequential (with overlap and loop detection), vocab-tree retrieval, spatial (GPS), transitive, and custom pair list.
- `Matcher` trait: brute-force with ratio test and cross-check (CPU via blocked matrix multiply with `faer`; GPU via wgpu tiled kernel), guided matching using known geometry.
- Vocabulary tree: port the COLMAP visual-index format so existing `.bin` vocab trees are loadable.
- Results written to the `matches` and `two_view_geometries` tables.

### 7.3 Geometry and RANSAC (`colmap-geometry`)

- Generic RANSAC framework: `Estimator`, `SupportMeasurer` (inlier count + residual sum, plus an MSAC/LO-RANSAC variant), adaptive iteration count, and local optimization.
- Minimal solvers: 5-point essential, 7/8-point fundamental, 4-point homography, P3P/EPnP for absolute pose, DLT triangulation, plus relative-pose decomposition with cheirality checks.
- Two-view geometry classifier replicating COLMAP's decision logic (calibrated vs. uncalibrated vs. planar/panoramic vs. watermark detection) so database outputs match.
- Seeded RNG (`rand_pcg`) threaded through every estimator for reproducibility.

### 7.4 Optimization (`colmap-optim`) — the critical piece

Bundle adjustment is the hot loop of SfM. Plan:

1. **Problem representation:** parameter blocks (camera poses, intrinsics, 3D points) with constant/variable masks and optional manifold handling for rotations (quaternion or Lie-algebra updates).
2. **Residuals:** reprojection error per observation with analytic Jacobians for each camera model, validated against finite differences in tests.
3. **Solver:** Levenberg–Marquardt with trust-region adaptation, Schur complement elimination of 3D points, reduced camera system solved via `faer` sparse Cholesky or preconditioned CG (Jacobi / block-Jacobi) for large problems.
4. **Robust losses:** Huber, Cauchy, soft-L1, applied through IRLS-style residual weighting (Triggs correction).
5. **Gauge fixing:** fix first camera and scale (as upstream does) or use priors on pose/position.
6. **Variants:** local BA over a window of recent images; global BA with periodic invocation; pose-prior BA for GPS; intrinsics refinement toggles (focal, principal point, extra params).
7. **Parallelism:** residual and Jacobian evaluation with rayon; Schur-block construction per point in parallel.

Validation strategy: replicate several upstream BA problems (exported as `.bal`-style fixtures) and compare final cost and parameter deltas against Ceres output. If native performance trails Ceres by more than \~1.5x on benchmark datasets, profile and optimize (SIMD via `std::simd`/`wide`, block-sparse layouts) before widening scope.

### 7.5 Incremental mapper (`colmap-sfm`)

Faithful port of the incremental flow:

1. Load database, build correspondence graph, compute two-view geometry stats.
2. Choose initial pair (many inliers, sufficient triangulation angle, good coverage).
3. Triangulate initial structure; run global BA.
4. Loop: select next best image (visible-point count with multi-resolution coverage score) → absolute pose via RANSAC + PnP → refine pose → triangulate new points → complete/merge tracks → local BA → filter (reprojection error, triangulation angle, observation count) → periodic global BA with adaptive frequency.
5. Retriangulation and re-registration for failed images; model splitting when registration fails (multiple models).
6. Post-processing: final BA, filtering, export.

The mapper is written as a state machine with explicit `Observer` callbacks (progress, per-step events) instead of Qt signals, enabling the CLI, GUI and tests to hook in.

### 7.6 Global mapper (phase 2)

- Rotation averaging (robust L1 + IRLS), global positioning (GLOMAP-style point-to-camera constraints), then global BA and track filtering.
- Reuses the same `colmap-optim` and `colmap-geometry` building blocks, which is a strong argument for getting those crates right early.

### 7.7 Dense reconstruction (`colmap-mvs`, `colmap-mesh`)

- **PatchMatch stereo:** port as wgpu compute shaders (random init, propagation with checkerboard scheduling, view selection, photometric and geometric consistency costs, depth/normal refinement). A CPU reference implementation with rayon is written first for correctness and testing.
- **Optional CUDA backend** through `cudarc` for users who want upstream-level speed on NVIDIA hardware, behind a cargo feature.
- **Stereo fusion:** depth-map consistency filtering and point merging into a dense PLY (pure Rust).
- **Meshing:** Phase 1 wraps PoissonRecon via a subprocess or FFI shim; Phase 2 native Delaunay + graph-cut meshing and a native screened Poisson port.
- **Undistortion:** image undistortion and export to common MVS layouts (NVM, Bundler, PMVS) as utilities.

### 7.8 I/O and compatibility (`colmap-io`)

- Full implementation of the COLMAP SQLite schema (`cameras`, `images`, `keypoints`, `descriptors`, `matches`, `two_view_geometries`, `pose_priors`, and so on) with schema-version checks.
- Text and binary model readers/writers for cameras, images, points3D, plus rig definitions when supported.
- Round-trip test: read an upstream model, write it, and byte-compare (binary) or semantically compare (text).
- Exporters: PLY, NVM, Bundler, VisSfM, plus a glTF export for modern viewers.

## 8. CLI design

Binary `colmap-rs` with subcommands mirroring upstream names so scripts port easily:

```
colmap-rs feature-extractor   --database db.db --image-path images/
colmap-rs exhaustive-matcher  --database db.db
colmap-rs sequential-matcher  --database db.db --overlap 10
colmap-rs mapper              --database db.db --image-path images/ --output-path sparse/
colmap-rs bundle-adjuster     --input sparse/0 --output sparse/0
colmap-rs image-undistorter   --input sparse/0 --output dense/
colmap-rs patch-match-stereo  --workspace dense/
colmap-rs stereo-fusion       --workspace dense/ --output fused.ply
colmap-rs automatic-reconstructor --workspace project/ --quality medium
```

Underscore-style aliases (`feature_extractor`) are accepted for drop-in script compatibility. All options can also come from a TOML project file.

## 9. GUI plan

- Phase 1: no GUI; export to existing viewers and optionally stream to **Rerun** for live visualization during mapping.
- Phase 2: `egui` + `wgpu` viewer showing cameras, sparse/dense points, tracks, and reconstruction progress.
- Phase 3: project management, database inspection, and parameter panels equivalent to the upstream workflows people actually use (automatic reconstruction, model viewer, BA controls).

## 10. Testing and validation

- **Unit tests:** per-camera-model project/unproject round trips, solver minimal-case tests, RANSAC statistics, schema checks.
- **Property tests (`proptest`):** geometry invariants (e.g., recovered pose consistency under random rigid transforms, triangulation under noise).
- **Golden datasets:** small public sets (Fountain, South Building, Gerrard Hall, ETH3D training scenes) with stored upstream outputs.
- **Metrics:** registered image count, mean reprojection error, track length statistics, number of 3D points, absolute trajectory error after Sim(3) alignment against upstream and against ground truth where available.
- **Acceptance thresholds:** registered images ≥ 99% of upstream, mean reprojection error within 5%, ATE within an agreed tolerance relative to scene scale.
- **Fuzzing:** `cargo-fuzz` targets for the database and model readers.
- **CI:** Linux, macOS and Windows; clippy, rustfmt, `cargo deny`, and `cargo miri` on safe-code crates where practical.

## 11. Performance targets

| Area | Target relative to upstream |
| --- | --- |
| SIFT extraction (CPU) | ≤ 1.5x slower, GPU path competitive |
| Exhaustive matching (GPU) | within 2x of CUDA path |
| Bundle adjustment | within 1.5x of Ceres on benchmark problems |
| Incremental mapper end-to-end | within 1.5x |
| Dense PatchMatch (wgpu) | within 2x of CUDA on equivalent hardware |
| Memory use | ≤ upstream |

Benchmarks run in CI with `criterion` and tracked over time so regressions are visible.

## 12. Licensing and attribution

Upstream is BSD-3-Clause, which allows a derivative port. The port keeps the original copyright notice, credits the COLMAP authors (Schönberger et al.), and cites the SfM and MVS papers in the README. Third-party code (PoissonRecon, vocabulary trees, SiftGPU-derived ideas) is reviewed for license compatibility individually before being ported or linked.

## 13. Roadmap

| Phase | Scope | Exit criteria |
| --- | --- | --- |
| **0 – Foundations** (weeks 1–4) | Workspace, core types, camera models, database and model I/O, round-trip tests | Reads/writes upstream models and databases losslessly |
| **1 – Geometry** (weeks 5–9) | RANSAC framework, minimal solvers, triangulation, PnP, two-view geometry classification | Matches upstream two-view outcomes on test pairs |
| **2 – Optimization** (weeks 8–14) | LM + Schur BA, robust losses, analytic Jacobians | BA cost parity and performance within target on fixtures |
| **3 – Features and matching** (weeks 12–18) | SIFT CPU, brute-force and sequential matching, vocab tree | Database from `colmap-rs` is usable by upstream mapper |
| **4 – Incremental mapper** (weeks 16–26) | Full mapper loop, multi-model handling | Golden dataset acceptance thresholds met |
| **5 – GPU acceleration** (weeks 24–34) | wgpu SIFT and matching kernels | Speedups on reference hardware |
| **6 – Dense** (weeks 30–44) | PatchMatch (CPU then GPU), fusion, Poisson wrapper | Dense clouds comparable to upstream on ETH3D scenes |
| **7 – Global mapper and polish** (weeks 40+) | Rotation averaging, global positioning, docs, PyO3 bindings, viewer | Public 0.1 release |

Phases overlap; 1 and 2 can be built in parallel by separate contributors.

## 14. Risks and mitigations

| Risk | Impact | Mitigation |
| --- | --- | --- |
| Native BA slower or less robust than Ceres | High | Build and benchmark BA first; fall back to FFI-wrapped Ceres behind a feature flag during transition |
| Numerical divergence from upstream | Medium | Aim for statistical, not bit-exact, parity; golden-data regression with tolerances |
| wgpu limits (no unified memory control, shader portability quirks) | Medium | CPU reference implementations; optional CUDA backend; keep kernels simple and tile-based |
| Scope creep from GUI and research features | Medium | Strict phase gates; GUI deferred; learned features only after parity |
| Upstream moves fast (global mapper, rigs, new models) | Medium | Track upstream release tags; pin a parity target version per milestone |
| Contributor bandwidth | High | Small crates with clear boundaries; good-first-issue labels on solvers and exporters |

## 15. Open questions

1. Which upstream version is the parity target (latest stable at project start)?
2. Is an FFI-wrapped Ceres acceptable as a stopgap, or must the first release be pure Rust?
3. How important is CUDA parity for dense reconstruction versus a vendor-neutral wgpu path?
4. Should the library expose a stable semver API from 0.1, or stay unstable until the mapper reaches parity?
5. Rig and multi-sensor support: include in the first mapper milestone or defer?
6. Do we ship Python bindings from day one to ease adoption by existing pycolmap users?

## 16. Next steps

1. Create the workspace and CI skeleton with `colmap-core` and `colmap-io`.
2. Implement the database schema and the binary/text model round trip; commit golden fixtures.
3. Prototype the BA solver on a single synthetic and one real fixture to retire the biggest risk.
4. Decide the parity target version and the answers to the open questions above.
