# colmap-rs

A Rust port of [COLMAP](https://colmap.github.io/) (structure-from-motion and
multi-view stereo). Keeps COLMAP's algorithms and file formats; replaces the
C++/CUDA/Qt/Ceres stack with a memory-safe, easily built, cross-platform Rust
workspace.

See `design.md` for the full design and `Todo.md` for progress.

## Status

Early. Phase 0 (foundations: core types + I/O) in progress.

## Build

```sh
cargo build
cargo test
cargo clippy --all-targets -- -D warnings
cargo fmt --check
```

Requires a recent stable Rust toolchain (see `rust-toolchain.toml`). No system
dependencies for the CPU path (SQLite is bundled).

## Parity target

Installed COLMAP 4.3.0.dev0 (commit 137f2168, CPU/no-GPU build) is used as
the reference for golden-data comparison.

## License

BSD-3-Clause. See `LICENSE`. Upstream COLMAP copyright retained.
