//! `colmap-rs` command-line entry point (minimal).

use std::env;

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() < 2 {
        print_usage();
        return;
    }
    match args[1].as_str() {
        "feature-extractor" | "feature_extractor" => {
            println!("feature-extractor: not yet implemented")
        }
        "exhaustive-matcher" | "exhaustive_matcher" => {
            println!("exhaustive-matcher: not yet implemented")
        }
        "mapper" => println!("mapper: not yet implemented"),
        "bundle-adjuster" | "bundle_adjuster" => println!("bundle-adjuster: not yet implemented"),
        "--help" | "-h" | "help" => print_usage(),
        other => {
            eprintln!("unknown command: {}", other);
            print_usage();
        }
    }
}

fn print_usage() {
    println!("Usage: colmap-rs <command> [options]");
    println!("Commands: feature-extractor, exhaustive-matcher, mapper, bundle-adjuster");
}
