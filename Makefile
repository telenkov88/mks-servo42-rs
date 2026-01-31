run:
	cargo run --example base

build:
	cargo build

lint:
	cargo clippy --all-targets -- -D warnings

fmt:
	cargo fmt

test:
	cargo test

clean:
	cargo clean

.PHONY: build lint fmt test clean
