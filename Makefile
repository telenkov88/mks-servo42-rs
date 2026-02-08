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

coverage:
	mkdir -p target/coverage
	rm -rf target/coverage/*
	export RUSTFLAGS="-C instrument-coverage" && \
	export LLVM_PROFILE_FILE="target/coverage/cargo-test-%p-%m.profraw" && \
	cargo test
	grcov . --binary-path ./target/debug/deps/ -s target/coverage -t html --branch --ignore-not-existing --ignore "/*" -o target/coverage/html
	grcov . --binary-path ./target/debug/deps/ -s target/coverage -t cobertura --branch --ignore-not-existing --ignore "/*" -o target/coverage/cobertura.xml

clean:
	cargo clean

install-deps:
	cargo install cargo-llvm-cov grcov

.PHONY: build lint fmt test coverage clean install-deps
