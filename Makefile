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
	grcov target/coverage --binary-path ./target/debug/deps/ -s . -t html --branch --ignore-not-existing --ignore "/*" --ignore "tests/*" --excl-line "#\[test\]" --excl-start "mod tests \{" --excl-stop "^\}" -o target/coverage/html
	grcov target/coverage --binary-path ./target/debug/deps/ -s . -t cobertura --branch --ignore-not-existing --ignore "/*" --ignore "tests/*" --excl-line "#\[test\]" --excl-start "mod tests \{" --excl-stop "^\}" -o target/coverage/cobertura.xml
	grcov target/coverage --binary-path ./target/debug/deps/ -s . -t markdown --branch --ignore-not-existing --ignore "/*" --ignore "tests/*" --excl-line "#\[test\]" --excl-start "mod tests \{" --excl-stop "^\}"

doc:
	cargo doc --no-deps --open

check-publish:
	cargo package --list
	cargo package

publish-dry-run:
	cargo publish --dry-run

publish:
	cargo publish

clean:
	cargo clean

install-deps:
	cargo install cargo-llvm-cov grcov

.PHONY: build lint fmt test coverage doc check-publish publish-dry-run publish clean install-deps
