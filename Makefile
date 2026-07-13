PREFIX ?= $(HOME)/.local
BINARY := px4-param-audit

.DEFAULT_GOAL := build

.PHONY: build install clean

build:
	cargo build --release
	cp target/release/$(BINARY) ./$(BINARY)

install: build
	install -m 0755 target/release/$(BINARY) "$(PREFIX)/bin/$(BINARY)"

clean:
	rm -f ./$(BINARY)
	cargo clean
