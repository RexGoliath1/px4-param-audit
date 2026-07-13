PREFIX ?= $(HOME)/.local
BINARY := px4-param-audit

.PHONY: build local install clean-local

build:
	cargo build --release

local: build
	cp target/release/$(BINARY) ./$(BINARY)

install: build
	install -m 0755 target/release/$(BINARY) "$(PREFIX)/bin/$(BINARY)"

clean-local:
	rm -f ./$(BINARY)
