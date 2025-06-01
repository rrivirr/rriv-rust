# rriv-rust

## Building for Different Board Versions

This project supports building firmware for multiple board versions using Cargo features. You can select the target board at compile time by enabling the appropriate feature.

### Available Board Features
- `rriv_board_0_4_2` (default)
- `rriv_board_0_4_0`

### How to Build for a Specific Board

**Build for the default board (currently rriv_board_0_4_2):**
```sh
cargo build
```

**Build for rriv_board_0_4_0:**
```sh
cargo build --no-default-features --features rriv_board_0_4_0
```

**Build for rriv_board_0_4_2 (explicit):**
```sh
cargo build --features rriv_board_0_4_2
```

**Note:**
- Only one board feature can be enabled at a time. The build will fail if multiple board features are enabled simultaneously.
- The `rust-analyzer` feature is available for IDE support and should not be used for actual builds.
