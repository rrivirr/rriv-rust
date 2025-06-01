// board_select.rs
// Board selection and build_board() logic for conditional compilation.
// This file contains all feature-gated board selection code.

use rriv_board::RRIVBoard;

#[cfg(all(feature = "rriv_board_0_4_2", not(feature = "rriv_board_0_4_0")))]
pub type SelectedBoard = rriv_board_0_4_2::Board;
#[cfg(all(feature = "rriv_board_0_4_0", not(feature = "rriv_board_0_4_2")))]
pub type SelectedBoard = rriv_board_0_4_0::Board;

#[cfg(all(feature = "rriv_board_0_4_2", not(feature = "rriv_board_0_4_0")))]
pub fn build_board() -> SelectedBoard {
    rriv_board_0_4_2::build()
}
#[cfg(all(feature = "rriv_board_0_4_0", not(feature = "rriv_board_0_4_2")))]
pub fn build_board() -> SelectedBoard {
    rriv_board_0_4_0::build()
}

// This fallback is only for Rust Analyzer to avoid false errors in the IDE.
// It is never used in real builds.
// To enable this, add the following to your app/Cargo.toml:
// [features]
// rust-analyzer = []
#[cfg(feature = "rust-analyzer")]
pub type SelectedBoard = ();
#[cfg(feature = "rust-analyzer")]
pub fn build_board() -> SelectedBoard {
    unimplemented!("This is a dummy for rust-analyzer only");
}

#[cfg(not(any(
    all(feature = "rriv_board_0_4_2", not(feature = "rriv_board_0_4_0")),
    all(feature = "rriv_board_0_4_0", not(feature = "rriv_board_0_4_2"))
)))]
compile_error!("You must enable exactly one board feature.");
