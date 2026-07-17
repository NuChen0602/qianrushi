# PMON source findings

Local search covered the project root and `LS2K0301_Library`, excluding Git metadata, build/install/log directories, virtual environments, archives, and common binaries. It found no `Loongson-PMON-V1.005-release202602`, `release202602`, `not_delay`, menu implementation, or boot configuration source. The only match was `library_patrol_final_demo/config/board_boot_dht11.cfg` containing `showmenu 0`; it is not evidence of the board’s PMON configuration.

Conclusion: exact vendor source, config, board directory, and build instructions are missing. No community implementation has been represented as equivalent to the board binary.
