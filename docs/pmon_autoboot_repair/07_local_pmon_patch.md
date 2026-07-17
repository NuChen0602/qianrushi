# Local PMON patch material

`patches/pmon_ignore_unknown_uart_input.patch` is a reference behavioral sketch only, because exact vendor source was not found. It makes unknown bytes—including `0x00` and `0xff`—continue the timer; CR/LF boots immediately; `c` enters maintenance; navigation selections retain a finite timer; and timeout zero boots safely. `tools/pmon_recovery/pmon_menu_state_test.py` models nine required cases. No firmware was built or flashed.
