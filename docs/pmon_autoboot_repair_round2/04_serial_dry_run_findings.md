# Serial dry-run findings

Log: `serial_recovery_20260716_061058.log`.

The port was opened read-only at 115200 8N1 with software/hardware flow control disabled and without DTR/RTS manipulation. No byte arrived during the initial five-second state window or the following observation window. State: `SERIAL_STATE_SILENT`; `bytes_before_write=0`; `whether_cr_written=False`; `exact_write_count=0`; final state silent.

This is not proof that the board is off or in PMON. Under the stated safety policy, the tool must not send CR in silent state unless a human explicitly confirms the on-site condition and manually invokes the force command.
