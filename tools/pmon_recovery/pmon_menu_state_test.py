#!/usr/bin/env python3
"""Reference-only model of the proposed PMON input policy; no board I/O."""
def step(timeout, data):
    maintenance = False
    for byte in data:
        if byte in (13, 10): return 'boot'
        if byte == ord('c'): maintenance = True
        # Unknown/line-noise and navigation input do not permanently freeze timer.
    if maintenance: return 'maintenance'
    return 'boot' if timeout <= 1 else step(timeout - 1, b'')

tests = [(3,b'', 'boot'), (1,b'', 'boot'), (3,b'\0','boot'), (3,b'\xff','boot'),
         (3,b'x','boot'), (3,b'\r','boot'), (3,b'c','maintenance'),
         (3,b'\x1b[A','boot'), (0,b'','boot')]
for timeout, data, expected in tests:
    got = step(timeout, data)
    assert got == expected, (timeout, data, got)
print('9 reference state-machine tests passed')
