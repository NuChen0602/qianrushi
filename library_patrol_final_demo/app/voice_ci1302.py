"""Deprecated in-process voice shim.

Real CI302 serial input is owned by scripts/voice_trigger_ssh_bridge.py.  This
module remains only for the existing web simulation endpoint.
"""


class VoiceCi302Stub:
    def __init__(self):
        self.callback = None
        self.running = False

    def start(self, callback):
        self.callback = callback
        self.running = True

    def stop(self):
        self.running = False
        self.callback = None

    def simulate_command(self, command_id):
        if self.running and self.callback:
            self.callback(str(command_id))


# Import compatibility for older code; no hardware I/O is performed here.
VoiceCi1302Stub = VoiceCi302Stub
