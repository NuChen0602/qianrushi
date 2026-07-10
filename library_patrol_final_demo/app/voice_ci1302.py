class VoiceCi1302Stub:
    """First version voice adapter: no serial access, only simulated commands."""

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


class VoiceCi1302Serial:
    """Reserved real CI1302 adapter.

    TODO:
    - Open configured serial port only when voice.enabled=true.
    - Read fixed command frames without writing to the port.
    - Map frames to A1..A9 command ids.

    This class intentionally does not open /dev/ttyS1 in the first demo
    scaffold.
    """

    def __init__(self, serial_port="/dev/ttyS1", baudrate=115200):
        self.serial_port = serial_port
        self.baudrate = baudrate

    def start(self, callback):
        raise NotImplementedError("real CI1302 serial adapter is not enabled yet")

    def stop(self):
        pass
