class BoardClockMapper:
    """Map board monotonic time to ROS time using minimum observed delay."""

    def __init__(self):
        self.minimum_offset_ns = None
        self.last_stamp_ns = None

    def map_ns(self, board_mono_ns, ros_receive_ns, sync_board_mono_ns=None):
        board_mono_ns = int(board_mono_ns)
        ros_receive_ns = int(ros_receive_ns)
        if board_mono_ns <= 0:
            raise ValueError('board monotonic timestamp must be positive')
        if sync_board_mono_ns is None:
            sync_board_mono_ns = board_mono_ns
        sync_board_mono_ns = int(sync_board_mono_ns)
        if sync_board_mono_ns <= 0:
            raise ValueError(
                'board synchronization timestamp must be positive')
        observed_offset = ros_receive_ns - sync_board_mono_ns
        if (self.minimum_offset_ns is None or
                observed_offset < self.minimum_offset_ns):
            self.minimum_offset_ns = observed_offset
        stamp_ns = board_mono_ns + self.minimum_offset_ns
        stamp_ns = min(stamp_ns, ros_receive_ns)
        if self.last_stamp_ns is not None:
            stamp_ns = max(stamp_ns, self.last_stamp_ns + 1)
        self.last_stamp_ns = stamp_ns
        return stamp_ns
