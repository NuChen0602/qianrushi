def detect_hazard_stub(mode):
    payloads = {
        "socket": {
            "has_hazard": True,
            "type": "socket",
            "message": "发现插座乱接和线缆绊倒风险。",
        },
        "fire": {
            "has_hazard": True,
            "type": "fire",
            "message": "传感器异常，存在起火风险。",
        },
        "exit": {
            "has_hazard": True,
            "type": "exit",
            "message": "发现安全出口杂物堆放。",
        },
    }
    return payloads.get(
        mode,
        {
            "has_hazard": False,
            "type": str(mode),
            "message": "未发现高危异常。",
        },
    )
