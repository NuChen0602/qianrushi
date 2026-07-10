class QwenVLClient:
    """Reserved visual API client.

    The first demo scaffold must not call the network. Later, migrate the
    real image encoding and Qwen-VL request logic from the E08 vision project
    into this class.
    """

    def __init__(self, api_key=None, base_url=None, model="qwen3-vl-flash"):
        self.api_key = api_key
        self.base_url = base_url
        self.model = model

    def analyze_image(self, image_path, mode):
        return {
            "ok": True,
            "stub": True,
            "model": self.model,
            "mode": mode,
            "image_path": str(image_path),
            "message": "视觉 API 当前为占位结果，未调用真实网络接口。",
        }
