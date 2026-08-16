"""VLM terrain classifier (ROS2 port of perception/terrain_classification/scripts/VLM_ros_depth_topic.py).

Subscribes to an RGB image topic, sends the frame to a local OpenAI-compatible
VLM endpoint (the OmniLRS Qwen3-VL vLLM server), parses the JSON classification,
and publishes the selected navigation mode on ``/navigation_mode``.

Endpoint defaults target the local OmniLRS deployment:

    QWEN_VL_BASE_URL=http://127.0.0.1:22002/v1
    QWEN_VL_MODEL=/home/lry/OmniLRS/deploy/qwen3vl/models/Qwen3-VL-8B-Instruct-AWQ-4bit

In topic mode the classifier is event-driven: it runs the moment a new terrain
image arrives (see ``_image_cb``), so a mode switch tracks the segment boundary
with only VLM inference latency.  ``evaluation_interval`` still drives the
standalone ``image_file`` fallback.
"""

import base64
import json
import os
import re

import cv2
import numpy as np
import rclpy
from openai import OpenAI
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

DEFAULT_BASE_URL = "http://127.0.0.1:22002/v1"
DEFAULT_MODEL = "/home/lry/OmniLRS/deploy/qwen3vl/models/Qwen3-VL-8B-Instruct-AWQ-4bit"

SYSTEM_PROMPT = (
    "You are an AI assistant analyzing terrain for rover navigation.\n"
    "Terrain Categories:\n"
    "1. Flat: Terrain with few obstacles and minimal elevation changes\n"
    "2. Rocky: Terrain with rocks and obstacles but no significant slopes\n"
    "3. Challenging: Terrain with elevation changes and rocks\n"
    "Evaluation Criteria:\n"
    "1. rock_distribution MUST be a numeric float from 0.0 to 1.0 for obstacle complexity\n"
    "2. slope MUST be a numeric float from 0.0 to 1.0 for elevation complexity\n"
    "Output: Return ONLY a JSON object with this schema: "
    '{"rock_distribution": 0.0, "slope": 0.0, '
    '"terrain_complexity": "flat|rocky|challenging", "explanation": "brief reason"}'
)

USER_TEXT = (
    "Analyze this rover terrain image. Evaluate rock distribution and slope, "
    "then classify as flat, rocky, or challenging."
)


def parse_vlm_json(text):
    text = text.strip()
    if text.startswith("```") and text.endswith("```"):
        text = text.strip("`").strip()
    if text.startswith("json"):
        text = text[len("json"):].strip()
    if not text:
        raise ValueError("Response content is empty")
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        match = re.search(r"\{.*\}", text, flags=re.DOTALL)
        if not match:
            raise
        return json.loads(match.group(0))


def normalize_score(value, fallback=None):
    if isinstance(value, bool):
        return float(value)
    if isinstance(value, (int, float)):
        return max(0.0, min(1.0, float(value)))
    if not isinstance(value, str):
        return fallback
    text = value.strip().lower()
    if any(t in text for t in ["none", "no ", "minimal", "flat", "few", "sparse", "gentle", "low"]):
        return 0.15
    if any(t in text for t in ["moderate", "medium", "some"]):
        return 0.5
    if any(t in text for t in ["dense", "many", "high", "steep", "severe", "challenging"]):
        return 0.85
    return fallback


def normalize_terrain_complexity(value, rock_distribution, slope):
    if isinstance(value, str) and value.strip().lower() in {"flat", "rocky", "challenging"}:
        return value.strip().lower()
    if slope is not None and slope >= 0.55:
        return "challenging"
    if rock_distribution is not None and rock_distribution >= 0.35:
        return "rocky"
    return "flat"


class VlmMode(Node):
    def __init__(self):
        super().__init__("vlm_mode")
        self.declare_parameter("base_url", os.getenv("QWEN_VL_BASE_URL", DEFAULT_BASE_URL))
        self.declare_parameter("api_key", os.getenv("QWEN_VL_API_KEY", "EMPTY"))
        self.declare_parameter("model", os.getenv("QWEN_VL_MODEL", DEFAULT_MODEL))
        self.declare_parameter("temperature", 0.1)
        self.declare_parameter("max_tokens", 256)
        self.declare_parameter("timeout", 120.0)
        self.declare_parameter("image_topic", "/terrain/image")
        self.declare_parameter("mode_topic", "/navigation_mode")
        self.declare_parameter("qa_topic", "/vlm/qa")
        self.declare_parameter("evaluation_interval", 2.0)
        # Standalone fallback: if True, classify this image file on an interval
        # instead of waiting for an image topic (no camera available).
        self.declare_parameter("image_file", "")

        # The OmniLRS endpoint is on localhost; ignore any user proxy settings
        # (e.g. a ``socks://`` ALL_PROXY). httpx otherwise tries to parse the
        # proxy URL and fails on the socks scheme before honouring no_proxy.
        for _k in list(os.environ):
            if _k.lower().endswith("proxy"):
                os.environ.pop(_k, None)

        self.client = OpenAI(
            api_key=self.get_parameter("api_key").value,
            base_url=self.get_parameter("base_url").value,
            timeout=float(self.get_parameter("timeout").value),
        )
        self.get_logger().info(f"VLM endpoint: {self.get_parameter('base_url').value}")
        self.get_logger().info(f"VLM model: {self.get_parameter('model').value}")

        self.current_bgr = None
        self.create_subscription(Image, self.get_parameter("image_topic").value, self._image_cb, 1)
        self.mode_pub = self.create_publisher(String, self.get_parameter("mode_topic").value, 1)
        self.qa_pub = self.create_publisher(String, self.get_parameter("qa_topic").value, 1)

        self.image_file = self.get_parameter("image_file").value
        # Topic mode is *event-driven*: classify the moment a new terrain image
        # arrives, so a mode switch tracks the segment boundary with only the
        # VLM inference latency (~1 s) and no timer round-trip lag.  A 2 s timer
        # here could leave the rover in Mode 2 for a full extra interval after
        # it crossed into the challenging segment -- enough to roll over on the
        # ridge before Mode 3 engages.  The timer survives only for the
        # standalone ``image_file`` fallback.
        if self.image_file:
            self.timer = self.create_timer(float(self.get_parameter("evaluation_interval").value), self._evaluate)
        else:
            self.timer = None
        self.get_logger().info("vlm_mode ready")

    def _image_cb(self, msg):
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8)
            arr = arr.reshape(msg.height, msg.width, 3)
            self.current_bgr = arr[:, :, ::-1].copy()  # rgb8 -> bgr
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"image conversion error: {exc}")
            return
        # Classify immediately on new imagery.  The VLM call is blocking, but
        # images arrive only on zone changes (seconds apart), so this is fine.
        if not self.image_file:
            self._evaluate()

    def _evaluate(self):
        bgr = None
        if self.image_file:
            bgr = cv2.imread(self.image_file)
            if bgr is None:
                self.get_logger().warn(f"cannot read image file {self.image_file}")
                return
        elif self.current_bgr is not None:
            bgr = self.current_bgr
        else:
            self.get_logger().info("waiting for terrain image...", throttle_duration_sec=10.0)
            return

        ok, buffer = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        if not ok:
            self.get_logger().error("failed to encode image")
            return
        b64 = base64.b64encode(buffer.tobytes()).decode("utf-8")

        try:
            response = self.client.chat.completions.create(
                model=self.get_parameter("model").value,
                temperature=float(self.get_parameter("temperature").value),
                max_tokens=int(self.get_parameter("max_tokens").value),
                messages=[
                    {"role": "system", "content": SYSTEM_PROMPT},
                    {
                        "role": "user",
                        "content": [
                            {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{b64}"}},
                            {"type": "text", "text": USER_TEXT},
                        ],
                    },
                ],
            )
        except Exception as exc:  # noqa: BLE001 - endpoint down / timeout
            self.get_logger().error(f"VLM request failed: {exc}")
            return

        raw = response.choices[0].message.content
        try:
            data = parse_vlm_json(raw)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"failed to parse VLM JSON: {exc}; raw={raw!r}")
            return

        rock = normalize_score(data.get("rock_distribution"))
        slope = normalize_score(data.get("slope"))
        complexity = normalize_terrain_complexity(data.get("terrain_complexity"), rock, slope)

        mode = {"flat": "Mode 1", "rocky": "Mode 2", "challenging": "Mode 3"}.get(complexity, "Mode 1")
        msg = String()
        msg.data = mode
        self.mode_pub.publish(msg)
        qa_msg = String()
        qa_msg.data = json.dumps(
            {
                "question": USER_TEXT,
                "answer_raw": raw,
                "terrain_complexity": complexity,
                "rock_distribution": rock,
                "slope": slope,
                "mode": mode,
                "explanation": data.get("explanation", ""),
            },
            ensure_ascii=False,
        )
        self.qa_pub.publish(qa_msg)
        self.get_logger().info(
            f"terrain={complexity} (rock={rock:.2f}, slope={slope:.2f}) -> {mode}"
        )


def main():
    rclpy.init()
    node = VlmMode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
