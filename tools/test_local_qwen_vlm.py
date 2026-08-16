#!/usr/bin/env python3
"""Smoke-test the local OmniLRS Qwen3-VL terrain classifier without ROS."""
import argparse
import base64
import json
import os
import re

import cv2
import httpx
from openai import OpenAI


DEFAULT_BASE_URL = "http://127.0.0.1:22002/v1"
DEFAULT_MODEL = "/home/lry/OmniLRS/deploy/qwen3vl/models/Qwen3-VL-8B-Instruct-AWQ-4bit"


def encode_image(path):
    image = cv2.imread(path)
    if image is None:
        raise FileNotFoundError(path)
    ok, buffer = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    if not ok:
        raise RuntimeError(f"Failed to encode image: {path}")
    return base64.b64encode(buffer.tobytes()).decode("utf-8")


def parse_json(text):
    text = text.strip()
    if text.startswith("```") and text.endswith("```"):
        text = text.strip("`").strip()
    if text.startswith("json"):
        text = text[len("json"):].strip()
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        match = re.search(r"\{.*\}", text, flags=re.DOTALL)
        if not match:
            raise
    return json.loads(match.group(0))


def normalize_score(value):
    if isinstance(value, (int, float)):
        return max(0.0, min(1.0, float(value)))
    if not isinstance(value, str):
        return None

    text = value.strip().lower()
    if any(token in text for token in ["none", "no ", "minimal", "flat", "few", "sparse", "gentle", "low"]):
        return 0.15
    if any(token in text for token in ["moderate", "medium", "some"]):
        return 0.5
    if any(token in text for token in ["dense", "many", "high", "steep", "severe", "challenging"]):
        return 0.85
    return None


def normalize_result(data):
    rock_distribution = normalize_score(data.get("rock_distribution"))
    slope = normalize_score(data.get("slope"))
    terrain_complexity = str(data.get("terrain_complexity", "")).strip().lower()

    if terrain_complexity not in {"flat", "rocky", "challenging"}:
        if slope is not None and slope >= 0.55:
            terrain_complexity = "challenging"
        elif rock_distribution is not None and rock_distribution >= 0.35:
            terrain_complexity = "rocky"
        else:
            terrain_complexity = "flat"

    data["rock_distribution"] = rock_distribution
    data["slope"] = slope
    data["terrain_complexity"] = terrain_complexity
    return data


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "image",
        nargs="?",
        default="docs/picture/perception/rough_env.png",
        help="local terrain image to classify",
    )
    parser.add_argument("--base-url", default=os.getenv("QWEN_VL_BASE_URL", DEFAULT_BASE_URL))
    parser.add_argument("--model", default=os.getenv("QWEN_VL_MODEL", DEFAULT_MODEL))
    parser.add_argument("--api-key", default=os.getenv("QWEN_VL_API_KEY", "EMPTY"))
    args = parser.parse_args()

    client = OpenAI(
        api_key=args.api_key,
        base_url=args.base_url,
        timeout=120.0,
        http_client=httpx.Client(timeout=120.0, trust_env=False),
    )
    image_b64 = encode_image(args.image)
    response = client.chat.completions.create(
        model=args.model,
        temperature=0.1,
        max_tokens=256,
        messages=[
            {
                "role": "system",
                "content": (
                    "You are analyzing rover terrain. Return ONLY JSON with keys "
                    "rock_distribution, slope, terrain_complexity, explanation. "
                    "rock_distribution and slope MUST be numeric floats from 0.0 to 1.0. "
                    "terrain_complexity must be flat, rocky, or challenging."
                ),
            },
            {
                "role": "user",
                "content": [
                    {
                        "type": "image_url",
                        "image_url": {"url": f"data:image/jpeg;base64,{image_b64}"},
                    },
                    {
                        "type": "text",
                        "text": "Classify this terrain for planetary rover navigation.",
                    },
                ],
            },
        ],
    )
    raw = response.choices[0].message.content
    print(json.dumps(normalize_result(parse_json(raw)), indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
