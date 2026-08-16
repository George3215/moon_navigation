"""Iteratively tune a synthetic 'challenging' image until the VLM calls it
challenging (not rocky).  Writes candidate PNGs and classifies them via the
local OmniLRS endpoint."""
import base64
import json
import os
import re

import cv2
import numpy as np
from openai import OpenAI

OUT = "/home/lry/mars_navigation/docs/picture/perception/synthetic"
BASE_URL = "http://127.0.0.1:22002/v1"
MODEL = "/home/lry/OmniLRS/deploy/qwen3vl/models/Qwen3-VL-8B-Instruct-AWQ-4bit"


def slope_rocks(n_rocks, slope_strength, ridges=6):
    h = w = 512
    rng = np.random.default_rng(int(1000 + n_rocks * 10 + slope_strength * 100))
    y, x = np.mgrid[0:h, 0:w].astype(np.float32)
    grad = slope_strength * (x / w) + 0.4 * (y / h)
    gray = np.clip(215.0 - 190.0 * grad + rng.normal(0, 5, (h, w)), 20, 235).astype(np.uint8)
    img = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR).astype(np.float32)
    # ridge / contour lines running perpendicular to the slope (diagonal)
    for k in range(1, ridges + 1):
        t = k / (ridges + 1)
        x0, y0 = int(t * w), 0
        x1, y1 = 0, int(t * h)
        cv2.line(img, (x0, y0), (x1, y1), (30, 30, 30), 2, lineType=cv2.LINE_AA)
    for _ in range(n_rocks):
        cx = int(rng.uniform(0, w))
        cy = int(rng.uniform(0, h))
        r = rng.uniform(4, 14)
        c = rng.uniform(20, 70)
        cv2.circle(img, (cx, cy), int(r), (c, c, c), -1, lineType=cv2.LINE_AA)
    return np.clip(img, 0, 255).astype(np.uint8)


def classify(path):
    for _k in list(os.environ):
        if _k.lower().endswith("proxy"):
            os.environ.pop(_k, None)
    img = cv2.imread(path)
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    b64 = base64.b64encode(buf.tobytes()).decode()
    client = OpenAI(api_key="EMPTY", base_url=BASE_URL, timeout=120.0)
    sys = (
        "You are an AI assistant analyzing terrain for rover navigation.\n"
        "Terrain Categories:\n1. Flat: few obstacles and minimal elevation changes\n"
        "2. Rocky: rocks and obstacles but no significant slopes\n"
        "3. Challenging: elevation changes and rocks\n"
        'Output: ONLY JSON {"rock_distribution": 0.0, "slope": 0.0, '
        '"terrain_complexity": "flat|rocky|challenging", "explanation": "..."}'
    )
    r = client.chat.completions.create(
        model=MODEL, temperature=0.1, max_tokens=256,
        messages=[{"role": "system", "content": sys},
                  {"role": "user", "content": [{"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{b64}"}},
                                               {"type": "text", "text": "Analyze this rover terrain image."}]}],
    )
    raw = r.choices[0].message.content
    m = re.search(r"\{.*\}", raw, re.DOTALL)
    return json.loads(m.group(0)) if m else {"raw": raw}


def main():
    candidates = [
        (15, 1.0, 8, "c1"),
        (20, 1.2, 6, "c2"),
        (8, 1.2, 10, "c3"),
        (12, 1.5, 8, "c4"),
    ]
    for n, s, ridges, tag in candidates:
        img = slope_rocks(n, s, ridges)
        path = f"{OUT}/challenging_{tag}.png"
        cv2.imwrite(path, img)
        d = classify(path)
        print(f"{tag}: rocks={n} slope_strength={s} ridges={ridges} -> "
              f"complexity={d.get('terrain_complexity')} rock={d.get('rock_distribution')} slope={d.get('slope')} | {d.get('explanation','')[:60]}")


if __name__ == "__main__":
    main()
