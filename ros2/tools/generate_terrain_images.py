"""Generate unambiguous synthetic terrain images for the VLM classifier.

The paper's real Mars-Yard photos are ambiguous to the local Qwen3-VL model
(``rough_env.png`` is read as *flat*, ``hazard_env.png`` as *rocky*), so the
multi-mode demo never left Mode 1.  These synthetic images are engineered to be
unmistakable so the VLM reliably maps flat -> Mode 1, rocky -> Mode 2, and
challenging -> Mode 3.

Each is written to ``docs/picture/perception/synthetic/<name>.png`` and can be
classified by ``tools/test_local_qwen_vlm.py``.
"""
import os

import cv2
import numpy as np

OUT_DIR = "/home/lry/mars_navigation/docs/picture/perception/synthetic"


def _sandy_background(h, w, seed):
    rng = np.random.default_rng(seed)
    y, x = np.mgrid[0:h, 0:w].astype(np.float32)
    # gentle large-scale dune undulation + fine noise
    base = 150.0 + 18.0 * np.sin(x / 120.0 + seed) + 14.0 * np.cos(y / 90.0 - seed)
    fine = rng.normal(0.0, 4.0, (h, w)).astype(np.float32)
    return np.clip(base + fine, 30, 235).astype(np.uint8)


def flat(h=512, w=512):
    """Uniform sandy surface, almost no features."""
    gray = _sandy_background(h, w, 7).astype(np.uint8)
    # very subtle horizontal texture only
    gray = cv2.GaussianBlur(gray, (0, 0), 2.0)
    return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)


def rocky(h=512, w=512):
    """Flat ground densely scattered with dark rocks of many sizes."""
    gray = _sandy_background(h, w, 11)
    img = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR).astype(np.float32)
    rng = np.random.default_rng(11)
    n = 140
    for _ in range(n):
        cx = int(rng.uniform(0, w))
        cy = int(rng.uniform(0, h))
        r = rng.uniform(5, 26)
        col = rng.uniform(20, 70)  # dark rock
        cv2.circle(img, (cx, cy), int(r), (col, col, col), -1, lineType=cv2.LINE_AA)
        # small highlight + shadow for a 3D rock look
        cv2.circle(img, (cx - int(r * 0.25), cy - int(r * 0.25)), max(1, int(r * 0.5)),
                   (col * 1.6, col * 1.6, col * 1.6), -1, lineType=cv2.LINE_AA)
    return np.clip(img, 0, 255).astype(np.uint8)


def challenging(h=512, w=512):
    """Steep hillside: strong diagonal elevation gradient + contour ridges + rocks.

    The diagonal brightness gradient and contour lines cue *slope*, and the
    scattered rocks cue *rocky* -- together the VLM reads this as
    ``challenging`` (elevation changes *and* rocks), not merely rocky.
    """
    rng = np.random.default_rng(99)
    y, x = np.mgrid[0:h, 0:w].astype(np.float32)
    grad = 0.7 * (x / w) + 0.5 * (y / h)
    gray = np.clip(210.0 - 180.0 * grad + rng.normal(0.0, 4.0, (h, w)), 20, 230).astype(np.uint8)
    img = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR).astype(np.float32)
    # contour ridges running perpendicular to the slope
    for k in range(1, 8):
        p0 = (0, int(k * h / 8 - 50))
        p1 = (int(k * w / 8 + 50), 0)
        cv2.line(img, p0, p1, (30, 30, 30), 2)
    for _ in range(30):
        cx = int(rng.uniform(0, w))
        cy = int(rng.uniform(0, h))
        r = rng.uniform(4, 16)
        col = rng.uniform(20, 70)
        cv2.circle(img, (cx, cy), int(r), (col, col, col), -1, lineType=cv2.LINE_AA)
    return np.clip(img, 0, 255).astype(np.uint8)


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    for name, fn in (("flat", flat), ("rocky", rocky), ("challenging", challenging)):
        img = fn()
        path = os.path.join(OUT_DIR, f"{name}.png")
        cv2.imwrite(path, img)
        g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        print(f"wrote {path}  ({img.shape[1]}x{img.shape[0]}, gray mean={g.mean():.0f} std={g.std():.0f})")


if __name__ == "__main__":
    main()
