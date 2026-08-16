# Reproduction metrics

## Isaac Sim physical closed loop (final form, 2026-08-16)

Start (-20.0, 0.0) -> goal (20.0, 0.0), arrival radius 2.0 m, on the
synthesised lunar heightmap (crater + boulders). Real rigid-body physics in
Isaac Sim 5.1.0 (Moon gravity 1.62 m/s²), OmniLRS LunarRegolith8k material +
sun lighting, collidable rocks, in-scene path-line visualization, and HUD
(mode / speed / VLM Q&A / mode-switch history) recorded into the videos.
Rollover is a **real physical backflip** detected by Isaac
(`tilt = acos(cos·roll·cos·pitch) > 60°`), not a slope lookup. Mode speeds
rescaled to 0.32 / 0.20 / 0.15 m/s (Leo wheel-velocity cap 0.375 m/s).

| run | success | reason | time | distance | avg speed |
| --- | --- | --- | --- | --- | --- |
| mode1 (efficient, straight) | ❌ | rollover @ boulder | 53.2 s | 10.5 m | 0.197 m/s |
| mode2 (rock-safe, slope-blind) | ❌ | rollover @ crater rim | 130.1 s | 14.7 m | 0.113 m/s |
| mode3 (slope+roughness cost) | ✅ | goal reached | 468.2 s | 42.3 m | 0.090 m/s |
| vlm (adaptive Mode1↔Mode3) | ✅ | goal reached | 287.7 s | 43.6 m | 0.152 m/s |

VLM drives Mode 1 (0.32 m/s) across the flat ~40 m, drops to the conservative
Mode 3 only around the crater rim (~3.5 m), and reaches the goal **~38% faster
than Mode 3** (287.7 vs 468.2 s) while surviving like Mode 3 — the paper's
adaptive multi-mode efficiency claim, reproduced with real physics. Videos and
per-frame PNGs under `ros2/runs_isaac/<mode>/` (`cam.mp4`, `orbit.mp4`,
`record/cam/*.png`, `record/orbit/*.png`).

### VLM multi-mode per-mode breakdown (Isaac run)

```
  Mode 1     time=248.9s (87%)  distance=39.9m (91%)
  Mode 2     time=2.2s   (1%)   distance=0.2m  (0%)
  Mode 3     time=38.1s  (13%)  distance=3.5m  (8%)
```

---

## Kinematic smoke test (earlier reference, slope-lookup rollover)

Start (-20.0, 0.0) -> goal (20.0, 0.0), arrival radius 2.0 m.

A slope-based rollover model differentiates the modes by *capability*
(Mode 1 limit 8 deg, Mode 2 limit 12 deg, Mode 3 limit 40 deg).  The
~15.8 deg ridge is the capability-differentiating terrain: Mode 1 and
Mode 2 roll over on it, while Mode 3 -- and the VLM multi-mode system,
which switches to Mode 3 before the ridge -- cross it successfully.

| run | success | reason | time | distance | avg speed |
| --- | --- | --- | --- | --- | --- |
| mode1 | ❌ | rollover | 7.70 s | 14.96 m | 1.943 m/s |
| mode2 | ❌ | rollover | 20.20 s | 15.77 m | 0.781 m/s |
| mode3 | ✅ | goal reached | 79.20 s | 39.07 m | 0.493 m/s |
| vlm | ✅ | goal reached | 42.44 s | 38.53 m | 0.908 m/s |

## VLM multi-mode per-mode breakdown

```
  Mode 1     time=7.00s (16%)  distance=12.23m (32%)
  Mode 2     time=6.88s (16%)  distance=5.56m (14%)
  Mode 3     time=24.07s (57%)  distance=11.98m (31%)
```