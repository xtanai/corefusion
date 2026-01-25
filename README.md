# CoreFusion

**Multi-stereo fusion and calibration engine for precise 3D hand and gesture key-poses.**

CoreFusion consumes **pre-computed 3D keypoints and ROI data** from multiple **EdgeTrack** stereo rigs over LAN.  
It performs **time alignment, multi-view calibration, bundle adjustment, and low-latency fusion**, and publishes a unified, high-confidence **3D key-pose stream** to downstream clients such as **MotionCoder**.

> **Status:** early prototype (WIP). Interfaces and schemas may evolve.

---

## Why CoreFusion?

* **Scale:** Fuse **2–4 independent stereo rigs** (4–8 cameras) to achieve robust, occlusion-resistant tracking.
* **Precision:** Edge-side stereo reconstruction combined with **multi-view geometry** and **reference features** (AprilTags, wristbands, fingertips) enables stable **metric Z-scale** and mm-level repeatability.
* **Determinism:** Supports **TDMStrobe phase metadata (A/B/C/D)** for deterministic cross-illumination and timing consistency.
* **Low latency:** Fusion operates on **3D keypoints and sparse ROI point clouds only** — no raw video transport required.
* **AI-assisted geometry:** Optional ML-based refinement improves stability in hard cases (fast motion, partial occlusions) and helps auto-tune geometry-related thresholds.
* **Interactive UI & tuning:** Built-in UI for live rig status, calibration workflows, and parameter tuning (filters, gating, outlier rules) without recompiling.
* **Visual inspection:** Optional video/ROI preview and overlay tools to verify alignment, references, and reconstruction quality during setup and debugging.
* **Separation of concerns:**  
  **EdgeTrack** handles capture, synchronization, stereo and pre-processing on the edge;  
  **CoreFusion** centralizes calibration, fusion, filtering, visualization, and system-level configuration.

---

## System Architecture

```
[TDMStrobe] ── TRIG A/B ─► [EdgeTrack #1..#N] ── LAN ─► [CoreFusion] ──► MotionCoder / Apps
                                 │                           │
                     Stereo → 3D keypoints,           Unified 3D key-poses
                     ROI point clouds, refs           (joints + confidence)
```

* **Inputs:** per‑rig streams (keypoints, ROI crops or sparse point clouds, reference features, timestamps, TDM phase IDs).
* **Core:** timebase alignment, outlier rejection, **per‑rig + global calibration**, triangulation, bundle adjust, temporal filtering.
* **Outputs:** unified **3D key‑poses** (with confidences & references), optional diagnostics/metrics.

---

## Features

* **Multi‑rig ingestion**: 3–4 stereo rigs over UDP/ZeroMQ/TCP.
* **Time alignment**: monotonic clock sync + per‑rig latency compensation.
* **Calibration tools**:

  * Intrinsics/extrinsics from **AprilTag L‑brackets** or Charuco panels.
  * One‑click **bundle adjustment** with robust loss and priors (Tag board, wristband, fingertip aids).
* **Fusion & filtering**:

  * Triangulation with uncertainty propagation.
  * Outlier culling (RANSAC/Huber/Tukey), temporal smoothing (Savitzky–Golay/One‑Euro).
  * Confidence gating & joint‑level fallbacks.
* **Low‑latency path**: ROI‑only option; avoids raw video transport.
* **Live monitoring**: web UI for per‑rig status, histograms, residual plots, and frame phase.
* **Exports**: stream to **MotionCoder**, CSV/JSON logs, optional ROS topic.

---

## I/O Schemas

### Inbound (from EdgeTrack)

```json
{
  "rig_id": "rig01",
  "frame_id": 1245678,
  "timestamp_ns": 1731400123456789,
  "phase": "A",

  "net": {
    "transport": "udp",
    "src_ip": "192.168.1.21",
    "src_port": 5555,
    "dst_ip": "192.168.1.10",
    "dst_port": 5557,
    "iface": "eth0",
    "seq": 18342
  },

  "keypoints_3d": {
    "hand": [[0.1,0.2,0.3,0.98]],
    "wrist": [0.0,0.0,0.0,0.99],
    "fingertips": [[0.1,0.2,0.95]]
  },
  "roi_points3d": [[0.01,0.02,0.03,0.9]],
  "refs": {
    "apriltag_corners": [[123.4, 56.7, 17]],
    "board_id": "LBRACKET_FRONT"
  },
  "intrinsics": {"fx": 0, "fy": 0, "cx": 0, "cy": 0, "k": []},
  "extrinsics": {"R": [], "t": []}
}
```

### Outbound (to MotionCoder / clients)

```json
{
  "pose3d": {
    "joints": [
      {"name":"wrist","X":... ,"Y":...,"Z":...,"conf":...},
      {"name":"thumb_tip", ...},
      {"name":"index_tip", ...}
    ],
    "frame_id": 1245678,
    "timestamp_ns": 1731400123456789,
    "rigs_used": ["rig01","rig02","rig03"],
    "quality": {"rms_reproj_px": 0.41, "num_inliers": 56}
  },
  "refs": {"apriltag_board": "LBRACKET_FRONT", "wristband": true},
  "meta": {"phase":"A","latency_ms": 7.8}
}
```

---

## Requirements

* **Host OS:** Linux (Ubuntu 22.04+). Windows and macOS planned.
* **GPU:** NVIDIA CUDA‑capable (e.g., RTX 3060/3080/3090); CPU‑only mode available with reduced throughput
* **Network:** Gigabit Ethernet (up to 4× RJ45 via PCIe/PCI; **direct NIC connections recommended**. **Network switches are not recommended** due to added latency and jitter).
* **Dependencies:**

  * Core: C++/Python, **Eigen**, **ceres‑solver** (BA), **OpenCV**, **Sophus**
  * Optional: **PyTorch** (MMPose), **Anipose**
  * Messaging: **ZeroMQ**/**UDP**/**TCP** (select at build/runtime)

---

## Quick Start

1. **Install deps** (`ceres`, `opencv`, `zeromq`, CUDA optional).
2. Launch **EdgeTrack** on each rig; verify they publish LAN packets.
3. Create a **config.yaml** (see below) and run `CoreFusion --config config.yaml`.
4. Open the **web UI** to confirm rig clocks, phase usage, and residuals.
5. Run **calibration** (tag solve + BA). Save the **global extrinsics**.
6. Start the **Pose stream** → subscribe from **MotionCoder**.

---

## Configuration Example (`config.yaml`)

```yaml
fusion:
  rigs: [rig01, rig02, rig03, rig04]
  transport: zeromq   # udp|tcp|zeromq
  max_latency_ms: 30
  outlier: { method: huber, delta: 1.0 }
  smoothing: { method: one_euro, min_cutoff: 1.0, beta: 0.05 }
  output:
    fps: 120
    endpoint: tcp://0.0.0.0:5557

rigs:
  rig01:
    subscribe: tcp://rig01.local:5555
    time_offset_ms: 0
    weight: 1.0
  rig02:
    subscribe: tcp://rig02.local:5555
    time_offset_ms: -2.3
    weight: 1.0
  rig03:
    subscribe: tcp://rig03.local:5555
    time_offset_ms: 1.1
    weight: 0.9

calibration:
  tag_family: apriltag_36h11
  board: LBRACKET_FRONT
  optimize: { intrinsics: false, extrinsics: true, scale: true }
  priors:
    wristband: true
    fingertips: true
```

---

## CLI

```
CoreFusion --config config.yaml \
            --log-level info \
            --web-ui 0.0.0.0:8080 \
            --record /data/sessions/run_001
```

**Subcommands**

* `calibrate`: solve tag board → init extrinsics → run BA
* `bench`: synthetic timing/latency & throughput test
* `replay`: ingest recorded LAN stream for offline tuning

---

## Performance Notes

* Prefer **edge-side stereo reconstruction** and ROI extraction to minimize bandwidth.
* Maintain **consistent TDM phases (A/B/C/D)** across rigs to avoid cross-lit frames.
* For 120 fps pipelines, target **< 10 ms end-to-end latency** (edge → fusion → client) on RTX-class GPUs.

---

## Roadmap

* Per‑rig **latency auto‑cal** via cross‑correlation.
* Multi‑rig **auto‑scale alignment** using redundant tag boards.
* IMU fusion hooks (wrist/hand) and per‑joint uncertainty output.
* ROS 2 bridge; gRPC streaming API.
* GUI tools for tag board layout & calibration reports.

---

## License

**Apache‑2.0** (code, docs). Calibration boards/mechanics may use **CERN‑OHL‑S**.

---

## Safety

CoreFusion processes data only, but it assumes a capture setup using **NIR illumination**. Follow the project’s **Safety** guidance (IEC 62471, baffling, interlocks). When in doubt, reduce pulse width/duty and shield emitters.
