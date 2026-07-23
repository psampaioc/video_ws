# Drone Vision Pipeline

This repository contains a ROS 2 visual telemetry extraction system. It ingests video feeds (via V4L2 hardware or `.mcap` datasets), isolates specific GUI regions (Latitude/Longitude), and performs CPU-based OCR inference using ONNX Runtime to publish structured geographical data.

## Architecture & Evolution

### V1: Modular Approach (Legacy)

* **Structure:** Separated into two distinct ROS 2 nodes: one for hardware video capture/drive and another for cropping and OCR inference.
* **Why:** The initial intention was strict separation of concerns—isolating hardware I/O from heavy neural network computations.
* **The Problem:** Passing raw, uncompressed video frames across ROS 2 topics introduced unnecessary Inter-Process Communication (IPC) overhead and latency.

### V2: Unified Approach (Active)

* **Structure:** A single, event-driven node (`unified_vision_node`).
* **Why:** Designed to eliminate the IPC bottleneck. Frame capture, geometric cropping, and OCR inference now happen sequentially in the same memory space. This "zero-copy" approach enforces the KISS principle, dramatically reduces latency, and ensures inference only triggers exactly when a new frame is available.

## Usage Guides

Detailed technical documentation, parameter configurations, and troubleshooting steps are maintained separately for each architecture.

* **For V1:** Refer to the guide inside the `V1/` directory.
* **For V2:** Refer to the `guide.md` inside the `V2/` directory for the active production setup.
