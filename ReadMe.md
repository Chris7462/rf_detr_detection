# RF-DETR Object Detection ROS2 Node

ROS2 wrapper around [`rf_detr_trt_backend`](https://github.com/Chris7462/rf_detr_trt_backend), running RF-DETR object detection on live or recorded camera streams via a TensorRT engine.

The node subscribes to an image topic, letterbox-pads each frame to the model's square input size, runs inference through the backend, and maps detections back to the original image's coordinates before publishing.

## Dependencies

- ROS2 (tested with Lyrical)
- [`rf_detr_trt_backend`](https://github.com/Chris7462/rf_detr_trt_backend) — must be built first; this package links against it and expects a compiled `.engine` file under its `share/engines` directory
- `rclcpp`, `sensor_msgs`, `vision_msgs`, `ament_index_cpp`, `cv_bridge`

## Build

```bash
cd ~/kitti_ws
colcon build --packages-select rf_detr_detection
source install/setup.bash
```

## Run

### KITTI data (raw bag)
```bash
ros2 launch rf_detr_detection rf_detr_detection_kitti_launch.py
```

### CARLA simulator
```bash
ros2 launch rf_detr_detection rf_detr_detection_carla_launch.py
```

This plays `/data/kitti/raw/2011_09_29_drive_0071_sync_bag` at real-time rate; update the bag path in `launch/rf_detr_detection_kitti_launch.py` to point at your own data.

## Parameters

Set in `param/rf_detr_detection.yaml`:

| Parameter | Default | Description |
|---|---|---|
| `input_topic` | `kitti/camera/color/left/image_raw` | Input image topic |
| `output_topic` | `rf_detr_detection` | `vision_msgs/Detection2DArray` output topic |
| `output_overlay_topic` | `rf_detr_detection_overlay` | Annotated image output topic (only published while subscribed) |
| `queue_size` | `5` | QoS history depth |
| `processing_frequency` | `50.0` | Inference timer rate (Hz) |
| `max_processing_queue_size` | `3` | Bounded input queue; oldest frame is dropped when full |
| `engine_package` | `rf_detr_trt_backend` | Package the engine file is resolved from |
| `engine_filename` | `rf_detr_large_704x704.engine` | Engine file name, relative to `<engine_package>/share/engines` |
| `score_threshold` | `0.5` | Minimum detection confidence |
| `warmup_iterations` | `2` | Engine warmup passes at startup |
| `log_level` | `2` | TensorRT log verbosity (0: Internal Error, 1: Error, 2: Warning, 3: Info, 4: Verbose) |

Model input size, decoder query count, and foreground class count are **not** parameters — `RFDetrTrtBackend` reads them directly from the loaded `.engine` file's own tensor shapes at construction time, and logs the resolved values at startup. This means they can never drift out of sync with whatever `engine_filename` actually points to.

To run against a fine-tuned checkpoint (e.g. a KITTI 3-class model), just point `engine_filename` at it.

## Topics

**Subscribes**

- `<input_topic>` (`sensor_msgs/Image`, BGR8) - camera frames

**Publishes**

- `<output_topic>` (`vision_msgs/Detection2DArray`) - detections in the input image's original pixel coordinates
- `<output_overlay_topic>` (`sensor_msgs/Image`) - input image with detection boxes and labels drawn on top

Both publishers are subscription-gated: the overlay image is only rendered and published while something is subscribed to it, and detections are only serialized when `<output_topic>` has subscribers.

## Notes

- Frames are letterbox-padded (not cropped or stretched) to the model's square input, so non-square sources like KITTI's 1242x375 images are handled correctly; detections are unletterboxed back to the source image's coordinates before publishing.
- The node runs on ROS2's `EventsCBGExecutor` with a single reentrant callback group.
