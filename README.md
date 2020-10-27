# Usage

Turn off duckiebot interface containers first to use camera

Can detect yellow and blue. Works with yellow duckies

Build it on the duckiebot. Assuming in the root directory of the repo
```
devel build -f -H <duckiebot name>.local
```

Running it on the duckiebot
```
docker -H <duckiebot name>.local run --rm --privileged --network host <name of the built image>
```

Publishes the debug images to the topic `~color_detector_node/debug_images/compressed`

Change the detection color using `rosparam set <vehicle name>/color_detector_node/color <color>` to `"yellow"` or `"blue"`