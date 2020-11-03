# Usage
Can detect yellow and blue. Works with yellow duckies

Build it on the laptop. Assuming in the root directory of the repo
```
dts devel build -f 
```

Running it on the laptop while recieving messages from the duckiebot
```
docker run --rm --network host -e VEHICLE_NAME="duckiebot name" -e ROS_MASTER_URI="Master URI on duckiebot" -e ROS_IP="laptop ip" <name of the built image>
```

Publishes the debug images to the topic `~color_detector_node/debug_images/compressed`

Change the detection color using `rosparam set <vehicle name>/color_detector_node/color <color>` to `"yellow"` or `"blue"`. Or modify the config file before launching the node.
