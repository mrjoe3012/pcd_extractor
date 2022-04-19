## pcd_extractor
This is a ros2 utility package which listens for PointCloud2 messages on a topic and outputs each one to a .pcd file.

## usage
after building and sourcing your local installation, execute the following command to run the package


```bash
ros2 run pcd_extractor pcd_extractor --ros-args -p topic:="/topic/name/here" -p output:="./output/directory/here"
```