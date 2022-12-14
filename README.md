# Introduction
This is a package for run arducam in ros. It has a efficient nodelet mode.

## Usage
```bash
$ roslaunch arducam_ros arducam.launch show:=true publish_split:=true
```

If enable show, it will display the image in a window.
If enable publish_split, it will publish the splited images, this is for quad-camera.

## Note
This is a third-party package, it is not maintained by the manufacturer. If you have any questions in hardware, please DO NOT contact me.

## LICENSE
LGPL-V3
