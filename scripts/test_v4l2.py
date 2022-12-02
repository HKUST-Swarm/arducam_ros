#!/usr/bin/python3
# from v4l2py import Device
# cam = Device.from_id(0)
# for i, frame in enumerate(cam):
#     print(f"frame #{i}: {len(frame)} bytes")
#     if i > 9:
#         break
import v4l2
import fcntl
vd = open('/dev/video0', 'r')
cp = v4l2.v4l2_capability()
fcntl.ioctl(vd, v4l2.VIDIOC_QUERYCAP, cp)
print(cp.card)