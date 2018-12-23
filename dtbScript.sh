#!/bin/sh
echo "Copy mcp23016.dtbo"
scp ~/eclipse-workspace-drivers-OPI/driver-mcp23016/DTS/mcp23016.dtbo root@192.168.0.103:/boot/overlay-user
echo "Copy mcp23016.ko"
scp ~/eclipse-workspace-drivers-OPI/driver-mcp23016/mcp23016.ko root@192.168.0.103:/lib/modules/4.14.78-sunxi/kernel/drivers/gpio
echo "Reboot"
ssh root@192.168.0.103 'reboot'