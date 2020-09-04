# Dependencies, and how they are installed on Jetson Nano

cmake, from apt
opencv, compiled from source, Version 4.3.0, GPU enabled
eigen, Installed from source, Version 3.3.7
opengv, compiled, no releases on this project
liblas from apt "liblas-c-dev" and "liblas-dev"
yaml-cpp, compiled from source, Version 0.6.3
Drivers from arducam camera (see their website)
v4l-utils, from apt, "v4l-utils"


Add the following to "/etc/udev/rules.d/99-com.rules"

```
SUBSYSTEM=="gpio", GROUP="gpio", MODE="0660"
```

Also, add your user to the "gpio" group

```
sudo usermod -aG gpio <user>
```

By default, the nvidia process "nvgetty" is taking the serial port we need (nvgetty provides a serial terminal). We need to disable that process permanently to get access to the port. Note that this will prevent you from using the serial console on this port.

```
sudo systemctl disable nvgetty
sudo usermod -aG dialout <user>
```

Always reboot after adding your user to groups

