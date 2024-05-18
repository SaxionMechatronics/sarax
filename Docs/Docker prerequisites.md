# Docker Prerequisites
To allow effective use of the Docker container, we can utilise dedicated graphics to improve performance of GUI applications.

## Installing NVIDIA drivers (Host)
If you are running on Ubuntu. Install NVIDIA drivers from the [PPA GPU drivers repository](https://launchpad.net/~graphics-drivers/+archive/ubuntu/ppa).

1. Add the PPA repository
```shell
sudo add-apt-repository ppa:graphics-drivers/ppa
```
2. Identify the GPU model and available drivers
```shell
ubuntu-drivers devices
```

The output should be similar to this
<details>
  <summary>
  Devices Output
  </summary>

```shell
== /sys/devices/pci0000:00/0000:00:01.0/0000:01:00.0 ==
modalias : pci:v000010DEd00001F91sv00001043sd0000106Fbc03sc00i00
vendor   : NVIDIA Corporation
model    : TU117M [GeForce GTX 1650 Mobile / Max-Q]
driver   : nvidia-driver-515 - third-party non-free
driver   : nvidia-driver-535-server - distro non-free
driver   : nvidia-driver-450-server - distro non-free
driver   : nvidia-driver-545-open - third-party non-free
driver   : nvidia-driver-460 - third-party non-free
driver   : nvidia-driver-465 - third-party non-free
driver   : nvidia-driver-418-server - distro non-free
driver   : nvidia-driver-525 - third-party non-free
driver   : nvidia-driver-545 - third-party non-free
driver   : nvidia-driver-520 - third-party non-free
driver   : nvidia-driver-535-open - distro non-free
driver   : nvidia-driver-535-server-open - distro non-free
driver   : nvidia-driver-495 - third-party non-free
driver   : nvidia-driver-450 - third-party non-free
driver   : nvidia-driver-455 - third-party non-free
driver   : nvidia-driver-535 - third-party non-free
driver   : nvidia-driver-550 - third-party non-free recommended # <-- Find the recommended driver
driver   : nvidia-driver-470-server - distro non-free
driver   : nvidia-driver-550-open - third-party non-free
driver   : nvidia-driver-470 - third-party non-free
driver   : nvidia-driver-510 - third-party non-free
driver   : xserver-xorg-video-nouveau - distro free builtin
```
</details>

3. Install the NVIDIA driver
```shell
sudo apt install nvidia-driver-550
```

4. Reboot the system
```shell
sudo reboot
```
> [!TIP]\
> If you are running on `WSL2`
> ```shell
> wget https://developer.download.nvidia.com/compute/cuda/repos/wsl-ubuntu/x86_64/cuda-wsl-ubuntu.pin && sudo mv cuda-wsl-ubuntu.pin /etc/apt/preferences.d/cuda-repository-pin-600 && wget https://developer.download.nvidia.com/compute/cuda/12.4.1/local_installers/cuda-repo-wsl-ubuntu-12-4-local_12.4.1-1_amd64.deb && sudo dpkg -i cuda-repo-wsl-ubuntu-12-4-local_12.4.1-1_amd64.deb && sudo cp /var/cuda-repo-wsl-ubuntu-12-4-local/cuda-*-keyring.gpg usr/share/keyrings/ && sudo apt-get update && sudo apt-get -y install cuda-toolkit-12-4
>```

## Installing NVIDIA Container Toolkit (Host)

1. Prepare the repositories and sources
```shell
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
```

2. Update the packages
```shell
sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
```
### Root mode (option 1)
> [!TIP]\
> If you have not setup Docker yet, follow the following [Installation options](https://docs.docker.com/engine/install/).

Configuring the Docker
```shell
sudo nvidia-ctk runtime configure --runtime=docker
```

Restart the Docker Daemon
```shell
sudo systemctl restart docker
```

### Rootless mode (option 2)
```shell
nvidia-ctk runtime configure --runtime=docker --config=$HOME/.config/docker/daemon.json
```

Restart the Docker daemon
```shell
systemctl --user restart docker
```

Perform some configuration
```shell
sudo nvidia-ctk config --set nvidia-container-cli.no-cgroups --in-place
```

### Running a sample workload
To test if a container is running with accellerated GPU from the host system, we can run a sample workload to test compatibility.
```shell
sudo docker run --rm --runtime=nvidia --gpus all ubuntu nvidia-smi
```

In the docker container, the output should be similar to this
```shell
+-----------------------------------------------------------------------------------------+
| NVIDIA-SMI 550.54.15              Driver Version: 550.54.15      CUDA Version: 12.4     |
|-----------------------------------------+------------------------+----------------------+
| GPU  Name                 Persistence-M | Bus-Id          Disp.A | Volatile Uncorr. ECC |
| Fan  Temp   Perf          Pwr:Usage/Cap |           Memory-Usage | GPU-Util  Compute M. |
|                                         |                        |               MIG M. |
|=========================================+========================+======================|
|   0  NVIDIA GeForce GTX 1650        Off |   00000000:01:00.0  On |                  N/A |
| N/A   37C    P8              3W /   50W |     710MiB /   4096MiB |     34%      Default |
|                                         |                        |                  N/A |
+-----------------------------------------+------------------------+----------------------+
                                                                                         
+-----------------------------------------------------------------------------------------+
| Processes:                                                                              |
|  GPU   GI   CI        PID   Type   Process name                              GPU Memory |
|        ID   ID                                                               Usage      |
|=========================================================================================|
+-----------------------------------------------------------------------------------------+
```