#!/usr/bin/env python3

import os
import subprocess

from typing import Tuple

def detect_os() -> str:
    if any(word in os.uname().release.lower() for word in ["microsoft", "wsl2"]):
        with open("/etc/os-release", "r") as f:
            content = f.read().lower()
            if "ubuntu" in content:
                return "wsl2"
            else:
                raise ValueError("WSL2 usage only supports Ubuntu")
    else:
        with open("/etc/os-release", "r") as f:
            content = f.read().lower()
            if "ubuntu" in content:
                return "ubuntu"
            elif "fedora" in content:
                return "fedora"
            else:
                raise ValueError("OS not supported")

def detect_gpu(host_os:str) -> Tuple[str, str, str]:
    gpu_vendor = None
    gpu_pci    = None
    gpu_card   = None
    gpu_render = None
    if host_os == 'wsl2':
        raise NotImplementedError("Currently not updated for WSL2")
    elif host_os == 'ubuntu' or host_os == 'fedora':
        renderers = subprocess.check_output(["lspci -D -nn | grep -E '3D|VGA|Display'"], universal_newlines=True, shell=True).splitlines()
        for description in renderers:
            if "NVIDIA" in description:
                gpu_vendor = "nvidia"
                gpu_pci = description.split()[0]
        if gpu_vendor == None:
            for description in renderers:
                if "Intel" in description:
                    gpu_vendor = "intel"
                    gpu_pci = description.split()[0]
        drm_paths = os.listdir(f"/sys/bus/pci/devices/{gpu_pci}/drm")
        for device in drm_paths:
            if "card" in device:
                gpu_card = f"/dev/dri/{device}"
            if "render" in device:
                gpu_render = f"/dev/dri/{device}"
    else:
        raise ValueError("OS not supported")

    return tuple([gpu_vendor, gpu_card, gpu_render])

host_os = detect_os()
gpu_vendor, gpu_card, gpu_render = detect_gpu(host_os)

if host_os == 'wsl2':
    raise NotImplementedError("Currently not updated for WSL2")
elif host_os == 'ubuntu':
    pass
elif host_os == 'fedora':
    pass
else:
    raise NotImplementedError("OS not supported")
