# Configuring .devcontainer
## **Introduction**
- **.devcontainer** directory contains the configuration files for the development container. This directory contains the **Dockerfile** and **devcontainer.json** files. Dockerfile is used to build the container and devcontainer.json is used to configure the container for VS Code.

- There is no need to modify Dockerfile however for the devcontainer.json file there are some changes that can be made according to the user's needs. Below are the changes that can be made to the devcontainer.json file.

**devcontainer.json** file has a structure like below:
```
{
    "name": "RHex ROS2 Development Container",
    "build": {
        "dockerfile": "Dockerfile",
        "context": "../.."
    },

    "containerEnv": {
        "DISPLAY": "${env:DISPLAY}",
        "NVIDIA_VISIBLE_DEVICES": "all",
        "NVIDIA_DRIVER_CAPABILITIES": "all"
    },

    "workspaceMount": "source=${localWorkspaceFolder},target=/home/rhex/mnt,type=bind,consistency=cached",
    "workspaceFolder": "/home/rhex/mnt/rhex_ws",

    "privileged": true,
    "runArgs": [
        "--net=host",
        "--gpus", "all"
    ],

    "mounts": [
       "source=/tmp/.X11-unix,target=/tmp/.X11-unix,type=bind,consistency=cached"
    ],

    "otherPortsAttributes": {
        "onAutoForward": "ignore"
    },

    "customizations": {
        "vscode": {
            "extensions": [
                "ms-vscode.cpptools",
                "ms-vscode.cpptools-extension-pack",
                "donjayamanne.python-extension-pack",
                "ms-iot.vscode-ros",
                "GitHub.copilot"
            ]
        }
    },

    "postCreateCommand": "chmod +x ../scripts/* && ../scripts/build_workspace.sh && echo 'export PATH=$PATH:/home/rhex/mnt/scripts/' >> ~/.bashrc"
}
```
## **Reconfiguring The devcontainer.json File**
### 1. **Non Nvidia GPUs:**
- If you don't have a Nvidia GPU or your computer has CPU embedded graphics card on your computer, remove the `--gpus`, `all` from the `runArgs` section. Then proceed for the building container steps. Result should be the following:
```
    "runArgs": [
        "--net=host"
    ],
```
### 2. **Customizing The Extensions:**
- If you want to add or remove extensions from the VS Code, you can do so by changing the `customizations` section. You can find the extension names from the [VS Code Marketplace](https://marketplace.visualstudio.com/).
- For an extension that you want to add go to the [VS Code Marketplace](https://marketplace.visualstudio.com/) and search for the extension that you want to add. Then copy the name of the extension and add it to the `customizations` section.
For example adding Material Icon Theme to the extensions go to VS Code Marketplace and then search it's name. Then click the extension, it will forward you to the extension page. Under the installation part just below the extension name and icon there will be a `Copy` button. Copy the part after the `install` and paste the copied text to the `customizations` section. Result would be like the following:
```
    "customizations": {
        "vscode": {
            "extensions": [
                "ms-vscode.cpptools",
                "ms-vscode.cpptools-extension-pack",
                "ms-python.python",
                "GitHub.copilot",
                "ms-iot.vscode-ros",
                "pkief.material-icon-theme"
            ]
        }
    },
```