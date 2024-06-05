# Docker Container Environment


## Table of Contents

- [Introduction](#introduction)
- [Customization](#customization)
    - [PCs with no Nvidia GPUs](#pcs-with-no-nvidia-gpus)
    - [Customizing The VSCode Extensions](#customizing-the-vscode-extensions)

## Introduction
**.devcontainer** directory contains the configuration files for the development container. This directory contains the **Dockerfile** and **devcontainer.json** files. Dockerfile is used to build the container and devcontainer.json is used to configure the container for VS Code.

There is no need to modify Dockerfile however for the devcontainer.json file there are some changes that can be made according to the user's needs. Below are the changes that can be made to the devcontainer.json file.

## Customization

### PCs with no Nvidia GPUs:
- If you don't have an Nvidia GPU or your computer has CPU embedded graphics card, remove the `"--gpus", "all"` from the `runArgs` section. Then proceed for the building container steps. Result should be the following:
```
    "runArgs": [
        "--net=host"
    ],
```

### Customizing The VSCode Extensions
- If you want to add or remove extensions from the VS Code, you can do so by changing the `customizations` section. You can find the extension names from the [VS Code Marketplace](https://marketplace.visualstudio.com/).
- For an extension that you want to add go to the [VS Code Marketplace](https://marketplace.visualstudio.com/) and search for the extension that you want to add. Then copy the name of the extension and add it to the `customizations` section.
For example adding Material Icon Theme to the extensions go to VS Code Marketplace and then search it's name. Then click the extension, it will forward you to the extension page. Under the installation part just below the extension name and icon there will be a `Copy` button. Copy the part after the `install` and paste the copied text to the `customizations` section. Result would be like the following:
```
    "customizations": {
        "vscode": {
            "extensions": [
                "ms-vscode.cpptools",
                "ms-vscode.cpptools-extension-pack",
                "donjayamanne.python-extension-pack",
                "ms-iot.vscode-ros",
                "pkief.material-icon-theme"
            ]
        }
    },
```
