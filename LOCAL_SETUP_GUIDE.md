# Local Development Guide


This guide provides information on how to set up a local development environment on your computer, with all the dependencies for the class preinstalled. The setup is pretty automatic, and relies on a docker image we have provided for you (the same that is used by Deepnote), as well as a VSCode plugin called Dev Containers that allows you to open an entire project within this docker container.
(NOTE: This guide also works for other IDEs that build on VSCode, such as Cursor.)


## Prerequisites (one-time on your machine)
1. Install and start [Docker Desktop](https://docs.docker.com/desktop/) (Mac, Windows, or Linux) or [Docker Engine](https://docs.docker.com/engine/install/) (Linux only)
2. Install VSCode or Cursor.
3. Install the VSCode extension Dev Containers (ms-vscode-remote.remote-containers).
4. (Recommended) Install Python extension + Jupyter extension in VS Code.


## Project layout
Start by creating these files and folders in your project root folder:
```
your-project/
 .devcontainer/
   devcontainer.json
    postCreate.sh
 .vscode/
   settings.json
 requirements.txt
 src/
   test_drake.py
```

The files should contain the following:
```json
// .devcontainer/devcontainer.json
{
  "name": "Drake DevContainer",
  "image": "russtedrake/manipulation:90658e5",
  "forwardPorts": [7000, 7001, 7002, 8000, 8888],
  "portsAttributes": {
    "7000": { "label": "MeshCat", "onAutoForward": "notify" },
    "7001": { "label": "MeshCat", "onAutoForward": "notify" },
    "7002": { "label": "MeshCat", "onAutoForward": "notify" },
    "8000": { "label": "MeshCat", "onAutoForward": "notify" },
    "8888": { "label": "MeshCat", "onAutoForward": "notify" },
  },


  "otherPortsAttributes": { "onAutoForward": "ignore" },
  "customizations": {
    "vscode": {
      "extensions": [
        "ms-python.python",
        "ms-python.vscode-pylance",
        "ms-toolsai.jupyter"
      ],
      "settings": {
        "remote.localPortHost": "127.0.0.1",
        "python.defaultInterpreterPath": "${workspaceFolder}/.venv/bin/python",
        "python.terminal.activateEnvironment": true
      }
    }
  },
  
  "postCreateCommand": [
  "bash",
  "-lc",
  "chmod +x .devcontainer/postCreate.sh && .devcontainer/postCreate.sh"
  ]
}
```

```sh
# .devcontainer/postCreate.sh
#!/usr/bin/env bash


set -e
apt-get update && apt-get install -y python3-venv python3-full >/dev/null || true
python3 -m venv .venv --system-site-packages
.venv/bin/pip install -U pip wheel
if [ -f requirements.txt ]; then .venv/bin/pip install -U -r requirements.txt; fi
if [ -f requirements.user.txt ]; then .venv/bin/pip install -U -r requirements.user.txt; fi


.vscode/settings.json
{
    "python.defaultInterpreterPath": "${workspaceFolder}/.venv/bin/python",
    "python-envs.defaultEnvManager": "ms-python.python:venv",
    "python.terminal.activateEnvironment": true,
    "python-envs.pythonProjects": []
}
requirements.txt
--extra-index-url https://drake-packages.csail.mit.edu/whl/nightly
manipulation==2025.9.22
Note: this may change throughout the semester. See Installing your own dependencies & updating `manipulation`


src/test_drake.py (just a test file)
from pydrake.all import RigidTransform, RollPitchYaw, StartMeshcat
from manipulation.letter_generation import create_sdf_asset_from_letter
import time


if __name__ == "__main__":
    print("Drake import OK.")
    X = RigidTransform(RollPitchYaw(0.1, 0.2, 0.3), [1, 2, 3])
    print("Transform:", X)
    print("Starting MeshCat… (check the VS Code Ports panel)")
    meshcat = StartMeshcat()
    time.sleep(30)
```

Open the project in the container
Now for the key step: opening the project folder inside the docker container. You will have to run this step every time you reopen VSCode (but it should be very quick!)
1. Make sure that Docker is running.
2. Open the project folder in VSCode.
3. Press ⌘⇧P / Ctrl⇧P → “Dev Containers: Reopen in Container”.
   1. The first time you do this: Wait for the container to download and build, and for postCreateCommand to set up the virtual environment inside the container (this will take ~15 minutes the first time you do this, but should be quick after)
4. VSCode should auto-select ${workspaceFolder}/.venv/bin/python as the Python interpreter. If not, pick it in the interpreter menu.
5. To close the Dev Container, select “File → Close Remote Connection”


## Running code & notebooks
* To test that everything is working correctly, run the provided Python test script: src/test_drake.py
   * You can run the script either by opening the script and clicking the “Run” arrow, or by running `./venv/bin/python src/test_drake.py` in a terminal inside VSCode.
   * Make sure to open http://localhost:7000 and verify that meshcat is running (the simulation will be empty and is set to only run for 30 seconds)
* MeshCat: open forwarded port in browser (localhost:7000 or localhost:7001)
   * Note: Meshcat defaults to port 7000. However, if this port is taken by another     process on your computer, Meshcat will get forwarded to another port
   * To make sure you open the right port, either click the pop-up on the bottom-right when you start meshcat or go to the ports console and look for Meshcat
* Jupyter: select kernel at ${workspaceFolder}/.venv/bin/python


Installing your own dependencies & updating `manipulation`
* We will be updating the `manipulation` python package throughout the semester. To upgrade it to the latest version, edit `requirements.txt` to update the manipulation version (see the latest release history here). Then run `.venv/bin/pip install -U -r requirements.txt` within your VSCode terminal
* Your project might require other dependencies than what we provide you with. To add dependencies of your own, you can either:
   * Run `.venv/bin/pip install SOME_PYTHON_PACKAGE`
   * OR Create a new file `requirements.user.txt` and add them to this file. Then run `.venv/bin/pip install -U -r requirements.user.txt` to install them
Other tips & gotchas
* Always use the .venv; don’t install anything into the docker container system Python.
* When starting meshcat with StartMeshcat(), it will likely print out an address like “localhost:7000”. This is the address it connects to in the docker container, but not necessarily the address connected to on your machine. If port 7000 is already taken, for example, meshcat will get forwarded to a different port. Check the ports tab of the terminal to make sure you navigate to the right place
* Your code will persist on your host machine via a bind mount.
* If the MeshCat port is missing, add manually in the Ports tab.
* On Windows, Docker Desktop requires WSL 2 and virtualization to be enabled.
     * To install/update WSL, ensure you’re in a 64-bit powershell environment. Check your environment by running:

```
[Environment]::Is64BitProcess        
```

* For enabling virtualization, you may need to enter your bios. Navigate to your CPU configuration and look for a setting like “SVM Mode”, “AMD-V”, or “Intel VT-x”
     * You may also need to enable Hyper-V
