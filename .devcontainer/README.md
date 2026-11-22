<!-- SPDX-License-Identifier: MIT -->
# Dev container

ROS 2 Jazzy desktop with the workspace of README.md (Prerequisites) built on create, for GitHub Codespaces and local Dev Containers. The workspace is ~/openamr_ws; its src/ links this checkout and holds openamr-platform-sw and the two OpenArm repositories at the pins of upstream_pins.yaml (.devcontainer/setup.sh). RViz shows in the browser through noVNC on port 6080 (desktop-lite feature, password vscode). Simulation tooling only: no hardware, no evidence.

`devcontainer.json` is kept as strict JSON so the repository-quality check (Validate tracked JSON and XML) can parse it.
