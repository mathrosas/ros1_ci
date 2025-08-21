# Checkpoint 24 — Task 1
## ROS1 CI for **TortoiseBot Waypoints** with **Jenkins + Docker** (Noetic)

This README documents a complete, reproducible **Continuous Integration** setup for **ROS1 (Noetic)** that builds a Docker image, launches **Gazebo headless**, starts the **TortoiseBot waypoints action server**, runs **`rostest`**, and reports success/failure in **Jenkins** every time the code changes.

- **Checkpoint**: 24 — *Continuous Integration*  
- **Task**: 1 — Jenkins Basics (automate build & tests for a ROS package)  
- **ROS repository under test**: `https://github.com/mathrosas/ros1_testing`  
  - Packages: `tortoisebot`, `tortoisebot_waypoints`
  - Tests: Python 3 (ROS Noetic)

This solution uses a **Freestyle** Jenkins job driven by shell steps and a purpose-built Docker image. No GUI is required during tests (Gazebo runs via **Xvfb**).

---

## Contents

- [What the pipeline does](#what-the-pipeline-does)
- [Prerequisites](#prerequisites)
- [Local quickstart (no Jenkins)](#local-quickstart-no-jenkins)
- [Start Jenkins](#start-jenkins)
- [Create the Jenkins job](#create-the-jenkins-job)
  - [Build steps (copy–paste)](#build-steps-copypaste)
  - [Trigger on Git changes (Poll SCM)](#trigger-on-git-changes-poll-scm)
- [Expected successful output](#expected-successful-output)
- [How the pieces fit together](#how-the-pieces-fit-together)
- [Troubleshooting](#troubleshooting)
- [Optional improvements](#optional-improvements)
- [License](#license)

---

## What the pipeline does

1. **Builds a CI Docker image** from `osrf/ros:noetic-desktop`, installing ROS/Gazebo deps, `rostest`, and `xvfb`.
2. **Clones** `ros1_testing` into `/simulation_ws/src` and **builds** the Catkin workspace.
3. **Runs headless Gazebo** using **Xvfb** with `DISPLAY=:1`; starts the **Waypoints Action Server**.
4. **Executes `rostest`** (`tortoisebot_waypoints/waypoints_test.test`) against the running sim.
5. **Cleans up** all processes (gzserver/gzclient/Xvfb) and exits with the **test result code**.
6. **Jenkins** shows pass/fail and stores console logs; it’s triggered automatically by **Poll SCM**.

---

## Prerequisites

- **Ubuntu 20.04+** (or compatible Linux host)  
- **Docker Engine** (and permissions for Jenkins to use it)  
- **Jenkins** (this repo includes a helper script to start it locally)

Install Docker & enable non-root usage:

```bash
sudo apt-get update
sudo apt-get install -y docker.io docker-compose
sudo service docker start

sudo usermod -aG docker $USER
newgrp docker
```

> CI notes: Allocate sufficient disk/RAM. The run step uses `--shm-size=2g` to keep Gazebo stable in headless mode.

---

## Local quickstart (no Jenkins)

You can build and run the CI container locally to verify everything works before wiring Jenkins.

```bash
# From the directory that contains this README and the Dockerfile
docker build -t tortoisebot-ros1-ci .

# Run tests headless; entrypoint will start Gazebo + action server + rostest
docker run --rm --shm-size=2g -e CI=1 tortoisebot-ros1-ci
```

Expected test summary:

```
SUMMARY
 * RESULT: SUCCESS
 * TESTS: 2
 * ERRORS: 0
 * FAILURES: 0
```

If the container exits with code `0`, the tests passed.

---

## Start Jenkins

This solution includes `run_jenkins.sh` to start a local Jenkins quickly:

```bash
bash run_jenkins.sh
```

What the script does:

- Sets `JENKINS_HOME=~/webpage_ws/jenkins/`
- Installs Java (OpenJDK 17)
- Downloads and runs `jenkins.war` (v2.463) in the background
- Prints **PID and URL** to `~/jenkins__pid__url.txt`

Open the printed URL, unlock Jenkins with the initial admin password (see console or `~/webpage_ws/jenkins/secrets/initialAdminPassword`), install suggested plugins, and go to the **Dashboard**.

---


### Jenkins login (credentials for this checkpoint)

For this exercise, use the following Jenkins admin credentials:

- **Username:** `admin`
- **Password:** `password`

> ⚠️ **Security note:** These credentials are provided for a learning/lab environment. 
> Do **not** reuse them in public or production systems. Change the password immediately if you expose Jenkins to any untrusted network.

## Create the Jenkins job

1. **Dashboard → New Item →** *Freestyle project*  
   Name it, for example: **ROS 1 CI – Tortoisebot Waypoints**.
2. **Description** (optional):  
   “Build Docker, run headless Gazebo + `rostest` for TortoiseBot waypoints.”
3. **Source Code Management → Git**  
   - Repository URL: `https://github.com/mathrosas/ros1_ci` *(this CI scaffolding repo)*  
   - Branches to build: `main`
4. **Build Triggers**: configure **Poll SCM** (below).  
5. **Build**: add the three shell steps in the next section.

### Build steps (copy–paste)

**Step 1 — Preflight & housekeeping**

```bash
set -euxo pipefail

echo "== Preflight =="

whoami
uname -a
docker --version
git --version
df -h

# allow Jenkins to talk to Docker (quick-and-dirty for learning envs)
sudo chmod 666 /var/run/docker.sock || true

# keep the host clean
docker image prune -f || true
docker container prune -f || true
```

**Step 2 — Build the CI image**

```bash
set -euxo pipefail

# Build the Docker image that clones and builds the ROS workspace
docker build --pull -t tortoisebot-ros1-ci \
  --build-arg REPO_URL=https://github.com/mathrosas/ros1_testing.git \
  --build-arg REPO_BRANCH=main \
  .
```

> Note: The current Dockerfile clones `ros1_testing` directly; the two args above are provided for future parameterization (you may see a “not consumed” warning).

**Step 3 — Run the tests headless**

```bash
set -euxo pipefail

# Make sure any old container is gone
docker rm -f tortoise-ros1-ci >/dev/null 2>&1 || true

# Run headless; entrypoint starts Xvfb + Gazebo + action server + rostest
docker run --name tortoise-ros1-ci --rm \
  --shm-size=2g \
  -e CI=1 \
  tortoisebot-ros1-ci
```

### Trigger on Git changes (Poll SCM)

- **Build Triggers → Poll SCM**: `* * * * *`  
  Jenkins will check the repository every minute and build only if a new commit is found.

---

## Expected successful output

Tail of a passing build (abridged):

```
Successfully tagged tortoisebot-ros1-ci:latest

+ docker run --name tortoise-ros1-ci --rm --shm-size=2g -e CI=1 tortoisebot-ros1-ci
[2025-08-20 22:25:10] Launching TortoiseBot world...
[2025-08-20 22:25:10] Waiting for ROS master...
[2025-08-20 22:25:12] Waiting for Gazebo...
[2025-08-20 22:25:14] Starting Waypoints Action Server...
[2025-08-20 22:25:16] Running rostest...
[ROSUNIT] Outputting test results to /root/.ros/test_results/tortoisebot_waypoints/rostest-test_waypoints_test.xml

[ROSTEST]-----------------------------------------------------------------------
[tortoisebot_waypoints.rosunit-test_waypoints/test_final_position][passed]
[tortoisebot_waypoints.rosunit-test_waypoints/test_final_yaw][passed]

SUMMARY
 * RESULT: SUCCESS
 * TESTS: 2
 * ERRORS: 0
 * FAILURES: 0

[2025-08-20 22:25:30] TEST RESULT: 0
[2025-08-20 22:25:30] Shutting down...
Finished: SUCCESS
```

---

## How the pieces fit together

- **Dockerfile**
  - Base image: `osrf/ros:noetic-desktop`
  - Installs: Gazebo ROS pkgs, controllers, `rostest`, `xvfb`, and utilities.
  - Creates `/simulation_ws`, **clones** `ros1_testing`, ensures Python 3 shebang on the action server script, and **builds** the workspace.
  - Persists env setup to `/root/.bashrc` and sets `ROS_MASTER_URI=http://localhost:11311`.

- **entrypoint.sh**
  - Starts **Xvfb** (`DISPLAY=:1`), launches **Gazebo** headless (`tortoisebot_playground.launch gui:=false`).
  - Waits for ROS master and Gazebo topics to be ready.
  - Starts the **Waypoints Action Server** and waits for `/tortoisebot_as`.
  - Runs **`rostest tortoisebot_waypoints waypoints_test.test --reuse-master`**.
  - Prints the test result and **cleans up** (action server, gzserver, Xvfb). Exits with the test code.

- **run_jenkins.sh**
  - Sets `JENKINS_HOME`, installs Java, downloads `jenkins.war`, launches Jenkins, and prints the **URL** & **PID** for convenience.

---

## Troubleshooting

- **“the input device is not a TTY” inside Jenkins**  
  Remove `-it` from `docker run` (this guide’s run step omits it).

- **Docker permissions for Jenkins**  
  If Docker commands fail with permissions errors, the preflight step uses:  
  `sudo chmod 666 /var/run/docker.sock` (fine for a classroom/lab).  
  Prefer adding the Jenkins user to the `docker` group in real deployments.

- **Gazebo headless / display issues**  
  Ensure `xvfb` is installed; `entrypoint.sh` sets `DISPLAY=:1`. Keep `--shm-size=2g` to avoid shared-memory crashes.

- **ROS env variables (Noetic + bash -u)**  
  The image sets `ROS_MASTER_URI=http://localhost:11311`. If you add strict `set -u`, ensure environment variables are defined before sourcing ROS scripts.

- **Python 3**  
  The Noetic stack uses Python 3. The entrypoint ensures the waypoints action server script has a Python 3 shebang and is executable.

- **Disk pressure**  
  Docker images are big; the preflight step prunes images/containers. Periodically `docker system prune -af` if the host is near capacity.

---

## Optional improvements

- **Parameterize the Dockerfile** to fully consume build args:
  ```dockerfile
  ARG REPO_URL="https://github.com/mathrosas/ros1_testing.git"
  ARG REPO_BRANCH="main"
  RUN git clone --branch "${REPO_BRANCH}" --depth 1 "${REPO_URL}" ros1_testing
  ```

- **Publish test results** using “Publish JUnit test result report” and point it to:  
  `/root/.ros/test_results/**/*.xml` to see test graphs/trends in Jenkins.

- **Enable BuildKit / buildx** to speed up image builds and caching.

- **Switch to a Jenkinsfile** (Pipeline as Code) once the freestyle flow is stable.

---

## License

This CI setup is provided as-is for educational and internal testing purposes.  
Use under this repository’s license and the upstream licenses of ROS packages and `ros1_testing`.

---

### TL;DR (end-to-end)

```bash
# 1) Docker
sudo apt-get update
sudo apt-get install -y docker.io docker-compose
sudo service docker start
sudo usermod -aG docker $USER
newgrp docker

# 2) Jenkins
bash run_jenkins.sh
# Open URL from ~/jenkins__pid__url.txt and complete setup

# 3) Jenkins Job (Freestyle)
#   - SCM: https://github.com/mathrosas/ros1_ci  (branch: main)
#   - Build: paste the three steps from this README
#   - Triggers: Poll SCM (* * * * *)

# 4) Commit & push
# Jenkins builds the image, runs Gazebo headless, executes rostest, and reports status.
```
