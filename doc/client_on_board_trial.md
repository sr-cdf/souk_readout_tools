# Client-on-board trial

This note tracks the development-branch experiment that runs the
`souk_readout_tools` client directly on the RFSoC ARM cores alongside the
existing server daemon. Use this procedure only while working on the
`client-on-board-trial` branch. Throughout the doc the adjective **on-board**
means "physically located on the RFSoC board" so it is not confused with any
"onboard" workflow for new users.

## Goals

- Prove that the RFSoC can host both `readout_server.py` and
  `readout_client.py` without breaking real-time streaming.
- Surface missing dependencies or installer guards that block the client on the
  Xilinx aarch64 image.
- Capture operational caveats (CPU load, storage pressure, config races) before
  deciding whether to merge the feature.

## RFSoC prerequisites

1. **Base OS**: the RFSoC ships with Ubuntu 18.04 LTS. Keep the existing
  service virtualenv (`/home/casper/py38venv`, Python 3.8) untouched because
  the server hard-depends on that runtime.
2. **Client Python**: install Python 3.10 specifically for the client. On
  Ubuntu 18.04 the simplest route is the deadsnakes PPA:
  ```bash
  sudo add-apt-repository ppa:deadsnakes/ppa
  sudo apt update
  sudo apt install python3.10 python3.10-venv python3.10-dev
  python3.10 -m venv /home/casper/venvs/souk-client-py310
  source /home/casper/venvs/souk-client-py310/bin/activate
  ```
  Always run client commands inside this Python 3.10 environment.
3. **System packages (headless)**: avoid GUI stacks entirely. Install only the
  math/plotting prerequisites SciPy and matplotlib need when driven by the Agg
  backend:
  ```bash
  sudo apt install build-essential libopenblas-dev liblapack-dev \
      libatlas-base-dev libfreetype6-dev libpng-dev pkg-config
  ```
4. **Matplotlib backend**: force Agg so no GUI bindings load. Set it once in
  the shell or the virtualenv `activate` script:
  ```bash
  export MPLBACKEND=Agg
  ```
  (Jupyter forwarding from the RFSoC can come later; keep it out of this
  trial.)
5. **Repository checkout**: clone or fetch this repo and switch to
  `client-on-board-trial`.

## Installing the client on-board

1. Activate the virtualenv.
2. Export the installer flag that re-enables client modules on Xilinx builds:
   ```bash
   export INSTALL_CLIENT=true
   ```
   (Leave `INSTALL_SERVER` unset so the existing server install remains.)
3. From the repo root run:
   ```bash
   pip install -e .
   ```
4. Verify that Python can import the client stack:
   ```bash
   python -c "import souk_readout_tools.client.readout_client as c; print(c.__file__)"
   ```

## Configuration updates

- Copy `src/souk_readout_tools/data/config/config.yaml` to the RFSoC data tree
  if needed, then set `rfsoc_host.address: 127.0.0.1`. The server keeps binding
  to `0.0.0.0`, so remote clients still function.
- Update `data_dir` (and other paths) to point at attached storage such as
  `/mnt/data` to avoid filling the eMMC with raw IQ captures.
- Keep `config.yaml` and calibrations under version control (git or plain
  copies) so the server and on-board client both have auditable state.

## Avoiding config races while the daemon runs

Instead of stopping the `readout_server` systemd service, use the existing RPCs
to serialize file edits through the server process itself:

1. Pull the live config into the client workspace:
   ```bash
   python - <<'PY'
   from souk_readout_tools.client.readout_client import ReadoutClient
   client = ReadoutClient()
   client.pull_config()
   PY
   ```
2. Edit the local copy (for example in `$HOME/.config/souk_readout_tools`).
3. Push the modification back via the same RPC so the daemon rewrites its copy
   atomically:
   ```bash
   python - <<'PY'
   from souk_readout_tools.client.readout_client import ReadoutClient
   client = ReadoutClient()
   client.push_config()
   PY
   ```
4. Use `push_calibration`/`pull_calibration` in the same way for calibration
   files.

Because every change flows through a single TCP request queue, the server never
observes partially-written files and can reload safely without service stops.

## Running the trial

1. **Smoke test the control RPCs**:
   ```bash
   python -m souk_readout_tools.client.client_scripts.souk_connection_test \
          --config /path/to/config.yaml
   ```
2. **Exercise stream reception locally**:
   ```bash
   python -m souk_readout_tools.client.client_scripts.receive_stream \
          --config /path/to/config.yaml --duration 30
   ```
3. **Skip GUI tooling for now**: headless clients must avoid PyQt and other GUI
  front-ends. If interactive debugging is required later, plan to run a remote
  Jupyter server on the RFSoC, but keep it out of this initial trial.

## Operational cautions

- **CPU contention**: Monitor ARM cores with `htop`. Consider pinning client
  processes with `taskset` if they interfere with the streaming server.
- **Storage usage**: High-rate captures can fill the RFSoC eMMC quickly. Rotate
  files or stream directly to network-mounted storage.
- **Dependency builds**: SciPy wheels may not exist for Ubuntu 18.04 aarch64;
  expect lengthy source builds even without GUI bindings.
- **Config locking**: Always use the `push_*` RPCs noted above rather than
  editing files in-place on disk.

## Next steps

1. Validate the Python 3.10 virtualenv on Ubuntu 18.04 by running the control
   RPC smoke test locally.
2. Capture CPU and memory profiles while the on-board client performs streaming
   so we can tune core pinning if necessary.
3. Exercise the config/calibration push workflow end-to-end and log any race
   conditions we still observe.
4. Decide whether the Agg-only matplotlib flow is sufficient or if a lightweight
   remote Jupyter setup is required for interactive debugging.

Document any findings from the trial here so we can decide on a production-grade
solution.
