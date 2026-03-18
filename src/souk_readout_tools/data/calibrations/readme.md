# Example Calibration Files

This directory contains example calibration files bundled with the package. On the server, these are copied to `~/.souk_readout_tools/pipeline_N/calibrations/` on first run.

You do not need to manually copy calibration files to the server. When you use `push_config()`, any calibration files referenced in the config are automatically transferred to the server. When you use `pull_config()`, referenced files are fetched into memory and saved alongside the config when `save_config()` is called.

For full calibration documentation, measurement procedures, and RF peripheral details, see [doc/calibration.md](../../../../doc/calibration.md).

## File Format

Calibration files are two-column text (frequency Hz, power dBm at full-scale). Lines starting with `#` are comments.

## Included Files

- `dac0.txt` — Example DAC0 calibration from the Cardiff KRM board
- `spectrum_analyser_cable.txt` — Example cable loss measurement

