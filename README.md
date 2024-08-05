# LocalAir_development

## Firmware for the Teensy hardware

This is the first version of the LocalAir microprocesser software, for use on the Teensy 4.0.

This version works, but is grim. A product version is currently under development.

The main code is in the `src` directory.

### Setup

This project uses [PlatformIO](https://platformio.org/).

To get started, install either:

- [PlatformIO Core](https://docs.platformio.org/en/latest/core/index.html), a CLI tool, or
- [PlatformIO IDE](https://docs.platformio.org/en/latest/integration/ide/pioide.html), an integration with VS Code.

### Building

Either:

- Run `pio run --target upload`, if using PlatformIO Core (CLI), or
- Use the `PlatformIO: Upload` button in VS Code, if using PlatformIO IDE.

## Data analysis

The code is in the `data_processing` directory.

This repo uses [nbstripout](https://pypi.org/project/nbstripout/) to stop private data being committed to this repo.

1. First install `nbstripout` in your Python environment.
2. Then run `nbstripout --install --attributes .gitattributes` from the root directory of the repo.

This will ensure that Jupyter notebooks are committed with no outputs present.
