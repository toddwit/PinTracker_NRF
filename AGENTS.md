# PinTracker NRF — Agent Notes

## Framework
- Zephyr RTOS (nRF) firmware project using Zephyr CMake build system
- Single CMake project under `pintracker_app/`

## Build
- `west build -b decawave_dwm3001cdk pintracker_app/` — requires `ZEPHYR_BASE` env var set
- Role is selected in `src/main.c`: set `PINTRACKER_BUILD_ANCHOR` to `0` for tag or `1` for anchor, and set `PINTRACKER_ANCHOR_ID` before building.
- `.vscode/settings.json` configures CMake to use `pintracker_app/` as source directory
- Build artifacts go to `pintracker_app/build/` (gitignored)

## Source layout
- `src/main.c` — entry point, tests raw SPI reads then probes DW3000 driver
- `src/dw3000/` — Decawave DW3000 API/registers
- `src/dw3000_port.c` / `src/port.c` — HAL/board porting layer
- `src/boards/decawave_dwm3001cdk.overlay` — devicetree overlay
- `prj.conf` — Kconfig flags (SPI, GPIO, UART)

## Key dependencies
- `west.yml` is present but minimal (no external west modules beyond Zephyr)
- DW3000 driver code is vendored in `src/dw3000/`

## Notes
- SPI and UART are working; DW3000 integration is in progress
- `deca_device_api.h` / `deca_probe_interface.h` are the main driver headers agents should reference
