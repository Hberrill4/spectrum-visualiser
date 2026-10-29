# Audio-Reactive Spectrum Visualiser

University of Liverpool EEE, Mini-Challenge 2 (October 2026).
ESP32 + MAX7219 LED matrix spectrum visualiser, developed through the CDIO cycle over six sessions.

## Team
| Name | Main responsibility |
|------|---------------------|
|      |                     |

## Product definition
_One or two sentences: what it does, who it is for, what frequency range, how success is judged._

## Signal chain
Input → conditioning → sampling → FFT → spectrum mapping → LED display
(see `docs/design-log.md` and `docs/calculations.md`)

## Repository layout
| Folder | Contents |
|--------|----------|
| `src/` | Firmware (PlatformIO, Arduino framework) |
| `docs/` | Design log, calculations, test plan, session reflections, images |
| `test-data/` | Raw test results (CSV) |
| `reference/` | Notes on the original prototype code and sources used |
| `hardware/` | Wiring diagrams, schematics, pin map |

## Current status
| Item | Status |
|------|--------|
| Original prototype running | ☐ |
| Own firmware sampling at fixed fs | ☐ |
| Tone-sweep accuracy test | ☐ |
| Limits tested (aliasing, silence, clipping) | ☐ |
| Improvement implemented and measured | ☐ |

## Acknowledgements
See `ATTRIBUTION.md`.
