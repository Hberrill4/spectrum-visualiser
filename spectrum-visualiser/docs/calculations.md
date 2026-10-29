# Design Calculations

| Parameter | Symbol | Value | Justification |
|-----------|--------|-------|---------------|
| Sample rate | fs | | |
| Nyquist frequency | fs / 2 | | |
| FFT size | N | | |
| Bin width (frequency resolution) | fs / N | | |
| Time to collect one frame | N / fs | | |
| Display update rate | | | |
| Number of display columns | | | |
| Lowest band edge | | | |
| Highest band edge | | | |
| Band spacing (linear / log) | | | |
| Input DC bias | | | |
| Anti-aliasing filter cut-off | fc = 1 / (2πRC) | | |

## Column-to-frequency map
| Column | Lower edge (Hz) | Upper edge (Hz) | FFT bins used |
|--------|-----------------|-----------------|---------------|
