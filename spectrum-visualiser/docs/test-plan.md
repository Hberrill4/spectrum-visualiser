# Test Plan

| ID | Test | Input | Expected result | Pass criterion | Result file | Pass/Fail |
|----|------|-------|-----------------|----------------|-------------|-----------|
| T1 | Matrix smoke test | none | every column lights | all LEDs work | | |
| T2 | Original prototype baseline | music + tones | | | | |
| T3 | Sine tone sweep | sine, 50 Hz to fs/2 | peak in correct column | peak within ±1 column | | |
| T4 | Above Nyquist | sine above fs/2 | false peak (aliasing) | behaviour recorded | | |
| T5 | Silence | no input | no lit columns | no flicker above row 1 | | |
| T6 | Overload | very loud input | | behaviour recorded | | |
| T7 | Rapid change | tone steps or drum hits | | response time recorded | | |
| T8 | Real music | 3 different genres | | qualitative notes + video | | |

## Test conditions to record every time
- Date, firmware commit hash, input source, input level/volume, frequency, notes
