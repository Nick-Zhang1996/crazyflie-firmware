### Exp 01
All zero: oscillates around the string and flipped eventually. All rotor responses are the same.

### Exp 02
Default pitch controller + default pitch rate controller: stabilzed along string very well. Rotor responses are two curves.

### Exp 03
Default pitch PD controller + default pitch rate controller: stablized along string very well. Rotor responses are two curves.

Exp 02 and 03 => I in pitch controller is not necessary. Since D = 0 in the default pitch controller, we know a simple P controller for pitch is sufficient.

### Exp 04
Default pitch P controller + default pitch rate PD controller: stablized along string very well. Rotor responses are two curves. Now we know P + PD are sufficient to have a good pitch controller.

### Exp 05
Default pitch P controller + default pitch rate P controller: vibrates at high frequency but low magnitude but was able to stablize along the string relatively well overall. Rotor responses show very high frequenct oscillations.

Exp 04 and 05 => This shows that D in pitch rate damps the motion, which I guess makes sense.

### Exp 06
Pitch PD controller only. We set P and D gains so that it is theoretically equivalent to Exp 05: P = pitch P * pitch rate P in Exp 05 and D = pitch rate P in Exp 05 (we have already converted the units). P = 1500; D = 250.
It started to vibrate immediately with an increasing magnitude that quickly went out of control. Eventually it flipped over and the experiment
was terminated. This is very similar to what we observe in our past experiments.

### Exp 07
Now we take a step back. In Exp 07, we implement Exp 05 in an equivalent way:
```
rollOutput = 0;
pitchOutput = saturateSignedInt16(1500 * (eulerPitchDesired - eulerRollActual) - 250 * pitchRateActual);
yawOutput = 0;
```
This also rules out the influence of output limits, lpf, etc, although I believe they are not active in the codebase. Behavor: it was going up and down at modest frequency (maybe 2 Hz) but it controled its attitude fairly well. I didn't observe the high frequency oscillation in Exp 05. I guess I should diff Exp 07 and 05 to understand where this difference comes from.

### Exp 08
I reproduced Exp 05 and observed the old behavior: high frequency osciliation but still managed to stablize itself along the string. So Exp 05 is indeed reproduceable.