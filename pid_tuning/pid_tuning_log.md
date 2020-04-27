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

### Exp 09
Commented out suspected useless code snippets in Exp 08. Turns out they are indeed useless. The behavior is almost identical to Exp 08 (and 05).

### April 27
Goal today is to have a single onboard PID controller for pitch attitude control. ( no separated rate and attitude controller)

### Exp 10
Start from exp09 code, fix the typo about roll angle (as shown earlier in this log)
result: stable oscillation, magnitude decreases as thrust goes up, from 10deg - 5deg
this is with P 1500 and D 250, all embedded in the DebugPID() function

Since there were still oscillation, lets try increasing D and see if that goes away. We will first do this qualitatively, then with a fixed throttle, record oscillation amplitude to quantify improvement

### Exp 11
try with P 1500, D 500
My subjective view: oscillation amplitude were smaller at given thrust, but at higher frequency. There were a lot of sideway oscillation, i.e., translational movement

### Exp 12
Let's try with P 1500, D350, and observe how the cf deals with a step input
Result: Quite responsive

Mathmatically speaking, how different is PD from PP?  -> transfer function 

### Exp 13
Recreate P-P controller in 5
with rate P = 250, attitude P = 6, vehilcle shows similar behavior to PD controller in exp 12, oscillates at lower thrust, but at 50% thrust oscillation is very small. However, sometimes is oscillates sideway quite aggressively, as in 12 

I suspect that this oscillation may be due to the string. Next I'll try implement the PD controller to roll and pitch, and leave in default yaw controller. See how the vehicle behave without constrain.  

### Exp 14
try exp2, no oscillation, so it has nothing to do with strings...

### Exp 15
default P-P controller oscillates quite a bit
default PD-P controller effectively damps this oscillation

OK, so maybe we need a damper on rate. A damper on rate essentially acts against external disturbences, who manifest as undesired angular acceleration.

Let's cheat a bit by adding a damper on rate, if I recall correctly there's a LPF in there. 

### Exp 16
used Tao's PD controller as in 9 and 10, but added a damper with default LPF on pitch rate

result: no more oscillation


