# Thermal control

The oscillator sits in a heated assembly with a TMP117 sensor and an external
heater switched by PB3. Two loops hold its temperature: an inner PID drives
the heater toward a temperature target, and a slow outer loop moves that
target so the heater runs near 50 % duty, keeping headroom both ways.

## 1. Inner temperature loop

The normal firmware runs a **parallel PID with derivative on raw measurement**.
Older PI-only profiles and names such as `thermal_pi` remain in the repository;
they do not mean normal AUTO_HYBRID has no derivative term.

| Compiled PID parameter | Value | Unit |
| --- | --- | --- |
| $K_{pT}$ | 2.6 | Duty/K |
| $K_{iT}$ | 0.016 | Duty/(K·s) |
| $K_{dT}$ | 16.9 | Duty·s/K |
| P/I temperature-filter time | 5 | s |
| Derivative-filter time $\tau_D$ | 3 | s |
| Derivative contribution limit | ±0.3 | Duty |
| Final heater duty range | 0–1 | Duty |

At each fresh accepted temperature update, with elapsed time $\Delta t_T$:

$$
\bar\theta_k=\bar\theta_{k-1}+
\frac{\Delta t_T}{5+\Delta t_T}(\theta_k-\bar\theta_{k-1}),
$$

$$
r^{ramp}_k=r^{ramp}_{k-1}+
\operatorname{clip}(r_k-r^{ramp}_{k-1},-q\Delta t_C,+q\Delta t_C),
\qquad q=\frac{328}{65536}\approx0.00500488\ \mathrm{K/s}.
$$

$\Delta t_C$ is time since the preceding control update or activation;
normally it equals $\Delta t_T$. This distinction avoids integrating time
before a controller start that falls between sensor updates.

The filtered raw-temperature derivative, PID terms and trial integral are

$$
w_k=\frac{\tau_D w_{k-1}+\theta_k-\theta_{k-1}}{\tau_D+\Delta t_T},
\qquad D_k=\operatorname{clip}(-K_{dT}w_k,-0.3,+0.3),
$$

$$
\epsilon_k=r^{ramp}_k-\bar\theta_k,\qquad
P_k=K_{pT}\epsilon_k,\qquad
J^{trial}_{k+1}=J_k+K_{iT}\epsilon_k\Delta t_C,
$$

$$
h^{cmd}_k=\operatorname{clip}(P_k+J_{k+1}+D_k,0,1).
$$

The derivative sign reduces heat when temperature is rising. Differentiating
measurement avoids a direct derivative kick from a setpoint change. P/I use
the 5 s filtered temperature; D uses raw sensor increments through its own 3 s
filter. This is not an unfiltered numerical derivative.

Integral updates are suppressed when they push an already saturated output
farther outward. A step crossing an output rail is projected to that rail;
the integral also has a fixed-point numerical bound of **±8 duty units**.
The final command is rounded to **0.001 duty** and applied in **1 s heater frames**.
Temperatures use signed Q16 arithmetic, and PID contributions use millionths of
full duty. The equations above omit those integer-rounding details.

Startup ramps the inner reference from current filtered temperature and
preloads its integral from prior duty. A normal outer goal update preserves
the inner integral, filter, derivative and ramp history.

Heating continues during clock HOLD/OFF or unavailable GNSS, provided heater
safety remains satisfied. Sensor faults/staleness, loss of actuator permission,
overtemperature and missed control/foreground deadlines stop heating. Relevant
deadlines are **2.5 s** for sensor/control freshness and a **250 ms foreground
lease** checked by the independent heater safety path. Raw trip is **70 °C**;
normal autonomous goals are bounded to **0–60 °C**.

The inner `stable` diagnostic retains stricter legacy thresholds (about 0.05 K
error, 0.01 K/min slope and 300 s dwell). It is separate from clock readiness
and does not gate PID output or automatic clock steering.

Sources: [thermal_control.c](../core/src/thermal_control.c),
[thermal_control.h](../core/include/simpleclock/thermal_control.h),
[thermal_probe.c](../platform/stm32c071/src/thermal_probe.c),
and [heater.c](../core/src/heater.c).

## 2. Outer temperature-goal loop

This loop moves temperature goal to obtain **50% external-heater duty**, leaving
heating headroom in both directions. It does not directly optimize oscillator
frequency or drive the heater pin.

It consumes **completed actual heater frames**, counted from on/off ticks,
rather than assuming the requested command was delivered. A time-weighted
**30 s duty window** establishes valid history and headroom. Separately, actual
duty is smoothed with a **100 s filter**:

$$
\bar h_k=\bar h_{k-1}+\alpha_h(h_k-\bar h_{k-1}),\qquad
\alpha_h\approx\min\left(1,\frac{2\Delta t_T}{200+\Delta t_T}\right).
$$

This is the implementation's rational approximation to $1-e^{-\Delta t_T/100}$,
with Q31 coefficient rounding. On centering entry, the filter is primed from
the valid 30 s mean. The window and filter have distinct jobs; this is not a
30 s moving average cascaded into the 100 s filter on every update.

Define the duty error and continuous deadband:

$$
e_h=0.5-\bar h,\qquad
\mathcal D(e_h)=
\begin{cases}
e_h-0.01,&e_h>0.01,\\
0,&|e_h|\le0.01,\\
e_h+0.01,&e_h<-0.01.
\end{cases}
$$

The target-rate request and target integration, when permitted, are

$$
\dot r=\operatorname{clip}\left(
\frac{G_\theta}{T_c}\mathcal D(e_h),-L_T,+L_T\right),\qquad
r_{k+1}=\operatorname{clip}(r_k+\dot r\Delta t_T,0,60)\ ^\circ\mathrm C.
$$

Defaults are $G_\theta=10$ K/unit duty and $T_c=1000$ s, giving
$G_\theta/T_c=0.01$ K/(s·unit duty). The requested slew cap is **0.004 K/s
(14.4 K/h)**; Q16 flooring makes the implemented cap **262/65536 ≈ 0.003997803 K/s**.
It covers a worst-case 10 K/h outdoor day-night ambient ramp with 1.44x margin:
in the synthetic plant (6 K heater authority, 250 s time constant) the old
0.001 K/s cap saturated the heater for 8,912 s of a 4 h cycle with 3.1 K error,
while 0.003–0.005 K/s held 0.034 K with no saturation (`test_hybrid_plant.c`).
At the regulated-temperature sensitivity of 0.04 ppb/°C, following 10 K/h is a
1.1e-4 ppb/s frequency ramp, far inside the 0.003 ppb/s clock slew cap.
Fractional target increments are retained in a remainder accumulator.
`Gtheta` is explicitly **provisional, not a measured thermal calibration**.

Low duty raises the goal; high duty lowers it. For example, filtered duty 0.60
gives deadband error −0.09 and requests **−0.0009 K/s** before permission guards.
Within 49–51% duty, the goal holds. `duty_centered` has a separate **48–52% band
and 300 s dwell**, with valid duty history and no temperature-goal rail limiting.

### Permission and directional guards

- It starts after startup target capture, valid inner regulation and a fresh
  duty window, and depends only on thermal-side conditions: safe sensor,
  completed thermal startup, running PID and no manual FCW trial.
- Clock state does not gate it: clock HOLD/OFF, unknown actuator, acquisition
  fault and GNSS loss (reported as `holdover` context) leave centering active.
- `outer pause` freezes the goal while inner PID continues; manual heater
  targets remain authoritative while the outer loop is explicitly paused.
  `outer resume` also clears a recoverable outer safety fault.
- If proposed goal motion would worsen raw goal-minus-temperature error beyond
  about **0.075 K**, that direction is blocked. After error exceeds **0.15 K
  for 10 s**, an error-pause latch permits only motion toward raw temperature;
  clearing it requires error within **0.075 K for 30 s**.
- Saturated duty can still request bounded corrective recentering; saturation
  is not a blanket freeze. Invalid/missed duty evidence rebuilds the window.
  Gaps over **3 s** reset continuity; no pause or gap creates catch-up motion.
  The first newly permitted update primes state without integrating.

Normal startup uses full heater power, then captures current temperature as
the goal after a qualified warming-slope test: **60 s initial delay, 30 s fit,
slope ≤0.4 °C/min for 20 s**. Flat/no-rise and ceiling paths can capture earlier.
It hands over at zero heater duty, then PID rebuilds holding power and the
outer loop adopts the captured goal. **39.399 °C is an initialization fallback,
not a required final operating temperature.**

Sources: [thermal_outer.c](../core/src/thermal_outer.c),
[thermal_boot.c](../core/src/thermal_boot.c), and `centering_permission`
in [hybrid_probe.c](../platform/stm32c071/src/hybrid_probe.c).

## 3. Timing summary

| Loop | Rate | Key parameters |
| --- | --- | --- |
| TMP117 sampling | One-shot every ≥ 1 s | 200 ms conversion; stale after 2.5 s |
| Heater PWM | 1 s frame, 1 ms resolution | 250 ms foreground lease; PID output older than 2.5 s is rejected |
| Startup boost | Once per power-up | 100 % duty; slope fit after 60 s over 30 s; capture when slope ≤ 0.4 °C/min for 20 s |
| Inner PID | 1 s | Kp 2.6 duty/K, Ki 0.016 duty/(K·s), Kd 16.9 duty·s/K; target ramp 0.005 K/s |
| Outer centering | Each completed heater frame | Mean duty to 50 % (±1 % deadband); 100 s duty filter; 30 s duty window; target rate ≤ 0.004 K/s |
| Outer pause/resume | Continuous | Pause when error > 0.15 °C for 10 s; resume when < 0.075 °C for 30 s |
| LED3 / `thermal_warning` | 1 s | Settled = error ≤ 0.25 K and slope ≤ 0.1 K/min |

Temperature control and clock discipline share no state: thermal conditions
only raise LED3 and `thermal_warning` indications.
