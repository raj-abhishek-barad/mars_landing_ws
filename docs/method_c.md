# Method C — Uncertainty-Aware Terrain-Relative Guidance

## Overview

Method C is a robust powered descent guidance law that combines three terms:

```
a_cmd = a_ZEM/ZEV  +  a_barrier  +  a_MSS
```

### 1. ZEM/ZEV (Zero Effort Miss / Zero Effort Velocity)

Classical optimal guidance that computes the acceleration needed to reach the target assuming no future control:

```
ZEM = rf - (r + v*tgo + 0.5*g*tgo²)    ← position miss if we do nothing
ZEV = vf - (v + g*tgo)                  ← velocity miss if we do nothing

a_ZEM/ZEV = (6/tgo²)*ZEM - (2/tgo)*ZEV
```

### 2. Barrier Penalty Term

Adds repulsive acceleration when the lander approaches terrain obstacles. The terrain is approximated as a stepped cone (funnel) defined by heights h[] and widths w[].

```
d = distance vector from lander to nearest barrier surface
φ = clipped penalty function of (d, v)
p_dot = gradient of barrier penalty

a_barrier = (tgo²/12) * p_dot
```

The barrier parameters h[], w[] are updated in real time from the perception node.

### 3. MSS Robust Term (Method C specific)

Modified Super-Twisting Sliding Mode term that rejects disturbances and model errors:

```
S = e_v + (k/tgo) * e_r          ← sliding surface
e_r = r - r_ref                   ← position error from reference
e_v = v - v_ref                   ← velocity error from reference

a_MSS = -K1 * tanh(S / φ_dynamic)
```

The boundary layer φ_dynamic is **uncertainty-aware**:
```
φ_dynamic = φ_base + β * trace(P_pos)
```
where `P_pos` is the 3x3 position covariance from the EKF. When the EKF is uncertain about position, the boundary layer widens — preventing chattering in high-uncertainty conditions.

### Thrust Saturation

```
T_min = 0.2 × 31000 = 6200 N
T_max = 0.8 × 31000 = 24800 N
Isp = 225 s
mdot = -|T| / (Isp × |g_earth|)
```

## Parameters

| Parameter | Value | Description |
|---|---|---|
| K1 | 8.0 | MSS gain |
| k | 3.0 | Sliding surface gain |
| φ_base | 10.0 | Base boundary layer |
| β | 2.0 | Uncertainty sensitivity |
| T_min | 6200 N | Min thrust |
| T_max | 24800 N | Max thrust |
| Isp | 225 s | Specific impulse |
| m0 | 2000 kg | Initial mass |

## Method Comparison

| Method | ZEM/ZEV | Barrier | MSS Robust |
|---|---|---|---|
| A | ✅ distance penalty | ❌ | ❌ |
| B | ✅ distance+velocity penalty | ❌ | ❌ |
| **C** | ✅ distance+velocity penalty | ✅ | ✅ |

Method C is the most robust — it handles disturbances (wind, model errors) that Methods A and B cannot reject.
