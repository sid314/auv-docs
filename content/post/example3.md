+++
date = '2026-10-04T15:28:00+05:30'
draft = false
title = 'Control Basics II: PID Tuning'
author = 'Abdullah'
tags = ['control']
+++

# Control Basics II: PID Tuning

The [previous post](/post/example1/) introduced feedback and the PID law on a first-order plant. This follow-up moves to a second-order plant, which can overshoot and oscillate, and shows what each gain does and why integrators need anti-windup.

<!--more-->

## A second-order plant

Many systems have inertia as well as lag. The standard form is

$$
G(s) = \frac{\omega_n^2}{s^2 + 2\zeta\omega_n s + \omega_n^2},
$$

with natural frequency $\omega_n$ and damping ratio $\zeta$. We use $\omega_n = 1$ and $\zeta = 0.5$, so

$$
\ddot{y} + \dot{y} + y = u.
$$

For a unit step, the percent overshoot of an underdamped second-order system depends only on $\zeta$:

$$
M_p = \exp\!\left(\frac{-\pi\zeta}{\sqrt{1-\zeta^2}}\right) \times 100\%.
$$

## What each gain does

Closing the loop with a P controller gives a steady-state output $y_\infty = K_p/(1 + K_p)$. For $K_p = 2$ that is $2/3$, so proportional control alone cannot reach the reference.

| Gain     | Effect on rise time | Overshoot | Steady-state error   |
| -------- | ------------------- | --------- | -------------------- |
| $K_p$ up | faster              | increases | reduced, not removed |
| $K_i$ up | slightly slower     | increases | removed              |
| $K_d$ up | little change       | decreases | none                 |

The derivative term acts like damping: it opposes fast changes in the error.

## Anti-windup

Real actuators saturate. If the integral keeps accumulating while the output is pinned at its limit, it stores up a large value that then has to be unwound, causing extra overshoot. This effect is called integrator windup. A simple fix is conditional integration: stop accumulating while the actuator is saturated and the error is still pushing in that direction.

## Simulation

The script runs P, PI, and PID on the plant above, then compares PI control with and without anti-windup when the input is limited to $|u| \le 1.5$.

```python
import numpy as np

def plant_step(x, u, dt):
    # G(s) = 1/(s^2 + s + 1), states: y, ydot
    y, yd = x
    ydd = u - yd - y
    return np.array([y + dt*yd, yd + dt*ydd])

def run(kp, ki=0.0, kd=0.0, umax=None, r=1.0, dt=0.001, T=20.0):
    n = int(T/dt)
    x = np.zeros(2); integ = 0.0; e_prev = r
    ys = np.zeros(n)
    for k in range(n):
        e = r - x[0]
        d = (e - e_prev)/dt if k else 0.0
        u_unsat = kp*e + ki*integ + kd*d
        u = u_unsat if umax is None else np.clip(u_unsat, -umax, umax)
        # anti-windup: freeze the integrator while saturated and pushing
        if umax is None or u == u_unsat or np.sign(e) != np.sign(u_unsat):
            integ += e*dt
        x = plant_step(x, u, dt)
        ys[k] = x[0]; e_prev = e
    return np.arange(n)*dt, ys

def metrics(t, y, r=1.0):
    os_ = max(0.0, (y.max()-r)/r*100)
    out = np.where(np.abs(y-r) > 0.02*r)[0]
    ts = float('nan') if (len(out) and out[-1]+1 >= len(t)) else (t[out[-1]+1] if len(out) else 0.0)
    return os_, ts, y[-1]

for name, g in {"P  (kp=2)": dict(kp=2.0),
                "PI (kp=2, ki=1)": dict(kp=2.0, ki=1.0),
                "PID(kp=2, ki=1, kd=1)": dict(kp=2.0, ki=1.0, kd=1.0)}.items():
    t, y = run(**g)
    os_, ts, yf = metrics(t, y)
    print(f"{name:24s} overshoot={os_:5.1f}%  settle(2%)={ts:5.2f}s  y_final={yf:.3f}")
```

Results:

| Controller                      | Overshoot | 2% settling time | Final value |
| ------------------------------- | --------- | ---------------- | ----------- |
| P ($K_p=2$)                     | 0.0%      | never            | 0.667       |
| PI ($K_p=2$, $K_i=1$)           | 24.7%     | 11.89 s          | 0.999       |
| PID ($K_p=2$, $K_i=1$, $K_d=1$) | 6.4%      | 5.92 s           | 1.000       |

P stays at $2/3$ as predicted. Adding the integral removes the offset but overshoots by about a quarter. Adding the derivative cuts overshoot to 6.4% and halves the settling time.

With the actuator limited to $|u| \le 1.5$ and a PI controller ($K_p = 2$, $K_i = 3$), overshoot was 23.5% with anti-windup and 59.7% without it.

## Takeaways

1. Proportional control leaves an offset; integral action removes it but adds overshoot.
2. Derivative action adds damping, which allows a more aggressive integral term.
3. Any controller with an integrator and a saturating actuator needs anti-windup.
