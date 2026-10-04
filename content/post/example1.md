+++
date = '2026-10-04T15:12:45+05:30'
draft = false
title = 'Control Basics'
author = 'Abdullah'
tags = ['control']
+++

# Control Basics

A control system measures what a system is doing, compares it with what we want, and adjusts the input to close the gap. This post covers the feedback loop, a first-order plant model, and the PID controller, with a short Python simulation at the end.

<!--more-->

## The feedback loop

Let $r(t)$ be the reference (what we want), $y(t)$ the measured output, and $u(t)$ the control input. The controller acts on the error

$$
e(t) = r(t) - y(t).
$$

The goal is to choose $u(t)$ so that $e(t) \to 0$ quickly, without large overshoot or oscillation.

## A first-order plant

A simple model for many physical systems, such as a heater or a motor speed loop, is a first-order lag:

$$
\tau \dot{y}(t) + y(t) = K\,u(t),
$$

where $K$ is the DC gain and $\tau$ is the time constant. Taking the Laplace transform gives the transfer function

$$
G(s) = \frac{Y(s)}{U(s)} = \frac{K}{\tau s + 1}.
$$

## PID control

The PID controller combines three terms:

$$
u(t) = K_p\, e(t) + K_i \int_0^t e(\sigma)\, d\sigma + K_d\, \dot{e}(t).
$$

- $K_p$ reacts to the present error.
- $K_i$ accumulates past error, which removes steady-state offset.
- $K_d$ reacts to the error trend and adds damping.

With proportional control only, the closed-loop steady-state output for a unit step is

$$
y_\infty = \frac{K K_p}{1 + K K_p}.
$$

For $K = 1$ and $K_p = 4$ this gives $y_\infty = 0.8$, so a pure P controller leaves a 20% error.

## Simulating it in Python

The script below simulates the plant with forward Euler under P and PI control.

```python
import numpy as np

def simulate(kp, ki=0.0, K=1.0, tau=1.0, r=1.0, dt=0.01, T=10.0):
    """Euler simulation of a first-order plant under PI control."""
    n = int(T / dt)
    t = np.arange(n) * dt
    y = np.zeros(n)
    integral = 0.0
    for k in range(1, n):
        e = r - y[k - 1]
        integral += e * dt
        u = kp * e + ki * integral
        y[k] = y[k - 1] + dt * (-y[k - 1] + K * u) / tau
    return t, y

for label, gains in {"P": dict(kp=4.0), "PI": dict(kp=4.0, ki=2.0)}.items():
    t, y = simulate(**gains)
    print(f"{label:>2}: y(10 s) = {y[-1]:.3f}")
```

Running it prints:

```text
 P: y(10 s) = 0.800
PI: y(10 s) = 0.998
```

The P controller settles at $0.8$, matching the formula above. Adding the integral term drives the output to the reference, $y \to 1$.

## Takeaways

1. Feedback works on the error $e = r - y$.
2. Proportional control alone leaves a steady-state error.
3. Integral action removes that error, at the cost of possible overshoot.
