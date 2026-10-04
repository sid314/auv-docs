+++
date = '2026-10-04T15:28:00+05:30'
draft = false
title = 'Perception Basics'
author = 'Abdullah'
tags = ['perception']
+++

# Perception Basics

A robot sees the world through a camera: a 3D scene is flattened into a 2D image. This post covers the pinhole camera model, how pixels relate to 3D points, and how a stereo pair recovers depth, with a short Python check.

<!--more-->

## The pinhole camera model

A 3D point $\mathbf{P} = (X, Y, Z)^\top$ in the camera frame, with $Z$ along the optical axis, projects to pixel coordinates

$$
u = f_x \frac{X}{Z} + c_x, \qquad v = f_y \frac{Y}{Z} + c_y.
$$

Here $f_x, f_y$ are the focal lengths in pixels and $(c_x, c_y)$ is the principal point. In matrix form, with the intrinsic matrix $K$,

$$
Z \begin{bmatrix} u \\ v \\ 1 \end{bmatrix}
= \underbrace{\begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix}}_{K}
\begin{bmatrix} X \\ Y \\ Z \end{bmatrix}.
$$

The division by $Z$ is why depth is lost in a single image: every point on the ray $\lambda\,(X, Y, Z)^\top$ lands on the same pixel.

## Distortion and calibration

Real lenses bend straight lines, so a distortion model is applied before $K$. The radial term is

$$
x_d = x\,(1 + k_1 r^2 + k_2 r^4), \qquad r^2 = x^2 + y^2,
$$

where $x = X/Z$ and $y = Y/Z$ are normalized coordinates. Calibration estimates $K$ and the distortion coefficients $k_i$ from images of a known pattern such as a checkerboard.{{% sidenote %}}Underwater, refraction at a flat port changes the effective field of view, so calibrate in water rather than in air.{{% /sidenote %}}

## Depth from stereo

Two cameras separated by a baseline $B$ see the same point at horizontal positions that differ by the disparity $d = u_L - u_R$. Depth follows from similar triangles:

$$
Z = \frac{f\,B}{d}.
$$

Differentiating shows how a disparity error $\delta d$ becomes a depth error:

$$
\delta Z \approx \frac{Z^2}{f\,B}\,\delta d.
$$

Depth error grows with the square of the distance, so stereo is precise up close and coarse far away.

## Checking it in Python

The script projects four 3D points, adds half-pixel noise, re-estimates $f_x$ and $c_x$ by least squares, and computes stereo depth for a camera with $f = 600$ px and $B = 0.05$ m.

```python
import numpy as np

K = np.array([[600.0, 0.0, 320.0],
              [0.0, 600.0, 240.0],
              [0.0,   0.0,   1.0]])

def project(K, P):
    """Project Nx3 camera-frame points to Nx2 pixel coordinates."""
    p = (K @ P.T).T
    return p[:, :2] / p[:, 2:3]

P = np.array([[ 0.0,  0.0, 2.0],
              [ 0.5,  0.2, 2.5],
              [-0.4,  0.3, 3.0],
              [ 0.3, -0.2, 4.0]])
uv = project(K, P)
print("pixels:\n", np.round(uv, 1))

# Estimate fx from noisy measurements of u = fx * X/Z + cx
rng = np.random.default_rng(0)
noisy = uv + rng.normal(0, 0.5, uv.shape)
x = P[:, 0] / P[:, 2]
A = np.column_stack([x, np.ones_like(x)])
(fx_hat, cx_hat), *_ = np.linalg.lstsq(A, noisy[:, 0], rcond=None)
print(f"fx_hat = {fx_hat:.1f}, cx_hat = {cx_hat:.1f}")

# Reprojection error with the true model
rmse = np.sqrt(np.mean(np.sum((noisy - uv) ** 2, axis=1)))
print(f"reprojection RMSE = {rmse:.2f} px")

# Stereo depth
f, B = 600.0, 0.05
for d in (30.0, 15.0, 7.5):
    print(f"disparity {d:5.1f} px -> Z = {f*B/d:.2f} m")
```

Output:

```text
pixels:
 [[320. 240.]
 [440. 288.]
 [240. 300.]
 [365. 210.]]
fx_hat = 602.1, cx_hat = 320.1
reprojection RMSE = 0.47 px
disparity  30.0 px -> Z = 1.00 m
disparity  15.0 px -> Z = 2.00 m
disparity   7.5 px -> Z = 4.00 m
```

Half a pixel of noise on four points moves $f_x$ from $600$ to about $602$. For the stereo pair, the error formula gives $\delta Z \approx 0.03$ m per pixel of disparity error at $1$ m, but about $0.5$ m per pixel at $4$ m.

## Takeaways

1. A pixel is a ray, not a point: depth needs a second view or another sensor.
2. The intrinsic matrix $K$ and distortion coefficients come from calibration.
3. Stereo depth error scales as $Z^2$, so plan for it when choosing the baseline $B$.
