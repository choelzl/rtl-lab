# Baseline 4x8 vs Square 4x8 SC — Dynamic Range Power Study (Constant Amplitude)

Power (mW) as a function of input dynamic range. Rows sweep the 4-bit A input
(halved 3 times: 4 → 3 → 2 → 1 effective bits). Columns sweep the 8-bit B
input (halved 7 times: 8 → 7 → … → 1 effective bits). Each cell is the total
power from post-synthesis dynamic power analysis at 1.35 ns clock period
(ASAP7 RVT, OpenSTA).

**Input distribution:** each input is drawn independently and uniformly at
random from the two-point set {+peak, −peak}, where
peak = 2^(RANGE_BITS−1) − 1. Inputs always toggle at maximum amplitude for
the given range; only the sign changes randomly each cycle.

## Baseline 4x8 — Total Power (mW)

| A \ B     | B=8 (1/1) | B=7 (1/2) | B=6 (1/4) | B=5 (1/8) | B=4 (1/16) | B=3 (1/32) | B=2 (1/64) | B=1 (1/128) |
|-----------|:---------:|:---------:|:---------:|:---------:|:----------:|:----------:|:----------:|:-----------:|
| A=4 (1/1) |   1.450   |   1.450   |   1.450   |   1.450   |   1.450    |   1.450    |   1.440    |    0.292    |
| A=3 (1/2) |   1.460   |   1.460   |   1.460   |   1.460   |   1.460    |   1.460    |   1.450    |    0.285    |
| A=2 (1/4) |   1.120   |   1.120   |   1.120   |   1.120   |   1.110    |   1.110    |   1.100    |    0.276    |
| A=1 (1/8) |   0.508   |   0.509   |   0.509   |   0.509   |   0.510    |   0.510    |   0.511    |    0.044    |

## Square 4x8 SC — Total Power (mW)

| A \ B     | B=8 (1/1) | B=7 (1/2) | B=6 (1/4) | B=5 (1/8) | B=4 (1/16) | B=3 (1/32) | B=2 (1/64) | B=1 (1/128) |
|-----------|:---------:|:---------:|:---------:|:---------:|:----------:|:----------:|:----------:|:-----------:|
| A=4 (1/1) |   1.140   |   1.150   |   1.140   |   1.110   |   1.110    |   1.120    |   1.110    |    0.523    |
| A=3 (1/2) |   1.180   |   1.110   |   1.100   |   1.090   |   1.040    |   1.040    |   1.090    |    0.496    |
| A=2 (1/4) |   1.120   |   1.080   |   1.030   |   1.000   |   0.926    |   0.984    |   1.000    |    0.476    |
| A=1 (1/8) |   0.752   |   0.725   |   0.677   |   0.630   |   0.611    |   0.626    |   0.630    |    0.066    |

## Baseline vs Square — Power Difference (%)

Positive = Baseline consumes more power than Square. Computed as
`(Bas − Sqr) / Sqr × 100`.

| A \ B     | B=8 (1/1) | B=7 (1/2) | B=6 (1/4) | B=5 (1/8) | B=4 (1/16) | B=3 (1/32) | B=2 (1/64) | B=1 (1/128) |
|-----------|:---------:|:---------:|:---------:|:---------:|:----------:|:----------:|:----------:|:-----------:|
| A=4 (1/1) |  +27.2%   |  +26.1%   |  +27.2%   |  +30.6%   |  +30.6%    |  +29.5%    |  +29.7%    |   −44.2%    |
| A=3 (1/2) |  +23.7%   |  +31.5%   |  +32.7%   |  +33.9%   |  +40.4%    |  +40.4%    |  +33.0%    |   −42.5%    |
| A=2 (1/4) |   +0.0%   |   +3.7%   |   +8.7%   |  +12.0%   |  +19.9%    |  +12.8%    |  +10.0%    |   −42.0%    |
| A=1 (1/8) |  −32.4%   |  −29.8%   |  −24.8%   |  −19.2%   |  −16.5%    |  −18.5%    |  −18.9%    |   −33.3%    |
