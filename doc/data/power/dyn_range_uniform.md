# Baseline 4x8 vs Square 4x8 SC — Dynamic Range Power Study (Uniform Random)

Power (mW) as a function of input dynamic range. Rows sweep the 4-bit A input
(halved 3 times: 4 → 3 → 2 → 1 effective bits). Columns sweep the 8-bit B
input (halved 7 times: 8 → 7 → … → 1 effective bits). Each cell is the total
power from post-synthesis dynamic power analysis at 1.35 ns clock period
(ASAP7 RVT, OpenSTA).

**Input distribution:** each input is drawn independently and uniformly at
random from the signed integer range [−peak, +peak], where
peak = 2^(RANGE_BITS−1) − 1. All 2·peak + 1 integer values within the range
are equally likely.

## Baseline 4x8 — Total Power (mW)

| A \ B     | B=8 (1/1) | B=7 (1/2) | B=6 (1/4) | B=5 (1/8) | B=4 (1/16) | B=3 (1/32) | B=2 (1/64) | B=1 (1/128) |
|-----------|:---------:|:---------:|:---------:|:---------:|:----------:|:----------:|:----------:|:-----------:|
| A=4 (1/1) |   1.450   |   1.450   |   1.440   |   1.430   |   1.420    |   1.400    |   1.340    |    1.220    |
| A=3 (1/2) |   1.360   |   1.360   |   1.350   |   1.340   |   1.320    |   1.300    |   1.270    |    1.180    |
| A=2 (1/4) |   1.140   |   1.130   |   1.120   |   1.120   |   1.110    |   1.110    |   1.100    |    1.050    |
| A=1 (1/8) |   1.100   |   1.100   |   1.090   |   1.090   |   1.090    |   1.080    |   1.060    |    0.742    |

## Square 4x8 SC — Total Power (mW)

| A \ B     | B=8 (1/1) | B=7 (1/2) | B=6 (1/4) | B=5 (1/8) | B=4 (1/16) | B=3 (1/32) | B=2 (1/64) | B=1 (1/128) |
|-----------|:---------:|:---------:|:---------:|:---------:|:----------:|:----------:|:----------:|:-----------:|
| A=4 (1/1) |   1.200   |   1.180   |   1.170   |   1.160   |   1.170    |   1.180    |   1.180    |    1.180    |
| A=3 (1/2) |   1.160   |   1.120   |   1.100   |   1.090   |   1.090    |   1.110    |   1.120    |    1.120    |
| A=2 (1/4) |   1.150   |   1.100   |   1.060   |   1.050   |   1.050    |   1.060    |   1.070    |    1.070    |
| A=1 (1/8) |   1.140   |   1.090   |   1.040   |   1.020   |   1.020    |   1.030    |   1.040    |    1.030    |

## Baseline vs Square — Power Difference (%)

Positive = Baseline consumes more power than Square. Computed as
`(Bas − Sqr) / Bas × 100`.

| A \ B     | B=8 (1/1) | B=7 (1/2) | B=6 (1/4) | B=5 (1/8) | B=4 (1/16) | B=3 (1/32) | B=2 (1/64) | B=1 (1/128) |
|-----------|:---------:|:---------:|:---------:|:---------:|:----------:|:----------:|:----------:|:-----------:|
| A=4 (1/1) |  +17.2%   |  +18.6%   |  +18.8%   |  +18.9%   |  +17.6%    |  +15.7%    |  +11.9%    |   +3.3%     |
| A=3 (1/2) |  +14.7%   |  +17.6%   |  +18.5%   |  +18.7%   |  +17.4%    |  +14.6%    |  +11.8%    |   +5.1%     |
| A=2 (1/4) |   −0.9%   |   +2.7%   |   +5.4%   |   +6.3%   |   +5.4%    |   +4.5%    |   +2.7%    |   −1.9%     |
| A=1 (1/8) |   −3.6%   |   +0.9%   |   +4.6%   |   +6.4%   |   +6.4%    |   +4.6%    |   +1.9%    |  −38.8%     |
