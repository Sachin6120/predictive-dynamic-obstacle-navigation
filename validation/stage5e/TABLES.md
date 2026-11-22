### Table 1 — final comparison (all 9 conditions pooled)

|  | CV | Global Reach | Class-Aware Reach |
|---|---:|---:|---:|
| Trials launched | 45 | 45 | 45 |
| Infrastructure faults (excluded) | 0 | 2 | 1 |
| Trials scored | 45 | 43 | 44 |
| Success | 45/45 (1.000) | 43/43 (1.000) | 44/44 (1.000) |
| Collisions | 1 | 0 | 0 |
| Near misses (< 0.2 m) | 5 | 5 | 7 |
| Timeouts | 0 | 0 | 0 |
| Planner/controller failures | 0 | 0 | 0 |
| Trials with no global plan | 0 | 0 | 0 |
| Collision-monitor interventions | 1 | 0 | 3 |
| Worst clearance, m | -0.128 | 0.123 | 0.069 |
| Mean clearance, m | 0.658 ± 0.315 | 0.668 ± 0.303 | 0.665 ± 0.325 |
| Median clearance, m | 0.691 | 0.703 | 0.695 |
| Nav time, s | 19.013 ± 3.281 | 18.708 ± 3.195 | 19.002 ± 3.135 |
| Path length, m | 6.019 ± 0.316 | 6.028 ± 0.309 | 6.013 ± 0.317 |
| Stops | 2.07 ± 1.03 | 1.84 ± 0.78 | 1.82 ± 0.76 |
| Global path changes | 4.62 ± 2.08 | 4.42 ± 2.00 | 4.80 ± 2.08 |
| Predictive cells (max/trial) | 1122.6 ± 289.8 | 1187.8 ± 216.7 | 1325.5 ± 252.3 |
| Layer updateCosts mean, us | 25.07 ± 2.25 | 24.93 ± 2.81 | 25.36 ± 2.71 |
| Layer updateCosts max, us | 62.7 ± 22.0 | 57.7 ± 14.4 | 63.5 ± 23.1 |
| Layer rate, Hz | 5.023 ± 0.005 | 5.023 ± 0.004 | 5.023 ± 0.005 |
| Layer rate shortfalls | 0 | 0 | 0 |
| Costmap update failures | 0 | 0 | 0 |
| Tracker reach stage, us | 0.090 ± 0.029 | 6.430 ± 1.334 | 6.895 ± 1.320 |
| Capability containment (pooled, offline) | 0.0098 | 0.0171 | 1.0000 |
| Observed containment (pooled, offline) | 0.9888 | 0.9956 | 1.0000 |
| Contract breaches (live) | 0 | 66 | 0 |
| Unsafe profile events | 0 | 0 | 0 |

### Table 2 — clearance distribution, m (conflict conditions only; `ped_close_pass` excluded)

|  | n | min | p10 | p25 | median | p75 | max |
|---|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| CV | 40 | -0.128 | 0.133 | 0.507 | 0.675 | 0.747 | 0.978 |
| Global Reach | 38 | 0.123 | 0.149 | 0.440 | 0.675 | 0.728 | 0.996 |
| Class-Aware Reach | 39 | 0.069 | 0.122 | 0.478 | 0.691 | 0.792 | 0.992 |

### Table 3 — per-condition breakdown (success · collisions · worst clearance m · mean clearance m · nav time s · max predictive cells)

| condition | CV | Global Reach | Class-Aware Reach |
|---|:---|:---|:---|
| `ped_cross` | 5/5 · 0 · 0.691 · 0.705 · 18.94 · 986 | 5/5 · 0 · 0.708 · 0.718 · 18.74 · 1053 | 5/5 · 0 · 0.694 · 0.712 · 18.66 · 1208 |
| `ped_longitudinal` | 5/5 · 0 · 0.112 · 0.137 · 16.02 · 1040 | 5/5 · 0 · 0.123 · 0.136 · 16.48 · 1183 | 5/5 · 0 · 0.099 · 0.121 · 16.55 · 1321 |
| `bike_cross` | 5/5 · 0 · 0.681 · 0.700 · 23.20 · 1462 | 5/5 · 0 · 0.620 · 0.679 · 21.81 · 1536 | 5/5 · 0 · 0.655 · 0.693 · 23.56 · 1719 |
| `veh_cross` | 5/5 · 0 · 0.832 · 0.863 · 23.26 · 956 | 4/4 · 0 · 0.847 · 0.863 · 23.40 · 1030 | 5/5 · 0 · 0.860 · 0.876 · 23.32 · 1150 |
| `veh_longitudinal` | 5/5 · 1 · -0.128 · 0.210 · 17.22 · 2400 | 5/5 · 0 · 0.234 · 0.307 · 15.93 · 1061 | 5/5 · 0 · 0.069 · 0.215 · 17.16 · 1181 |
| `unknown_cross` | 5/5 · 0 · 0.962 · 0.971 · 20.37 · 1014 | 5/5 · 0 · 0.964 · 0.980 · 21.65 · 1030 | 5/5 · 0 · 0.975 · 0.981 · 20.13 · 1145 |
| `ped_oblique` | 5/5 · 0 · 0.554 · 0.561 · 19.85 · 1026 | 5/5 · 0 · 0.556 · 0.565 · 19.41 · 1144 | 5/5 · 0 · 0.560 · 0.566 · 19.84 · 1331 |
| `ped_close_pass` | 5/5 · 0 · 1.090 · 1.123 · 13.37 · 1119 | 5/5 · 0 · 1.100 · 1.128 · 13.41 · 1217 | 5/5 · 0 · 1.128 · 1.150 · 13.39 · 1303 |
| `ped_corner` | 5/5 · 0 · 0.620 · 0.651 · 18.88 · 1569 | 4/4 · 0 · 0.640 · 0.672 · 18.41 · 1677 | 4/4 · 0 · 0.643 · 0.672 · 18.24 · 2034 |

### Table 4 — pairwise differences (right − left) with the two-sigma-of-the-difference band

A difference whose magnitude is INSIDE the band supports no claim of superiority.

**CV → Global Reach** — Does consuming the global reachable set instead of the CV covariance change navigation behaviour?

| metric | left | right | diff | 2σ band | outside band? |
|---|---:|---:|---:|---:|---:|
| `min_clearance_m` | 0.658 | 0.668 | 0.010 | 0.132 | no |
| `nav_time_s` | 19.013 | 18.708 | -0.305 | 1.381 | no |
| `path_length_m` | 6.019 | 6.028 | 0.008 | 0.133 | no |
| `max_pred_cells` | 1122.556 | 1187.814 | 65.258 | 108.793 | no |
| `layer_update_us_mean` | 25.067 | 24.930 | -0.137 | 1.088 | no |
| `n_stops` | 2.067 | 1.837 | -0.230 | 0.390 | no |
| `primary_reaction_time_s` | 4.069 | 4.004 | -0.065 | 0.253 | no |
| `success / collisions / timeouts / near misses` | 45 / 1 / 0 / 5 | 43 / 0 / 0 / 5 | — | — | — |

**CV → Class-Aware Reach** — Does consuming the class-aware reachable set instead of the CV covariance change navigation behaviour?

| metric | left | right | diff | 2σ band | outside band? |
|---|---:|---:|---:|---:|---:|
| `min_clearance_m` | 0.658 | 0.665 | 0.007 | 0.136 | no |
| `nav_time_s` | 19.013 | 19.002 | -0.011 | 1.360 | no |
| `path_length_m` | 6.019 | 6.013 | -0.006 | 0.134 | no |
| `max_pred_cells` | 1122.556 | 1325.523 | 202.967 | 115.117 | YES |
| `layer_update_us_mean` | 25.067 | 25.359 | 0.292 | 1.058 | no |
| `n_stops` | 2.067 | 1.818 | -0.248 | 0.383 | no |
| `primary_reaction_time_s` | 4.069 | 3.991 | -0.078 | 0.246 | no |
| `success / collisions / timeouts / near misses` | 45 / 1 / 0 / 5 | 44 / 0 / 0 / 7 | — | — | — |

**Global Reach → Class-Aware Reach** — Does semantic class add anything to the reachable set at the planner level? (Stage-5D asked this; Stage-5E re-asks it on the wider condition set.)

| metric | left | right | diff | 2σ band | outside band? |
|---|---:|---:|---:|---:|---:|
| `min_clearance_m` | 0.668 | 0.665 | -0.003 | 0.135 | no |
| `nav_time_s` | 18.708 | 19.002 | 0.293 | 1.358 | no |
| `path_length_m` | 6.028 | 6.013 | -0.015 | 0.134 | no |
| `max_pred_cells` | 1187.814 | 1325.523 | 137.709 | 100.769 | YES |
| `layer_update_us_mean` | 24.930 | 25.359 | 0.429 | 1.184 | no |
| `n_stops` | 1.837 | 1.818 | -0.019 | 0.331 | no |
| `primary_reaction_time_s` | 4.004 | 3.991 | -0.013 | 0.237 | no |
| `success / collisions / timeouts / near misses` | 43 / 0 / 0 / 5 | 44 / 0 / 0 / 7 | — | — | — |

### Table 5 — CV prediction error by horizon, mean of per-trial means, m (measured in EVERY arm: the tracker publishes CV predictions in all three, only consumption differs)

| horizon, s | CV | Global Reach | Class-Aware Reach |
|---|---:|---:|---:|
| 0.5 | 0.2604 ± 0.1692 | 0.2555 ± 0.1601 | 0.2578 ± 0.1635 |
| 1.0 | 0.2797 ± 0.1588 | 0.2746 ± 0.1488 | 0.2757 ± 0.1515 |
| 1.5 | 0.3081 ± 0.1488 | 0.3023 ± 0.1387 | 0.3021 ± 0.1417 |
| 2.0 | 0.3459 ± 0.1421 | 0.3395 ± 0.1317 | 0.3379 ± 0.1341 |
| 2.5 | 0.3926 ± 0.1376 | 0.3851 ± 0.1284 | 0.3840 ± 0.1296 |
| 3.0 | 0.4457 ± 0.1373 | 0.4384 ± 0.1297 | 0.4374 ± 0.1310 |
| ADE | 0.3303 ± 0.1472 | 0.3241 ± 0.1374 | 0.3242 ± 0.1394 |
| FDE (3 s) | 0.4265 ± 0.1320 | 0.4191 ± 0.1226 | 0.4178 ± 0.1245 |
| tracking position error | 0.2504 ± 0.1757 | 0.2468 ± 0.1671 | 0.2494 ± 0.1702 |

### Table 6 — offline prediction quality, one replay set (2547 rows, 57 tracks)

`cells` is the planner-visible count inside the 0.55 m window; 529 is the whole window, i.e. SATURATED.

| horizon, s | CV cap | CV obs (k2.0 / k1.5) | CV area m² / cells | GR cap | GR obs | GR area m² / cells | CA cap | CA obs | CA area m² / cells |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0.5 | 0.0318 | 0.9590 / 0.9373 | 0.47 / 54 | 0.0365 | 0.9787 | 0.60 / 123 | 1.0000 | 1.0000 | 7.26 / 529 |
| 1.0 | 0.0082 | 0.9841 / 0.9620 | 2.33 / 260 | 0.0130 | 0.9982 | 3.62 / 529 | 1.0000 | 1.0000 | 97.70 / 529 |
| 1.5 | 0.0047 | 0.9981 / 0.9855 | 7.37 / 529 | 0.0082 | 1.0000 | 13.12 / 529 | 1.0000 | 1.0000 | 475.87 / 529 |
| 2.0 | 0.0047 | 1.0000 / 0.9995 | 18.52 / 529 | 0.0086 | 1.0000 | 35.66 / 529 | 1.0000 | 1.0000 | 1479.36 / 529 |
| 2.5 | 0.0047 | 1.0000 / 1.0000 | 39.92 / 529 | 0.0090 | 1.0000 | 79.12 / 529 | 1.0000 | 1.0000 | 3558.89 / 529 |
| 3.0 | 0.0047 | 1.0000 / 1.0000 | 76.86 / 529 | 0.0271 | 1.0000 | 151.74 / 529 | 1.0000 | 1.0000 | 7291.32 / 529 |

### Table 7 — live capability-profile usage (cumulative tracker counters, whole campaign)

| counter | CV | Global Reach | Class-Aware Reach |
|---|---:|---:|---:|
| `profile_samples_total` | 0 | 41712 | 43182 |
| `class_closure_rate` | — | 0.0000 | 0.5807 |
| `fallback_rate` | — | 0.0000 | 0.4193 |
| `transition_rate` | — | 0.0000 | 0.0000 |
| `global_profile_rate` | — | 1.0000 | 0.0000 |
| `unsafe_profile_events` | 0 | 0 | 0 |
| `prof_global` | 0 | 41712 | 0 |
| `prof_class` | 0 | 0 | 25074 |
| `prof_fb_unknown` | 0 | 0 | 16194 |
| `prof_fb_lowconf` | 0 | 0 | 1794 |
| `prof_fb_immature` | 0 | 0 | 120 |
| `prof_fb_badconfig` | 0 | 0 | 0 |
| `prof_transition` | 0 | 0 | 0 |

### Table 8 — arm integrity, read back per trial

| check | CV | Global Reach | Class-Aware Reach |
|---|:---|:---|:---|
| layer `prediction_mode` seen | cv_covariance | reachability | cv_covariance, reachability |
| mode mismatches | 0 | 2 | 1 |
| `max_influence_radius` recorded | 0.55 | 0.55 | 0.55 |
| window mismatches | 0 | 0 | 0 |
| class-aware banner seen | False | False | True |
| banner mismatches | 0 | 0 | 0 |

The mode mismatches counted here are exactly the three excluded infrastructure-fault trials, in which Nav2 or Gazebo never came up and no layer mode was ever read back. Among the 132 SCORED trials every arm read back its own layer mode, a 0.55 m window and the expected class-aware banner, with zero mismatches.

### Table 9 — tracker per-scan cost, median per trial then mean ± sd over trials, us

| G2_PERF stage | CV | Global Reach | Class-Aware Reach |
|---|---:|---:|---:|
| `scan_us` | 10500.66 ± 1483.23 | 10515.75 ± 1521.08 | 10278.12 ± 18.49 |
| `cluster_us` | 6.15 ± 2.21 | 6.07 ± 2.20 | 6.17 ± 2.20 |
| `deblend_us` | 7.41 ± 1.78 | 7.28 ± 1.69 | 7.39 ± 1.65 |
| `compound_us` | 0.68 ± 0.04 | 0.67 ± 0.04 | 0.68 ± 0.06 |
| `association_us` | 4.66 ± 1.00 | 4.64 ± 0.99 | 4.67 ± 0.94 |
| `publish_us` | 89.82 ± 3.95 | 104.16 ± 4.60 | 104.41 ± 5.60 |
| `callback_us` | 10745.13 ± 1469.00 | 10768.45 ± 1501.94 | 10527.93 ± 48.10 |
| `reach_us` | 0.090 ± 0.029 | 6.430 ± 1.334 | 6.895 ± 1.320 |
