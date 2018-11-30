Number of literals: 22
Constructing lookup tables: [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%] [110%] [120%] [130%] [140%]
Post filtering unreachable actions:  [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%] [110%] [120%] [130%] [140%]
[01;34mNo analytic limits found, not considering limit effects of goal-only operators[00m
24% of the ground temporal actions in this problem are compression-safe
Initial heuristic = 11.000
b (10.000 | 90.000)b (8.000 | 150.001)b (7.000 | 180.002)
Resorting to best-first search
b (10.000 | 90.000)b (8.000 | 150.001)b (7.000 | 180.002)b (4.000 | 210.003)b (2.000 | 240.003);;;; Solution Found
; States evaluated: 128
; Cost: 270.003
; Time 0.06
0.000: (setup pr2_0)  [90.000]
90.001: (goto_location pr2_0 commode tablelaas)  [60.000]
150.002: (pick pr2_0 tablelaas cube0_0)  [30.000]
180.003: (placeon pr2_0 tablelaas cube0_0 cube0)  [30.000]
210.003: (pick pr2_0 tablelaas cube0_1)  [30.000]
240.003: (placeon pr2_0 tablelaas cube0_1 cube0_0)  [30.000]
