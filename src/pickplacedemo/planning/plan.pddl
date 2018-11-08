Number of literals: 35
Constructing lookup tables: [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
Post filtering unreachable actions:  [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
[01;34mNo analytic limits found, not considering limit effects of goal-only operators[00m
20% of the ground temporal actions in this problem are compression-safe
Initial heuristic = 14.000
b (13.000 | 60.000)b (12.000 | 60.000)b (11.000 | 90.001)b (10.000 | 90.001)b (9.000 | 90.001)b (8.000 | 150.002)b (7.000 | 150.002)b (4.000 | 180.002)b (2.000 | 180.002);;;; Solution Found
; States evaluated: 118
; Cost: 210.002
; Time 0.02
0.000: (goto_location pr2_1 commode tablelaas)  [60.000]
0.000: (pick pr2_1 commode cube0)  [30.000]
0.000: (goto_location pr2_0 tablelaas commode)  [60.000]
60.001: (placeat pr2_1 tablelaas cube0)  [30.000]
60.001: (pick pr2_0 commode cube0_1)  [30.000]
60.001: (goto_location pr2_1 tablelaas commode)  [60.000]
90.002: (placeat pr2_0 commode cube0_1)  [30.000]
90.002: (goto_location pr2_0 commode tablelaas)  [60.000]
120.002: (pick pr2_0 commode cube0_0)  [30.000]
120.002: (pick pr2_1 commode cube0_1)  [30.000]
120.002: (goto_location pr2_1 commode tablelaas)  [60.000]
150.002: (placeon pr2_0 tablelaas cube0 cube0_0)  [30.000]
180.002: (placeon pr2_1 tablelaas cube0_0 cube0_1)  [30.000]
