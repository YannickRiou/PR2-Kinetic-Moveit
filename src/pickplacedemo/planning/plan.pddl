Number of literals: 18
Constructing lookup tables: [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
Post filtering unreachable actions:  [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
[01;34mNo analytic limits found, not considering limit effects of goal-only operators[00m
All the ground actions in this problem are compression-safe
Initial heuristic = 7.000
b (6.000 | 30.000)b (5.000 | 90.002)b (4.000 | 90.002)b (3.000 | 90.002)b (2.000 | 120.002)b (1.000 | 150.002);;;; Solution Found
; States evaluated: 11
; Cost: 150.002
; Time 0.00
0.000: (goto_location pr2_0 tablelaas commode)  [30.000]
0.000: (goto_location pr2_1 tablelaas commode)  [30.000]
30.001: (pick pr2_0 commode cube0)  [30.000]
30.001: (pick pr2_1 commode cube0_0)  [30.000]
60.002: (place pr2_0 tablelaas cube0)  [30.000]
60.002: (place pr2_1 tablelaas cube0_0)  [30.000]
90.002: (pick pr2_0 commode cube0_1)  [30.000]
120.002: (place pr2_0 tablelaas cube0_1)  [30.000]
