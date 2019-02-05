Number of literals: 22
Constructing lookup tables: [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%] [110%] [120%] [130%] [140%]
Post filtering unreachable actions:  [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%] [110%] [120%] [130%] [140%]
[01;34mNo analytic limits found, not considering limit effects of goal-only operators[00m
65% of the ground temporal actions in this problem are compression-safe
Initial heuristic = 9.000
b (8.000 | 90.000)b (6.000 | 150.001)b (5.000 | 180.002)b (3.000 | 210.003)b (2.000 | 240.003);;;; Solution Found
; States evaluated: 11
; Cost: 270.003
; Time 0.00
0.000: (setup pr2_0)  [90.000]
90.001: (goto_location pr2_0 commode ikea_table_torsbyl)  [60.000]
150.002: (pick pr2_0 ikea_table_torsbyl cube_green)  [30.000]
180.003: (placeon pr2_0 ikea_table_torsbyl cube_green cube_red)  [30.000]
210.003: (pick pr2_0 ikea_table_torsbyl cube_red)  [30.000]
240.003: (placeon pr2_0 ikea_table_torsbyl cube_red cube_blue)  [30.000]
