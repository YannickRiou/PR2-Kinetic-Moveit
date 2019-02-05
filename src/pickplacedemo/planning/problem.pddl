(define (problem task)
(:domain pickplacedemo)
(:objects
    ikea_table_torsbyl commode - location
    pr2_0 - robot
    cube_blue cube_red cube_green - cube
)
(:init
    (cube_at cube_blue ikea_table_torsbyl)
    (cube_at cube_red ikea_table_torsbyl)
    (cube_at cube_green ikea_table_torsbyl)


    (robot_at pr2_0 commode)

    (empty_hand pr2_0)



    (top_free cube_blue)
    (top_free cube_red)
    (top_free cube_green)

    (not_ready pr2_0)


)
(:goal (and
    (cube_at cube_blue ikea_table_torsbyl)
    (cube_at cube_red ikea_table_torsbyl)
    (on_top cube_blue cube_red)
    (on_top cube_red cube_green)
))
)
