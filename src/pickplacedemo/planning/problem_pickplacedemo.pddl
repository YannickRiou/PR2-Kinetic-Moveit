(define (problem task)
    (:domain pickplacedemo)
    (:objects
        table commode - location
        cube_blue cube_red cube_green - cube
        pr2_0 - robot
    )
    (:init
        (robot_at pr2_0 commode)
        (cube_at cube_blue table)
        (cube_at cube_red table)
        (cube_at cube_green table)
        (empty_hand pr2_0)
        (top_free cube_blue)
        (top_free cube_red)
        (top_free cube_green)
        (not_ready pr2_0)
    )
    (:goal 
        (and
        (cube_at cube_blue table)
        (cube_at cube_red table)
        (on_top cube_blue cube_red)
        (on_top cube_red cube_green)
        )
    )
)