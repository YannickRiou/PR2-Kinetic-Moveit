(define (problem task)
(:domain pickplacedemo)
(:objects
    tablelaas commode - location
    pr2_0 - robot
    cube0 cube0_0 cube0_1 - cube
)
(:init
    (cube_at cube0 commode)
    (cube_at cube0_0 commode)
    (cube_at cube0_1 commode)

    (occupied tablelaas)
    (not (occupied commode))

    (robot_at pr2_0 tablelaas)

    (empty_hand pr2_0)



    (top_free cube0)
    (top_free cube0_0)
    (top_free cube0_1)

)
(:goal (and
    (cube_at cube0 tablelaas)
    (cube_at cube0_0 tablelaas)
    (on_top cube0 cube0_0)
    (on_top cube0_0 cube0_1)
))
)
