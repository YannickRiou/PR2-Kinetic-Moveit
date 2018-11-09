(define (problem task)
(:domain pickplacedemo)
(:objects
    tablelaas commode - location
    pr2_0 - robot
    cube0 cube0_0 cube0_1 - cube
)
(:init
    (cube_at cube0 tablelaas)
    (cube_at cube0_0 tablelaas)
    (cube_at cube0_1 tablelaas)


    (robot_at pr2_0 commode)

    (empty_hand pr2_0)



    (top_free cube0)
    (top_free cube0_0)
    (top_free cube0_1)

    (unready pr2_0)

)
(:goal (and
    (cube_at cube0 tablelaas)
    (cube_at cube0_0 tablelaas)
    (on_top cube0 cube0_0)
    (on_top cube0_0 cube0_1)
))
)
