(define (problem task)
(:domain pickplacedemo)
(:objects
    tablelaas commode - location
    pr2_0 pr2_1 - robot
    cube0 cube0_0 cube0_1 - cube
)
(:init
    (cube_at cube0 commode)
    (cube_at cube0_0 commode)
    (cube_at cube0_1 commode)


    (robot_at pr2_0 tablelaas)
    (robot_at pr2_1 tablelaas)



    (empty_hand pr2_0)
    (empty_hand pr2_1)


)
(:goal (and
    (cube_at cube0 tablelaas)
    (cube_at cube0_0 tablelaas)
    (cube_at cube0_1 tablelaas)
))
)
