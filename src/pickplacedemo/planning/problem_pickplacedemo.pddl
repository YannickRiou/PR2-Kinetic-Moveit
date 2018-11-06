(define (problem task)
(:domain pickplacedemo)
(:objects
    tableLaas commode - location
    cube0 cube0_0 cube0_1 - cube
    pr2_0 pr2_1 - robot
)
(:init
    (robot_at pr2_0 tableLaas)
    (robot_at pr2_1 tableLaas)
    (cube_at cube0 commode)
    (cube_at cube0_0 commode)
    (cube_at cube0_1 commode)
    (empty_hand pr2_0)
    (empty_hand pr2_1)
)
(:goal 
    (and
    (cube_at cube0 tableLaas)
    (cube_at cube0_0 tableLaas)
    (cube_at cube0_1 tableLaas)
)))