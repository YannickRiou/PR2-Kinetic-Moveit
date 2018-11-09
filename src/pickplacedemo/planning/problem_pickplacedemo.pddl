(define (problem task)
(:domain pickplacedemo)
(:objects
    tableLaas commode - location
    cube0 cube0_0 cube0_1 - cube
    pr2_0 - robot
)
(:init
    (unready pr2_0)
    (robot_at pr2_0 commode)
    (cube_at cube0 tableLaas)
    (cube_at cube0_0 tableLaas)
    (cube_at cube0_1 tableLaas)
    (empty_hand pr2_0)
    (top_free cube0)
    (top_free cube0_0)
    (top_free cube0_1)
)
(:goal 
    (and
    (cube_at cube0 tableLaas)
    (cube_at cube0_0 tableLaas)
    (on_top cube0 cube0_0)
    (on_top cube0_0 cube0_1)
)))