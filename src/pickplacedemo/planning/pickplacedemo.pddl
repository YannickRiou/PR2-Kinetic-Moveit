(define (domain pickplacedemo)

(:requirements :strips :typing :fluents :disjunctive-preconditions :durative-actions)

(:types
	location
	robot
	cube
	pile
)

(:predicates
	(cube_at ?c - cube ?loc - location)
	(occupied ?loc - location)
	(robot_at ?r - robot ?loc -location)
	(in ?c - cube ?p - pile)
	(on ?c - cube ?p - pile)
	(holding ?r - robot ?c - cube)
)

(:durative-action goto_location
	

)

(:durative-action pick
	

)

(:durative-action place
	

)

(:durative-action check_landing
	

)

(:durative-action check_picking
	

)

(:durative-action check_location
	

)

(:durative-action remove_obstacles
	

)


)



;; Durative action template
;;(:durative-action dock
;;	:parameters ()
;;	:duration ()
;;	:condition ()
;;	:effect ()
;;)
