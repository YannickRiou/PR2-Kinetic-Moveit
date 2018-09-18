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



)



;; Durative action template
;;(:durative-action dock
;;	:parameters ()
;;	:duration ()
;;	:condition ()
;;	:effect ()
;;)
