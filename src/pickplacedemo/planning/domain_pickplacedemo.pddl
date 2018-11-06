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
	(robot_at ?r - robot ?loc - location)
	(in ?c - cube ?p - pile)
	(on ?c - cube ?p - pile)
	(empty_hand ?r - robot)
	(in_hand ?r - robot ?c - cube)
)

(:durative-action goto_location
	:parameters (?r -robot ?source ?destination - location)
	:duration ( = ?duration 30)
	:condition (at start (robot_at ?r ?source))
	:effect (at end(robot_at ?r ?destination)) 
)

(:durative-action pick
	:parameters (?r - robot ?l - location ?c - cube) 
	:duration ( = ?duration 30)
	:condition 	(and 
				(over all (robot_at ?r ?l))
				(over all (cube_at ?c ?l))
				(over all (empty_hand ?r))
				)
	:effect 	(and 
				(at end (not (empty_hand ?r))) 
				(at end (in_hand ?r ?c))
				(at end (not (cube_at ?c ?l)))
				)
)

(:durative-action place
	:parameters (?r - robot ?l - location ?c - cube)
	:duration ( = ?duration 30)
	:condition 	(and 
				(over all (robot_at ?r ?l))
				(over all (in_hand ?r ?c))
				)
	:effect 	(and 
				(at end (empty_hand ?r))
				(at end (cube_at ?c ?l))
				)
)

;;(:durative-action check_landing
;;)

;;(:durative-action check_picking
;;)

;;(:durative-action check_location
;;)

;;(:durative-action remove_obstacles
;;)

)



;; Durative action template
;;(:durative-action dock
;;	:parameters ()
;;	:duration ()
;;	:condition ()
;;	:effect ()
;;)
