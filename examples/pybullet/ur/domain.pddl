;Header and description

(define (domain ur-tamp)

;remove requirements that are not needed
(:requirements :strips :equality)


(:predicates
    
    (Arm ?a)
    (CanMove)

    ;static predicates
    (CuttingBoard ?o)
    (Pose ?o ?p)
    (Conf ?q)
    
    ;fluent predicates
    (AtConf ?q)
    (AtPose ?o ?p)

)


;define actions here
(:action move
    :parameters (?q1 ?o ?p ?t) ; configuration, object and target pose, trajectory?
    :precondition (and 

        (AtConf ?q1) (CanMove)
    )
    :effect (and (AtPose ?o ?p) (not (AtConf ?q1)) (not (CanMove))) ; should we use conf if we use cartesian planning?


)
)