(define (stream ur-tamp)

  ; Calls a motion planner to generate safe trajectories between pairs of configurations
  (:stream inverse-kinematics  ; should rename to 'motion'
    :inputs (?o ?p)
    :domain (and (Pose ?o ?p))
    :outputs (?t)
    :certified (and (Traj ?t))
  )
)
