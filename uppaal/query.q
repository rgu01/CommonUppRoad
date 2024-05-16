strategy safe = control:A[] !dStateEgo.detection.collide && !dStateEgo.detection.outside
strategy reachSafe = minE(D) [<=MAXTIME] {egoController.location}->{dStateEgo.position.x, dStateEgo.position.y, dStateEgo.velocity, dStateEgo.orientation, dStateEgo.acceleration, dStateEgo.yawRate}: <> time>=MAXTIME under safe
simulate [<=MAXTIME] { obs1.vars.x, obs1.vars.y, obs1.vars.orientation, obs1.vars.velocity, obs1.vars.acceleration, varsEgo.x, varsEgo.y, varsEgo.orientation, varsEgo.velocity, varsEgo.acceleration } under reachSafe
