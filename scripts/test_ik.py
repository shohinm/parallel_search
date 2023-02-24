from abb_fkik_solver import ABBFkIkSolver

fkik = ABBFkIkSolver("assets/abb/irb_1600/irb1600_6_12_generated.urdf")
print(fkik.computeFK([0,0,0,0,0,0]))
print(fkik.computeIK([0.815,0,0.9615], [0,0,0]))
