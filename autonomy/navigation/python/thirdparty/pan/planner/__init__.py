"""Tour-then-plan planner: imitate the navmesh shortest-path expert given tour keyframes as spatial context.

    model.py       the Planner and its building blocks
    env.py         navmesh and simulator access, kinematics, closed-loop scoring
    expert.py      the privileged expert: shortest-path actions, episodes, chunk targets
    dataset.py     token stores, training items, frozen banks
    train.py       DDP imitation training
    eval.py        closed-loop evaluation
    eval_accum.py  closed-loop evaluation with online context accumulation

The model never imports env.py or expert.py, and the closed-loop decode never calls the expert.
The baselines live in the separate baselines/ repository.
"""
