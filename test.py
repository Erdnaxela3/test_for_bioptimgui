"""
This file was automatically generated using BioptimGUI version 0.0.1
"""

from bioptim import (
    BiorbdModel,
    OptimalControlProgram,
    DynamicsList,
    DynamicsFcn,
    BoundsList,
    InitialGuessList,
    ObjectiveList,
    ObjectiveFcn,
    ConstraintFcn,
    ConstraintList,
    InterpolationType,
    BiMappingList,
    Solver,
    Node,
)

import numpy as np

def prepare_ocp():
    """
    This function build an optimal control program and instantiate it.
    It can be seen as a factory for the OptimalControlProgram class.

    Parameters
    ----------
    # TODO fill this section

    Returns
    -------
    The OptimalControlProgram ready to be solved
    """


    # Declaration of generic elements
    bio_model = BiorbdModel(r"/home/aweng/afs/trampoOCP/models/AdCh.bioMod")

    n_shooting = 50
    phase_time = 1.0 # user-input
    n_somersault = 1
    n_half_twist = 2

    # Declaration of the constraints and objectives of the ocp
    constraints = ConstraintList()
    objective_functions = ObjectiveList()

    objective_functions.add(ObjectiveFcn.Lagrange.MINIMIZE_CONTROL, key="tau", node=Node.ALL_SHOOTING, weight=100)

    # Declaration of the dynamics function used during integration
    dynamics = DynamicsList()

    dynamics.add(DynamicsFcn.TORQUE_DRIVEN, expand=True)

    # Define control path constraint
    tau_min, tau_max, tau_init = -100, 100, 0    # KEEP ? minuscule TODO

    n_q = bio_model.nb_q
    n_qdot = bio_model.nb_qdot
    n_tau = bio_model.nb_tau

    # Declaration of optimization variables bounds and initial guesses
    # Path constraint
    x_bounds = BoundsList()
    x_bounds["q"] = bio_model.bounds_from_ranges("q")
    x_bounds["qdot"] = bio_model.bounds_from_ranges("qdot")

    x_initial_guesses = InitialGuessList()

    u_bounds = BoundsList()
    u_initial_guesses = InitialGuessList()

    # TOCHECK
    x_bounds.add(
        "q",
        min_bound = [
            [
                -1, # pelvis translation X
                -1, # pelvis translation Y
                -0.001, # pelvis translation Z
                -0.001, # pelvis rotation X, somersault
                -np.pi / 4, # pelvis rotation Y, tilt
                -np.pi, # pelvis rotation Z, twist
                -0.65, # right upper arm rotation Z
                -0.05, # right upper arm rotation Y
                -2, # left upper arm rotation Z
                -3, # left upper arm rotation Y
            ]
        ],
        max_bound = [
            [
                -0.1, # transX
                -0.1, # transY
                -0.1, # transZ
                2 * np.pi * n_somersault - 0.1, # somersault
                np.pi / 4, # tilt
                np.pi * n_half_twist, # twist
                2,
                3,
                0.65,
                0.05,
            ]
        ],
        interpolation=InterpolationType.CONSTANT_WITH_FIRST_AND_LAST_DIFFERENT,
    )

    x_initial_guesses.add(
        "q",
        initial_guess=[[0.0]],
        interpolation=InterpolationType.CONSTANT,
    )

    # TOCHECK
    x_bounds.add(
        "qdot",
        min_bound=[[tau_min] * 10],
        max_bound=[[tau_max] * 10],
        interpolation=InterpolationType.CONSTANT_WITH_FIRST_AND_LAST_DIFFERENT,
    )
    x_initial_guesses.add(
        "qdot",
        initial_guess=[[0.0]],
        interpolation=InterpolationType.CONSTANT,
    )

    # TOCHECK
    u_bounds.add(
        "tau",
        min_bound=[[tau_min] * n_tau],
        max_bound=[[tau_max] * n_tau],
        interpolation=InterpolationType.CONSTANT_WITH_FIRST_AND_LAST_DIFFERENT,
    )
    u_initial_guesses.add(
        "tau",
        initial_guess=[[tau_init] * n_tau],
        interpolation=InterpolationType.CONSTANT,
    ) 
 
    # TOCHECK
    mapping = BiMappingList()
    mapping.add("tau", [None, None, None, None, None, None, 0, 1, 2, 3], [6, 7, 8, 9])

    # Construct and return the optimal control program (OCP)
    return OptimalControlProgram(
        bio_model=bio_model,
        n_shooting=n_shooting,
        phase_time=phase_time,
        dynamics=dynamics,
        x_bounds=x_bounds,
        u_bounds=u_bounds,
        x_init=x_initial_guesses,
        u_init=u_initial_guesses,
        constraints=constraints,
        objective_functions=objective_functions,
        variable_mappings=mapping, # TOCHECK
        use_sx=True,
    )


def main():
    """
    If this file is run, then it will perform the optimization
    """

    # --- Prepare the ocp --- #
    ocp = prepare_ocp()

    # --- Solve the ocp --- #
    sol = ocp.solve(solver=Solver.IPOPT())
    sol.graphs(show_bounds=True) # TOCHECK
    sol.animate()


if __name__ == "__main__":
    main()
