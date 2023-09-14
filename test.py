"""
This file was automatically generated using BioptimGUI version 0.0.1
"""

import numpy as np
from bioptim import (
    BiorbdModel,
    OptimalControlProgram,
    DynamicsList,
    DynamicsFcn,
    BoundsList,
    InitialGuessList,
    ObjectiveList,
    ObjectiveFcn,
    ConstraintList,
    InterpolationType,
    BiMappingList,
    Solver,
    Node,
)
from casadi import MX, Function


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
    phase_time = 1.0  # TODO user-input to add
    final_time = 3  # TO CHECK
    n_somersault = 1
    n_half_twist = 2
    preferred_twist_side = "right"

    # Declaration of the constraints and objectives of the ocp
    constraints = ConstraintList()
    objective_functions = ObjectiveList()

    objective_functions.add(
        ObjectiveFcn.Lagrange.MINIMIZE_CONTROL,
        key="tau",
        node=Node.ALL_SHOOTING,
        weight=100,
    )

    # Declaration of the dynamics function used during integration
    dynamics = DynamicsList()

    dynamics.add(DynamicsFcn.TORQUE_DRIVEN, expand=True)

    # Define control path constraint
    tau_min, tau_max, tau_init = -100, 100, 0

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

    # Initial bounds
    x_bounds["q"].min[:, 0] = [
        -1,  # pelvis translation X
        -1,  # pelvis translation Y
        -0.001,  # pelvis translation Z
        0,  # pelvis rotation X, somersault
        0,  # pelvis rotation Y, tilt
        0,  # pelvis rotation Z, twist
        0,  # right upper arm rotation Z
        2.9,  # right upper arm rotation Y
        0,  # left upper arm rotation Z
        -2.9,  # left upper arm rotation Y
    ]
    x_bounds["q"].max[:, 0] = [
        1,  # pelvis translation X
        1,  # pelvis translation Y
        0.001,  # pelvis translation Z
        0,  # pelvis rotation X, somersault
        0,  # pelvis rotation Y, tilt
        0,  # pelvis rotation Z, twist
        0,  # right upper arm rotation Z
        2.9,  # right upper arm rotation Y
        0,  # left upper arm rotation Z
        -2.9,  # left upper arm rotation Y
    ]

    # Intermediate bounds
    x_bounds["q"].min[:, 1] = [
        -1,  # transX
        -1,  # transY
        -0.1,  # transZ
        -0.2,  # somersault
        -np.pi / 4,  # tilt
        -0.2,  # twist
        -0.65,  # right upper arm rotation Z
        -0.05,  # right upper arm rotation Y
        -2,  # left upper arm rotation Z
        -3,  # left upper arm rotation Y
    ]
    x_bounds["q"].max[:, 1] = [
        1,  # transX
        1,  # transY
        10,  # transZ
        2 * np.pi * n_somersault + 0.2,  # somersault
        np.pi / 4,  # tilt
        np.pi * n_half_twist + 0.2,  # twist
        2,  # right upper arm rotation Z
        3,  # right upper arm rotation Y
        0.65,  # left upper arm rotation Z
        0.05,  # left upper arm rotation Y
    ]

    # Final bounds
    x_bounds["q"].min[:, 2] = [
        -1,  # transX
        -1,  # transY
        -0.1,  # transZ
        2 * np.pi * n_somersault - 0.1,  # somersault
        -0.1,  # tilt
        np.pi * n_half_twist - 0.1,  # twist
        -0.1,  # right upper arm rotation Z
        2.9 - 0.1,  # right upper arm rotation Y
        -0.1,  # left upper arm rotation Z
        -2.9 - 0.1,  # left upper arm rotation Y
    ]
    x_bounds["q"].max[:, 2] = [
        1,  # transX
        1,  # transY
        0.1,  # transZ
        2 * np.pi * n_somersault + 0.1,  # somersault
        0.1,  # tilt
        np.pi * n_half_twist + 0.1,  # twist
        0.1,  # right upper arm rotation Z
        2.9 + 0.1,  # right upper arm rotation Y
        0.1,  # left upper arm rotation Z
        -2.9 + 0.1,  # left upper arm rotation Y
    ]

    x_init = np.array(
        [
            [0, 0, 0, 0, 0, 0, 0, 2.9, 0, -2.9],
            [
                0,
                0,
                0,
                2 * np.pi * n_somersault,
                0,
                np.pi * n_half_twist,
                0,
                2.9,
                0,
                -2.9,
            ],
        ]
    ).T
    x_initial_guesses.add(
        "q",
        initial_guess=x_init,
        interpolation=InterpolationType.LINEAR,
    )

    vzinit = (
        9.81 / 2 * final_time
    )  # vitesse initiale en z du CoM pour revenir a terre au temps final

    # decalage entre le bassin et le CoM
    CoM_Q_sym = MX.sym("CoM", n_q)
    CoM_Q_init = x_bounds[0].min[
        :n_q, 0
    ]  # min ou max ne change rien a priori, au DEBUT ils sont egaux normalement
    CoM_Q_func = Function(
        "CoM_Q_func", [CoM_Q_sym], [bio_model[0].center_of_mass(CoM_Q_sym)]
    )
    bassin_Q_func = Function(
        "bassin_Q_func",
        [CoM_Q_sym],
        [bio_model[0].homogeneous_matrices_in_global(CoM_Q_sym, 0).to_mx()],
    )  # retourne la RT du bassin

    x_bounds["qdot"].min = [[tau_min] * n_q]
    x_bounds["qdot"].max = ([[tau_max] * n_q],)

    x_initial_guesses.add(
        "qdot",
        initial_guess=[0.0] * n_qdot,
        interpolation=InterpolationType.CONSTANT,
    )

    u_bounds.add(
        "tau",
        min_bound=[tau_min] * n_tau,
        max_bound=[tau_max] * n_tau,
        interpolation=InterpolationType.CONSTANT,
    )
    u_initial_guesses.add(
        "tau",
        initial_guess=[tau_init] * n_tau,
        interpolation=InterpolationType.CONSTANT,
    )

    # TOCHECK
    mapping = BiMappingList()
    mapping.add(
        "tau",
        to_second=[None, None, None, None, None, None, 0, 1, 2, 3],
        to_first=[6, 7, 8, 9],
    )

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
        variable_mappings=mapping,
        use_sx=False,
        assume_phase_dynamics=True,
    )


def main():
    """
    If this file is run, then it will perform the optimization
    """

    # --- Prepare the ocp --- #
    ocp = prepare_ocp()

    # --- Solve the ocp --- #
    sol = ocp.solve(solver=Solver.IPOPT())
    sol.graphs(show_bounds=True)  # FOR CHECK PURPOSE
    sol.animate()


if __name__ == "__main__":
    main()
