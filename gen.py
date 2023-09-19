"""
This file was automatically generated using BioptimGUI version 0.0.1
"""

import pickle as pkl

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

    class SomersaultDirection:
        FORWARD = "forward"
        BACKWARD = "backward"

    class PreferredTwistSide:
        RIGHT = "right"
        LEFT = "left"

    # Declaration of generic elements
    bio_model = BiorbdModel(r"/home/aweng/afs/trampoOCP/models/AdCh.bioMod")

    n_shooting = 24
    phase_time = 1.5  # TODO user-input to add
    final_time = 1.5  # TO CHECK
    n_somersault = 1
    n_half_twist = 0
    preferred_twist_side = PreferredTwistSide.LEFT
    somersault_direction = (
        SomersaultDirection.BACKWARD
        if n_half_twist % 2 == 0
        else SomersaultDirection.FORWARD
    )

    # Declaration of the constraints and objectives of the ocp
    constraints = ConstraintList()
    objective_functions = ObjectiveList()

    objective_functions.add(
        ObjectiveFcn.Lagrange.MINIMIZE_CONTROL,
        key="tau",
        node=Node.ALL_SHOOTING,
        weight=100,
    )

    objective_functions.add(
        ObjectiveFcn.Mayer.MINIMIZE_TIME, min_bound=0.0, max_bound=final_time, weight=1
    )

    # Declaration of the dynamics function used during integration
    dynamics = DynamicsList()

    dynamics.add(DynamicsFcn.TORQUE_DRIVEN, expand=True)

    # Define control path constraint
    tau_min, tau_max, tau_init = -100, 100, 0

    n_q = bio_model.nb_q
    n_qdot = bio_model.nb_qdot
    n_tau = bio_model.nb_tau - bio_model.nb_root

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
        (
            -0.2
            if somersault_direction == SomersaultDirection.FORWARD
            else -(2 * np.pi * n_somersault + 0.2)
        ),  # somersault
        -np.pi / 4,  # tilt
        (
            -0.2
            if preferred_twist_side == PreferredTwistSide.LEFT
            else -(np.pi * n_half_twist + 0.2)
        ),  # twist
        -0.65,  # right upper arm rotation Z
        -0.05,  # right upper arm rotation Y
        -2,  # left upper arm rotation Z
        -3,  # left upper arm rotation Y
    ]
    x_bounds["q"].max[:, 1] = [
        1,  # transX
        1,  # transY
        10,  # transZ
        (
            2 * np.pi * n_somersault + 0.2
            if somersault_direction == SomersaultDirection.FORWARD
            else 0.2
        ),  # somersault
        np.pi / 4,  # tilt
        (
            np.pi * n_half_twist + 0.2
            if preferred_twist_side == PreferredTwistSide.LEFT
            else 0.2
        ),  # twist
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
        (
            2 * np.pi * n_somersault - 0.1
            if somersault_direction == SomersaultDirection.FORWARD
            else -2 * np.pi * n_somersault - 0.1
        ),  # somersault
        -0.1,  # tilt
        (
            np.pi * n_half_twist - 0.1
            if preferred_twist_side == PreferredTwistSide.LEFT
            else -np.pi * n_half_twist - 0.1
        ),  # twist
        -0.1,  # right upper arm rotation Z
        2.9 - 0.1,  # right upper arm rotation Y
        -0.1,  # left upper arm rotation Z
        -2.9 - 0.1,  # left upper arm rotation Y
    ]
    x_bounds["q"].max[:, 2] = [
        1,  # transX
        1,  # transY
        0.1,  # transZ
        (
            2 * np.pi * n_somersault + 0.1
            if somersault_direction == SomersaultDirection.FORWARD
            else -2 * np.pi * n_somersault + 0.1
        ),  # somersault
        0.1,  # tilt
        (
            np.pi * n_half_twist + 0.1
            if preferred_twist_side == PreferredTwistSide.LEFT
            else -np.pi * n_half_twist + 0.1
        ),  # twist
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
                (
                    2 * np.pi * n_somersault
                    if somersault_direction == SomersaultDirection.FORWARD
                    else -2 * np.pi * n_somersault
                ),  # somersault
                0,
                (
                    np.pi * n_half_twist
                    if preferred_twist_side == PreferredTwistSide.LEFT
                    else -np.pi * n_half_twist
                ),  # twist
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

    # Taken from https://github.com/EveCharbie/AnthropoImpactOnTech/blob/main/TechOpt83.py
    vzinit = (
        9.81 / 2 * final_time
    )  # vitesse initiale en z du CoM pour revenir a terre au temps final

    # decalage entre le bassin et le CoM
    co_m_q_sym = MX.sym("CoM", n_q)
    co_m_q_init = x_bounds["q"].min[
        :n_q, 0
    ]  # min ou max ne change rien a priori, au DEBUT ils sont egaux normalement
    co_m_q_func = Function(
        "co_m_q_func", [co_m_q_sym], [bio_model.center_of_mass(co_m_q_sym)]
    )
    bassin_q_func = Function(
        "bassin_q_func",
        [co_m_q_sym],
        [bio_model.homogeneous_matrices_in_global(co_m_q_sym, 0).to_mx()],
    )  # retourne la RT du bassin

    r = (
        np.array(co_m_q_func(co_m_q_init)).reshape(1, 3)
        - np.array(bassin_q_func(co_m_q_init))[-1, :3]
    )  # selectionne seulement la translation de la RT

    x_bounds["qdot"].min[:, 0] = [
        -0.5,  # pelvis translation X speed
        -0.5,  # pelvis translation Y
        -vzinit - 0.5,  # pelvis translation Z
        (
            0.5 if somersault_direction == SomersaultDirection.FORWARD else -20
        ),  # pelvis rotation X, somersault
        0,  # pelvis rotation Y, tilt
        0,  # pelvis rotation Z, twist
        0,  # right upper arm rotation Z
        0,  # right upper arm rotation Y
        0,  # left upper arm rotation Z
        0,  # left upper arm rotation Y
    ]
    x_bounds["qdot"].max[:, 0] = [
        0.5,  # pelvis translation X
        0.5,  # pelvis translation Y
        vzinit + 0.5,  # pelvis translation Z
        (
            20 if somersault_direction == SomersaultDirection.FORWARD else -0.5
        ),  # pelvis rotation X, somersault
        0,  # pelvis rotation Y, tilt
        0,  # pelvis rotation Z, twist
        0,  # right upper arm rotation Z
        0,  # right upper arm rotation Y
        0,  # left upper arm rotation Z
        0,  # left upper arm rotation Y
    ]

    if somersault_direction == SomersaultDirection.FORWARD :
        borne_inf = (x_bounds["q"].min[0:3, 0] + np.cross(r, x_bounds["qdot"].min[3:6, 0]))[
            0
        ]
        borne_sup = (x_bounds["q"].min[0:3, 0] + np.cross(r, x_bounds["qdot"].max[3:6, 0]))[
            0
        ]
    else:
        borne_inf = (x_bounds["q"].min[0:3, 0] - np.cross(r, x_bounds["qdot"].min[3:6, 0]))[
            0
        ]
        borne_sup = (x_bounds["q"].min[0:3, 0] - np.cross(r, x_bounds["qdot"].max[3:6, 0]))[
            0
        ]

    x_bounds["qdot"].min[0:3, 0] = (
        min(borne_sup[0], borne_inf[0]),
        min(borne_sup[1], borne_inf[1]),
        min(borne_sup[2], borne_inf[2]),
    )

    x_bounds["qdot"].max[0:3, 0] = (
        max(borne_sup[0], borne_inf[0]),
        max(borne_sup[1], borne_inf[1]),
        max(borne_sup[2], borne_inf[2]),
    )

    # Intermediate bounds
    x_bounds["qdot"].min[:, 1] = [
        -10,
        -10,
        -100,
        (0.5 if somersault_direction == SomersaultDirection.FORWARD else -20),
        -100,
        -100,
        -100,
        -100,
        -100,
        -100,
    ]
    x_bounds["qdot"].max[:, 1] = [
        10,
        10,
        100,
        (20 if somersault_direction == SomersaultDirection.FORWARD else -0.5),
        100,
        100,
        100,
        100,
        100,
        100,
    ]

    # Final bounds
    x_bounds["qdot"].min[:, 2] = [
        -10,
        -10,
        -100,
        (0.5 if somersault_direction == SomersaultDirection.FORWARD else -20),
        -100,
        -100,
        -100,
        -100,
        -100,
        -100,
    ]
    x_bounds["qdot"].max[:, 2] = [
        10,
        10,
        100,
        (20 if somersault_direction == SomersaultDirection.FORWARD else -0.5),
        100,
        100,
        100,
        100,
        100,
        100,
    ]

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

    solver = Solver.IPOPT()
    # solver.set_maximum_iterations(0) # debug purpose
    # --- Solve the ocp --- #
    sol = ocp.solve(solver=solver)
    # sol.graphs(show_bounds=True)
    sol.animate()

    out = sol.integrate(merge_phases=True)
    state, time_vector = out._states["unscaled"], out._time_vector

    save = {
        "solution": sol,
        "unscaled_state": state,
        "time_vector": time_vector,
    }

    del sol.ocp
    with open(f"somersault.pkl", "wb") as f:
        pkl.dump(save, f)


if __name__ == "__main__":
    main()
