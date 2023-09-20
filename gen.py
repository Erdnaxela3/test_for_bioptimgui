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
    InterpolationType,
    BiMappingList,
    Solver,
    Node,
    ConstraintList,
)


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
    n_shooting = (
        24,
        24,
        24,
        24,
    )
    phase_time = (  # time of each phase
        0.75,
        0.75,
        0.75,
        0.75,
    )
    final_time_margin = 0.1
    n_somersault = 4
    n_half_twist = (
        2,
        0,
        1,
        6,
    )
    preferred_twist_side = PreferredTwistSide.LEFT
    somersault_direction = (
        SomersaultDirection.BACKWARD
        if sum(n_half_twist) % 2 == 0
        else SomersaultDirection.FORWARD
    )

    bio_model = [
        BiorbdModel(r"/home/aweng/afs/trampoOCP/models/AdCh.bioMod")
        for _ in range(n_somersault)
    ]  # can't use * to have multiple, needs duplication

    # Declaration of the objectives of the ocp
    constraints = ConstraintList()  # keep empty for now

    objective_functions = ObjectiveList()

    for phase in range(n_somersault):
        objective_functions.add(
            ObjectiveFcn.Lagrange.MINIMIZE_CONTROL,
            key="tau",
            node=Node.ALL_SHOOTING,
            weight=100,
            phase=phase,
        )

        objective_functions.add(
            ObjectiveFcn.Mayer.MINIMIZE_TIME,
            min_bound=phase_time[phase] - final_time_margin,  # use current phase
            max_bound=phase_time[phase] + final_time_margin,
            weight=1,
            phase=phase,
        )

    # Declaration of the dynamics function used during integration
    dynamics = DynamicsList()

    for phase in range(n_somersault):
        dynamics.add(
            DynamicsFcn.TORQUE_DRIVEN,
            expand=True,  # TODO test with/without expand
            phase=phase,  # don't need it, but keep it for clarity
        )

    # Define control path constraint
    tau_min, tau_max, tau_init = -100, 100, 0

    n_q = bio_model[0].nb_q
    n_qdot = bio_model[0].nb_qdot
    n_tau = bio_model[0].nb_tau - bio_model[0].nb_root

    # Declaration of optimization variables bounds and initial guesses
    # Path constraint
    x_bounds = BoundsList()

    for phase in range(n_somersault):
        x_bounds.add("q", bio_model[phase].bounds_from_ranges("q"), phase=phase)
        x_bounds.add("qdot", bio_model[phase].bounds_from_ranges("qdot"), phase=phase)

    # Initial bounds
    x_bounds[0]["q"].min[:, 0] = [
        -0.001,  # pelvis translation X
        -0.001,  # pelvis translation Y
        -0.001,  # pelvis translation Z
        0,  # pelvis rotation X, somersault
        0,  # pelvis rotation Y, tilt
        0,  # pelvis rotation Z, twist
        0,  # right upper arm rotation Z
        2.9,  # right upper arm rotation Y
        0,  # left upper arm rotation Z
        -2.9,  # left upper arm rotation Y
    ]
    x_bounds[0]["q"].max[:, 0] = [
        0.001,  # pelvis translation X
        0.001,  # pelvis translation Y
        0.001,  # pelvis translation Z
        0,  # pelvis rotation X, somersault
        0,  # pelvis rotation Y, tilt
        0,  # pelvis rotation Z, twist
        0,  # right upper arm rotation Z
        2.9,  # right upper arm rotation Y
        0,  # left upper arm rotation Z
        -2.9,  # left upper arm rotation Y
    ]

    for phase in range(n_somersault):
        if phase != 0:
            # initial bounds, same as final bounds of previous phase
            x_bounds[phase]["q"].min[:, 0] = x_bounds[phase - 1]["q"].min[:, 2]
            x_bounds[phase]["q"].max[:, 0] = x_bounds[phase - 1]["q"].max[:, 2]

        # Intermediate bounds, same for every phase
        x_bounds[phase]["q"].min[:, 1] = [
            -1,  # transX
            -1,  # transY
            -0.1,  # transZ
            (
                2 * np.pi * phase
                if somersault_direction == SomersaultDirection.FORWARD
                else -(2 * np.pi * (phase + 1))
            ),  # somersault
            -np.pi / 4,  # tilt
            (
                np.pi * sum(n_half_twist[:phase]) - 0.2
                if preferred_twist_side == PreferredTwistSide.LEFT
                else -(np.pi * sum(n_half_twist[: phase + 1])) - 0.2
            ),  # twist
            -0.65,  # right upper arm rotation Z
            -0.05,  # right upper arm rotation Y
            -2,  # left upper arm rotation Z
            -3,  # left upper arm rotation Y
        ]
        x_bounds[phase]["q"].max[:, 1] = [
            1,  # transX
            1,  # transY
            10,  # transZ
            (
                2 * np.pi * (phase + 1)
                if somersault_direction == SomersaultDirection.FORWARD
                else -(2 * np.pi * phase)
            ),  # somersault
            np.pi / 4,  # tilt
            (
                np.pi * sum(n_half_twist[: phase + 1]) + 0.2
                if preferred_twist_side == PreferredTwistSide.LEFT
                else -(np.pi * sum(n_half_twist[:phase])) + 0.2
            ),  # twist
            2,  # right upper arm rotation Z
            3,  # right upper arm rotation Y
            0.65,  # left upper arm rotation Z
            0.05,  # left upper arm rotation Y
        ]

        # Final bounds, used for next phase initial bounds
        x_bounds[phase]["q"].min[:, 2] = [
            -1,  # transX
            -1,  # transY
            -0.1,  # transZ
            (
                2 * np.pi * (phase + 1)
                if somersault_direction == SomersaultDirection.FORWARD
                else -2 * np.pi * (phase + 1)
            ),  # somersault
            -np.pi / 4,  # tilt
            (
                np.pi * sum(n_half_twist[: phase + 1]) - 0.2
                if preferred_twist_side == PreferredTwistSide.LEFT
                else -(np.pi * sum(n_half_twist[: phase + 1])) - 0.2
            ),  # twist
            -0.65,  # right upper arm rotation Z
            -0.05,  # right upper arm rotation Y
            -2,  # left upper arm rotation Z
            -3,  # left upper arm rotation Y
        ]
        x_bounds[phase]["q"].max[:, 2] = [
            1,  # transX
            1,  # transY
            10,  # transZ
            (
                2 * np.pi * (phase + 1)
                if somersault_direction == SomersaultDirection.FORWARD
                else -2 * np.pi * (phase + 1)
            ),  # somersault
            np.pi / 4,  # tilt
            (
                np.pi * sum(n_half_twist[: phase + 1]) + 0.2
                if preferred_twist_side == PreferredTwistSide.LEFT
                else -(np.pi * sum(n_half_twist[: phase + 1])) + 0.2
            ),  # twist
            2,  # right upper arm rotation Z
            3,  # right upper arm rotation Y
            0.65,  # left upper arm rotation Z
            0.05,  # left upper arm rotation Y
        ]

    # Final and last bounds
    x_bounds[n_somersault - 1]["q"].min[:, 2] = [
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
            np.pi * sum(n_half_twist) - 0.1
            if preferred_twist_side == PreferredTwistSide.LEFT
            else -np.pi * sum(n_half_twist) - 0.1
        ),  # twist
        -0.1,  # right upper arm rotation Z
        2.9 - 0.1,  # right upper arm rotation Y
        -0.1,  # left upper arm rotation Z
        -2.9 - 0.1,  # left upper arm rotation Y
    ]
    x_bounds[n_somersault - 1]["q"].max[:, 2] = [
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
            np.pi * sum(n_half_twist) + 0.1
            if preferred_twist_side == PreferredTwistSide.LEFT
            else -np.pi * sum(n_half_twist) + 0.1
        ),  # twist
        0.1,  # right upper arm rotation Z
        2.9 + 0.1,  # right upper arm rotation Y
        0.1,  # left upper arm rotation Z
        -2.9 + 0.1,  # left upper arm rotation Y
    ]

    x_initial_guesses = InitialGuessList()

    x_inits = np.zeros((n_somersault, 2, n_q))

    x_inits[0] = np.array([0, 0, 0, 0, 0, 0, 0, 2.9, 0, -2.9])

    for phase in range(n_somersault):
        if phase != 0:
            x_inits[phase][0] = x_inits[phase - 1][1]

        x_inits[phase][1] = np.array(
            [
                0,
                0,
                0,
                (
                    2 * np.pi * (phase + 1)
                    if somersault_direction == SomersaultDirection.FORWARD
                    else -2 * np.pi * (phase + 1)
                ),  # somersault
                0,
                (
                    np.pi * sum(n_half_twist[: phase + 1])
                    if preferred_twist_side == PreferredTwistSide.LEFT
                    else -np.pi * sum(n_half_twist[: phase + 1])
                ),  # twist
                0,
                2.9,
                0,
                -2.9,
            ]
        )

        x_initial_guesses.add(
            "q",
            initial_guess=x_inits[phase].T,
            interpolation=InterpolationType.LINEAR,
            phase=phase,
        )

    # Taken from https://github.com/EveCharbie/AnthropoImpactOnTech/blob/main/TechOpt83.py
    vzinit = (
        9.81 / 2 * phase_time[0]
    )  # vitesse initiale en z du CoM pour revenir a terre au temps final

    # Initial bounds
    x_bounds[0]["qdot"].min[:, 0] = [
        -0.5,  # pelvis translation X speed
        -0.5,  # pelvis translation Y
        vzinit - 2,  # pelvis translation Z
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
    x_bounds[0]["qdot"].max[:, 0] = [
        0.5,  # pelvis translation X
        0.5,  # pelvis translation Y
        vzinit + 2,  # pelvis translation Z
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

    for phase in range(n_somersault):
        if phase != 0:
            # initial bounds, same as final bounds of previous phase
            x_bounds[phase]["qdot"].min[:, 0] = x_bounds[phase - 1]["qdot"].min[:, 2]
            x_bounds[phase]["qdot"].max[:, 0] = x_bounds[phase - 1]["qdot"].max[:, 2]

        # Intermediate bounds
        x_bounds[phase]["qdot"].min[:, 1] = [
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
        x_bounds[phase]["qdot"].max[:, 1] = [
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

        # Final bounds, same as intermediate
        x_bounds[phase]["qdot"].min[:, 2] = x_bounds[phase]["qdot"].min[:, 1]
        x_bounds[phase]["qdot"].max[:, 2] = x_bounds[phase]["qdot"].max[:, 1]

    x_initial_guesses.add(
        "qdot",
        initial_guess=[0.0] * n_qdot,
        interpolation=InterpolationType.CONSTANT,
        phase=0,
    )

    u_bounds = BoundsList()
    u_initial_guesses = InitialGuessList()

    for phase in range(n_somersault):
        u_bounds.add(
            "tau",
            min_bound=[tau_min] * n_tau,
            max_bound=[tau_max] * n_tau,
            interpolation=InterpolationType.CONSTANT,
            phase=phase,
        )

    u_initial_guesses.add(
        "tau",
        initial_guess=[tau_init] * n_tau,
        interpolation=InterpolationType.CONSTANT,
        # TODO check default behavior
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
        objective_functions=objective_functions,
        variable_mappings=mapping,
        use_sx=False,
        assume_phase_dynamics=True,
        constraints=constraints,
    )


def main():
    """
    If this file is run, then it will perform the optimization
    """

    # --- Prepare the ocp --- #
    ocp = prepare_ocp()

    # solver = Solver.IPOPT(show_online_optim=True, show_options={"show_bounds": True}) # debug purpose
    solver = Solver.IPOPT()
    # solver.set_maximum_iterations(0) # debug purpose
    # --- Solve the ocp --- #
    sol = ocp.solve(solver=solver)
    # sol.graphs(show_bounds=True)  # debug purpose
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
