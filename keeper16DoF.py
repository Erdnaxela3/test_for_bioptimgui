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


class SomersaultDirection:
    FORWARD = "forward"
    BACKWARD = "backward"


class PreferredTwistSide:
    RIGHT = "right"
    LEFT = "left"


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
    n_shooting = [24, 24, 24, 24]
    phase_time = [1.0, 1.0, 1.0, 1.0]
    final_time_margin = 0.1
    n_somersault = 1
    n_half_twist = [1, 0, 2, 4]
    preferred_twist_side = PreferredTwistSide.LEFT
    somersault_direction = (
        SomersaultDirection.BACKWARD
        if sum(n_half_twist) % 2 == 0
        else SomersaultDirection.FORWARD
    )

    bio_model = [BiorbdModel(r"models/AdChfull.bioMod") for _ in range(n_somersault)]
    # can't use * to have multiple, needs duplication

    # Declaration of the constraints and objectives of the ocp
    constraints = ConstraintList()
    objective_functions = ObjectiveList()
    objective_functions.add(
        objective=ObjectiveFcn.Lagrange.MINIMIZE_CONTROL,
        key="tau",
        node=Node.ALL_SHOOTING,
        weight=100.0,
    )
    objective_functions.add(
        objective=ObjectiveFcn.Mayer.MINIMIZE_TIME,
        min_bound=0.9,
        max_bound=1.1,
        node=Node.END,
        weight=1.0,
    )

    # Declaration of the dynamics function used during integration
    dynamics = DynamicsList()

    for phase in range(n_somersault):
        dynamics.add(
            DynamicsFcn.TORQUE_DRIVEN,
            expand=True,
            phase=phase,
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
    x_bounds[0]["q"].min[:, 0] = [0] * n_q
    x_bounds[0]["q"].min[:3, 0] = -0.001
    x_bounds[0]["q"].min[[7, 11], 0] = 2.9, -2.9

    x_bounds[0]["q"].max[:, 0] = -x_bounds[0]["q"].min[:, 0]
    x_bounds[0]["q"].max[[7, 11], 0] = 2.9, -2.9

    intermediate_min_bounds = [
        -1,  # transX
        -1,  # transY
        -0.1,  # transZ
        0,  # somersault to adapt
        -np.pi / 4,  # tilt
        0,  # twist to adapt
        -0.65,  # right upper arm rotation Z
        -0.05,  # right upper arm rotation Y
        -1.8,  # right forearm rotation Z
        -2.65,  # right forearm arm rotation X
        -2,  # left upper arm rotation Z
        -3,  # left upper arm rotation Y
        1.1,  # left forearm rotation Z
        -2.65,  # left forearm arm rotation X
        -2.7,  # thigh rotation X
        -0.1,  # thigh rotation Y
    ]

    intermediate_max_bounds = [
        1,  # transX
        1,  # transY
        10,  # transZ
        0,  # somersault to adapt
        np.pi / 4,  # tilt
        0,  # twist to adapt
        2,  # right upper arm rotation Z
        3,  # right upper arm rotation Y
        1.1,  # right forearm rotation Z
        0,  # right forearm arm rotation X
        0.65,  # left upper arm rotation Z
        0.05,  # left upper arm rotation Y
        1.8,  # left forearm rotation Z
        0,  # left forearm arm rotation X
        0.3,  # thigh rotation X
        0.1,  # thigh rotation Y
    ]

    for phase in range(n_somersault):
        if phase != 0:
            # initial bounds, same as final bounds of previous phase
            x_bounds[phase]["q"].min[:, 0] = x_bounds[phase - 1]["q"].min[:, 2]
            x_bounds[phase]["q"].max[:, 0] = x_bounds[phase - 1]["q"].max[:, 2]

        # Intermediate bounds, same for every phase
        x_bounds[phase]["q"].min[:, 1] = intermediate_min_bounds
        x_bounds[phase]["q"].min[3, 1] = (
            2 * np.pi * phase
            if somersault_direction == SomersaultDirection.FORWARD
            else -(2 * np.pi * (phase + 1))
        )
        x_bounds[phase]["q"].min[5, 1] = (
            np.pi * sum(n_half_twist[:phase]) - 0.2
            if preferred_twist_side == PreferredTwistSide.LEFT
            else -(np.pi * sum(n_half_twist[: phase + 1])) - 0.2
        )

        x_bounds[phase]["q"].max[:, 1] = intermediate_max_bounds
        x_bounds[phase]["q"].max[3, 1] = (
            2 * np.pi * (phase + 1)
            if somersault_direction == SomersaultDirection.FORWARD
            else -(2 * np.pi * phase)
        )
        x_bounds[phase]["q"].max[5, 1] = (
            np.pi * sum(n_half_twist[: phase + 1]) + 0.2
            if preferred_twist_side == PreferredTwistSide.LEFT
            else -(np.pi * sum(n_half_twist[:phase])) + 0.2
        )

        # Final bounds, used for next phase initial bounds
        x_bounds[phase]["q"].min[:, 2] = intermediate_min_bounds
        x_bounds[phase]["q"].min[3, 2] = (
            2 * np.pi * (phase + 1)
            if somersault_direction == SomersaultDirection.FORWARD
            else -2 * np.pi * (phase + 1)
        )
        x_bounds[phase]["q"].min[5, 2] = (
            np.pi * sum(n_half_twist[: phase + 1]) - 0.2
            if preferred_twist_side == PreferredTwistSide.LEFT
            else -(np.pi * sum(n_half_twist[: phase + 1])) - 0.2
        )

        x_bounds[phase]["q"].max[:, 2] = intermediate_max_bounds
        x_bounds[phase]["q"].max[3, 2] = (
            2 * np.pi * (phase + 1)
            if somersault_direction == SomersaultDirection.FORWARD
            else -2 * np.pi * (phase + 1)
        )
        x_bounds[phase]["q"].max[5, 2] = (
            np.pi * sum(n_half_twist[: phase + 1]) + 0.2
            if preferred_twist_side == PreferredTwistSide.LEFT
            else -(np.pi * sum(n_half_twist[: phase + 1])) + 0.2
        )

    # Final and last bounds
    x_bounds[n_somersault - 1]["q"].min[:, 2] = (
        np.array([-0.9, -0.9, 0, 0, 0, 0, 0, 2.9, 0, 0, 0, -2.9, 0, 0, 0, 0]) - 0.1
    )
    x_bounds[n_somersault - 1]["q"].min[3, 2] = (
        2 * np.pi * n_somersault - 0.1
        if somersault_direction == SomersaultDirection.FORWARD
        else -2 * np.pi * n_somersault - 0.1
    )
    x_bounds[n_somersault - 1]["q"].min[5, 2] = (
        np.pi * sum(n_half_twist) - 0.1
        if preferred_twist_side == PreferredTwistSide.LEFT
        else -np.pi * sum(n_half_twist) - 0.1
    )

    x_bounds[n_somersault - 1]["q"].max[:, 2] = (
        np.array([0.9, 0.9, 0, 0, 0, 0, 0, 2.9, 0, 0, 0, -2.9, 0, 0, 0, 0]) + 0.1
    )
    x_bounds[n_somersault - 1]["q"].max[3, 2] = (
        2 * np.pi * n_somersault + 0.1
        if somersault_direction == SomersaultDirection.FORWARD
        else -2 * np.pi * n_somersault + 0.1
    )
    x_bounds[n_somersault - 1]["q"].max[5, 2] = (
        np.pi * sum(n_half_twist) + 0.1
        if preferred_twist_side == PreferredTwistSide.LEFT
        else -np.pi * sum(n_half_twist) + 0.1
    )

    # Taken from https://github.com/EveCharbie/AnthropoImpactOnTech/blob/main/TechOpt83.py
    vzinit = (
        9.81 / 2 * phase_time[0]
    )  # vitesse initiale en z du CoM pour revenir a terre au temps final

    # Initial bounds
    x_bounds[0]["qdot"].min[:, 0] = [0] * n_qdot
    x_bounds[0]["qdot"].min[:2, 0] = -0.5
    x_bounds[0]["qdot"].min[2, 0] = vzinit - 2
    x_bounds[0]["qdot"].min[3, 0] = (
        0.5 if somersault_direction == SomersaultDirection.FORWARD else -20
    )

    x_bounds[0]["qdot"].max[:, 0] = -x_bounds[0]["qdot"].min[:, 0]
    x_bounds[0]["qdot"].max[2, 0] = vzinit + 2
    x_bounds[0]["qdot"].max[3, 0] = (
        20 if somersault_direction == SomersaultDirection.FORWARD else -0.5
    )

    for phase in range(n_somersault):
        if phase != 0:
            # initial bounds, same as final bounds of previous phase
            x_bounds[phase]["qdot"].min[:, 0] = x_bounds[phase - 1]["qdot"].min[:, 2]
            x_bounds[phase]["qdot"].max[:, 0] = x_bounds[phase - 1]["qdot"].max[:, 2]

        # Intermediate bounds
        x_bounds[phase]["qdot"].min[:, 1] = [-100] * n_qdot
        x_bounds[phase]["qdot"].min[:2, 1] = -10
        x_bounds[phase]["qdot"].min[3, 1] = (
            0.5 if somersault_direction == SomersaultDirection.FORWARD else -20
        )

        x_bounds[phase]["qdot"].max[:, 1] = [100] * n_qdot
        x_bounds[phase]["qdot"].max[:2, 1] = 10
        x_bounds[phase]["qdot"].max[3, 1] = (
            20 if somersault_direction == SomersaultDirection.FORWARD else -0.5
        )

        # Final bounds, same as intermediate
        x_bounds[phase]["qdot"].min[:, 2] = x_bounds[phase]["qdot"].min[:, 1]
        x_bounds[phase]["qdot"].max[:, 2] = x_bounds[phase]["qdot"].max[:, 1]

    x_inits = np.zeros((n_somersault, 2, n_q))

    x_inits[0] = np.array([0, 0, 0, 0, 0, 0, 0, 2.9, 0, 0, 0, -2.9, 0, 0, 0, 0])

    for phase in range(n_somersault):
        if phase != 0:
            x_inits[phase][0] = x_inits[phase - 1][1]

        x_inits[phase][1][3] = (
            2 * np.pi * (phase + 1)
            if somersault_direction == SomersaultDirection.FORWARD
            else -2 * np.pi * (phase + 1)
        )

        x_inits[phase][1][5] = (
            np.pi * sum(n_half_twist[: phase + 1])
            if preferred_twist_side == PreferredTwistSide.LEFT
            else -np.pi * sum(n_half_twist[: phase + 1])
        )

        x_inits[phase][1][[7, 11]] = 2.9, -2.9

    x_initial_guesses = InitialGuessList()

    for phase in range(n_somersault):
        x_initial_guesses.add(
            "q",
            initial_guess=x_inits[phase].T,
            interpolation=InterpolationType.LINEAR,
            phase=phase,
        )

    x_initial_guesses.add(
        "qdot",
        initial_guess=[0.0] * n_qdot,
        interpolation=InterpolationType.CONSTANT,
        phase=0,
    )

    u_bounds = BoundsList()
    for phase in range(n_somersault):
        u_bounds.add(
            "tau",
            min_bound=[tau_min] * n_tau,
            max_bound=[tau_max] * n_tau,
            interpolation=InterpolationType.CONSTANT,
            phase=phase,
        )

    u_initial_guesses = InitialGuessList()
    u_initial_guesses.add(
        "tau",
        initial_guess=[tau_init] * n_tau,
        interpolation=InterpolationType.CONSTANT,
    )

    mapping = BiMappingList()
    mapping.add(
        "tau",
        to_second=[None, None, None, None, None, None, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9],
        to_first=[6, 7, 8, 9, 10, 11, 12, 13, 14, 15],
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

    # solver = Solver.IPOPT()
    # --- Solve the ocp --- #
    solver = Solver.IPOPT(
        show_online_optim=True, show_options={"show_bounds": True}
    )  # debug purpose
    sol = ocp.solve(solver=solver)
    sol.graphs(show_bounds=True)  # debug purpose
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
