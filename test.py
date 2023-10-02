import numpy as np
import pytest
from bioptim import BoundsList, BiorbdModel

import gen
from gen import SomersaultDirection, PreferredTwistSide


def x_init_func_base(n_somersault, n_q, n_half_twist, preferred_twist_side):
    somersault_direction = (
        SomersaultDirection.BACKWARD
        if sum(n_half_twist) % 2 == 0
        else SomersaultDirection.FORWARD
    )

    x_inits = np.zeros((n_somersault, 2, n_q))

    x_inits[0] = np.array([0, 0, 0, 0, 0, 0, 0, 2.9, 0, 0, 0, -2.9, 0, 0, 0, 0])

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
                0,
                0,
                -2.9,
                0,
                0,
                0,
                0,
            ]
        )

    return x_inits


def x_bounds_func_base(
    n_somersault,
    bio_model,
    phase_time,
    n_half_twist,
    preferred_twist_side,
):
    somersault_direction = (
        SomersaultDirection.BACKWARD
        if sum(n_half_twist) % 2 == 0
        else SomersaultDirection.FORWARD
    )

    n_q = bio_model[0].nb_q
    n_qdot = bio_model[0].nb_qdot
    n_tau = bio_model[0].nb_tau - bio_model[0].nb_root

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
        0,  # right forearm rotation Z
        0,  # right forearm arm rotation X
        0,  # left upper arm rotation Z
        -2.9,  # left upper arm rotation Y
        0,  # left forearm rotation Z
        0,  # left forearm arm rotation X
        0,  # thigh rotation X
        0,  # thigh rotation Y
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
        0,  # right forearm rotation Z
        0,  # right forearm arm rotation X
        0,  # left upper arm rotation Z
        -2.9,  # left upper arm rotation Y
        0,  # left forearm rotation Z
        0,  # left forearm arm rotation X
        0,  # thigh rotation X
        0,  # thigh rotation Y
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
            -1.8,  # right forearm rotation Z
            -2.65,  # right forearm arm rotation X
            -2,  # left upper arm rotation Z
            -3,  # left upper arm rotation Y
            1.1,  # left forearm rotation Z
            -2.65,  # left forearm arm rotation X
            -2.7,  # thigh rotation X
            -0.1,  # thigh rotation Y
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
            1.1,  # right forearm rotation Z
            0,  # right forearm arm rotation X
            0.65,  # left upper arm rotation Z
            0.05,  # left upper arm rotation Y
            1.8,  # left forearm rotation Z
            0,  # left forearm arm rotation X
            0.3,  # thigh rotation X
            0.1,  # thigh rotation Y
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
            -1.8,  # right forearm rotation Z
            -2.65,  # right forearm arm rotation X
            -2,  # left upper arm rotation Z
            -3,  # left upper arm rotation Y
            1.1,  # left forearm rotation Z
            -2.65,  # left forearm arm rotation X
            -2.7,  # thigh rotation X
            -0.1,  # thigh rotation Y
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
            1.1,  # right forearm rotation Z
            0,  # right forearm arm rotation X
            0.65,  # left upper arm rotation Z
            0.05,  # left upper arm rotation Y
            1.8,  # left forearm rotation Z
            0,  # left forearm arm rotation X
            0.3,  # thigh rotation X
            0.1,  # thigh rotation Y
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
        -0.1,  # right forearm rotation Z
        -0.1,  # right forearm rotation X
        -0.1,  # left upper arm rotation Z
        -2.9 - 0.1,  # left upper arm rotation Y
        -0.1,  # left forearm rotation Z
        -0.1,  # left forearm rotation X
        -0.1,  # thigh rotation X
        -0.1,  # thigh rotation Y
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
        0.1,  # right forearm rotation Z
        0.1,  # right forearm rotation X
        0.1,  # left upper arm rotation Z
        -2.9 + 0.1,  # left upper arm rotation Y
        0.1,  # left forearm rotation Z
        0.1,  # left forearm rotation X
        0.1,  # thigh rotation X
        0.1,  # thigh rotation Y
    ]

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
        0,  # right forearm rotation Z
        0,  # right forearm rotation X
        0,  # left upper arm rotation Z
        0,  # left upper arm rotation Y
        0,  # left forearm rotation Z
        0,  # left forearm rotation X
        0,  # thigh rotation X
        0,  # thigh rotation Y
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
        0,  # right forearm rotation Z
        0,  # right forearm rotation X
        0,  # left upper arm rotation Z
        0,  # left upper arm rotation Y
        0,  # left forearm rotation Z
        0,  # left forearm rotation X
        0,  # thigh rotation X
        0,  # thigh rotation Y
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

    return x_bounds


@pytest.mark.parametrize(
    "n_somersault, phase_time, n_half_twist, preferred_twist_side",
    [
        (1, [0.5], [0], PreferredTwistSide.LEFT),
        (1, [0.5], [1], PreferredTwistSide.RIGHT),
        (1, [0.5], [2], PreferredTwistSide.LEFT),
        (1, [0.5], [3], PreferredTwistSide.RIGHT),
        (2, [0.5, 0.5], [0, 0], PreferredTwistSide.LEFT),
        (2, [0.5, 0.5], [0, 1], PreferredTwistSide.LEFT),
        (2, [0.5, 0.5], [1, 0], PreferredTwistSide.RIGHT),
        (2, [0.5, 0.5], [1, 1], PreferredTwistSide.RIGHT),
    ],
)
def test_bounds(
    n_somersault,
    phase_time,
    n_half_twist,
    preferred_twist_side,
):
    bio_model = [BiorbdModel(r"models/AdChfull.bioMod") for _ in range(n_somersault)]
    expected = x_bounds_func_base(
        n_somersault=n_somersault,
        bio_model=bio_model,
        phase_time=phase_time,
        n_half_twist=n_half_twist,
        preferred_twist_side=preferred_twist_side,
    )
    actual = gen.x_bounds_func(
        n_somersault=n_somersault,
        bio_model=bio_model,
        phase_time=phase_time,
        n_half_twist=n_half_twist,
        preferred_twist_side=preferred_twist_side,
    )

    for phase in range(n_somersault):
        for key in ["qdot", "q"]:
            for i in range(3):
                assert np.all(
                    expected[phase][key].min[:, i] == actual[phase][key].min[:, i]
                )
                assert np.all(
                    expected[phase][key].max[:, i] == actual[phase][key].max[:, i]
                )


@pytest.mark.parametrize(
    "n_somersault, n_half_twist, preferred_twist_side",
    [
        (1, [0], PreferredTwistSide.LEFT),
        (1, [1], PreferredTwistSide.RIGHT),
        (1, [2], PreferredTwistSide.LEFT),
        (1, [3], PreferredTwistSide.RIGHT),
        (2, [0, 0], PreferredTwistSide.LEFT),
        (2, [0, 1], PreferredTwistSide.LEFT),
        (2, [1, 0], PreferredTwistSide.RIGHT),
        (2, [1, 1], PreferredTwistSide.RIGHT),
    ],
)
def test_init(
    n_somersault,
    n_half_twist,
    preferred_twist_side,
):
    bio_model = [BiorbdModel(r"models/AdChfull.bioMod") for _ in range(n_somersault)]

    n_q = bio_model[0].nb_q

    expected = x_init_func_base(
        n_somersault=n_somersault,
        n_q=n_q,
        n_half_twist=n_half_twist,
        preferred_twist_side=preferred_twist_side,
    )
    actual = gen.x_init_func(
        n_somersault=n_somersault,
        n_q=n_q,
        n_half_twist=n_half_twist,
        preferred_twist_side=preferred_twist_side,
    )

    for phase in range(n_somersault):
        for i in range(2):
            assert np.all(expected[phase][i] == actual[phase][i])
