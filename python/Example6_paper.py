from __future__ import annotations

from pathlib import Path

import numpy as np

from PID import PID
from process_and_simulation_data import load_process_and_simulation_data
from show_graphical_results import show_graphical_results

CURRENT_DIR = Path(__file__).resolve().parent

#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#%%%%%%%%%%%%%% Example 6: Gain scheduling %%%%%%%%%%%%%%%%%%%%%%%%%%%
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#
# This example shows how to use the proposed code for the implementation of 
# gain-scheduling approaches. We consider the classical tank level control 
# problem, which process dynamics is given by the following differential equation:
#
# dy(t)/dt=-a/A\sqrt(2gy(t))+u(t)/A
#
# The gain-scheduling control scheme can be implemented in two different ways: 
# by running all the controllers in parallel and switching among them based 
# on the current operating point; or by using a single controller and updating 
# its parameters based on the current operating point.Both solutions have 
# been used in this example and the code is presented below
#
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

def main() -> None:
    # Load process and simulation data
    example_id = 6
    d = load_process_and_simulation_data(example_id, CURRENT_DIR)

    dt = d["dt"]
    l = d["l"]

    # Instantiate PID controllers and initialization
    pid1 = PID()
    pid1.Dt = dt

    ti1 = d["t1"]
    kp1 = d["t1"] / (d["k1"] * (d["tc"] + l))
    ki1 = kp1 / ti1
    kd1 = 0.0

    pid1.kp = kp1
    pid1.ki = ki1
    pid1.kd = kd1
    pid1.b = 1.0
    pid1.c = 0.0
    pid1.Tt = ti1
    pid1.maxlim = float("inf")
    pid1.minlim = float("-inf")
    pid1.dumax = float("inf")
    pid1.dumin = float("-inf")
    pid1.Tf = dt

    uff = 0.0
    uman = 0.0
    utrack = 0.0

    # Define the second controller
    pid2 = pid1.copy()
    ti2 = d["t2"]
    kp2 = d["t2"] / (d["k2"] * (d["tc"] + l))
    ki2 = kp2 / ti2
    kd2 = 0.0
    pid2.kp = kp2
    pid2.ki = ki2
    pid2.kd = kd2
    pid2.Tt = ti2

    # Define the third controller
    pid3 = pid1.copy()
    ti3 = d["t3"]
    kp3 = d["t3"] / (d["k3"] * (d["tc"] + l))
    ki3 = kp3 / ti3
    kd3 = 0.0
    pid3.kp = kp3
    pid3.ki = ki3
    pid3.kd = kd3
    pid3.Tt = ti3

    u1 = d["q10"]
    u2 = d["q10"]
    u3 = d["q10"]

    # State initialization (r0,y0,u0,uff0)
    pid1.init(d["r"], d["h10"], u1, 0.0)
    pid2.init(d["r"], d["h10"], u2, 0.0)
    pid3.init(d["r"], d["h10"], u3, 0.0)

    # Controllers mode
    mode1 = "AUTO"
    mode2 = "TRACK"
    mode3 = "TRACK"

    # Control Loop Simulation
    n = len(d["uc"])
    lp = d["lp"]

    for i in range(lp + 1, n):
        # Simulation of process dynamics
        d["yc"][i] = d["yc"][i - 1] + dt * ((-d["a_orifice"] / d["a_tank"]) * np.sqrt(2.0 * d["g"] * d["yc"][i - 1]) + d["uc"][i - 1] / d["a_tank"])

        # Controller 1
        d["uc1"][i] = pid1.control(d["r"], d["yc"][i], uff, uman, utrack, mode1)
        # Controller 2
        d["uc2"][i] = pid2.control(d["r"], d["yc"][i], uff, uman, utrack, mode2)
        # Controller 3
        d["uc3"][i] = pid3.control(d["r"], d["yc"][i], uff, uman, utrack, mode3)

        # Simulation scenarios      
        if d["yc"][i] < 12.0:
            d["uc"][i] = d["uc1"][i]
            mode1, mode2, mode3 = "AUTO", "TRACK", "TRACK"
            d["modes"][i] = 1.0
        elif d["yc"][i] < 20.0:
            d["uc"][i] = d["uc2"][i]
            mode1, mode2, mode3 = "TRACK", "AUTO", "TRACK"
            d["modes"][i] = 2.0
        else:
            d["uc"][i] = d["uc3"][i]
            mode1, mode2, mode3 = "TRACK", "TRACK", "AUTO"
            d["modes"][i] = 3.0

        utrack = d["uc"][i]

        if (i + 1) * dt >= 10.0 and (i + 1) * dt < 200.0:
            d["r"] = 22.0
        elif (i + 1) * dt < 400.0:
            d["r"] = 3.0
        elif (i + 1) * dt < 600.0:
            d["r"] = 15.0

        d["rc"][i] = d["r"]

    # Second simulation for a single controller updating the parameters based on the operaiting poing
    # pid1 is the controller
    pid1.kp = kp1
    pid1.ki = ki1
    pid1.kd = kd1
    d["r"] = d["h10"]
    pid1.init(d["r"], d["h10"], u1, 0.0)

    uff = 0.0
    uman = 0.0
    utrack = 0.0
    mode = "AUTO"

    for i in range(lp + 1, n):
        d["ycs"][i] = d["ycs"][i - 1] + dt * ((-d["a_orifice"] / d["a_tank"]) * np.sqrt(2.0 * d["g"] * d["ycs"][i - 1]) + d["ucs"][i - 1] / d["a_tank"])

        if d["ycs"][i] < 12.0:
            pid1.kp = kp1
            pid1.ki = ki1
            pid1.kd = kd1
            d["modess"][i] = 1.0
        elif d["ycs"][i] < 20.0:
            pid1.kp = kp2
            pid1.ki = ki2
            pid1.kd = kd2
            d["modess"][i] = 2.0
        else:
            pid1.kp = kp3
            pid1.ki = ki3
            pid1.kd = kd3
            d["modess"][i] = 3.0

        if (i + 1) * dt >= 10.0 and (i + 1) * dt < 200.0:
            d["r"] = 22.0
        elif (i + 1) * dt < 400.0:
            d["r"] = 3.0
        elif (i + 1) * dt < 600.0:
            d["r"] = 15.0

        d["ucs"][i] = pid1.control(d["r"], d["ycs"][i], uff, uman, utrack, mode)
        d["rcs"][i] = d["r"]

    show_graphical_results(example_id, d)


if __name__ == "__main__":
    main()
