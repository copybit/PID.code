from __future__ import annotations

from pathlib import Path

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
    example_id = 7
    d = load_process_and_simulation_data(example_id, CURRENT_DIR)

    dt = d["dt"]
    t = d["t"]
    l = d["l"]
    k = d["k"]

    # Instantiate PID controllers and initialization
    pid1 = PID()

    pid1.Dt = dt                    # Sampling time
    ti = t                          # Integral time
    pid1.kp = t / (k * (t + l))     # Proportional gain
    pid1.ki = pid1.kp / ti          # Integral gain
    pid1.kd = 0.0                   # Derivative gain
    pid1.b = 1.0                    # b weigth for set-point weighting
    pid1.c = 0.0                    # c weigth for set-point weighting
    pid1.Tt = ti                    # Tracking constant for back-calculation scheme
    pid1.maxlim = float("inf")      # Maximum control limit signal
    pid1.minlim = float("-inf")     # Minimum control limit signal
    pid1.dumax = float("inf")       # Maximum Increment control limit signal
    pid1.dumin = float("-inf")      # Minimum Increment control limit signal
    pid1.Tf = dt                    # Filter Time constant

    # Input signals
    uff = 0.0                       # Feedforward signal
    uman = 0.0                      # Manual control signal
    utrack = 0.0                    # Tracking control signal   
    mode1 = "AUTO"                  
    mode2 = "TRACK"

    # Define the second controller
    pid2 = pid1.copy()

    u1 = 0.0
    u2 = 0.0
    pid1.init(d["r1"], d["r1"], u1, 0.0)
    pid2.init(d["r2"], d["r2"], u2, 0.0)

    n = len(d["uc"])
    lp = d["lp"]

    # Control Loop Simulation
    for i in range(lp + 1, n):

        # Simulation of process dynamics
        d["yc1"][i] = d["ap"] * d["yc1"][i - 1] + d["bp"] * (d["uc"][i - lp - 1] - d["v"][i])
        d["yc2"][i] = d["ap"] * d["yc2"][i - 1] + d["bp"] * d["uc"][i - lp - 1]

        # Controller 1
        d["uc1"][i] = pid1.control(d["r1"], d["yc1"][i], uff, uman, utrack, mode1)

        # Controller 2
        d["uc2"][i] = pid2.control(d["r2"], d["yc2"][i], uff, uman, utrack, mode2)

        # Selector
        if d["uc1"][i] < d["uc2"][i]:
            d["uc"][i] = d["uc1"][i]
            mode1 = "AUTO"
            mode2 = "TRACK"
        else:
            d["uc"][i] = d["uc2"][i]
            mode1 = "TRACK"
            mode2 = "AUTO"

        utrack = d["uc"][i]

    # Plotting graphical results
    show_graphical_results(example_id, d)


if __name__ == "__main__":
    main()
