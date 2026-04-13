from __future__ import annotations

from pathlib import Path

from PID import PID
from process_and_simulation_data import load_process_and_simulation_data
from show_graphical_results import show_graphical_results

CURRENT_DIR = Path(__file__).resolve().parent

#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#%%%%%%%%%%%%%% Example 5: Noise filtering %%%%%%%%%%%%%%%%%%%%%%%%%%%
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#
# This example shows how the control signal can be properly filtered by 
# tuning the T_f parameter as input to the filter function. The same process 
# transfer function and PI-controller as used in Example 1 have been used 
# for this simulation, where a white noise signal was added to the process output. 
# Three cases are simulated: without filter, with filter for T_f=0.01T_i, 
# and $T_f=0.1T_i$, respectively. 
#
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

def main() -> None:
    example_id = 5
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
    mode = "MAN"                    # Controller mode

    # Define two extra controllers to evaluate different filtering effects
    # - pid1: No filter
    # - pid2: ilter with Tf=0.1*Ti
    # - pid3: ilter with Tf=0.01*Ti
    pid2 = pid1.copy()
    pid2.Tf = 0.1 * ti
    pid3 = pid1.copy()
    pid3.Tf = 0.01 * ti

    # Control Loop Simulation
    n = len(d["uc1"])
    lp = d["lp"]

    for i in range(lp + 1, n):
        # Simulation of process dynamics
        d["yc1"][i] = d["ap"] * d["yc1"][i - 1] + d["bp"] * d["uc1"][i - lp - 1] + d["noise_signal"][i]
        d["yc2"][i] = d["ap"] * d["yc2"][i - 1] + d["bp"] * d["uc2"][i - lp - 1] + d["noise_signal"][i]
        d["yc3"][i] = d["ap"] * d["yc3"][i - 1] + d["bp"] * d["uc3"][i - lp - 1] + d["noise_signal"][i]

        # Controller 
        d["uc1"][i] = pid1.control(d["r"], d["yc1"][i], uff, uman, utrack, mode)
        yf2 = pid2.filter(d["yc2"][i])
        d["uc2"][i] = pid2.control(d["r"], yf2, uff, uman, utrack, mode)
        yf3 = pid3.filter(d["yc3"][i])
        d["uc3"][i] = pid3.control(d["r"], yf3, uff, uman, utrack, mode)

        # AUTO mode and set-point changes
        if (i + 1) * dt > d["tsim"] / 16.0 + l:
            mode = "AUTO"
            d["r"] = 3.0

        d["rc"][i] = d["r"]
        
    # Plotting graphical results
    show_graphical_results(example_id, d)


if __name__ == "__main__":
    main()
