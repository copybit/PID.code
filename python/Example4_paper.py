from __future__ import annotations

from pathlib import Path

import numpy as np

from PID import PID
from process_and_simulation_data import load_process_and_simulation_data
from show_graphical_results import show_graphical_results

CURRENT_DIR = Path(__file__).resolve().parent

#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#%%%%%%%%%%%%%% Example 4: Feedforward and anti-windup %%%%%%%%%%%%%%%
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#
# This example focuses on analyzing the anti-windup solutions included in 
# the PID code for the control signal saturation problem, and also the 
# combination of the PID controller with a feedforward compensator to deal 
# with measurable disturbances. The effect of anti-windup clamping and t
# racking solutions implemented will be analyzed and compared. Moreover, 
# the capability of the new PID code that includes the feedforward control 
# signal inside the PID controller will also be explored.

#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

def main() -> None:
    # Load process and simulation data
    example_id = 4
    d = load_process_and_simulation_data(example_id, CURRENT_DIR)

    dt = d["dt"]
    t = d["t"]
    l = d["l"]
    k = d["k"]

    # Instantiate PID controllers and initialization
    pid1 = PID()

    pid1.Dt = dt                    # Sampling time
    ti = t                          # Integral time
    pid1.kp = t / (k * (0.2 * t + l))     # Proportional gain
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
    mode = "AUTO"                    # Controller mode

    # State initialization (r0,y0,u0,uff0)
    pid1.init(0.0, 0.0, 0.0, 0.0)

    # Define three extra controllers to evaluate different anti-windup techqniues:
    # - pid1: with no saturation limits
    # - pid2: with only external satuartion limits
    # - pid3: with satuartion limits using clamping
    # - pid4: with satuartion limits using tracking

    umax = 3.5
    umin = -3.5
    pid2 = pid1.copy()
    pid3 = pid1.copy()
    pid3.maxlim = umax
    pid3.minlim = umin
    pid3.Tt = dt
    pid4 = pid1.copy()
    pid4.maxlim = umax
    pid4.minlim = umin

    # Feedforward parameters
    kff = d["kd"] / d["k"]
    tz = t
    tp = d["td"]
    lff = d["ld"] - l

    # Discrete feedforward parameters
    aff = np.exp(-dt / tp)
    bff = np.exp(-dt / tz)
    affd = kff * (1.0 - aff) / (1.0 - bff)
    bffd = affd * bff
    lffd = int(round(lff / dt))

    n = len(d["uc1"])
    lp = d["lp"]
    lpd = d["lpd"]

    # Simulation Loop
    for i in range(lp + 1, n):

        # Simulation of process dynamics
        d["yc1"][i] = d["ap"] * d["yc1"][i - 1] + d["bp"] * d["uc1"][i - lp - 1]
        d["yc2"][i] = d["ap"] * d["yc2"][i - 1] + d["bp"] * d["uc2"][i - lp - 1]
        d["yc3"][i] = d["ap"] * d["yc3"][i - 1] + d["bp"] * d["uc3"][i - lp - 1]
        d["yc4"][i] = d["ap"] * d["yc4"][i - 1] + d["bp"] * d["uc4"][i - lp - 1]

        # Disturbance dynamics
        d["yd"][i] = d["apd"] * d["yd"][i - 1] + d["bpd"] * d["ud"][i - lpd - 1]

        # Total process output
        d["y1"][i] = d["yc1"][i] + d["yd"][i]
        d["y2"][i] = d["yc2"][i] + d["yd"][i]
        d["y3"][i] = d["yc3"][i] + d["yd"][i]
        d["y4"][i] = d["yc4"][i] + d["yd"][i]

        # Disturbance signal change   
        if (i + 1) * dt > 45.0 + l:
            d["ud"][i] = 1.5

        # Feedforward compensator   
        idx1 = max(i - lffd, 0)
        idx2 = max(i - lffd - 1, 0)
        d["uff"][i] = aff * d["uff"][i - 1] + affd * d["ud"][idx1] - bffd * d["ud"][idx2]

        # Controller         
        d["uc1"][i] = pid1.control(d["r"], d["y1"][i], -d["uff"][i], uman, utrack, mode)
        d["uc2"][i] = pid2.control(d["r"], d["y2"][i], -d["uff"][i], uman, utrack, mode)
        d["uc3"][i] = pid3.control(d["r"], d["y3"][i], -d["uff"][i], uman, utrack, mode)
        d["uc4"][i] = pid4.control(d["r"], d["y4"][i], -d["uff"][i], uman, utrack, mode)

        # External saturation for case 2
        d["uc2"][i] = max(min(d["uc2"][i], umax), umin)

        if (i + 1) * dt > 2.0 - l and (i + 1) * dt < 20.0 + l:
            d["r"] = 4.5
        elif (i + 1) * dt >= 20.0 + l:
            d["r"] = 0.0

        d["rc"][i] = d["r"]

    # Plotting graphical results
    show_graphical_results(example_id, d)


if __name__ == "__main__":
    main()
