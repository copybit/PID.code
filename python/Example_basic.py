from __future__ import annotations

from pathlib import Path

from PID import PID
from process_and_simulation_data import load_process_and_simulation_data
from show_graphical_results import show_graphical_results

CURRENT_DIR = Path(__file__).resolve().parent

#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#%%%%%%%%%%%%%%% Example: basic control loop %%%%%%%%%%%%%%%%%%%%%%%%%%
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#%
#% To illustrate how the code can be used for simulation purposes, this 
#% example shows the code for a basic control loop using the filtering 
#% capabilities of the proposed PID control code.
#%
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

def main() -> None:
    # Load process and simulation data
    example_id = 0
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

    # State initialization (r0,y0,u0,uff0)
    pid1.init(0.0, 0.0, 0.0, 0.0)

    # Control Loop Simulation
    n = len(d["uc"])
    lp = d["lp"]
    for i in range(lp + 1, n):

        # Simulation of process dynamics
        d["yc"][i] = d["ap"] * d["yc"][i - 1] + d["bp"] * d["uc"][i - lp - 1] + d["noise_signal"][i]

        # Controller 
        yf = pid1.filter(d["yc"][i])
        u = pid1.control(d["r"], yf, uff, uman, utrack, mode)
        d["uc"][i] = u

        # AUTO mode
        if (i + 1) * dt > d["tsim"] / 16.0 + l:
            mode = "AUTO"

        # Set-point change
        if (i + 1) * dt > 2.0 - l:
            d["r"] = 3.0

        d["rc"][i] = d["r"]

    # Plotting graphical results
    show_graphical_results(example_id, d)


if __name__ == "__main__":
    main()
