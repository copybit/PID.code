from __future__ import annotations

from pathlib import Path

from PID import PID
from process_and_simulation_data import load_process_and_simulation_data
from show_graphical_results import show_graphical_results

CURRENT_DIR = Path(__file__).resolve().parent

#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#%%%%%%%%%%%%%%% Example 3: Set-point weightihg %%%%%%%%%%%%%%%%%%%%%%%
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#%
#% The PID code used here includes the capabilities for setpoint handling 
#% to obtain a smoother control signal when strong changes of setpoints 
#% (like step changes) are required. To show this idea, in this example we 
#% use the same process and controller as presented in Example 1 to show the 
#% effect of the b parameter on the responses of the control system. 
#% Three different cases are analyzed for b=0, b=0.5, and b=1. 
#%
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

def main() -> None:

    # Load process and simulation data
    example_id = 3
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
    mode = "AUTO"                   # Controller mode

   # State initialization (r0,y0,u0,uff0)
    pid1.init(0.0, 0.0, 0.0, 0.0)

    # Define two extra controllers only changing b parameters
    # PID with b=0.5    
    pid2 = pid1.copy()
    pid2.b = 0.5

    # PID with b=0
    pid3 = pid1.copy()
    pid3.b = 0.0

    # Control Loop Simulation
    n = len(d["uc1"])
    lp = d["lp"]
    for i in range(lp + 1, n):

        # Simulation of process dynamics
        d["yc1"][i] = d["ap"] * d["yc1"][i - 1] + d["bp"] * (d["uc1"][i - lp - 1] + d["dist"])
        d["yc2"][i] = d["ap"] * d["yc2"][i - 1] + d["bp"] * (d["uc2"][i - lp - 1] + d["dist"])
        d["yc3"][i] = d["ap"] * d["yc3"][i - 1] + d["bp"] * (d["uc3"][i - lp - 1] + d["dist"])

        # Controller 
        d["uc1"][i] = pid1.control(d["r"], d["yc1"][i], uff, uman, utrack, mode)
        d["uc2"][i] = pid2.control(d["r"], d["yc2"][i], uff, uman, utrack, mode)
        d["uc3"][i] = pid3.control(d["r"], d["yc3"][i], uff, uman, utrack, mode)

        # Disturbance signal change
        if (i + 1) * dt > d["tsim"] / 2.0 + l:
            d["dist"] = 1.0

        # Set-point change
        if (i + 1) * dt > 2.0 - l:
            d["r"] = 3.0

        d["rc"][i] = d["r"]

    # Plotting graphical results
    show_graphical_results(example_id, d)

if __name__ == "__main__":
    main()
