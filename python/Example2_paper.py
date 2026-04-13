from __future__ import annotations

from pathlib import Path

from PID import PID
from process_and_simulation_data import load_process_and_simulation_data
from show_graphical_results import show_graphical_results

CURRENT_DIR = Path(__file__).resolve().parent

#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#%%%%%%%%%%%%%%% Example 2: P, PD control and rate limitation %%%%%%%%%
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#%
#% This example shows how the proposed code can be used to implement P- or 
#% PD-controllers, that is, controllers without integral action. 
#% The rate limitation function is also illustrated. 
#% We use the same process and controller as presented in Example 1.
#% The simulation runs without rate limits by default. To evaluate the rate 
#% limitation function, the code lines before the simulation loop must be
#% uncommented.
#%
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

def main() -> None:
    # Load process and simulation data    
    example_id = 2
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
    pid1.init(d["r"], d["r"], d["u0"], 0.0)

    # Control Loop Simulation
    n = len(d["uc"])
    lp = d["lp"]
    for i in range(lp + 1, n):

        # Simulation of process dynamics
        d["yc"][i] = d["ap"] * d["yc"][i - 1] + d["bp"] * d["uc"][i - lp - 1]

        # Controller type swithing: PI, P, and PI
        if (i + 1) * dt > 0.333 * d["tsim"] + l and (i + 1) * dt <= 0.675 * d["tsim"] + l:
            pid1.ki = 0.0
            pid1.u0 = 2.0
        else:
            pid1.ki = pid1.kp / ti

        # Set-point change
        if (i + 1) * dt < 2.0 - l:
            d["r"] = 1.0
        elif (i + 1) * dt <= 0.498 * d["tsim"] + l:
            d["r"] = 3.0
        else:
            d["r"] = 1.0

        # Controller 
        d["uc"][i] = pid1.control(d["r"], d["yc"][i], uff, uman, utrack, mode)

        d["rc"][i] = d["r"]

    # Plotting graphical results
    show_graphical_results(example_id, d)


if __name__ == "__main__":
    main()
