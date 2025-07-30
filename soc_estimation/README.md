# State of charge estimation using extended Kalman filter


## How to generate simulation data

1. Open soc_estimation.slx file in matlab folder on top level (simulink file)
2. Load battery_data.mat
3. Adjust Ts if needed (e.g. in matlab terminal: Ts = 1)
4. Run the simulation
5. Execute in matlab terminal:  `xlswrite('export_u.csv', out.voltage_current_sim_data.signals(1).values)` 


## Run the simulation

Create a virtual environment and install the following dependencies:
- numpy
- scipy
- pandas

e.g.: `pip install numpy`

```py
source .venv/bin/activate
python3 soc_estimation.py
```


## Tuning the Kalman Filer

Pay attention to the variable `Q` for the uncertainty of the process model
and the variable R, which is the uncertainty of the measurements.
