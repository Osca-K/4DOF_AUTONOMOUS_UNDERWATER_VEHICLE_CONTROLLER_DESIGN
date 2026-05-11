# 4DOF AUV Controller Design - Quick Run Order

This guide shows the simplest order to run the main MATLAB scripts.

## Before You Start
1. Open MATLAB.
2. Set the Current Folder to this project root:
   `4DOF_AUTONOMOUS_UNDERWATER_VEHICLE_CONTROLLER_DESIGN`
3. Make sure Control System Toolbox is available.

## Run These Files In Order

### 1) Build trim point and linear model (must run first)
Run:
```matlab
Calculate_Trim_and_Linearise_4DOF
```
What it does:
- Solves trim conditions.
- Linearizes the nonlinear model.
- Saves `AUV_4DOF_Linearised_Model.mat`.

---

### 2) Check if trim is valid (recommended)
Run:
```matlab
Check_Trim_4DOF
```
What it does:
- Simulates nonlinear model at trim input.
- Confirms velocity perturbations stay near zero.
- Saves a figure in `Results/`.

---

### 3) Generate transfer functions and poles/zeros
Run:
```matlab
TransferFunctions
```
What it does:
- Loads the linear model.
- Builds transfer functions and prints poles/zeros.

---

### 4) Plot step responses
Run:
```matlab
Step_Input_Response
```
What it does:
- Generates step response plots for key channels.
- Saves figures in `Results/`.

---

### 5) Plot frequency response (Bode)
Run:
```matlab
Blode_Plot
```
What it does:
- Generates Bode plots for key channels.
- Saves figures in `Results/`.

---

### 6) Plot pole-zero maps
Run:
```matlab
Pole_Zero_Map
```
What it does:
- Generates pole-zero maps.
- Saves figures in `Results/`.

---

### 7) Plot root locus
Run:
```matlab
RootsLocus
```
What it does:
- Generates root locus plots.
- Saves figures in `Results/`.

## Optional (Controller Tuning)
Run these after the main model/TF workflow:

```matlab
PD_Tuner
PID_Tuner
PD_and_PID_Tuner_StepInfo
```

## Common Issue
If you get an error about missing linear model data, run this again first:
```matlab
Calculate_Trim_and_Linearise_4DOF
```
