# Dependencies

## CasADi Installation for MATLAB
Official Website:

- https://web.casadi.org/

Download CasADi:

- https://web.casadi.org/get/

---

## 1. Download CasADi

Download the MATLAB package matching your:

- Operating System
- MATLAB version

Official Download:

- https://web.casadi.org/get/

---

## 2. Extract CasADi

Extract the downloaded package to a convenient location.

Example paths:

### Windows

```text
C:\casadi
```

### macOS

```text
/Users/yourname/casadi
```

---

## 3. Add CasADi to MATLAB Path

In MATLAB:

### Windows

```matlab
addpath('C:\casadi')
savepath
```

### macOS

```matlab
addpath('/Users/yourname/casadi')
savepath
```
## 4. Verify Installation

Run the following in MATLAB:

```matlab
import casadi.*

x = SX.sym('x');
disp(x)
```

# How to Run the Simulation

## 1. Configure Parameters

Before running the simulation, define all required parameters in `params`, including:

- Controller selection (`controller_type`)  

Set the controller type as either:

```matlab
params.controller_type = 'SS';   % Single Shooting
% or
params.controller_type = 'MS';   % Multiple Shooting