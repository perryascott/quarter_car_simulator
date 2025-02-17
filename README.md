# Quarter Car Simulator

Simulates vehicle quarter-car model under various disturbances.
Apply differenct controllers to active force suspension for several objectives.

### Quickstart 

```
cd ./c
cmake .
make
./quarter_car ../python/test.csv
python3 ../python/plot_csv.py ../python/test.csv
```

### TODO:

- [ ] make PID minimize jerk
- [ ] Full state FB
- [ ] requirements.txt Python code
- [ ] rk4 solver
- [ ] LQR controller
- [ ] MPC
- [ ] MPC with preview control
- [ ] optionally add noise to state when appying control law
