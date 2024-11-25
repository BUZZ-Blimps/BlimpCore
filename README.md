# BlimpCore
This repository houses the code that will directly power the OrangePi onboard a given blimp

To-Do:
- [ ] break out the Test Bench into the Test Bench Repo
- [ ] build out Wiki
- [ ] Re-organize state machine into separate functions for each state, for readability and encapsulation.
- [ ] Make use of custom message types, also for readability. Instead of float64 arrays make custom msg package with labeled arrays of fixed size.
- [ ] Implement custom state message that sends more information back to the base station (for example, I already added the catch count to the current state using float64 array as a quick and dirty patch)

To launch catching blimp node on Orange Pi as root after sourcing ros2_ws:
```
ros2 launch ros2_blimp [Blimp Name].launch.py
```
For example,
```
ros2 launch ros2_blimp GameChamber.launch.py
```
