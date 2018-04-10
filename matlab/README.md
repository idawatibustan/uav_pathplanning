#Trajectory planning

Source file from <a>https://github.com/stormmax/quadrotor/traj_planning</a>
- `generate_ts.m`
- `traj_opt7.m`

## How To
- run `find_path.m`
  <br/>(marius's PRM without spline smoothing with orientation modification and dynamic obstacle from obs file )
- run `generate_pos.m`
  <br/>(generate smooth trajectory and `cruising.txt` )
- run `concatenator.m`
  <br/>(concatenate `takeoff.txt`, `cruising.txt`, and `landing.txt` as `path.txt`)
