## 20180116    
- Remove pre-func from Sim_data in imu_sim.py because pre-func cause ambiguity;
- Output Allan std instead of Allan var in demo Allan algorithm in allan_analysis.py;

## 20180117
- Add the functionality of automatic associated data generation. For example, your algorithm outputs only quaternions, but you want a plot or statistics of Euler angles (errors). **gnss-ins-sim** will automatically calculate Euler angles and gives results you want.
- Limit the summary of gnss-ins-sim to include error statistics of Euler angle, position and velocity.

## 20180123
- split demos in demo.py into different files: demo_xxx.py

## 20180125
- separate demos from gnss-ins-sim