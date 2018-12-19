@ECHO off
set arrive_rate = "0.1 0.2 0.3 0.4 0.5"


FOR /L %%s IN (0,10,100) DO (
	FOR /L %%l IN (1,1,3) DO (
		FOR /L %%r IN (3,3,30) DO (
			echo %%s %%l %%r
			python.exe -O simulation.py --pc --output experiment_data/partial_consensus/data_%%s_sl%%l_rate%%r.txt --seed %%s --rate %%r --safety_level %%l
			rem python.exe -O simulation.py  --pc --output experiment_data/normal_simulation/data_%%s_pc%%c.txt --seed %%s --rate 18 --pc-comp 0
			rem python.exe -O simulation.py --output experiment_data/normal_simulation/data_%%s_pc%%c.txt --seed %%s
			)
		)
	
	)
pause