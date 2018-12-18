@ECHO off
set arrive_rate = "0.1 0.2 0.3 0.4 0.5"


FOR /L %%s IN (0,2,100) DO (
	echo %%s
	FOR /L %%l IN (1,1,3) DO (
		echo %%m
		python.exe -O simulation.py --pc --output experiment_data/partial_consensus/data_%%s_decision%%m.txt --seed %%s --rate 18 --safety_level %%l
		rem python.exe -O simulation.py  --pc --output experiment_data/normal_simulation/data_%%s_pc%%c.txt --seed %%s --rate 18 --pc-comp 0
		rem python.exe -O simulation.py --output experiment_data/normal_simulation/data_%%s_pc%%c.txt --seed %%s
		)
	
	)
pause