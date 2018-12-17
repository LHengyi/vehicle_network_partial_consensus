@ECHO off
set arrive_rate = "0.1 0.2 0.3 0.4 0.5"


FOR /L %%s IN (0,1,100) DO (
	echo %%s
	FOR /L %%m IN (2,1,4) DO (
		echo %%m
		python.exe -O simulation.py --pc --output experiment_data/partial_consensus/data_%%s_decision%%m.txt --seed %%s --rate 18 --max_decision %%m
		rem python.exe -O simulation.py  --pc --output experiment_data/normal_simulation/data_%%s_pc%%c.txt --seed %%s --rate 18 --pc-comp 0
		rem python.exe -O simulation.py --output experiment_data/normal_simulation/data_%%s_pc%%c.txt --seed %%s
		)
	
	)
pause