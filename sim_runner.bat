@ECHO off
set arrive_rate = "0.1 0.2 0.3 0.4 0.5"


FOR /L %%s IN (0,1,100) DO (
	echo %%s
	FOR /L %%c IN (0,1,10) DO (
		echo %%c
		python.exe -O simulation.py --pc --output experiment_data/partial_consensus/data_%%s_pc%%c.txt --seed %%s --rate 18 --pc-comp %%c
		rem python.exe -O simulation.py  --pc --output experiment_data/normal_simulation/data_%%s_pc%%c.txt --seed %%s --rate 18 --pc-comp 0
		rem python.exe -O simulation.py --output experiment_data/normal_simulation/data_%%s_pc%%c.txt --seed %%s
		)
	
	)
pause