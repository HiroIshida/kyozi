# Usage
store one epoch data. Ctrl-C will terminate teaching procedure.
```
rosrun kyozi teach.py
```

Assuming you run `teach.py` multiple times, the following command summarize epoch data into a single pickle file by filtering with specified hz. If you set larger hz, but sampling point itself has sparse data, the process finished with assertion error. In such as case, please lower the hz value. 
```
rosrun kyozi summarize.py -hz 8
```

Assuming you run `summarize.py`, by the following command, you can see how is the summarized epoch data looks like. The command dumps the gif file.
```
rosrun kyozi debug.py
```
