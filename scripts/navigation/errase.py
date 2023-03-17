import pandas as pd 

filename = "/ws/src/octomap_mapping/octomap_server/scripts/inputpointclouds.csv"
# opening the file with w+ mode truncates the file
f = open(filename, "w+")
f.truncate()
f.close()