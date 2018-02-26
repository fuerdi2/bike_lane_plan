import numpy as np
import networkx as nx
import shapefile
from lanertFunc import lanertFunc
from bike_lane_planner import planner
lf = lanertFunc()
pl = planner()
g = lf.shp2networkx('street.shp','FROMNODE','TONODE')
pl.init(g,[],'LENGTH','LENGTH',1.01,3)
pl.trajs = lf.trajsgenerate(g,1000,pl.path_number_kv)
pl.make_plan(1000.0)
