#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
import networkx as nx
import shapefile
from bike_lane_planner import planner
from bike_lane_planner import shp2graph
sh = shp2graph()
g = sh.shp2networkx('street.shp','FROMNODE','TONODE')
pl = planner(g,'LENGTH','LENGTH')
trajs = sh.trajsgenerate(g,10000,pl.path_number_kv)
pl.make_plan(trajs,1000.0,3,1.01)
