import numpy as np
import networkx as nx
class planner:
    def __init__(self,graph,trajs,len_field='LENGTH',cost_field='LENGTH'):
        #初始化参数格式：
        #graph:networkx Graph(),trajs:tuples,
        self.graph = graph
        self.min_len = min([graph.get_edge_data(*e)[len_field] for e in graph.edges()])
        self.number_path_kv = self.create_number_path_kv(graph,len_field,cost_field)
        self.path_number_kv = self.create_path_number_kv(graph,self.number_path_kv)
        self.neighbors_of_number_path_kv = self.create_path_neighbors_kv(graph,self.number_path_kv,self.path_number_kv)
        self.pathes_plan = set()
        self.score = 0.0
    def initialize_pathes(self,k):
        initial_path_score_kv = dict(zip(self.number_path_kv.keys(),[self.calculate_gain(k) for k in self.number_path_kv.keys()]))
        initial_path_score_rank = sorted(initial_path_score_kv.items(),key = lambda v:v[1],reverse=True)
        initial_pathes = [k[0] for k in initial_path_score_rank][:k]
        return (set(initial_pathes))
    def create_number_path_kv(self,graph,len_field,cost_field):
        # parameters graph:networkx; len_field:string; cost_field:string
        # return:dictionary, such as {1:(3,4),2:(4,5)}.
        np_kv = {}
        for n,e in enumerate(graph.edges()):
            np_kv[n]={}
            np_kv[n]['path'] = e
            np_kv[n]['norm_l'] = graph.get_edge_data(*e)[len_field]/self.min_len
            np_kv[n]['cost'] = graph.get_edge_data(*e)[cost_field]
        return(np_kv)
    def create_path_number_kv(self,graph,np_kv):
        # parameters graph:networkx; np_kv:dictionary,such as {1:(3,4),2:(4,5)}.
        # return:dictionary, such as {(3,4):1,(4,5):2,(4,3):1}.
        pn_kv = {}
        for n in np_kv.keys():
            path = np_kv[n]['path']
            pn_kv[(path[0],path[1])] = n
            pn_kv[(path[1],path[0])] = n
        return (pn_kv)
    def create_path_neighbors_kv(self,graph,np_kv,pn_kv):
        # parameters graph:networkx; np_kv:dictionary,such as {1:(3,4),2:(4,5)};
        # pn_kv:dictionary, such as {(3,4):1,(4,5):2,(4,3):1}.
        # return:dictionary, sucha as {1:(2,3,4),2:(4,5,6)}
        number_path_neighbors_kv = {}
        for k in np_kv.keys():
            number_path_neighbors_kv[k] = ()
            path = np_kv[k]['path']
            n1,n2 = path[0],path[1]
            for nb in set(graph.neighbors(n1))-set((n2,)):
                number_path_neighbors_kv[k]+(pn_kv[(n1,nb)],)
            for nb in set(graph.neighbors(n2))-set((n1,)):
                number_path_neighbors_kv[k]+(pn_kv[(n2,nb)],)
        return(number_path_neighbors_kv)
    def find_ctps(self,traj,pathes):
        # find the set of continous road segments for traj in plan
        # parameters traj:a trajectory,such as (2,3,4,5);pathes:bike lanes,such
        # as (3,4,5).
        # return:tuples,such as ((1,2),3,(6,7,8))
        length = len(traj)
        if length == 0:
            return ()
        for i in range(length):
            if traj[i] not in pathes:
                return traj[:i] + find_ctps(traj[i+1:],pathes)
            if i == length-1:
                return (traj[:],)
    def calculate_trajs_score(self,pathes_plan):
        # calculate user's benefit score
        # parameter pathes_plan:set, such as (2,3,4,5)
        # return:float
        trajs_score = 0.0
        for traj in self.trajs:
            ctps = self.find_ctps(traj,pathes_plan)
            ctps_len = [sum([self.number_path_kv[path]['norm_l'] for path in ctp]) for ctp in ctps]
            traj_score = sum([ctp_len*(self.alpha**ctp_len) for ctp_len in ctps_len])
            trajs_score = trajs_score+traj_score
        return(trajs_score)

    def calculate_gain(self,new_path):
        # calculate benefit gain for new path
        # parameter new_path:tuple, such as (1,) or (4,)
        # return:float
        new_score = self.calculate_trajs_score(self.pathes_plan+(new_path,))
        if(self.score == 0):
            gain = (new_score-self.score)
        else:
            gain = (new_score-self.score)/self.score
        return(gain)

    def make_plan(self,trajs,budget,topk,alpha):
        # make bike lanes plan
        # parameters trajs:tuples; budget:float; topk:int; alpha:float
        self.trajs = trajs
        self.k = topk
        self.alpha = alpha
        self.budget = budget
        self.pathes_plan = self.initialize_pathes(topk)
        self.score = self.calculate_trajs_score(self.pathes_plan)
        self.remain_budget = budget-sum([self.number_path_kv[path]['cost'] for path in self.pathes_plan])
        for path in self.pathes_plan:
            nbs = self.neighbors_of_number_path_kv[path]
            self.candidate_pathes_set = self.candidate_pathes_set|set(nbs)-set(self.pathes_plan)
        while(self.remain_budget>0):
            if(self.candidate_pathes_set == set()):
                break
            else:
                max_gain = 0.0
                for cd in self.candidate_pathes_set:
                    gain = self.calculate_gain(cd)
                    if(gain>max_gain):
                        new_path = cd
                self.pathes_plan+(new_path,)
                self.score = self.calculate_trajs_score(self.pathes_plan)
                self.remain_budget = self.remain_budget -  self.number_path_kv[new_path]['cost']
                self.candidate_pathes_set = self.candidate_pathes_set|set(self.neighbors_of_number_path_kv[new_path])-set(self.pathes_plan)
class shp2graph:
    def shp2networkx(self,shpLineFile,fromNodeField,toNodeField):
        sf = shapefile.Reader(shpLineFile)
        fields = [ f[0] for f in sf.fields[1:]]
        records = sf.records()
        graph = nx.Graph()
        for record in records:
            recDict = dict(zip(fields,record))
            graph.add_edges_from([(recDict[fromNodeField],recDict[toNodeField],recDict)])
        return(graph)
    def trajsgenerate(self,graph,trajsNumber,pn_kv):
        trajs = ()
        nodes = graph.nodes()
        for i in range(trajsNumber):
            nodes = np.random.permutation(nodes)
            fn,tn = nodes[0],nodes[1]
            traj = nx.shortest_path(graph,fn,tn)
            l = len(traj)
            traj = [pn_kv[(traj[i-1],traj[i])] for i in range(1,l)]
            trajs+(traj,)
        return(trajs)
