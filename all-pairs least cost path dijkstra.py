import networkx as nx
class TransitGraph(nx.Graph):
    '''
    The nodes in the input graph must have a "line" attribute that holds a list
    of public transport lines that stop at that particular node (station).
    '''
    def __init__(self, incoming_graph_data=None, **attr):
        nx.Graph.__init__(self, incoming_graph_data, **attr)
        for n in self:
            Node = self.node[n]
            Node['predecessor'] = None
            Node['successors']=set(nx.neighbors(self, n))
            Node['is_visited']=False
            Node['cost'] = float('inf')
            Node['dist'] = 0
            Node['trf'] = 0
            Node['trf_benchmark'] = Node['line']
    def set_predecessor(self, node, predecessor):
        Node = self.node[node]
        Node['predecessor'] = predecessor
        Node['successors'].discard(predecessor)
    def set_cost(self, node, cost):
        Node = self.node[node]
        Node['cost'] = cost
    def set_dist(self, node, dist):
        self.node[node]['dist'] = dist
    def transfer_took_place(self, node):
        self.node[node]['trf']= self.node[node]['trf']+1
    def inherit_transfer(self, node):
        if self.get_predecessor(node) == None:
            self.node[node]['trf'] = 0
        else:
            self.node[node]['trf'] = self.get_transfers(self.get_predecessor(node))

    def set_visited(self, node):
        self.node[node]['is_visited'] = True
    def set_benchmark(self, node, benchmark_node):
        self.node[node]['trf_benchmark'] = self.node[benchmark_node]['line']
    
    def get_predecessor(self, node):
        return self.node[node]['predecessor']
    def get_successors(self, node):
        return self.node[node]['successors']
    def is_visited(self, node):
        return self.node[node]['is_visited']
    def get_cost(self, node):
        return self.node[node]['cost']
    def get_dist(self, node):
        return self.node[node]['dist']
    def get_transfers(self, node):
        return self.node[node]['trf']
    def get_line(self, node):
        return self.node[node]['line']
    def get_benchmark(self, node):
        return self.node[node]['trf_benchmark']

class Line(TransitGraph):
    def __init__(self, lineID, fare, stations = [], speed, headway):
        self._ID = lineID
        self._fare = fare
        self._speed = speed
        self._headway = headway
        self._stations=set(stations)
        
    def ID(self):
        return self._ID
    def getFare(self):
        return self._fare
    def changeFare(self, newFare):
        self._fare= newFare
    def changeSpeed(self, newSpeed):
        self._speed = newSpeed
    def changeHeadway(self, newHeadway):
        self._headway = newHeadway
    def getStations(self):
        return self._stations
    def add_stations(self, stations2add):
        self._stations.update(set(stations2add))
    def remove_stations(self, stations2remove):
        self._stations.difference_update(set(stations2remove))
        
import heapq
def dijkstra(graph, fare, time_value, speed, waiting_time):
    aplcp = {k:{} for k in graph}
    cost_km = {k:{} for k in graph}
    dist_km = {k:{} for k in graph}

    for k in graph:
        G=TransitGraph(graph) #reinitialization
        G.set_cost(k, 0)
        G.set_visited(k)
        unvisited_Q = [(G.get_cost(node), node) for node in G]
        heapq.heapify(unvisited_Q)
        
        while len(unvisited_Q):
            uv = heapq.heappop(unvisited_Q)
            current = uv[1]
            if not G.is_visited(current):
                G.set_benchmark(current, G.get_predecessor(current))

            G.set_visited(current)
            for successor in G.get_successors(current):
                if G.is_visited(successor):
                    continue
                G.inherit_transfer(successor)
                has_transfered = False
                if not bool(G.get_line(successor) & G.get_benchmark(current)):
                        has_transfered = True

                dist_ij = G[current][successor]['weight']
                cost_ij = dist_ij/speed*time_value + has_transfered*(fare + waiting_time/time_value)
                new_cost = G.get_cost(current) + cost_ij 
                new_dist = G.get_dist(current) + dist_ij

                if new_cost < G.get_cost(successor):
                    G.set_cost(successor, new_cost)
                    G.set_dist(successor, new_dist)
                    G.set_predecessor(successor, current)
                    G.inherit_transfer(successor)
                    
                    if has_transfered:
                        G.transfer_took_place(successor)
                        G.set_benchmark(successor, current)

            unvisited_Q = [(G.get_cost(n), n) for n in G if not G.is_visited(n) ]
            heapq.heapify(unvisited_Q)
        
        ''' 
        to find the farthest nodes from the source node so that when the 
        shortest path from the source to this particular node is found, there 
        would be more overlapping routes saved and skipped afterwards, facilitating
        the whole process.
        '''
        node_cost = {k: {m: G.get_cost(m) for m in G}}
        node_cost = sorted(node_cost[k].items(), key = lambda i: i[1])
        
        while len(node_cost):
            farthest = node_cost.pop(-1)[0]
            if not farthest in aplcp[k]:
                aplcp, cost_km, dist_km = __shortest__(G, farthest, aplcp, cost_km, dist_km,  [])
    trf_km = all_pairs_transfer_count(G, aplcp)
    return (aplcp, cost_km, dist_km, trf_km)

def __shortest__(G, node,  aplcp, cost_km, dist_km, path=[]):
    if not G.get_predecessor(node):
        path.append(node)
        for other in path:
            aplcp[other][node]= path[path.index(other):]
            aplcp[node][other]=aplcp[other][node][::-1]
            
            dist_km[node][other] = abs(G.get_dist(other)-G.get_dist(node))
            dist_km[other][node] = dist_km[node][other]

            cost_km[node][other] = abs(G.get_cost(other)-G.get_cost(node))
            cost_km[other][node] = cost_km[node][other]
        return (aplcp, cost_km, dist_km)
    else:
        path.append(node)
        for other in path:
            if other in aplcp[node]:
                continue
            else:
                aplcp[other][node]= path[path.index(other):]
                aplcp[node][other]=aplcp[other][node][::-1]

                dist_km[node][other] = abs(G.get_dist(other)-G.get_dist(node))
                dist_km[other][node] = dist_km[node][other]

                cost_km[node][other] = abs(G.get_cost(other)-G.get_cost(node))
                cost_km[other][node] = cost_km[node][other]
        return __shortest__(G, G.get_predecessor(node), aplcp, cost_km, dist_km, path)
    
def all_pairs_transfer_count(G, aplcp):
    '''calculation of transfers between all pairs in the public transit network'''
    import copy
    apsp = copy.deepcopy(aplcp)
    trf={(i,j):0 for j in G for i in G} 
    for i in apsp:
        del(apsp[i][i])
        while bool(apsp[i]): 
            j = apsp[i][max([apsp[i][j] for j in apsp[i]])[-1]][-1]
            benchMark = i
            apsp[i][j].remove(i)
            for n in apsp[i][j]:
                if not bool(G.node[n]['line'] & G.node[benchMark]['line']):
                    trf[(i,j)]+=1
                    trf[(j,i)]+=1
                    trf[(benchMark,n)],trf[(n,benchMark)] = 1,1
                    try:
                        del(apsp[benchMark][n])
                        if n!=benchMark:
                            del(apsp[n][benchMark])
                    except:
                        pass
                    benchMark = aplcp[i][j][aplcp[i][j].index(n)-1]
                else:
                    trf[(benchMark,n)],trf[(n,benchMark)] = 0,0
                    try:
                        del(apsp[benchMark][n])
                        if benchMark != n:
                            del(apsp[n][benchMark])
                    except:
                        pass
            trf[(i,n)] , trf[(n,i)] = trf[(i,j)] , trf[(j,i)]
            try:
                del(apsp[i][n])
                if n != i:
                    del(apsp[n][i])
            except: 
                pass
        try:
            del(apsp[i][j],apsp[j][i])
        except:
            pass
    return trf