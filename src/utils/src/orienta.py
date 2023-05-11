import numpy as np
import math
from src.navigation.src.graphs import graph
import matplotlib.pyplot as plt
import tkinter as tk

def str2tuple(tuple_str):
    return tuple(map(float, tuple_str.replace('(','').replace(')','').split(',')))

def read_graph(filename):    
    infile = open(filename, 'r')
    V=set()
    E=dict()
    
    for line in infile:
        nodes=line.split()
        if nodes[0] not in V:
            node0 = str2tuple(nodes[0])
            V.add(node0)
        if nodes[1] not in V:
            node1 = str2tuple(nodes[1])
            V.add(node1)
        E[(node0,node1)]=int(nodes[2])
    infile.close()
    return V, E

def build_graph(V, E, is_directed = False, vertex_map: dict = None) -> tuple([graph.Graph, dict]):
    G = graph.Graph(directed=is_directed)
    
    """
    NOTE: La funzione insert_edge() vuole esattamente un riferimento ad un vertice del grafo
    Quindi, se passassi un nuovo oggetto Vertex, anche se con lo stesso valore di quello che è presente nel grafico, non lo accetterebbe perché non ne troverebbe l'istanza nel grafo
    Il dizionario vertexMap, serve proprio a salvare il riferimento tra l'elemento e il vertice. L'univocità dell'elemento mi è garantita dal vatto che è contenuto nel set V
    """    
    vertex_map = vertex_map if vertex_map is not None else dict()
    for vertex in V:
        vertex_map[vertex] = G.insert_vertex(vertex)

    for edge in E:
        origin = vertex_map[edge[0]]
        destination = vertex_map[edge[1]]
        element = E[edge]
        G.insert_edge(origin, destination, element)

    return G, vertex_map

class Orienta():
        
    def __init__(self, vertices):
        self.fig, self.ax = plt.subplots()
        self.vertices = vertices
        cid = self.fig.canvas.mpl_connect('button_press_event', self.onclick)
        self._drawed_point = None
        
        self._items = list()
        
        for vx, vy in self.vertices:
            self.ax.scatter(vx, vy,c='blue')
        self.ax.grid()
        plt.show()
        
    # funzione chiamata quando viene fatto clic sul grafico
    def onclick(self,event):
        # estrai le coordinate del punto in cui è stato fatto clic
        x, y = event.xdata, event.ydata
        
        if self._drawed_point:
            # print(self._drawed_point)
            self._drawed_point[0].remove()
            
        if len(self._items) > 0:
            for e in self._items:
                e.remove()
            self._items = list()

        # disegna un punto sul grafico alle coordinate del clic
        self._drawed_point = plt.plot(x, y, 'ro')
        plt.draw()
        
        theta = input('\nInsert theta: ')
        self.set_robot_pose((x,y,math.radians(float(theta))))
        self.eval()
        
    def set_robot_pose(self, pose):
        self.u = np.array([np.cos(pose[2]),np.sin(pose[2]),0])
        self.robot_position = np.array([pose[0],pose[1],0])
    
    def eval(self):        
        if not isinstance(self.robot_position,np.ndarray):
            self.robot_position = np.array(self.robot_position)
        
        print('point','vettoriale','scalare',sep='\t')
        for vx, vy in self.vertices:
            self.ax.scatter(vx, vy,c='blue')
            point = np.array([vx,vy,0])
            v = point - self.robot_position
            
            ue = np.array([self.u[0],self.u[1],0])
            ve = np.array([v[0],v[1],0])
            cross = np.cross(ue,ve)[2]
            inner = np.inner(self.u,v)
            print(point,cross,inner,sep='\t')

            if abs(cross) > abs(inner):
                if cross > 0:
                    self._items.append(self.ax.annotate('sx', (vx, vy)))
                else :
                    self._items.append(self.ax.annotate('dx', (vx, vy)))
            else:
                if inner >= 0:
                    self._items.append(self.ax.annotate('front', (vx, vy)))
                else:    
                    self._items.append(self.ax.annotate('back', (vx, vy)))
        plt.show()

if __name__ == '__main__':
    import os
    from pathlib import Path
    
    V, E = read_graph(Path(os.environ['WS_DIR']) / 'src/navigation/trajectories/reach.txt')
    reach, vertex_map = build_graph(V, E)
    cc = Orienta(list(vertex_map.keys()))
    