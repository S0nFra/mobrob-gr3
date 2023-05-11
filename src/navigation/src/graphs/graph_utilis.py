from src.navigation.src.graphs import graph

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