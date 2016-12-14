  def grafo(self):   
        #creacion del grafo
        posiciones = {}
        with open("puntos.txt","r") as f:
            for line in f:
                l=line.strip("\n").split()
                if l[0]=="N":
                    g.add_node(l[1], x= float(l[2]),y=float(l[3]),name="")
                    posiciones[l[1]] = (float(l[2]),float(l[3]))
                else:
                    g.add_edge(l[1], l[2])

        print posiciones
        img = plt.imread("plano.png")
        plt.imshow(img, extent = [-12284,25600,-3840,9023])
        nx.draw_networkx(g, posiciones)
       
        print "Haciendo camino minimo"
        print nx.shortest_path(g,source="1", target="6")
       
        plt.show()
       

def nodoCercano(self):
        bState = RoboCompDifferentialRobot.Bstate()
        differentialrobot_proxy.getBaseState(bState)
        r = (bState.x , bState.z)
        dist = lambda r,n: (r[0]-n[0])**2+(r[1]-n[1])**2
        #funcion que devuele el nodo mas cercano al robot
        return  sorted(list (( n[0] ,dist(n[1],r)) for n in posiciones.items() ), key=lambda s: s[1])[0][0]
