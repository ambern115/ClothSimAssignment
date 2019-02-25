class SpringSystem {
  ArrayList<Node> nodes = new ArrayList();
  int num_nodes = 0;
  
  // constants
  float gravity = 9885;
  float k = .5; // stiffness
  float kv = .05; // damping velocity???
  float add_dist_x = 40;
  float floor_height = 1000;
  
  SpringSystem(float _k, float _kv) {
    k = _k;
    kv = _kv;
  }
  
  // spring system with floor height given
  SpringSystem(float _k, float _kv, float floor_h) {
    k = _k;
    kv = _kv;
    floor_height = floor_h;
  }
  
  // by default, adds new node to the end of the string
  // adds node using "default" parameters 
  void add_node() {
    println("in add node"+num_nodes);
    float h = num_nodes*50;
    Node n = new Node(45, h-50, h, 1, radius, floor_height);
    nodes.add(n);
    num_nodes++;
  }
   
   
//void update(float prev_node_velY,float num_nodes,float total_forces_below)
  void update(double dt) {
    // update each spring in between the nodes...
    // do this starting from the bottom up to make calculating total_forces_below easier
    float total_forces_below = 0;
    
    for (int i = num_nodes-1; i > -1; i--) {
      if (i > 0) {
        nodes.get(i).setTop(nodes.get(i-1).nodeY-nodes.get(i-1).radius);
        nodes.get(i).update(nodes.get(i-1).prev_velY, num_nodes, total_forces_below, dt);
        total_forces_below += nodes.get(i).total_forceY/nodes.get(i).mass;
      } else {
        nodes.get(i).update(0, num_nodes, total_forces_below, dt);
        total_forces_below += nodes.get(i).total_forceY/nodes.get(i).mass;
      }
    }
  }
  
  void run(int timesteps, double dt) {
    for (int i = 0; i < timesteps; i++) {
      update(dt);
    }
    for (int i = 0; i < num_nodes; i++) {
      nodes.get(i).run();
    }
  }
}
