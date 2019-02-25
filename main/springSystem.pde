class SpringSystem {
  ArrayList<Spring> springs = new ArrayList();
  int num_springs = 0;
  
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
    println("in add node"+num_springs);
    float h = 200+num_springs*50;
    PtVector top = new PtVector(0,h-50,0);
    PtVector bottom = new PtVector(0,h,0);
    
    Spring n = new Spring(40, top, bottom, 30, 10, floor_height);
    springs.add(n);
    num_springs++;
  }
   
   
//void update(float prev_node_velY,float num_nodes,float total_forces_below)
  void update(double dt) {
    // update each spring in between the nodes..

    for (int i = 0; i < num_springs; i++) {
      if (i == 0) {
        if (num_springs > 1) {
          springs.get(i).update(dt, 0, springs.get(i+1).yForce/springs.get(i+1).mass);
        } else {
          springs.get(i).update(dt, 0, 0);
        }
      } else if (i == num_springs-1) { // lowest spring
        springs.get(i).update(dt, springs.get(i-1).vel.y, 0);
      } else if (i > 0) {
        springs.get(i).update(dt, 
                              springs.get(i-1).vel.y, 
                              springs.get(i+1).yForce/springs.get(i+1).mass); 
      }                    
    }
  }
  
  void run(int timesteps, double dt) {
    for (int i = 0; i < timesteps; i++) {
      update(dt);
    }
    for (int i = 0; i < num_springs; i++) {
      springs.get(i).run();
    }
  }
}
