class SpringSystem {
  ArrayList<Spring> springs = new ArrayList();
  ArrayList<Spring> oldSprings;
  int num_springs = 0;
  
  // constants
  double gravity = 500000000L;
  double k =       1000000000000L;
  double kv =      -1000;
  float sprRestLen = 50;
  float floorHeight = 600;
  float springMass = 1;
  float nodeRadius = 15;
  
  SpringSystem(double _k, double _kv) {
    k = _k;
    kv = _kv;
  }
  
  // spring system with floor height given
  SpringSystem(double _k, double _kv, float floor_h) {
    k = _k;
    kv = _kv;
    floor_height = floor_h;
  }
  
  // by default, adds new spring to the end of the string
  // adds spring using "default" parameters 
  void add_spring() {
    println("adding node number "+num_springs);
    float h;
    if (num_springs == 0) { h = floor_height - height + 100; } //place near top of screen
    else { h = (float) (springs.get(num_springs - 1).bottom.y); } // place on bottom of last spring
    PtVector top;
    if (num_springs == 0) { top = new PtVector(400, h, 0); }
    else { top = springs.get(num_springs - 1).bottom; } // make spring top above spring's bottom
    PtVector bottom = new PtVector(400, h + sprRestLen, 0);
    
    Spring n = new Spring(sprRestLen, top, bottom, springMass, nodeRadius, k, kv, gravity, floorHeight);
    springs.add(n);
    num_springs++;
  }
   
  //performs physics calculations on each spring for timestep dt.
  void update(double dt) {
    // update each spring in between the nodes..
    for (int i = 0; i < num_springs; i++) {
      double vY_above = 0;
      if (i != 0) { vY_above = oldSprings.get(i-1).vel.y; }
      double yForce_below = 0;
      if (i != num_springs - 1) { yForce_below = oldSprings.get(i+1).yForce; }
      springs.get(i).update(dt, vY_above, yForce_below);
    }
  }
  
  //runs the spring system for one frame.
  //performs all spring physics updates, then renders each spring.
  void run(int timesteps, double dt) {
    for (int i = 0; i < timesteps; i++) {
      oldSprings = new ArrayList(springs);
      update(dt);
    }
    for (int i = 0; i < num_springs; i++) {
      springs.get(i).render();
    }
  }
}
