class SpringSystem {
  ArrayList<Spring> springs = new ArrayList();
  ArrayList<Spring> oldSprings;
  int num_springs = 0;
  
  // constants
  double gravity = 500000000L;
  double k =       100000000000L;
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
    if (num_springs == 0) { top = new PtVector(400, h, -20); }
    else { top = springs.get(num_springs - 1).bottom; } // make spring top above spring's bottom
    PtVector bottom = new PtVector(400 + random(-100, 100), h + sprRestLen + random(-20, 20), -20 + random(-100,10));
    
    Spring n = new Spring(sprRestLen, top, bottom, springMass, nodeRadius, k, kv, gravity, floorHeight);
    springs.add(n);
    num_springs++;
  }
  
  // adds a force to the end of the spring (the bottom node)
  void AddForceToBottomSpring(PtVector force) {
    //add a fraction of the force to all other springs so they move more naturally
    for (int i = num_springs - 2; i >= 0; i--) {
      PtVector result = new PtVector(force);
      result.divByCon(2.0/max(i,1));
      springs.get(i).userForce = result;
    }
    springs.get(num_springs - 1).userForce = new PtVector(force);
  }
   
  //performs physics calculations on each spring for timestep dt.
  void update(double dt) {
    // update each spring in between the nodes..
    for (int i = 0; i < num_springs; i++) {
      PtVector velAbove = new PtVector(0,0,0);
      //println("1");
      if (i != 0) { velAbove = new PtVector(oldSprings.get(i-1).vel); }
      //println("2");
      PtVector yForce_below = new PtVector(0,0,0);
      if (i != num_springs - 1) { yForce_below = new PtVector(oldSprings.get(i+1).overallForce); }
      //println("3");
      springs.get(i).update(dt, velAbove, yForce_below);
    }
  }
  
  //runs the spring system for one frame.
  //performs all spring physics updates, then renders each spring.
  void run(int timesteps, double dt, PtVector bottomForce) {
    for (int i = 0; i < timesteps; i++) {
      oldSprings = new ArrayList(springs);
      AddForceToBottomSpring(bottomForce);
      update(dt);
    }
    for (int i = 0; i < num_springs; i++) {
      springs.get(i).render();
    }
  }
}
