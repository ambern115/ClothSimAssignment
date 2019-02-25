class Spring {
  
  // global variables and constants
  double gravity;
  float restLen;
  
  double k;
  double kv;
  
  PtVector overallForce = new PtVector(0,0,0);
  
  float mass;
  float radius;
  float floor;
  PtVector vel; // velocity
  PtVector top; // top end of spring
  PtVector bottom; // bottom end of spring. Ball rendered at bottom of spring...
  
  Spring(float restL, PtVector T, PtVector B, float m, float r, double k_, double kv_, double grav, float flr) {
    restLen = restL;
    mass = m;
    radius = r;
    floor = flr;
    
    k = k_;
    kv = kv_;
    gravity = grav;
    
    vel = new PtVector(0,0,0);
    top = T;
    bottom = B; // bottom of spring, where a ball may be rendered 
  }
  
  void update(double dt, PtVector vel_above, PtVector forceBelow) {
    PtVector lengths = bottom.subtractVector(top);
    
    // compute (damped) Hooke's law for this spring, and all other forces
    double stringF = -k * (lengths.getLen() - restLen);
    PtVector forceDirections = lengths.divideByConstant(lengths.getLen());
    PtVector dampingForce = vel.subtractVector(vel_above).getMultByCon(-kv);
    overallForce = forceDirections.getMultByCon(stringF).getAddVectors(dampingForce);
    
    //add integrated forces to acceleration, and trickle integration down
    PtVector acc = (overallForce.divideByConstant(mass).subtractVector(forceBelow.divideByConstant(mass))).divideByConstant(2.0);
    acc.addVec(new PtVector(0,gravity,0));
    vel.addVec(acc.getMultByCon(dt));
    bottom.addVec(vel.getMultByCon(dt));
    
    /*double stringF_Y = -k*((bottom.y - top.y) - restLen);
    double dampF_Y = -kv*(vel - vel_above);
    yForce = stringF_Y + dampF_Y;
    
    // Eulerian integration...
    double acc_Y = gravity + .5*yForce/mass - .5*force_below/mass;
    vel.y += (acc_Y*dt);
    bottom.y += (vel.y*dt);*/
    
    // collision detection and response...
    if (bottom.y+radius > floor_height) {
      vel.y *= -.9;
      bottom.y = floor_height - radius;
    }
  }
  
  void render() {
    pushMatrix();
    fill(0,255,0);
    stroke(5);
    line((float) top.x, (float) top.y, (float) top.z, (float) bottom.x, (float) bottom.y, (float) bottom.z);
    translate((float)bottom.x, (float)bottom.y, (float)bottom.z);
    noStroke();
    sphere(radius);
    popMatrix();
  }
}
