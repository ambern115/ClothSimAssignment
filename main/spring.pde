class Spring {
  
  // global variables and constants
  float gravity = 1000;//0.98065;
  float prev_velY = 0;
  
  float yForce;
  float restLen;
  
  float k = 20; // stiffness
  float kv = 20; // damping velocity??????
  //float dt = .25;
  float mass;
  float radius;
  float floor;
  PtVector vel; // velocity
  PtVector top; // top end of spring
  PtVector bottom; // bottom end of spring. Ball rendered at bottom of spring...
  
  Spring(float restL, PtVector T, PtVector B, float m, float r, float flr) {
    restLen = restL;
    mass = m;
    radius = r;
    floor = flr;
    
    vel = new PtVector(0,0,0);
    top = T;
    bottom = B; // where a ball may be rendered 
  }
  
  void update(double dt, float sp_above_vY, float force_below) {
    // compute (damped) Hooke's law for this spring
    float stringF_Y = -k*((bottom.y - top.y) - restLen);
    float dampF_Y = -kv*(vel.y - sp_above_vY);
    yForce = stringF_Y + dampF_Y;
    
    // Eulerian integration...
    float acc_Y = gravity + .5*stringF_Y/mass - .5*force_below;
    vel.y = vel.y + (float)(acc_Y*dt);
    bottom.y = bottom.y + (float)(vel.y*dt);
    
    // collision detection and response...
    if (bottom.y+radius > floor_height) {
      vel.y *= -.9;
      bottom.y = floor_height - radius;
    }
  }
  
  void run() {
    pushMatrix();
      line(200,top.y,200,bottom.y);
      translate(200,bottom.y,0);
      sphere(radius);
    popMatrix();
  }
}
