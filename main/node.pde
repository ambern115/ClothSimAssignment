class Node {
  
  double gravity = 500000000L;
  double velY;
  double prev_velY = 0;

  float restLen;
  float stringTop;
  float nodeY; // node's Y position
  double k = 100000000L; //TRY-IT: How does changing k affect resting length?
  double kv = 10000;
  float mass;
  float radius;
  float floor;
  double total_forceY;
  
  
  Node(float restL, float stringT, float nY, float m, float r, float flr) {
    restLen = restL;
    stringTop = stringT;
    nodeY = nY;
    mass = m;
    radius = r;
    floor = flr;
  }
  
  void setTop(float y) {
    stringTop = y;
  }
  
  void update(double prev_node_velY, int num_nodes, double total_forces_below, double dt) {
    prev_velY = velY;
    
    // string force
    // question!!! does stringTop just mean the y velocity of the node immediately 
    // above the current one, or the cumulative velocity of all nodes above???
    double stringF = -1*k*((nodeY - stringTop) - restLen);
    double dampF = -1*kv*(velY - prev_node_velY);
    
    // sum of forces up and down
    double forceY = dampF + stringF + gravity*mass;
    total_forceY = forceY;
    
    // acceleration on y axis
    double accY = gravity + (1.0/num_nodes)*forceY/mass - (1.0/num_nodes)*total_forces_below; 
    velY += accY*dt;
    nodeY += velY*dt;
    
    if (nodeY+radius >= floor) {
      velY *= -.9;
      nodeY = floor - radius;
    }
  }
  
  void run() {
    pushMatrix();
      fill(0,0,0);
      // need to make string top dynamic....
      rect(500,stringTop,2,nodeY);
      translate(500,nodeY,0);
      sphere(radius);
    popMatrix();
  }
}
