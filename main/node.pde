class Node {
  
  float gravity = 0.98065;
  float velY;
  float prev_velY = 0;

  float restLen;
  float stringTop;
  float nodeY; // node's Y position
  float k = .01; // stiffness
  float kv = .05; // damping velocity???
  float dt = .25;
  float mass;
  float radius;
  float floor;
  float total_forceY;
  
  
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
  
  void update(float prev_node_velY,float num_nodes,float total_forces_below) {
    prev_velY = velY;
    
    // string force
    // question!!! does stringTop just mean the y velocity of the node immediately 
    // above the current one, or the cummulative velocity of all nodes above???
    float stringF = -1*k*((nodeY - stringTop) - restLen);
    float dampF = -1*kv*(velY - prev_node_velY);
    
    // sum of forces up and down
    float forceY = dampF + stringF + gravity*mass;
    total_forceY = forceY;
    
    // acceleration on y axis
    float accY = gravity + (1/num_nodes)*forceY/mass - (1/num_nodes)*total_forces_below; 
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
