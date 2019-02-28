//class for the nodes that rest between springs
class SpringNode {
  
  double gravity;
  float collisionDamp = 0.3; //the factor the node is multiplied by upon collisions with solid objects
  
  PtVector overallForce = new PtVector(0,0,0);
  PtVector userForce = new PtVector(0,0,0);
  
  float mass;
  float radius;
  float restLen; // the rest length between this node and other nodes
  double k; //the spring constant
  double kd; //the damping constant
  float floor;
  PtVector vel; // velocity
  PtVector pos; //position of the node
  
  PtVector accForces = new PtVector(0,0,0); //accumulated forces to be integrated at the end of timestep
  
  SpringNode[][] neighbors; //a 3x3 array of all this node's immediate neighbors, in [row][column] format
  int neighborsLen = 3;
  // neighbors[1][1] should be this node
  
  //Standard SpringNode constructor initializes necessary info
  SpringNode(PtVector p) {
    mass = ClothParams.mass;
    radius = ClothParams.radius;
    
    restLen = ClothParams.restLen;
    k = ClothParams.k;
    kd = ClothParams.kd;
    
    floor = ClothParams.floor_height;
    gravity = ClothParams.gravity;
    
    vel = new PtVector(0,0,0);
    pos = p;
  }
  
  // properly sets this node's neighbors so that they can be referenced later.
  // if there isn't a neighbor at index i,j, then neighbors[i][j] = null.
  // setDiags is a flag that's set if we want our node to also have springs attached
  // to diagonals.
  void set_neighbors(SpringNode[][] nodes, int myRow, int myCol, boolean setDiags) {
    neighbors = new SpringNode[neighborsLen][neighborsLen];
    int nRow = 0;
    for (int i = myRow-1; i <= myRow+1; i++) {
      int nCol = 0;
      for (int j = myCol-1; j <= myCol+1; j++) {
        if ((i >= 0 && j >= 0 && i < nodes.length && j < nodes[i].length) && (setDiags || abs(i-j) != 2)) {
          neighbors[nRow][nCol] = nodes[i][j];
        } else { neighbors[nRow][nCol] = null; }
        nCol++;
      }
      nRow++;
    }
  }
  
  // sets the physics for this node for the next timestep
  //uses spring forces between it and all neighbor nodes to calculate this
  void update() {
    for (int nRow = 0; nRow < neighborsLen; nRow++) {
      for (int nCol = 0; nCol < neighborsLen; nCol++) {
        // first, ensure this is a valid neighbor to calculate forces for
        if (neighbors[nRow][nCol] == null || nRow == nCol) { continue; }
        else {
          SpringNode neighbor = neighbors[nRow][nCol];
          /*
          r = r2 - r1
          l = |r|
          e = r / |r|
          F1 = [ -ks*(l0 - l) - kd*dot(v1 - v2, e) ] e
          */
          // calculate unit length vector between two nodes
          //println("neighbor pos is: " + neighbor.pos + ", my pos is: " + this.pos);
          PtVector e = neighbor.pos.getSubtractedVector(this.pos); // e* = r2 - r1
          //println("e immediately is: " + e.toString());
          double dist = e.getLen(); // l = |e*|
          e.divByCon(dist); // e = e*/l
          //println("e after division is: " + e.toString());
          // compute Hooke's law force along this spring's axis
          double stringF = -k * (restLen - dist); //f_s = -k_s(l_o - l)
          
          PtVector velDiff = vel.getSubtractedVector(neighbor.vel); //v_diff = v1 - v2
          // compute damping force along this spring's axis
          double dampF = kd * velDiff.dotVec(e); //f_d = -k_d * dot(v_diff, e)
          
          PtVector overallForce = e.getMultByCon(stringF - dampF); //f = (f_s - f_d) * e
          // push this side of the spring, and pull the other side
          //println("accForces before add is: " + accForces.toString());
          //println("overallForce is: " + overallForce.toString());
          accForces.addVec(overallForce);
          //println("accForces after add is: " + accForces.toString());
          neighbor.accForces.subtractVector(overallForce);
        }
      }
    }
  }
  
  // collision detection and response...
  // currently only works with floor surface
  void checkCollisions() {
    if (pos.y+radius > floor) {
      vel.y *= -collisionDamp;
      pos.y = floor - radius;
    }
  }
  
  //add accumulated forces to acceleration, and trickle integration down
  //currently done via Eulerian integration
  void integrate(double dt) {
    //println("My pos is: " + pos.toString());
    //println("accForces is: " + accForces.toString());
    PtVector acc = accForces.divideByConstant(mass); //a = F/m
    //println("acc is: " + acc.toString());
    acc.addVec(new PtVector(0,gravity,0)); //a += G
    vel.addVec(acc.getMultByCon(dt)); //v += a*dt
    //println("my vel is: " + vel.toString());
    pos.addVec(vel.getMultByCon(dt)); //p += v*dt
    //println("My pos is: " + pos.toString());
    accForces = new PtVector(0,0,0);
  }
  
  //Maybe it's best to render things as one system, given this implementation?
  /*void render() {
    pushMatrix();
    fill(0,255,0);
    stroke(5);
    line((float) top.x, (float) top.y, (float) top.z, (float) bottom.x, (float) bottom.y, (float) bottom.z);
    translate((float)bottom.x, (float)bottom.y, (float)bottom.z);
    noStroke();
    sphere(radius);
    popMatrix();
  }*/
}
