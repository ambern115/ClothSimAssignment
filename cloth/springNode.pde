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
  
  int row; //the row of this particle in the node list
  int col; //the column of this particle in the node list
  
  PtVector accForces = new PtVector(0,0,0); //accumulated forces to be integrated at the end of timestep
  
  SpringNode[][] neighbors; //a 3x3 array of all this node's immediate neighbors, in [row][column] format
  //true means force has already been calculated, false means it hasn't been done yet
  boolean[][] neighborForceDone; //holds the status of the spring force for each neighbor. 
  int neighborsLen = 3;
  // neighbors[1][1] should be this node
  
  //Standard SpringNode constructor initializes necessary info
  SpringNode(PtVector p, int r, int c) {
    mass = ClothParams.mass;
    radius = ClothParams.radius;
    
    restLen = ClothParams.restLen;
    k = ClothParams.k;
    kd = ClothParams.kd;
    
    floor = ClothParams.floor_height;
    gravity = ClothParams.gravity;
    
    vel = new PtVector(0,0,0);
    pos = p;
    //uncomment the line below for some random fun
    //pos.addVec(new PtVector(random(-20, 20), random (-20, 20), 0));
    
    row = r;
    col = c;
  }
  
  // properly sets this node's neighbors so that they can be referenced later.
  // if there isn't a neighbor at index i,j, then neighbors[i][j] = null.
  // setDiags is a flag that's set if we want our node to also have springs attached
  // to diagonals.
  void set_neighbors(SpringNode[][] nodes, int myRow, int myCol, boolean setDiags) {
    //println("I am node " + this.toString() + " and I am setting my neighbors");
    neighbors = new SpringNode[neighborsLen][neighborsLen];
    int nRow = 0;
    for (int i = myRow-1; i <= myRow+1; i++) {
      int nCol = 0;
      for (int j = myCol-1; j <= myCol+1; j++) {
        //println("setting neighbors in loop, i is " + i + ", j is " + j);
        if ((i >= 0 && j >= 0 && i < nodes.length && j < nodes[i].length) && (setDiags || abs(nRow-nCol) != 2)) {
          neighbors[nRow][nCol] = nodes[i][j];
          //println("just set neighbor " + nRow + ", " + nCol);
        } else { neighbors[nRow][nCol] = null;  println("just got no neighbor at " + nRow + ", " + nCol); }
        nCol++;
      }
      nRow++;
    }
    //this.print_neighbors();
    //println("\n\n");
  }
  
  //prints basic identifying info about this node
  String toString() {
    return "Row: " + row + ", Column: " + col;
  }
  
  //prints information about this node's neighbors
  void print_neighbors() {
    println("I am a node at position: " + this.toString());
    println("Here are my neighbors:");
    for (int i = 0; i < neighbors.length; i++) {
      for (int j = 0; j < neighbors[0].length; j++) {
        if (neighbors[i][j] != null) {
          println("  Neighbor at " + i + ", " + j + " has position: " + neighbors[i][j].toString());
        } else {
          println("  I have no neighbor at " + i + ", " + j);
        }
      }
    }
  }
  
  // sets the physics for this node for the next timestep
  //uses spring forces between it and all neighbor nodes to calculate this
  void update() {
    //println("I am particle at row " + row + " and col " + col);
    accForces.addVec(userForce);
    for (int nRow = 0; nRow < neighborsLen; nRow++) {
      for (int nCol = 0; nCol < neighborsLen; nCol++) {
        // first, ensure this is a valid neighbor to calculate forces for
        if (neighbors[nRow][nCol] == null || nRow == nCol) { continue; }
        else {
          SpringNode neighbor = neighbors[nRow][nCol];
          // calculate unit length vector between two nodes
          PtVector e = neighbor.pos.getSubtractedVector(this.pos); // e* = r2 - r1
          double dist = e.getLen(); // l = |e*|
          e.divByCon(dist); // e = e*/l
          // compute Hooke's law force along this spring's axis
          double stringF = -k * (restLen - dist); //f_s = -k_s(l_o - l)
          
          PtVector velDiff = vel.getSubtractedVector(neighbor.vel); //v_diff = v1 - v2
          // compute damping force along this spring's axis
          double dampF = kd * velDiff.dotVec(e); //f_d = -k_d * dot(v_diff, e)
          
          PtVector overallForce = e.getMultByCon(stringF - dampF); //f = (f_s - f_d) * e
          //println("My overallForce at current step is: " + overallForce.toString());
          // push this side of the spring, and pull the other side
          //println("overallForce is: " + overallForce.toString());
          accForces.addVec(overallForce);
          neighbor.accForces.subtractVector(overallForce);
        }
      }
    }
    //println("I, particle at row " + row + " and col " + col + " have decided my accForces is: " + accForces.toString());
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
    //println("accForces on node in row " + row + " and column " + col + " is: " + accForces.toString());
    //println("My pos before integration is: " + pos.toString());
    PtVector acc = accForces.divideByConstant(mass); //a = F/m
    //println("acc is: " + acc.toString());
    acc.addVec(new PtVector(0,gravity,0)); //a += G
    vel.addVec(acc.getMultByCon(dt)); //v += a*dt
    //println("my vel is: " + vel.toString());
    pos.addVec(vel.getMultByCon(dt)); //p += v*dt
    
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
