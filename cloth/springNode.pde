//class for the nodes that rest between springs
class SpringNode {
  
  double gravity;
  float collisionYDamp = 0.1; //the factor the node is multiplied by upon collisions with solid objects
  float collisionXDamp = 0.6;
  float collisionZDamp = 0.6;
  
  PtVector overallForce = new PtVector(0,0,0);
  PtVector userForce = new PtVector(1,1,1);
  
  float mass;
  float radius;
  float restLen; // the rest length between this node and other nodes
  
  double k; //the spring constant
  double kd; //the damping constant
  float floor;
  
  PtVector airVel;
  
  PtVector vel; // velocity
  PtVector pos; //position of the node
  PtVector og_vel; // original velocity
  PtVector og_pos; // original position
  
  int row; //the row of this particle in the node list
  int col; //the column of this particle in the node list
  
  boolean calcDiags = false; //whether or not to use diagonal spring-dampers
  boolean fixed = false; //is this spring node fixed and immovable?
  PtVector accForces = new PtVector(0,0,0); //accumulated forces to be integrated at the end of timestep
  
  SpringNode[][] neighbors; //a 3x3 array of all this node's immediate neighbors, in [row][column] format
  //true means force has already been calculated, false means it hasn't been done yet
  boolean[][] neighborForceDone; //holds the status of the spring force for each neighbor. 
  int neighborsLen = 3;
  // neighbors[1][1] should be this node
  
  //Standard SpringNode constructor initializes necessary info
  SpringNode(PtVector p, int r, int c, PtVector airV, boolean f) {
    mass = ClothParams.mass;
    radius = ClothParams.radius;
    
    restLen = ClothParams.restLen;
    k = ClothParams.k;
    kd = ClothParams.kd;
    
    floor = ClothParams.floor_height;
    gravity = ClothParams.gravity;
    
    airVel = airV;
    
    vel = new PtVector(0,0,0);
    pos = p;
    //uncomment the line below for some random fun
    //pos.addVec(new PtVector(random(-20, 20), random (-20, 20), 0));
    
    row = r;
    col = c;
    
    fixed = f;
  }
  
  // properly sets this node's neighbors so that they can be referenced later.
  // if there isn't a neighbor at index i,j, then neighbors[i][j] = null.
  // setDiags is a flag that's set if we want our node to also have springs attached
  // to diagonals.
  void set_neighbors(SpringNode[][] nodes, int myRow, int myCol, boolean calcDiags) {
    this.calcDiags = calcDiags;
    neighbors = new SpringNode[neighborsLen][neighborsLen];
    int nRow = 0;
    for (int i = myRow-1; i <= myRow+1; i++) {
      int nCol = 0;
      for (int j = myCol-1; j <= myCol+1; j++) {
        if ((i >= 0 && j >= 0 && i < nodes.length && j < nodes[i].length)) {
          neighbors[nRow][nCol] = nodes[i][j];
        } else { neighbors[nRow][nCol] = null; }
        nCol++;
      }
      nRow++;
    }
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
    //println("as a node, my airvel rn is: " + airVel.toString());
    synchronized(accForces) { accForces.addVec(userForce); } //force from user input
    for (int nRow = 0; nRow < neighborsLen; nRow++) {
      for (int nCol = 0; nCol < neighborsLen; nCol++) {
        // first, ensure this is a valid neighbor to calculate forces for
        if (neighbors[nRow][nCol] == null || (nRow == 1 && nCol == 1) || 
           (calcDiags == false && abs(nRow - nCol) % 2 == 0)) { continue; }
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
          // push this side of the spring, and pull the other side
          synchronized(accForces) { accForces.addVec(overallForce); }
          synchronized(neighbor.accForces) { neighbor.accForces.subtractVector(overallForce); }
        }
      }
    }
  }
  
  // function that calculates drag on the triangles 
  // defined by this point and the points below it,
  // then adds drag to overall force
  void addDrag() {
    //look at neighbors below to form triangles, then calculate with them
    for (int triangles = 0; triangles < 2; triangles++) {
      if (neighbors[2][triangles] != null && neighbors[2][triangles+1] != null) {
        SpringNode n1 = neighbors[2][triangles];
        SpringNode n2 = neighbors[2][triangles+1];
        SpringNode n3 = this;
        
        PtVector dragVel = n1.vel.getAddVectors(n2.vel).getAddVectors(n3.vel).divideByConstant(3); //(v1 + v2 + v3) / 3
        //println("in drag, airVel is " + airVel + " and dragVel is " + dragVel.toString());
        dragVel.subtractVector(airVel); //v = ((v1 + v2 + v3) / 3) - v_air
        //println("dragVel after subtraction of airVel is " + dragVel.toString());
        
        // n* = (r2 - r1) x (r3 - r1)
        PtVector n_star = n2.pos.getSubtractedVector(n1.pos).getCross(n3.pos.getSubtractedVector(n1.pos));
        // |v|^2an = ((|v|v.dot(n*)) / 2|n*|)n*
        PtVector lenVSq_a_n = n_star.getMultByCon((dragVel.getLen() * (dragVel.dotVec(n_star))) / (2 * n_star.getLen()));
        //f_aero = -(1/2)*p*|v|^2*c_d*a*n
        PtVector dragForce = lenVSq_a_n.getMultByCon(-0.5).getMultByCon(ClothParams.airDensity).getMultByCon(ClothParams.cd);
        dragForce.divByCon(3); //f_aero on each particle is f_aero / 3
        
        synchronized(n1.accForces) { n1.accForces.addVec(dragForce); }
        synchronized(n2.accForces) { n2.accForces.addVec(dragForce); }
        synchronized(n3.accForces) { n3.accForces.addVec(dragForce);}
      }
    }
  }
  
  // collision detection and response...
  // currently only works with floor surface
  void checkCollisions() {
    if (pos.y >= floor) {
      synchronized (vel) {
        vel.x *= -collisionXDamp;
        vel.y *= 0;
        vel.z *= collisionZDamp;
      }
      synchronized (pos) { pos.y = floor - radius; }
    }
  }
  
  //add accumulated forces to acceleration, and trickle integration down
  //currently done via Eulerian integration
  void integrate(double dt) {
    if (!fixed) { //only integrate if this node is allowed to move
      PtVector acc = accForces.divideByConstant(mass); //a = F/m
      acc.addVec(new PtVector(0,gravity,0)); //a += G
      synchronized (pos) { pos.addVec(vel.getMultByCon(dt)); } //p += v*dt
      vel.addVec(acc.getMultByCon(dt)); //v += a*dt
      
    }
    //check for breakages in neighbor springs
    if (ClothParams.tearable) {
      for (int row = 0; row < neighbors.length; row++) {
        for (int col = 0; col < neighbors.length; col++) {
          //if this is a valid spring connection and the length of the spring is over breakLen
          if (neighbors[row][col] != null && (row != 1 || col != 1) && 
             (calcDiags == true || abs(row - col) % 2 != 0) && 
             Math.abs(pos.getLen() - neighbors[row][col].pos.getLen()) >= ClothParams.breakLen) {
             //println("breaking a node at pos " + this.toString() + ", connects to node at pos " + neighbors[row][col].toString()); //<>//
             SpringNode thisInNeighbor = neighbors[row][col].neighbors[1+(1 - row)][1+(1 - col)];
             if (thisInNeighbor != null) { synchronized (thisInNeighbor) { thisInNeighbor = null; } }
             if (neighbors[row][col] != null) { synchronized(neighbors[row][col]) { neighbors[row][col] = null; } }
           }
        }
      }
    }
    accForces = new PtVector(0,0,0);
    userForce = new PtVector(1,1,1);
  }
  
  // integrate half eulerian
  void integrateHalf(double dt) {
    og_vel = new PtVector(vel);
    og_pos = new PtVector(pos);
    
    if (!fixed) { //only integrate if this node is allowed to move
      PtVector acc = accForces.divideByConstant(mass); //a = F/m
      acc.addVec(new PtVector(0,gravity,0)); //a += G
      vel.addVec(acc.getMultByCon(0.5*dt)); // v += a*dt
      synchronized (pos) { pos.addVec(vel.getMultByCon(0.5*dt)); } //p += v*dt
    }
    //check for breakages in neighbor springs
    if (ClothParams.tearable) {
      for (int row = 0; row < neighbors.length; row++) {
        for (int col = 0; col < neighbors.length; col++) {
          //if this is a valid spring connection and the length of the spring is over breakLen
          if (neighbors[row][col] != null && (row != 1 || col != 1) && 
             (calcDiags == true || abs(row - col) % 2 != 0) && 
             Math.abs(pos.getLen() - neighbors[row][col].pos.getLen()) >= ClothParams.breakLen) {
             //println("breaking a node at pos " + this.toString() + ", connects to node at pos " + neighbors[row][col].toString()); //<>//
             SpringNode thisInNeighbor = neighbors[row][col].neighbors[1+(1 - row)][1+(1 - col)];
             if (thisInNeighbor != null) { synchronized (thisInNeighbor) { thisInNeighbor = null; } }
             if (neighbors[row][col] != null) { synchronized(neighbors[row][col]) { neighbors[row][col] = null; } }
           }
        }
      }
    }
    accForces = new PtVector(0,0,0);
  }
  
  // integrate the full time step to finish midpoint integration
  void integrateFull(double dt) {    
    if (!fixed) { //only integrate if this node is allowed to move
      PtVector acc = accForces.divideByConstant(mass); //a = F/m
      acc.addVec(new PtVector(0,gravity,0)); //a += G
      vel = og_vel.getAddVectors(acc.getMultByCon(dt)); // v += a*dt
      synchronized (pos) { pos = og_pos.getAddVectors(vel.getMultByCon(dt)); } //p += v*dt
    }
    //check for breakages in neighbor springs
    if (ClothParams.tearable) {
      for (int row = 0; row < neighbors.length; row++) {
        for (int col = 0; col < neighbors.length; col++) {
          //if this is a valid spring connection and the length of the spring is over breakLen
          if (neighbors[row][col] != null && (row != 1 || col != 1) && 
             (calcDiags == true || abs(row - col) % 2 != 0) && 
             Math.abs(pos.getLen() - neighbors[row][col].pos.getLen()) >= ClothParams.breakLen) {
             //println("breaking a node at pos " + this.toString() + ", connects to node at pos " + neighbors[row][col].toString()); //<>//
             SpringNode thisInNeighbor = neighbors[row][col].neighbors[1+(1 - row)][1+(1 - col)];
             if (thisInNeighbor != null) { synchronized (thisInNeighbor) { thisInNeighbor = null; } }
             if (neighbors[row][col] != null) { synchronized(neighbors[row][col]) { neighbors[row][col] = null; } }
           }
        }
      }
    }
    accForces = new PtVector(0,0,0);
    userForce = new PtVector(1,1,1);
  }
}
