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
  
  PtVector airVel;
  
  PtVector vel; // velocity
  PtVector pos; //position of the node
  
  int row; //the row of this particle in the node list
  int col; //the column of this particle in the node list
  
  boolean calcDiags = false; //whether or not to use diagonal spring-dampers
  PtVector accForces = new PtVector(0,0,0); //accumulated forces to be integrated at the end of timestep
  
  SpringNode[][] neighbors; //a 3x3 array of all this node's immediate neighbors, in [row][column] format
  //true means force has already been calculated, false means it hasn't been done yet
  boolean[][] neighborForceDone; //holds the status of the spring force for each neighbor. 
  int neighborsLen = 3;
  // neighbors[1][1] should be this node
  
  //Standard SpringNode constructor initializes necessary info
  SpringNode(PtVector p, int r, int c, PtVector airV) {
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
  }
  
  // properly sets this node's neighbors so that they can be referenced later.
  // if there isn't a neighbor at index i,j, then neighbors[i][j] = null.
  // setDiags is a flag that's set if we want our node to also have springs attached
  // to diagonals.
  void set_neighbors(SpringNode[][] nodes, int myRow, int myCol, boolean calcDiags) {
    //println("I am node " + this.toString() + " and I am setting my neighbors");
    this.calcDiags = calcDiags;
    neighbors = new SpringNode[neighborsLen][neighborsLen];
    int nRow = 0;
    for (int i = myRow-1; i <= myRow+1; i++) {
      int nCol = 0;
      for (int j = myCol-1; j <= myCol+1; j++) {
        //println("setting neighbors in loop, i is " + i + ", j is " + j);
        if ((i >= 0 && j >= 0 && i < nodes.length && j < nodes[i].length)) {
          neighbors[nRow][nCol] = nodes[i][j];
          //println("just set neighbor " + nRow + ", " + nCol);
        } else { neighbors[nRow][nCol] = null; }//  println("just got no neighbor at " + nRow + ", " + nCol); }
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
          //println("overallForce is: " + overallForce.toString());
          synchronized(accForces) { accForces.addVec(overallForce); }
          synchronized(neighbor.accForces) { neighbor.accForces.subtractVector(overallForce); }
        }
      }
    }
    //println("I, particle at row " + row + " and col " + col + " have decided my accForces is: " + accForces.toString());
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
        dragVel.subtractVector(airVel); //v = ((v1 + v2 + v3) / 3) - v_air
        
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
    if (pos.y+radius > floor) {
      vel.y *= -collisionDamp;
      synchronized (pos) { pos.y = floor - radius; }
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
    synchronized (pos) { 
      pos.addVec(vel.getMultByCon(dt));
      //check for breakages in neighbor springs
      if (ClothParams.tearable) {
        for (int row = 0; row < neighbors.length; row++) {
          for (int col = 0; col < neighbors.length; col++) {
            //if this is a valid spring connection and the length of the spring is over breakLen
            if (neighbors[row][col] != null && (row != 1 || col != 1) && 
             (calcDiags == true || abs(row - col) % 2 != 0) && 
             Math.abs(pos.getLen() - neighbors[row][col].pos.getLen()) >= ClothParams.breakLen) {
               println("breaking a node at pos " + this.toString() + ", connects to node at pos " + neighbors[row][col].toString());
               SpringNode thisInNeighbor = neighbors[row][col].neighbors[1+(1 - row)][1+(1 - col)];
               if (thisInNeighbor != null) { synchronized (thisInNeighbor) { thisInNeighbor = null; } }
               if (neighbors[row][col] != null) { synchronized(neighbors[row][col]) { neighbors[row][col] = null; } }
               println("done breaking node at pos " + this.toString()); //<>//
             }
          }
        }
      }
    } //p += v*dt
    
    accForces = new PtVector(0,0,0);
    userForce = new PtVector(0,0,0);
  }
}
