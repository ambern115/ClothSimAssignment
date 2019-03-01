class SpringSystem {
  SpringNode[][] nodes; // the nodes, connected by springs, that form the simulation
  
  // the number of nodes, horizontally and vertically, in the SpringSystem
  int systemLength = 6;
  
  //the position of the anchored spring at the top of the system
  PtVector springTopLeftPos = new PtVector(width / 5, 10, -20); 
  
  //default constructor
  SpringSystem() {
    makeNodes();
  }
  
  // spring system with tunable size and starting pos
  SpringSystem(int len, PtVector topLeftPos) {
    systemLength = len;
    springTopLeftPos = topLeftPos;
    
    makeNodes();
  }
  
  //uses systemLength to add all necessary nodes to this system
  //each node is one restLen away from all neighbors
  void makeNodes() {
      nodes = new SpringNode[systemLength][systemLength];
      //first, make the nodes at the proper positions
      for (int row = 0; row < systemLength; row++) {
        PtVector currNodePos = new PtVector(springTopLeftPos);
        currNodePos.addVec(new PtVector(0,(ClothParams.restLen * row),0));
        for (int col = 0; col < systemLength; col++) {
          SpringNode n = new SpringNode(currNodePos, row, col);
          nodes[row][col] = n;
          currNodePos = new PtVector(currNodePos.getAddVectors(new PtVector(ClothParams.restLen,0,0)));
        }
      }
    
    //next, set up the nodes' neighbors
    for (int row = 0; row < systemLength; row++) {
        for (int col = 0; col < systemLength; col++) {
          nodes[row][col].set_neighbors(nodes, row, col, ClothParams.useDiags);
        }
      }
  }
  
  // returns a string that describes all nodes in the spring system
  String toString() {
    String result;
    result = "-----\nSpring system of size " + Integer.toString(systemLength) + "\n";
    for (int row = 0; row < systemLength; row++) {
      for (int col = 0; col < systemLength; col++) {
        result += "Node at row " + Integer.toString(row) + " and column " + Integer.toString(col);
        result += " has position of " + nodes[row][col].pos.toString() + "\n";
      }
      result += "\n";
    }
    result += "Is the first node the same one as the second node? " + String.valueOf(nodes[0][0] == nodes[0][1]) + "\n";
    result += "-----\n\n";
    return result;
  }
  
  // adds a force to the end of the spring (the bottom nodes)
  // i'll think about how to re-implement this later!
  
  void AddForceToBottomNodes(PtVector force) {
    //also adds a fraction of the force to all other springs so they move more naturally
    for (int row = systemLength - 2; row >= 0; row--) {
      for (int col = 0; col < systemLength; col++) {
        PtVector result = new PtVector(force);
        result.multByCon( ((float) row) / ((float) systemLength));
        nodes[row][col].userForce = result;
      }
    }
    // finally, adds a force to the end of the spring (the bottom nodes)
    for (int col = 0; col < systemLength; col++) {
        nodes[systemLength-1][col].userForce = new PtVector(force);
    }
  }
   
  //performs physics calculations on each node for timestep dt.
  void update(double dt) {
    // first, update all nodes' forces based on their neighbors
    for (int row = 0; row < systemLength; row++) {
      for (int col = 0; col < systemLength; col++) {
        // only update every other node in a row
        if (row % 2 == col % 2) { nodes[row][col].update(); }
      }
    }
    
    //next, check collisions and integrate forces
    //NOTE: don't bother with doing this for top row, they're fixed
    for (int row = 1; row < systemLength; row++) {
      for (int col = 0; col < systemLength; col++) {
        nodes[row][col].checkCollisions();
        nodes[row][col].integrate(dt);
      }
    }
  }
  
  // Display all spring nodes to the screen
  void renderNodes() {
    stroke(5);
    for (int row = 0; row < systemLength; row++) {
      for (int col = 0; col < systemLength; col++) {
        SpringNode n = nodes[row][col];
        if (col+1 < systemLength) { //This node ---> node to the right
          line((float) n.pos.x, (float) n.pos.y, (float) n.pos.z, 
            (float) nodes[row][col+1].pos.x, (float) nodes[row][col+1].pos.y, (float) nodes[row][col+1].pos.z);
        }
        if (row+1 < systemLength) { //This node ---> node underneath
          line((float) n.pos.x, (float) n.pos.y, (float) n.pos.z, 
            (float) nodes[row+1][col].pos.x, (float) nodes[row+1][col].pos.y, (float) nodes[row+1][col].pos.z);
        }
      }
    }
  }
  
  //runs the spring system for one frame.
  //performs all node physics updates, then renders each node.
  void run(int timesteps, double dt, PtVector bottomForce) {
    for (int i = 0; i < timesteps; i++) {
      AddForceToBottomNodes(bottomForce);
      update(dt);
    }
    renderNodes();
    //println(this.toString());
  }
}
