//these imports are for multithreading purposes
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.CyclicBarrier;

class SpringSystem {
  SpringNode[][] nodes; // the nodes, connected by springs, that form the simulation
  
  // the number of nodes, horizontally and vertically, in the SpringSystem
  int systemLength = 6;
  PtVector airVel = new PtVector(0,0,0);
  
  //the position of the anchored spring at the top of the system
  PtVector springTopLeftPos = new PtVector(width / 5, 10, -20); 
  
  //a barrier lock used by the physics threads to ensure concurrency
  final CyclicBarrier calcBarrier = new CyclicBarrier(ClothParams.numPhysThreads);
  
  //default constructor
  SpringSystem() {
    makeNodes();
  }
  
  // spring system with tunable size and starting pos
  SpringSystem(int len, PtVector topLeftPos, PtVector airV) {
    systemLength = len;
    springTopLeftPos = topLeftPos;
    airVel = airV;
    
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
          SpringNode n = new SpringNode(currNodePos, row, col, airVel);
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
        synchronized(nodes[row][col].userForce) { nodes[row][col].userForce = result; }
      }
    }
    // finally, adds a force to the end of the spring (the bottom nodes)
    for (int col = 0; col < systemLength; col++) {
        synchronized(nodes[systemLength-1][col].userForce) { nodes[systemLength-1][col].userForce = new PtVector(force); }
    }
  }
   
  //performs single-thread physics calculations on each node for timestep dt.
  void update(double dt) {
    for (int row = 0; row < systemLength; row++) {
      for (int col = 0; col < systemLength; col++) {
        // only update every other node in a row
        nodes[row][col].addDrag();
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
    //stroke(5);
    noStroke();
    fill(0,0,0);
    beginShape(TRIANGLE_STRIP);
    //texture(img);
    fill(255,0,0);
    
    for (int row = 0; row < systemLength; row++) {
      for (int col = 0; col < systemLength; col++) {
          if (row+1 < systemLength) {
            // if row odd! consider 0 even
            if (row % 2 != 0) {
              normal(0, 0, -1);
              synchronized (nodes[row][col].pos) {
                vertex((float) nodes[row][col].pos.x, (float) nodes[row][col].pos.y, 
                       (float) nodes[row][col].pos.z, (float) col/systemLength, (float) row/(systemLength-1));
              }
              synchronized (nodes[row+1][col].pos) {
                vertex((float) nodes[row+1][col].pos.x, (float) nodes[row+1][col].pos.y, 
                       (float) nodes[row+1][col].pos.z, (float) col/systemLength, (float) (row+1)/(systemLength-1));
              }
          } else {
              normal(0, 0, 1);
              synchronized (nodes[row][systemLength-col-1].pos) {
                vertex((float) nodes[row][systemLength-col-1].pos.x, (float) nodes[row][systemLength-col-1].pos.y, 
                       (float) nodes[row][systemLength-col-1].pos.z, (float) (systemLength-col-1)/systemLength, (float) row/(systemLength-1));
              }
              synchronized (nodes[row+1][systemLength-col-1].pos) {
                vertex((float) nodes[row+1][systemLength-col-1].pos.x, (float) nodes[row+1][systemLength-col-1].pos.y, 
                       (float) nodes[row+1][systemLength-col-1].pos.z, (float) (systemLength-col-1)/systemLength, (float) (row+1)/(systemLength-1));
              }
            }
        }
      }
    }
    endShape();
  }
  
  //runs the spring system for one frame on one single thread.
  //performs all node physics updates, then renders each node.
  void run_single_thread(int timesteps, double dt, PtVector bottomForce) {
    for (int i = 0; i < timesteps; i++) {
      AddForceToBottomNodes(bottomForce);
      update(dt);
    }
    renderNodes();
  }
}
