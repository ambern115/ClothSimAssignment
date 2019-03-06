//these imports are for multithreading purposes
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.CyclicBarrier;

class SpringSystem {
  SpringNode[][] nodes; // the nodes, connected by springs, that form the simulation
  
  // the number of nodes, horizontally and vertically, in the SpringSystem
  int systemHeight = 15;
  int systemLength = 20;
  FixedMethod fixMeth = FixedMethod.LEFT;
  
  PtVector airVel = new PtVector(0,0,0);
  
  //the position of the anchored spring at the top of the system
  PtVector springTopLeftPos = new PtVector(width / 5, 10, -20); 
  
  //a barrier lock used by the physics threads to ensure concurrency
  final CyclicBarrier calcBarrier = new CyclicBarrier(ClothParams.numPhysThreads);
  
  //default constructor
  SpringSystem() {
    makeNodes();
  }
  
  // spring system with tunable size, fixed side, and starting pos
  SpringSystem(int h, int len, FixedMethod fm, PtVector topLeftPos, PtVector airV) {
    systemHeight = h;
    systemLength = len;
    fixMeth = fm;
    springTopLeftPos = topLeftPos;
    airVel = airV;
    
    makeNodes();
  }
  
  //uses systemLength to add all necessary nodes to this system
  //each node is one restLen away from all neighbors
  void makeNodes() {
      nodes = new SpringNode[systemHeight][systemLength];
      //first, make the nodes at the proper positions
      for (int row = 0; row < systemHeight; row++) {
        PtVector currNodePos = new PtVector(springTopLeftPos);
        currNodePos.addVec(new PtVector(0,(ClothParams.restLen * row),0));
        for (int col = 0; col < systemLength; col++) {
          SpringNode n;
          
          //if this node should be a fixed node, make it fixed
          if ((fixMeth == FixedMethod.TOP && row == 0) ||
              (fixMeth == FixedMethod.LEFT && col == 0) ||
              (fixMeth == FixedMethod.BOTTOM && row == systemHeight - 1) ||
              (fixMeth == FixedMethod.RIGHT && col == systemLength - 1) || 
              (fixMeth == FixedMethod.FLAG_LEFT && col == 0 && (row < systemHeight * .2 || row > systemHeight * .8)) ||
              (fixMeth == FixedMethod.FLAG_DISPLAYED && row == 0 && (col < systemLength * .1 || col > systemLength * .9))) {
                n = new SpringNode(currNodePos, row, col, airVel, true);
              } else { n = new SpringNode(currNodePos, row, col, airVel, false); }
          
          nodes[row][col] = n;
          currNodePos = new PtVector(currNodePos.getAddVectors(new PtVector(ClothParams.restLen,0,0)));
        }
      }
    
    //next, set up the nodes' neighbors
    for (int row = 0; row < systemHeight; row++) {
        for (int col = 0; col < systemLength; col++) {
          nodes[row][col].set_neighbors(nodes, row, col, ClothParams.useDiags);
        }
      }
  }
  
  // returns a string that describes all nodes in the spring system
  String toString() {
    String result;
    result = "-----\nSpring system of size " + Integer.toString(systemLength) + "\n";
    for (int row = 0; row < systemHeight; row++) {
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
    for (int row = systemHeight - 2; row >= 0; row--) {
      for (int col = 0; col < systemLength; col++) {
        PtVector result = new PtVector(force);
        result.multByCon( ((float) row) / ((float) systemHeight));
        synchronized(nodes[row][col].userForce) { nodes[row][col].userForce = result; }
      }
    }
    // finally, adds a force to the end of the spring (the bottom nodes)
    for (int col = 0; col < systemLength; col++) {
        synchronized(nodes[systemHeight-1][col].userForce) { nodes[systemHeight-1][col].userForce = new PtVector(force); }
    }
  }
   
  //performs single-thread physics calculations on each node for timestep dt.
  void update(double dt) {
    if (euler) { 
      for (int row = 0; row < systemHeight; row++) {
        for (int col = 0; col < systemLength; col++) {
          // only update every other node in a row
          nodes[row][col].addDrag();
          if (row % 2 == col % 2) { nodes[row][col].update(); }
        }
      }
      //next, check collisions and integrate forces
      for (int row = 0; row < systemHeight; row++) {
        for (int col = 0; col < systemLength; col++) {
          nodes[row][col].checkCollisions();
          nodes[row][col].integrate(dt);
        }
      }
    } else {
      // first, update all nodes' forces based on their neighbors
      for (int row = 0; row < systemHeight; row++) {
        for (int col = 0; col < systemLength; col++) {
          // only update every other node in a row
          nodes[row][col].addDrag();
          if (row % 2 == col % 2) { nodes[row][col].update(); }
        }
      }
      
      // second, set all nodes to have HALF velocity and HALF position with eurlerian integration..
      for (int row = 1; row < systemHeight; row++) {
        for (int col = 0; col < systemLength; col++) {
          //nodes[row][col].checkCollisions();      
          nodes[row][col].integrateHalf(dt);
        }
      }
      
      // third recompute all nodes' forces based on their neighbors
      for (int row = 0; row < systemHeight; row++) {
        for (int col = 0; col < systemLength; col++) {
          // only update every other node in row
          nodes[row][col].addDrag();
          if (row % 2 == col % 2) { nodes[row][col].update(); }
        }
      }
      
      // fourth, use the forces from the halfway point at the original positions/velocities with the full timestep
      for (int row = 1; row < systemHeight; row++) {
        for (int col = 0; col < systemLength; col++) {
          //nodes[row][col].checkCollisions();
          nodes[row][col].integrateFull(dt);
        }
      }
    }
  }
  
  // Display all spring nodes to the screen
  void renderNodes() {
    noStroke();
    fill(255,255,255);
    beginShape(TRIANGLE_STRIP);
    texture(img);
    
    for (int row = 0; row < systemHeight; row++) {
      for (int col = 0; col < systemLength; col++) {
          if (row+1 < systemHeight) {
            // if row odd! consider 0 even
            if (row % 2 != 0) {
              normal(0, 0, -1);
                vertex((float) nodes[row][col].pos.x, (float) nodes[row][col].pos.y, 
                       (float) nodes[row][col].pos.z, (float) col/systemLength, (float) row/(systemHeight-1));
                vertex((float) nodes[row+1][col].pos.x, (float) nodes[row+1][col].pos.y, 
                       (float) nodes[row+1][col].pos.z, (float) col/systemLength, (float) (row+1)/(systemHeight-1));
          } else {
              normal(0, 0, 1);
                vertex((float) nodes[row][systemLength-col-1].pos.x, (float) nodes[row][systemLength-col-1].pos.y, 
                       (float) nodes[row][systemLength-col-1].pos.z, (float) (systemLength-col-1)/systemLength, (float) row/(systemHeight-1));
                vertex((float) nodes[row+1][systemLength-col-1].pos.x, (float) nodes[row+1][systemLength-col-1].pos.y, 
                       (float) nodes[row+1][systemLength-col-1].pos.z, (float) (systemLength-col-1)/systemLength, (float) (row+1)/(systemHeight-1));
            }
        }
      }
    }
    endShape();
  }
  
  // Display all spring nodes to the screen, via triangles between them
  void renderNodeTriangles() {
    noStroke();
    //fill(0,0,0);
    texture(img);
    //fill(255,0,0);
    
    for (int row = 0; row < systemHeight - 1; row++) {
      for (int col = 0; col < systemLength; col++) {
        //println("rendering at node " + nodes[row][col].toString());
        SpringNode nVertex1 = nodes[row][col];
        SpringNode nVertex2, nVertex3;
        if (nVertex1.neighbors[2][0] != null && nVertex1.neighbors[2][1] != null &&
            nVertex1.neighbors[2][0].neighbors[1][2] != null) { //left-down triangle is valid
        //println("making a left-down triangle");
          nVertex1 = nodes[row+1][col-1];
          nVertex2 = nodes[row][col];
          nVertex3 = nodes[row+1][col];
          beginShape(TRIANGLE);
          texture(img);
          vertex((float) nVertex1.pos.x, (float) nVertex1.pos.y, (float) nVertex1.pos.z, 
                 (float) (col-1)/(systemLength-1), (float) (row+1)/(systemHeight-1));
          vertex((float) nVertex2.pos.x, (float) nVertex2.pos.y, (float) nVertex2.pos.z, 
                 (float) col/(systemLength-1), (float) row/(systemHeight-1));
          vertex((float) nVertex3.pos.x, (float) nVertex3.pos.y, (float) nVertex3.pos.z, 
                 (float) col/(systemLength-1), (float) (row+1)/(systemHeight-1));
          endShape();
        }
        nVertex1 = nodes[row][col];
        if (nVertex1.neighbors[2][1] != null && nVertex1.neighbors[1][2] != null &&
            nVertex1.neighbors[2][1].neighbors[0][2] != null) { //right-up triangle is valid
          //println("making a right-up triangle, row and col of vertex 3 is " + nVertex1.neighbors[1][2].toString());
          nVertex2 = nodes[row+1][col];
          nVertex3 = nodes[row][col+1];
          beginShape(TRIANGLE);
          texture(img);
          vertex((float) nVertex1.pos.x, (float) nVertex1.pos.y, (float) nVertex1.pos.z, 
                 (float) col/(systemLength-1), (float) row/(systemHeight-1));
          vertex((float) nVertex2.pos.x, (float) nVertex2.pos.y, (float) nVertex2.pos.z, 
                 (float) col/(systemLength-1), (float) (row+1)/(systemHeight-1));
          vertex((float) nVertex3.pos.x, (float) nVertex3.pos.y, (float) nVertex3.pos.z, 
                 (float) (col+1)/(systemLength-1), (float) row/(systemHeight-1));
          endShape();
        }
      }
    }
  }
  
  // Display all spring nodes to the screen
  void renderNodesAsGrid() {
    stroke(5);
    for (int row = 0; row < systemHeight; row++) {
      for (int col = 0; col < systemLength; col++) {
        SpringNode n = nodes[row][col];
        if (col+1 < systemLength && n.neighbors[1][2] != null) { //This node ---> node to the right
          synchronized(n.pos) { synchronized (nodes[row][col+1].pos) {
            line((float) n.pos.x, (float) n.pos.y, (float) n.pos.z, 
              (float) nodes[row][col+1].pos.x, (float) nodes[row][col+1].pos.y, (float) nodes[row][col+1].pos.z); 
          } }
        }
        if (row+1 < systemHeight && n.neighbors[2][1] != null) { //This node ---> node underneath
          synchronized(n.pos) { synchronized (nodes[row+1][col].pos) {
            line((float) n.pos.x, (float) n.pos.y, (float) n.pos.z, 
              (float) nodes[row+1][col].pos.x, (float) nodes[row+1][col].pos.y, (float) nodes[row+1][col].pos.z);
          } }
        }
        if (ClothParams.useDiags) {
          if (col-1 > 0 && row-1 > 0 && n.neighbors[0][0] != null) { //This node ---> node at top left
            synchronized(n.pos) { synchronized (nodes[row-1][col-1].pos) {
              line((float) n.pos.x, (float) n.pos.y, (float) n.pos.z, 
                (float) nodes[row-1][col-1].pos.x, (float) nodes[row-1][col-1].pos.y, (float) nodes[row-1][col-1].pos.z);
            } }
          }
          if (col+1 < systemLength && row+1 < systemHeight && n.neighbors[0][2] != null) { //This node ---> node at top right
            synchronized(n.pos) { synchronized (nodes[row-1][col+1].pos) {
              line((float) n.pos.x, (float) n.pos.y, (float) n.pos.z, 
                (float) nodes[row-1][col+1].pos.x, (float) nodes[row-1][col+1].pos.y, (float) nodes[row-1][col+1].pos.z);
            } }
          }
        }
      }
    }
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
