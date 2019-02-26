class SpringSystem {
  // springs are stored within two double-ArrayLists, one for rows of horizontal springs,
  // and one for rows of vertical springs
  ArrayList<ArrayList<Spring>> horizSprings = new ArrayList();
  ArrayList<ArrayList<Spring>> vertSprings = new ArrayList();
  
  ArrayList<ArrayList<Spring>> oldHorizSprings;
  ArrayList<ArrayList<Spring>> oldVertSprings;
  
  // the number of squares, horizontally and vertically, in the SpringSystem
  // squares are formed by four springs coming together to form a square shape
  int systemLength = 5;
  
  // constants
  double gravity = 10000000L;
  double k;
  double kv;
  float sprRestLen = 40;
  float floorHeight = 600;
  //the position of the anchored spring at the top of the system
  PtVector springTopLeftPos = new PtVector(width / 5, floor_height - height + 100, -20); 
  float springMass = 1;
  float nodeRadius = 4;
  
  SpringSystem(double _k, double _kv) {
    k = _k;
    kv = _kv;
    
    addSprings();
  }
  
  // spring system with floor height given
  SpringSystem(double _k, double _kv, float floor_h) {
    k = _k;
    kv = _kv;
    floor_height = floor_h;
    
    addSprings();
  }
  
  // spring system with many tuning parameters
  SpringSystem(double _k, double _kv, double grav, int len, PtVector topLeftPos, float floor_h) {
    k = _k;
    kv = _kv;
    floor_height = floor_h;
    gravity = grav;
    systemLength = len;
    springTopLeftPos = topLeftPos;
    
    addSprings();
  }
  
  //uses systemLength to add all necessary springs to this system
  void addSprings() {
    //first, create all vertical springs in the system
    for (int row = 0; row < systemLength; row++) {
      //create the ArrayList for this column
      ArrayList<Spring> vertRow = new ArrayList<Spring>();
      vertSprings.add(vertRow);
      //vertical springs must have one extra column to make a systemLengthXsystemLength series of squares
      for (int col = 0; col < systemLength + 1; col++) {
        PtVector vertTop;
        if (col == 0 && row == 0) { //top left corner
          vertTop = new PtVector(springTopLeftPos);
        } else if (row == 0) { //top row, but not top left
          vertTop = new PtVector(vertRow.get(col-1).top.x + sprRestLen, springTopLeftPos.y, springTopLeftPos.z);
        } else { //anywhere else in the system
          vertTop = vertSprings.get(row-1).get(col).bottom;
        }
        PtVector vertBottom = new PtVector(vertTop.x, vertTop.y + sprRestLen, vertTop.z);
        Spring vertSpring = new Spring(sprRestLen, vertTop, vertBottom, springMass, nodeRadius, k, kv, gravity, floorHeight);
        vertRow.add(vertSpring);
      }
    }
    
    //next, create all horizontal springs
    //horizontal springs must have one extra row to make a systemLengthXsystemLength series of squares
    for (int row = 0; row < systemLength + 1; row++) {
      //create the ArrayList for this column
      ArrayList<Spring> horizRow = new ArrayList<Spring>();
      horizSprings.add(horizRow);
      for (int col = 0; col < systemLength; col++) {
        //tie this horizontal spring to the same anchors as its nearby verticals
        PtVector horizLeft;
        if (row < systemLength) { horizLeft = vertSprings.get(row).get(col).top; }
        else { horizLeft = vertSprings.get(row-1).get(col).bottom; }
        
        PtVector horizRight;
        if (row < systemLength) { horizRight = vertSprings.get(row).get(col+1).top; }
        else { horizRight = vertSprings.get(row-1).get(col+1).bottom; }
        
        Spring horizSpring = new Spring(sprRestLen, horizLeft, horizRight, springMass, nodeRadius, k, kv, gravity, floorHeight);
        horizRow.add(horizSpring);
      }
    }
  }
  
  // returns a string that describes all nodes in the spring system
  String toString() {
    String result;
    result = "-----\nSpring system of size " + Integer.toString(systemLength) + "\n";
    for (int row = 0; row < systemLength+1; row++) {
      for (int col = 0; col < systemLength+1; col++) {
        if (row < systemLength) {
          result += "Vertical spring at row " + Integer.toString(row) + " and column " + Integer.toString(col);
          result += " has top position of " + vertSprings.get(row).get(col).top.toString() + "\n";
        }
        
        if (col < systemLength) {
          result += "Horizontal spring at row " + Integer.toString(row) + " and column " + Integer.toString(col);
          result += " has left position of " + horizSprings.get(row).get(col).top.toString() + "\n";
        }
      }
      result += "\n";
    }
    result += "-----\n\n";
    return result;
  }
  
  // adds a force to the end of the spring (the bottom nodes)
  void AddForceToBottomSprings(PtVector force) {
    //also adds a fraction of the force to all other springs so they move more naturally
    for (int row = systemLength - 1; row >= 0; row--) {
      for (int col = 0; col < systemLength + 1; col++) {
        PtVector result = new PtVector(force);
        result.multByCon( ((float) row) / ((float) systemLength));
        if (row < systemLength - 1) { vertSprings.get(row).get(col).userForce = result; }
        if (col < systemLength) { horizSprings.get(row).get(col).userForce = result; }
      }
    }
    // finally, adds a force to the end of the spring (the bottom nodes)
    for (int col = 0; col < systemLength + 1; col++) {
        vertSprings.get(systemLength - 1).get(col).userForce = new PtVector(force);
        if (col < systemLength) { horizSprings.get(systemLength).get(col).userForce = new PtVector(force); }
    }
  }
   
  //performs physics calculations on each spring for timestep dt.
  void update(double dt) {
    for (int row = 0; row < systemLength+1; row++) {
      for (int col = 0; col < systemLength+1; col++) {
        if (row < systemLength) { //update this vertical spring
          PtVector velAbove, forceBelow;
          if (row == 0) { velAbove = new PtVector(0,0,0); }
          else { velAbove = new PtVector(oldVertSprings.get(row-1).get(col).vel); }
          if (row == systemLength - 1) { forceBelow = new PtVector(0,0,0); }
          else { forceBelow = new PtVector(oldVertSprings.get(row+1).get(col).overallForce); }
          vertSprings.get(row).get(col).update(dt, velAbove, forceBelow);
        }
        
        if (col < systemLength) { //update this horizontal spring
          if (row == 0) { //top row of springs shouldn't move
            continue;
          } else { //all other springs can move
            PtVector velLeft, forceRight;
            if (col == 0) { velLeft = new PtVector(0,0,0); }
            else { velLeft = new PtVector(oldHorizSprings.get(row).get(col-1).vel); }
            if (col == systemLength - 1) { forceRight = new PtVector(0,0,0); }
            else { forceRight = new PtVector(oldHorizSprings.get(row).get(col+1).overallForce); }
            horizSprings.get(row).get(col).update(dt, velLeft, forceRight);
          }
        }
      }
    }
  }
  
  //runs the spring system for one frame.
  //performs all spring physics updates, then renders each spring.
  void run(int timesteps, double dt, PtVector bottomForce) {
    for (int i = 0; i < timesteps; i++) {
      oldVertSprings = new ArrayList(vertSprings);
      oldHorizSprings = new ArrayList(horizSprings);
      AddForceToBottomSprings(bottomForce);
      update(dt);
    }
    for (int row = 0; row < systemLength+1; row++) {
      for (int col = 0; col < systemLength+1; col++) {
        if (row < systemLength) { vertSprings.get(row).get(col).render(); }
        if (col < systemLength) { horizSprings.get(row).get(col).render(); }
      }
    }
  }
}
