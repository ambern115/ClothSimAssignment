class PhysicsUpdateThread implements Runnable {
   private Thread t;
   private int tnum;
   private String threadName = "Physics Update Thread";
   SpringSystem system;
   int startRow = 0; //the row we start doing calculations on
   int endRow = 0; //the row we end calculations on
   
   int numPhysThreads = ClothParams.numPhysThreads;
   double dt = ClothParams.goalDT;

   PhysicsUpdateThread(SpringSystem ss, int sr, int er, int tn) {
      system = ss;
      startRow = sr;
      endRow = er;
      tnum = tn;
      threadName = "Physics Update Thread " + Integer.toString(tnum);
   }
   
   //runs as long as the thread hasn't been shutdown, and executes as long as it's active
   public void run() {
     while (true) {
       //println("active loop, framerate is " + frameRate);
       updateNodes();
     }
   }
   
   
   //perform physics updates and position integrations on this thread's
   //portion of the total number of nodes
   void updateNodes() {
     if (ClothParams.eulerian || ClothParams.semiImplicit) { // integrate using pure eulerian
       for (int row = startRow; row < endRow; row++) {
         for (int col = 0; col < ss.nodes[0].length; col++) {
           // only update every other node in a row
           ss.nodes[row][col].addDrag();
           if (row % 2 == col % 2) { ss.nodes[row][col].update(); }
         }
       }
     
       try { ss.calcBarrier.await(); } //wait for all threads to catch up to this point
       catch (Exception e) { println(e + " at barrier following updates by " + startRow); }
       
       //next, check collisions and integrate forces
       for (int row = startRow; row < endRow; row++) {
         for (int col = 0; col < ss.nodes[0].length; col++) {
           if (ClothParams.eulerian) { ss.nodes[row][col].integrate(dt); }
           else { ss.nodes[row][col].integrateSemiImplicit(dt); }
           //ss.nodes[row][col].checkCollisions();
         }
       }
     
     } else { // integrate using the midpoint method
       for (int row = startRow; row < endRow; row++) {
         for (int col = 0; col < ss.nodes[0].length; col++) {
           // only update every other node in a row
           ss.nodes[row][col].addDrag();
           if (row % 2 == col % 2) { ss.nodes[row][col].update(); }
         }
       }
       
       try { ss.calcBarrier.await(); } //wait for all threads to catch up to this point
       catch (Exception e) { println(e + " at barrier following updates by " + startRow); }
       
       //next, check collisions and integrate forces with half time step
       for (int row = startRow; row < endRow; row++) {
         for (int col = 0; col < ss.nodes[0].length; col++) {
           ss.nodes[row][col].integrateHalf(dt);
           //ss.nodes[row][col].checkCollisions();
         }
       }
       
       try { ss.calcBarrier.await(); } //wait for all threads to catch up to this point
       catch (Exception e) { println(e + " at barrier following integrations by " + startRow); }
       
       // recompute the forces applied to each node based on the half velocity and half position
       for (int row = startRow; row < endRow; row++) {
         for (int col = 0; col < ss.nodes[0].length; col++) {
           // only update every other node in a row
           ss.nodes[row][col].addDrag();
           if (row % 2 == col % 2) { ss.nodes[row][col].update(); }
         }
       }
       
       try { ss.calcBarrier.await(); } //wait for all threads to catch up to this point
       catch (Exception e) { println(e + " at barrier following updates by " + startRow); }
   
       //finally, check collisions and integrate forces for full time step
       for (int row = startRow; row < endRow; row++) {
         for (int col = 0; col < ss.nodes[0].length; col++) {
           ss.nodes[row][col].integrateFull(dt);
           //ss.nodes[row][col].checkCollisions();
         }
       }
     }
     
      try { ss.calcBarrier.await(); } //wait for all threads to catch up to this point
      catch (Exception e) { println(e + " at barrier following integrations by " + startRow); }
      //last one out pings the spring system
      if (tnum == 0) { ss.ping(); }
   }
   
  public void start () {
      //System.out.println("Starting " +  threadName);
      if (t == null) {
         t = new Thread(this, threadName);
         t.start();
      }
   }
}
