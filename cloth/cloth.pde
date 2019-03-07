// Cloth Simulation
// Amber Nelson (nels9242) & Michael Biggers (bigge019)

// initialize global variables

//import peasy.*;
//PeasyCam camera;

double startTime;
double elapsedTime;

// camera-control variables
boolean up_pressed = false; // up arrow
boolean dn_pressed = false; // down arrow
boolean lf_pressed = false; // left arrow
boolean rt_pressed = false; // right arrow
boolean sh_pressed = false; // shift key
boolean ctrl_pressed = false; // CTRL key
boolean w_pressed = false; // W key
boolean a_pressed = false; // A key
boolean s_pressed = false; // S key
boolean d_pressed = false; // D key
boolean mouse_lf_pressed = false; // left mouse button
float xStart, yStart;
float rotXStart, rotYStart;
float rotY, rotX;
float zoom;
float moveX, moveY;

PImage img;
PImage grass;

enum FixedMethod {
    LEFT,
    FLAG_LEFT,
    FLAG_DISPLAYED,
    RIGHT,
    BOTTOM,
    TOP
  }

// static class accessible from anywhere that holds our tuning 
// parameters for the simulation
static class ClothParams {
  static double goalDT = 0.0002; //the fraction of a second we simulate with each timestep
  static final boolean multithread = true; //do we want to turn on multithreading?
  static final boolean tearable = false; //cloth can be torn if it stretches too much
  static final short numPhysThreads = 12; //the number of physics threads running in the background
  static boolean useDiags = false; //Do we want to use diagonal springs for extra stability?
  
  static float floor_height = 600; //location of the floor in Y coordinates
  static float radius = 2; //radius of particle sphere
  static float mass = 1; //mass of particle
  
  static float restLen = 12; //the resting length of each springs
  static float breakLen = restLen * 1.6; //the length a spring must be for it to break
  static double k = 10000; //stiffness of the spring
  static double kd = 1000; //damping factor of spring motion
  static double gravity = 250; //acceleration due to gravity
  
  static double airDensity = .0012; //the density of the air, for calculating drag
  static double cd = 0.00001; //the drag coefficient
  
  // three components of air velocity, for drag. Can't make this into a static PtVector
  static double airVelX =  1; //50000;
  static double airVelY = 1;//- 5000;
  static double airVelZ = 1;//  10000;//55000;
  static double userAirVelAdd = 100; //how much velocity per frame the user can add to air direction
  
  static double userPullValue = 5000; //strength of force added by user pull on spring
  
  static int springSystemHeight = 26; //the number of nodes on the tall side of the spring system
  static int springSystemLength = 35; //the number of nodes on the long side of the spring system
  static FixedMethod fixedSide = FixedMethod.FLAG_LEFT; //the side of the spring system that's fixed
}

// standing velocity of the air as a PtVector
PtVector airVel = new PtVector(ClothParams.airVelX, ClothParams.airVelY, ClothParams.airVelZ);

PtVector userForce = new PtVector(0,0,0); //vector storing user pulls on the string

SpringSystem ss;

PhysicsUpdateThread[] physThreads; // the array of physics threads
PhysicsUpdateThread physThread1; //the first thread on which physics are calculated
PhysicsUpdateThread physThread2; //the second thread on which physics are calculated

boolean sim_started = false; //true if the spring rendering has begun

// initialize window
void setup() {
  size(800, 600, P3D);
  surface.setTitle("Really Cool Flag!");
  //arguments: SprngSystem(double _k, double _kv, double grav, PtVector topPos, float floor_h)
  ss = new SpringSystem(ClothParams.springSystemHeight, ClothParams.springSystemLength, 
                        ClothParams.fixedSide, new PtVector(width/10, 0, -100), airVel);
  //print(ss.toString());
  img = loadImage("transpride.png");
  grass = loadImage("grass.jpg");
  textureMode(NORMAL);
  
  if (ClothParams.multithread) {
    // get the physics threads rolling
    physThreads = new PhysicsUpdateThread[ClothParams.numPhysThreads];
    int extraRows = ss.systemHeight % ClothParams.numPhysThreads;
    int rowIncrement = ss.systemHeight / ClothParams.numPhysThreads;
    int startingRow = 0;
    int endingRow = ss.systemHeight / ClothParams.numPhysThreads;
    for (int i = 0; i < ClothParams.numPhysThreads; i++) {
      endingRow = startingRow + rowIncrement;
      if (extraRows > 0) { endingRow++; extraRows--; }
      physThreads[i] = new PhysicsUpdateThread(ss, startingRow, endingRow, i);
      physThreads[i].start();
      startingRow = endingRow;
    }
  }
  
  startTime = millis();
}

void keyPressed() {
  if (keyCode == ENTER) {
    sim_started = !sim_started;
  }
  
  if (keyCode == UP) {
    up_pressed = true;
  } else if (keyCode == DOWN) {
    dn_pressed = true;
  } else if (keyCode == LEFT) {
    lf_pressed = true;
  } else if (keyCode == RIGHT) {
    rt_pressed = true;
  } else if (keyCode == SHIFT) {
     sh_pressed = true;
  } else if (keyCode == CONTROL) {
    ctrl_pressed = true;
  } else if (keyCode == 87) { //W
    w_pressed = true;
  } else if (keyCode == 65) { //A
    a_pressed = true;
  } else if (keyCode == 83) { //S
    s_pressed = true;
  } else if (keyCode == 68) { //D
    d_pressed = true;
  }
}

void keyReleased() {
  if (keyCode == UP) {
    up_pressed = false;
  } else if (keyCode == DOWN) {
    dn_pressed = false;
  } else if (keyCode == LEFT) {
    lf_pressed = false;
  } else if (keyCode == RIGHT) {
    rt_pressed = false;
  } else if (keyCode == SHIFT) {
     sh_pressed = false;
  } else if (keyCode == CONTROL) {
    ctrl_pressed = false;
  } else if (keyCode == 87) { //W
    w_pressed = false;
  } else if (keyCode == 65) { //A
    a_pressed = false;
  } else if (keyCode == 83) { //S
    s_pressed = false;
  } else if (keyCode == 68) { //D
    d_pressed = false;
  }
}

void mousePressed() {
  if (mouseButton == LEFT) {
    mouse_lf_pressed = true;
    xStart = mouseX;
    yStart = mouseY;
    rotXStart = rotX;
    rotYStart = rotY;
  } 
}

void mouseReleased() {
  if (mouseButton == LEFT) {
    mouse_lf_pressed = false;
  }
}

void mouseWheel(MouseEvent event) {
  float e = event.getCount();
  zoom = e*-50+zoom;
}

void rotate_cam() {
  rotX = ((mouseX-xStart)/300)+rotXStart;
  rotY = ((mouseY-yStart)/300)+rotYStart;
}

void translate_cam() {
    if (w_pressed) {
    moveY += 10;
  }
  if (a_pressed) {
    moveX += 10;
  }
  if (s_pressed) {
    moveY -= 10;
  } 
  if (d_pressed) {
    moveX -= 10;
  }
}

//adds air velocity to the system from user input
void addUserAirVel() {
  if (lf_pressed) { //wind left
    airVel.x -= ClothParams.userAirVelAdd;
    //println("airVel changed to " + airVel.toString());
  }
  if (rt_pressed) { //wind right
    airVel.x += ClothParams.userAirVelAdd;
  }
  if (up_pressed) { //wind up
    airVel.y -= ClothParams.userAirVelAdd;
  }
  if (dn_pressed) { //wind down
    airVel.y += ClothParams.userAirVelAdd;
  }
  if (sh_pressed) { //wind away from screen
    airVel.z -= ClothParams.userAirVelAdd;
  }
  if (ctrl_pressed) { //wind toward screen
    airVel.z += ClothParams.userAirVelAdd;
  }
}

//displays all text that goes in the corner of the screen
void displayHUD() {
  textSize(14);
  fill(255,255,255,255);
  text(("                 Number of Spring Nodes: " + ClothParams.springSystemHeight*ClothParams.springSystemLength +
          " (" + ClothParams.springSystemHeight + "x" + ClothParams.springSystemLength + ")" +
          "\n                  Wind Speed: " + airVel.toString() +
          "\n                  Main Thread Framerate: " + frameRate +
          "\n                  Physics Updates per Frame: " + ss.currPhysFramerate), 4, -18);
}

/**
cylinder taken from http://wiki.processing.org/index.php/Cylinder
@author matt ditton
Modified by Michael Biggers to draw at a specified location with a specific color
*/
 
void cylinder(float w, float h, float xPos, float yPos, float zPos, float r, float g, float b, int sides)
{
  pushMatrix();
  
  fill(r,g,b);
  translate(xPos, yPos - h/2, zPos);
  
  float angle;
  float[] x = new float[sides+1];
  float[] z = new float[sides+1];
  
  float[] x2 = new float[sides+1];
  float[] z2 = new float[sides+1];
 
  //get the x and z position on a circle for all the sides
  for(int i=0; i < x.length; i++){
    angle = TWO_PI / (sides) * i;
    x[i] = sin(angle) * w;
    z[i] = cos(angle) * w;
  }
  
  for(int i=0; i < x.length; i++){
    angle = TWO_PI / (sides) * i;
    x2[i] = sin(angle) * w;
    z2[i] = cos(angle) * w;
  }
 
  //draw the bottom of the cylinder
  beginShape(TRIANGLE_FAN);
 
  vertex(0,   -h/2,    0);
 
  for(int i=0; i < x.length; i++){
    vertex(x[i], -h/2, z[i]);
  }
 
  endShape();
 
  //draw the center of the cylinder
  beginShape(QUAD_STRIP); 
 
  for(int i=0; i < x.length; i++){
    vertex(x[i], -h/2, z[i]);
    vertex(x2[i], h/2, z2[i]);
  }
 
  endShape();
 
  //draw the top of the cylinder
  beginShape(TRIANGLE_FAN); 
 
  vertex(0,   h/2,    0);
 
  for(int i=0; i < x.length; i++){
    vertex(x2[i], h/2, z2[i]);
  }
 
  endShape();
  
  popMatrix();
}

void setupLights() {
  ambientLight(100,100,100);
  directionalLight(70,70,70,0,.4,-1);
  lightFalloff(1, 0, 0);
  lightSpecular(0, 0, 0);
  float sxPos = width/5+200;
  float syPos = height-1;
  float szPos = 100;
  spotLight(255,255,255,sxPos,syPos,szPos,-.1,-.5,-.2,PI/15,75);
}

// function called every frame for rendering
void draw() {
  elapsedTime = (millis() - startTime) / 1000.0;
  startTime = millis();
  background(128, 43, 0); //orange
  //lights();
  setupLights();
  translate_cam();
  noStroke();
  
  if (mouse_lf_pressed) {
    beginCamera();
    float camz = (height/2.0) / tan(PI*30.0 / 180.0);
    camera(width/2.0, 400, camz,
           width/2.0, height/2.0,  0,
           0, 1, 0);
    // increase vision distance
    perspective(PI/3.0, width/height, camz/10.0, camz*20.0); 
    rotate_cam();
  } else {
    beginCamera();
    float camz = (height/2.0) / tan(PI*30.0 / 180.0);
    camera(width/2.0, 400, camz,  
          width/2.0, height/2.0,  0, 
            0, 1, 0);
    // increase vision distance
    perspective(PI/3.0, width/height, camz/10.0, camz*20.0);
  }
  // draw text here so it's not affected by cam movement
  displayHUD();
  fill(100,100,100);
  translate(moveX,moveY,zoom);
  rotateY(rotX);
  rotateX(rotY);
  endCamera();  
  
  /*//check for forces being applied by user
  if (lf_pressed) {
    userForce.addVec(new PtVector(-ClothParams.userPullValue,0,0));
  }
  if (rt_pressed) {
    userForce.addVec(new PtVector(ClothParams.userPullValue,0,0));
  }
  if (dn_pressed) {
    userForce.addVec(new PtVector(0,0,ClothParams.userPullValue));
  }
  if (up_pressed) {
    userForce.addVec(new PtVector(0,0,-ClothParams.userPullValue));
  }
  // reset user force if no input (menaing no force applied)
  if (!lf_pressed && !dn_pressed && !rt_pressed && !up_pressed) {
    //println("userForce resetting");
    userForce = new PtVector(0,0,0);
  }*/
  //else { println("lf_pressed is " + lf_pressed + ", dn_pressed is " + dn_pressed + ", rt_pressed is " + rt_pressed + ", up_pressed is " + up_pressed); }
  
  addUserAirVel();
  
  if (!ClothParams.multithread) {
    double timesteps = elapsedTime / ClothParams.goalDT;
    double extraChance = timesteps - ((int) timesteps);
    if (random(0,1) <= extraChance) {
      timesteps += 1;
    }
    double dt = ClothParams.goalDT / (int) timesteps;
    
    ss.run_single_thread(min((int) timesteps, 200), dt, userForce);
  } else { //when multithreading, the physics threads perform most physics calculations for us
  //ss.AddForceToBottomNodes(userForce);
  if (!ClothParams.tearable) { ss.renderNodeTriangles(); }
  else { ss.renderNodeTriangles(); }
  //ss.renderNodesAsGrid();
  }
  
  // ground....
  beginShape();
  fill(255,255,255);
  textureWrap(REPEAT);
  texture(grass);
  vertex(-width*10, height*2, 1000, 0, 0);
  vertex(width*11, height*2, 1000, 1, 0);
  vertex(width*11, height*2, -100000, 1, 1);
  vertex(-width*10, height*2, -100000, 0, 1);
  endShape();
  textureWrap(CLAMP);
  
  //left wall
  /*beginShape();
  fill(153,102,51);
  vertex(0, height, 20);
  vertex(0, height, -1000);
  vertex(0, 0, -1000);
  vertex(0, 0, 20);
  endShape();*/
  
  //flagpole
  cylinder(11, height*2, width/10, height*2, -100, 125, 125, 125, 30);  
  //right wall
  /*beginShape();
  fill(153,102,51);
  vertex(width, height, 20);
  vertex(width, height, -1000);
  vertex(width, 0, -1000);
  vertex(width, 0, 20);
  endShape();*/
}
  
