// Thread Simulation
// Amber Nelson (nels9242) & Michael Biggers (bigge019)

// initialize global variables

//import peasy.*;
//PeasyCam camera;

double startTime;
double elapsedTime;

// camera-control variables
boolean up_pressed = false;
boolean dn_pressed = false;
boolean lf_pressed = false;
boolean rt_pressed = false;
boolean w_pressed = false;
boolean a_pressed = false;
boolean s_pressed = false;
boolean d_pressed = false;
boolean mouse_lf_pressed = false;
float xStart, yStart;
float rotXStart, rotYStart;
float rotY, rotX;
float zoom;
float moveX, moveY;

// class accessible from anywhere else that holds our tuning 
// parameters for the simulation
static class ClothParams {
  static double goalDT = 0.0005; //the fraction of a second we simulate with each timestep
  static boolean useDiags = false; //Do we want to use diagonal springs for extra stability?
  
  static float floor_height = 1500; //location of the floor in Y coordinates
  static float radius = 2; //radius of particle sphere
  static float mass = 1; //mass of particle
  
  static float restLen = 20; //the resting length of each spring
  static double k = 600000L; //stiffness of the spring
  static double kd = 3000; //damping factor of spring motion
  static double gravity = 600000L; //acceleration due to gravity
  
  static double userPullValue = 8000L; //strength of force added by user pull on spring
  
  static int springSystemLength = 30; //the number of nodes on each side of the the spring system
}

PtVector userForce = new PtVector(0,0,0); //vector storing user pulls on the string

SpringSystem ss;

boolean sim_started = false; //true if the spring rendering has begun

// initialize window
void setup() {
  size(800, 600, P3D);
  surface.setTitle("Homework2_5611_Thread_Sim");
  //arguments: SprngSystem(double _k, double _kv, double grav, PtVector topPos, float floor_h)
  ss = new SpringSystem(ClothParams.springSystemLength, new PtVector(-100, -300, -20));
  print(ss.toString());
  startTime = millis();
  //camera = new PeasyCam(this, 400, 300, 0, 300);
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

void setupLights() {
  ambientLight(240,240,240);
  directionalLight(200,200,200,0,-1,0);
}

// function called every frame for rendering
void draw() {
  elapsedTime = (millis() - startTime) / 1000.0;
  startTime = millis();
  background(220,220,240);
  lights();  
  //setupLights();
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
    
    // draw text here so it's not affected by cam movement
    textSize(14);
    fill(0,0,0,255);
    text(("                 frame rate: " + frameRate), 4, -18);
    translate(moveX,moveY,zoom);
    rotateY(rotX);
    rotateX(rotY);
    endCamera();
  } else {
    beginCamera();
    float camz = (height/2.0) / tan(PI*30.0 / 180.0);
    camera(width/2.0, 400, camz,  
          width/2.0, height/2.0,  0, 
            0, 1, 0);
    // increase vision distance
    perspective(PI/3.0, width/height, camz/10.0, camz*20.0);
    
    // draw text here so it's not affected by cam movement
    textSize(14);
    fill(0,0,0,255);
    text(("                 frame rate: " + frameRate), 4, -18);    
    translate(moveX,moveY,zoom);
    rotateY(rotX);
    rotateX(rotY);
    endCamera();  
  }
  
  //check for forces being applied by user
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
    userForce = new PtVector(0,0,0);
  }
  
  // draw objects in the system
  double timesteps = elapsedTime / ClothParams.goalDT;
  double extraChance = timesteps - ((int) timesteps);
  if (random(0,1) <= extraChance) {
    timesteps += 1;
  }
  double dt = ClothParams.goalDT / (int) timesteps;
  
  // only run if the user has hit enter
  if (sim_started) { ss.run((int) timesteps, dt, userForce); }
  
  // ground....
  beginShape();
  fill(255,0,0);
  vertex(0, height, 20);
  vertex(width, height, 20);
  vertex(width, height, -1000);
  vertex(0, height, -1000);
  endShape();
  
  //left wall
  beginShape();
  fill(153,102,51);
  vertex(0, height, 20);
  vertex(0, height, -1000);
  vertex(0, 0, -1000);
  vertex(0, 0, 20);
  endShape();
  
  //right wall
  /*beginShape();
  fill(153,102,51);
  vertex(width, height, 20);
  vertex(width, height, -1000);
  vertex(width, 0, -1000);
  vertex(width, 0, 20);
  endShape();*/
}
  
