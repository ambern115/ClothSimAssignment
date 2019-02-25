// Thread Simulation
// Amber Nelson (nels9242) & Michael Biggers (bigge019)

// initialize global variables

//import peasy.*;
//PeasyCam camera;

double goalDT = 0.00004;
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

// objects
float floor_height = 600;
float radius = 20;

double k = 100000000L; //TRY-IT: How does changing k affect resting length?
double kv = 10000;

//NodeOnString nos1 = new NodeOnString(300,0,20,1,radius,floor_height);
SpringSystem ss = new SpringSystem(k, kv, floor_height);

// initialize window
void setup() {
  size(800, 600, P3D);
  surface.setTitle("Homework2_5611_Thread_Sim");
  ss.add_spring();
  ss.add_spring();
  ss.add_spring();
  startTime = millis();
  //camera = new PeasyCam(this, 400, 300, 0, 300);
}

void keyPressed() {
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
    text(("                 frame rate: " + frameRate 
          + "\n                 # particles: " + '0'), 4, -18);
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
    text(("                 frame rate: " + frameRate 
          + "\n                 # particles: " + '0'), 4, -18);    
    translate(moveX,moveY,zoom);
    rotateY(rotX);
    rotateX(rotY);
    endCamera();  
  }
  
  // draw objects in the system
  double timesteps = elapsedTime / goalDT;
  double extraChance = timesteps - ((int) timesteps);
  if (random(0,1) <= extraChance) {
    timesteps += 1;
  }
  double dt = goalDT / (int) timesteps;
  ss.run((int) timesteps, dt);
  
  // ground....
  //rect(0,floor_height-radius/2,1000,1);
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
  
