// Thread Simulation
// Amber Nelson (nels9242) & Michael Biggers (bigge019)

// initialize global variables

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
float floor_height = 2000;
float radius = 20;
//NodeOnString nos1 = new NodeOnString(300,0,20,1,radius,floor_height);
SpringSystem ss = new SpringSystem(0.9, 0.1, floor_height);

// initialize window
void setup() {
  size(800, 600, P3D);
  surface.setTitle("Homework2_5611_Thread_Sim");
  ss.add_node();
  ss.add_node();
  ss.add_node();

  
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
  background(220,220,240);
  setupLights();
  translate_cam();
  
  noStroke();
  lights();
  
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
  ss.run();
  
  // ground....
  rect(0,floor_height-radius/2,1000,1);
}
  
