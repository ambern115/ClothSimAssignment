import java.lang.*; //for Math.sqrt and Math.acos, they work with doubles

class PtVector {
  
  double x;
  double y;
  double z;
  
  PtVector() {
    x = 0;
    y = 0;
    z = 0;
  }
  
  PtVector(double newX, double newY, double newZ) {
    x = newX;
    y = newY;
    z = newZ;
  }
  
  //copy constructor
  PtVector(PtVector other) {
    x = other.x;
    y = other.y;
    z = other.z;
  }
  
  //returns a string representation for the PtVector
  String toString() {
    return "(" + x + ", " + y + ", " + z + ")";
  }
  
  // magnitude
  double getLen() {
    return Math.sqrt(x*x + y*y + z*z);
  }
  
  void addVec(PtVector v) {
    x += v.x;
    y += v.y;
    z += v.z;
  }
  
  PtVector getAddVectors(PtVector v) {
    double newX = x+v.x;
    double newY = y+v.y;
    double newZ = z+v.z;
    PtVector newV = new PtVector(newX,newY,newZ);
    return newV;
  }
  
  void mulVec(PtVector v) {
    x = x*v.x;
    y = y*v.y;
    z = z*v.z;
  }
    
  PtVector getMulVec(PtVector v) {
    double newX = x*v.x;
    double newY = y*v.y;
    double newZ = z*v.z;
    PtVector newV = new PtVector(newX,newY,newZ);
    return newV;
  }
  
  // multiple vector by a constant
  void multByCon(double n) {
    x *= n;
    y *= n;
    z *= n;
  }
  
  // multiple vector by a constant
  PtVector getMultByCon(double n) {
    double newx = x*n;
    double newy = y*n;
    double newz = z*n;
    return new PtVector(newx,newy,newz);
  }
  
  // divide vector by a constant
  void divByCon(double n) {
    x = x/n;
    y = y/n;
    z = z/n;
  }
  
  // divide vector by a constant
  PtVector divideByConstant(double n) {
    double newX = x/n;
    double newY = y/n;
    double newZ = z/n;
    return new PtVector(newX, newY, newZ);
  }
  
  // scalar projection of this vector onto v...
  // informally measures vector "similarity"
  double dotVec(PtVector v) {
    return x*v.x + y*v.y + z*v.z;
  }
  
  // returns angle between this and another vector
  double getTheta(PtVector v) {
    return Math.acos(this.dotVec(v) / (this.getLen() * v.getLen()));
  }
  
  // component of this vector in the direction of v
  double vectorProject(PtVector v) {
    return this.dotVec(v) * v.getLen();
  }
  
  void normalizeV() {
    double len = this.getLen();
    x = x/len;
    y = y/len;
    z = z/len;
  }
  
  void crossP(PtVector v) {
    double newX = y*v.z - z*v.y;
    double newY = z*v.x - x*v.z;
    double newZ = x*v.y - y*v.x;
    x = newX;
    y = newY;
    z = newZ;
  }
  
  // returns vector perpendicular (orthogonal) to both this vector and v
  PtVector getCross(PtVector v) {
    double newX = y*v.z - z*v.y;
    double newY = z*v.x - x*v.z;
    double newZ = x*v.y - y*v.x;
    PtVector newV = new PtVector(newX,newY,newZ);
    return newV;
  }
  
  //returns a brand new vector with the other subtracted from it
  PtVector getSubtractedVector(PtVector other) {
    double newX = this.x - other.x;
    double newY = this.y - other.y;
    double newZ = this.z - other.z;
    return new PtVector(newX, newY, newZ);
  }  
  
  //subtracts other from this vector
  void subtractVector(PtVector other) {
    this.x -= other.x;
    this.y -= other.y;
    this.z -= other.z;
  }
  
  PtVector subtractConstant(double constant) {
    double newX = this.x - constant;
    double newY = this.y - constant;
    double newZ = this.z - constant;
    return new PtVector(newX, newY, newZ);
  }
}
