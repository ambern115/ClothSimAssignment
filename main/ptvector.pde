class PtVector {
  
  float x;
  float y;
  float z;
  
  PtVector() {
    x = 0;
    y = 0;
    z = 0;
  }
  
  PtVector(float newX, float newY, float newZ) {
    x = newX;
    y = newY;
    z = newZ;
  }
  
  // magnitude
  float getLen() {
    return sqrt(x*x + y*y + z*z);
  }
  
  void addVec(PtVector v) {
    x += v.x;
    y += v.y;
    z += v.z;
  }
  
  PtVector getAdd(PtVector v) {
    float newX = x+v.x;
    float newY = y+v.y;
    float newZ = z+v.z;
    PtVector newV = new PtVector(newX,newY,newZ);
    return newV;
  }
  
  void mulVec(PtVector v) {
    x = x*v.x;
    y = y*v.y;
    z = z*v.z;
  }
    
  PtVector getMulVec(PtVector v) {
    float newX = x*v.x;
    float newY = y*v.y;
    float newZ = z*v.z;
    PtVector newV = new PtVector(newX,newY,newZ);
    return newV;
  }
  
  // multiple vector by a constant
  void multByCon(float n) {
    x *= n;
    y *= n;
    z *= n;
  }
  
  // multiple vector by a constant
  PtVector getMultByCon(float n) {
    float newx = x*n;
    float newy = y*n;
    float newz = z*n;
    return new PtVector(newx,newy,newz);
  }
  
  // divide vector by a constant
  void divByCon(float n) {
    x = x/n;
    y = y/n;
    z = z/n;
  }
  
  
  
  // scalar projection of this vector onto v...
  // informally measures vector "similarity"
  float dotVec(PtVector v) {
    return x*v.x + y*v.y + z*v.z;
  }
  
  // returns angle between this and another vector
  float getTheta(PtVector v) {
    return acos(this.dotVec(v) / (this.getLen() * v.getLen()));
  }
  
  // component of this vector in the direction of v
  float vectorProject(PtVector v) {
    return this.dotVec(v) * v.getLen();
  }
  
  void normalizeV() {
    float len = this.getLen();
    x = x/len;
    y = y/len;
    z = z/len;
  }
  
  void crossP(PtVector v) {
    float newX = y*v.z - z*v.y;
    float newY = z*v.x - x*v.z;
    float newZ = x*v.y - y*v.x;
    x = newX;
    y = newY;
    z = newZ;
  }
  
  // returns vector perpendicular (orthogonal) to both this vector and v
  PtVector getCross(PtVector v) {
    float newX = y*v.z - z*v.y;
    float newY = z*v.x - x*v.z;
    float newZ = x*v.y - y*v.x;
    PtVector newV = new PtVector(newX,newY,newZ);
    return newV;
  }
}
