/*
 * Copyright © 2011-2012. All Rights Reserved.
 * Author: Brian Chan
 * Contact: bchanx@gmail.com
 */

//import cc.arduino.*;
import hypermedia.net.*;
import processing.net.*;
import processing.serial.*;

/*
 * Global variables.
 */
  //Arduino _arduino;
  Serial _port;
  UDP _udp;
  Timer _timer;
  boolean _run = false;
  HashMap<String, Coord> _old_coords = new HashMap<String, Coord>();
  HashMap<String, Coord> _new_coords = new HashMap<String, Coord>();

  // Motors
  final int BASE_MOTOR = 1;
  final int SHOULDER_MOTOR = 2;
  final int ELBOW_MOTOR = 3;
  final int WRIST_MOTOR = 4;
  final int HAND_MOTOR = 5;
  
  // Directional input keys
  final char BASE_LEFT = 'Q';
  final char BASE_RIGHT = 'W';
  final char SHOULDER_UP = 'E';
  final char SHOULDER_DOWN = 'R';
  final char ELBOW_UP = 'A';
  final char ELBOW_DOWN = 'S';
  final char WRIST_UP = 'D';
  final char WRIST_DOWN = 'F';
  final char HAND_OPEN = 'Z';
  final char HAND_CLOSE = 'X';
  final char STOP_ALL = 'O';
  
  // directional commands
  final int GO_LEFT = 1;
  final int GO_RIGHT = 2;
  final int GO_UP = 3;
  final int GO_DOWN = 4;
  final int GO_OPEN = 5;
  final int GO_CLOSE = 6;
  
  // Skeleton tracking positions
  final String SHOULDER = "SHOULDER";
  final String ELBOW = "ELBOW";
  final String WRIST = "WRIST";
  final String HAND = "HAND";

  // Motor-degree Conversion Rates
  final float UP_CONV_RATE = 63.5;
  final float DOWN_CONV_RATE = 57.0;


//===== COORD TEST
void coord_test () {
  Coord old_vec = new Coord(0, 0, -3);
  Coord new_vec = new Coord(-3, 0, -3);
  
  println("old: "+old_vec.toStr());
  println("new: "+new_vec.toStr());
  
  //float angle = old_vec.angleBetweenLines(new_vec);
  //println(angle);
  println("left? "+str(new_vec.isLeft(old_vec)));
  
  //angle = tmp_vec.angleBetweenVectors(new_vec);
  //println(angle);
  //println("above? "+str(new_vec.isAbove(tmp_vec)));
}

//===== SERIAL TEST
void serial_test() {
  _port = new Serial(this, Serial.list()[0], 9600);
  int i = 2;
  int cmd;
  while (true) {
      if (i%2 == 0) cmd = GO_UP;
      else cmd = GO_DOWN; 
      float angle = 15.0;
      moveArm(angle, cmd, ELBOW_MOTOR);
      i++;
  }
}

//===== UDP TEST
void udp_test() {
  _udp = new UDP(this, 6000);
  //_udp.log(true); // printout the connection activity
  _udp.listen(true);
}

//===== ROTATION TEST
void rotation_test() {
  Coord old_vec = new Coord(0, 0, -1);
  Coord new_vec = new Coord(-1,0, 0);
  println(old_vec.toStr());
  println(new_vec.toStr());

  old_vec.rotateVector(45.0, new_vec);
  println(old_vec.toStr());
  //new_vec = new Coord(0, 0, -1);
  //old_vec.rotateVector(90.0, old_vec.cross_product(new_vec));
  //println(old_vec.toStr());
}

void setup () {
  //coord_test();
  //serial_test();
  udp_test();
  //rotation_test();
}

void draw () {
  if (_run && _valid(_old_coords, _new_coords)) { // && _old_coords/_new_coords not empty
    //println("IN THE RUN LOOP!");
    // implement update loop here
    // get the new coords
    
    // CHECK ELBOW (up/down)
      // if up/down diff > 5 degrees:
        // move shoulder motor 5 degrees
        // old coord elbow up/down update +/- 5 degrees
        // old coord wrist up/down update +/- 5 degrees
    // CHECK WRIST (left/right and up/down) angle between new_xz and old_xz
      // if left/right diff > 5 degrees:
        // move base motor 5 degrees left/right
        // update old coord xz vec 5 degrees left/right
        // update old coord vec same as above
      // TMP OLD COORD assuming new (x/z)... check if up/down diff > 5 degrees:
        // move elbow motor 5 degrees up/down
        // generate tmp vector thats above/below current old_coord (if xz==0, then use vec_xz with y == 0 else y++)
        // find cross product of the two vectors
        // old coord wrist update +/- 5 degrees

    Coord new_elbow = _new_coords.get(ELBOW);
    Coord old_elbow = _old_coords.get(ELBOW);
    
    Coord new_wrist = _new_coords.get(WRIST);
    Coord old_wrist = _old_coords.get(WRIST);
    
    Coord transform_wrist = new Coord(old_wrist);
    
    // check ELBOW position
    println("NEW_ELBOW: "+new_elbow.toStr());
    println("OLD_ELBOW: "+old_elbow.toStr());
    float angle = new_elbow.angleBetween(old_elbow);
    println("  angleBtwn: "+angle);
    if(angle > ANGLE_THRESHOLD) {
      int dir = (new_elbow.isAbove(old_elbow)) ? GO_UP : GO_DOWN;
      // moveArm(ANGLE_THRESHOLD, dir, SHOULDER_MOTOR);
      old_elbow.rotateVector(ANGLE_THRESHOLD, new_elbow);
      Coord tmp_wrist = create_tmp(dir, old_wrist);
      old_wrist.rotateVector(ANGLE_THRESHOLD, tmp_wrist);
      println("  --- "+old_elbow.toStr());
      //println("  --- "+old_wrist.toStr());
      
      transform_wrist.rotateVector(angle, tmp_wrist);
      println("  --- transform! "+transform_wrist.toStr());
    }
    
    // check WRIST position
    println("NEW_WRIST: "+new_wrist.toStr());
    println("OLD_WRIST: "+old_wrist.toStr());
    {  // check left/right
      angle = abs(new_wrist.angle_xz - old_wrist.angle_xz);
      println(angle);
      if (angle > ANGLE_THRESHOLD) {
        int dir = (new_wrist.isLeft(old_wrist)) ? GO_LEFT : GO_RIGHT;
        println(dir);
        // moveArm(ANGLE_THRESHOLD, dir, ELBOW_MOTOR);
        old_wrist.rotateVector(ANGLE_THRESHOLD, dir);
        println("  @@@ "+old_wrist.toStr());
        
        transform_wrist.rotateVector(angle, dir);
        println("  @@@ transform! "+transform_wrist.toStr());
      }
      // check up/down
    }
    // check HAND position
  }
}

// Angle treshold
final float ANGLE_THRESHOLD = 5.0;
  
boolean _valid (HashMap<String, Coord> _oc, HashMap<String, Coord> _nc) {
  if (!_oc.containsKey(ELBOW)) return false;
  if (!_oc.containsKey(WRIST)) return false;
  if (!_nc.containsKey(ELBOW)) return false;
  if (!_nc.containsKey(WRIST)) return false;
  return true;
}

Coord create_tmp (int dir, Coord old) {
  float tmp_x, tmp_y, tmp_z;
  if (old.vec.x == 0 && old.vec.z == 0) {
    tmp_x = 0;
    tmp_y = 0;
    tmp_z = -1;
  } else {
    tmp_x = old.vec.x;
    tmp_y = (dir == GO_UP) ? old.vec.y + 1 : old.vec.y - 1;
    tmp_z = old.vec.z;
  }
  Coord tmp = new Coord(tmp_x, tmp_y, tmp_z);
  return tmp;
}

/*
 * Coord class for storing (x, y, z) coords
 */
class Coord {
  /*
   * Constructor and variables.
   */
  PVector vec; // actual vector in 3d space
  float angle_xz; // direction angle of vector in xz plane
  
  Coord (float x, float y, float z) {
    vec = new PVector(x, y, z);
    vec.normalize();
    _angleXZ();
  }
  Coord (Coord old) {
    vec = new PVector(old.vec.x, old.vec.y, old.vec.z);
    vec.normalize();
    angle_xz = old.angle_xz;
  }

  /*
   * Calculate the angle of vector in the (x,z) plane.
   * This function is quite .. messy.
   * Used to set the angle_xz field upon Coord initilization.
   */
  void _angleXZ() {
    if (this.vec.x == 0 && this.vec.z == 0) {
      this.angle_xz = 90.0;
      return;
    }
    float new_z = sqrt(1 - sq(this.vec.y));
    if (this.vec.z < 0) new_z *= -1;
    PVector tmp = new PVector(0, this.vec.y, new_z);
    tmp.normalize();
    
    float theta = (tmp.z*this.vec.x - this.vec.z*tmp.x)/ ((sq(this.vec.z) + sq(this.vec.x)));
    if (abs(theta) > 1) theta += abs(theta)%1; // sometimes goes beyond -1 due to precision rounding, which causes asin() to fail
    float angle_xz = asin(theta);
    if (Float.isNaN(angle_xz)) this.angle_xz = 0;
    else this.angle_xz = 90-degrees(angle_xz);
  }
  
  /*
   * Prints coord as string (x, y, z)
   */
  String toStr() {
    return("3d vec: "+vec.toString()+" / angle: "+"("+angle_xz+")");
  }

  /*
   * Rotate vector by theta degrees in the left/right direction (x,z plane)
   * Silently sets vec to new coordinates once rotation is complete.
   * Automatically increment/decrements angle_xz field.
   */
  void rotateVector(float theta, int dir) {
    PVector cross_p;
    if (dir == GO_LEFT) cross_p = new PVector(0, 1, 0);
    else cross_p = new PVector(0, -1, 0);
    _rotate(theta, this.vec, cross_p);
    if (dir == GO_LEFT) this.angle_xz -= theta;
    else if (dir == GO_RIGHT) this.angle_xz += theta;
  }   
  
  /*
   * Rotate vector by theta degrees towards direction of new_vec (x,y,z plane)
   * Silently sets vec to new coordinates once rotation is complete.
   */   
  void rotateVector(float theta, Coord new_vec) {
    PVector cross_p = this.vec.cross(new_vec.vec);
    cross_p.normalize();
    _rotate(theta, this.vec, cross_p);
  }
  
  /*
   * Rotate a vector 'theta' degrees around axis 'u'.
   */
  void _rotate(float theta, PVector old_vec, PVector u) {
    // get theta in radians
    theta = radians(theta);
    
    // get new (x, y, z) coords
    //float new_x =
    old_vec.x =
    u.x*(u.x*old_vec.x + u.y*old_vec.y + u.z*old_vec.z)*(1-cos(theta)) +
    old_vec.x*cos(theta) +
    (-u.z*old_vec.y + u.y*old_vec.z)*sin(theta);
    
    //float new_y =
    old_vec.y = 
    u.y*(u.x*old_vec.x + u.y*old_vec.y + u.z*old_vec.z)*(1-cos(theta)) +
    old_vec.y*cos(theta) +
    (u.z*old_vec.x - u.x*old_vec.z)*sin(theta);
    
    //float new_z =
    old_vec.z = 
    u.z*(u.x*old_vec.x + u.y*old_vec.y + u.z*old_vec.z)*(1-cos(theta)) +
    old_vec.z*cos(theta) +
    (-u.y*old_vec.x + u.x*old_vec.y)*sin(theta);
  }
  
  /*
   * NOTE: be careful when vectors are not perpendicular to (x,y,z) planes.
   * Find angle between two vectors in (x,y,z) plane.
   * @return angle in degrees
   */   
  float angleBetween(Coord other) {
    return degrees(PVector.angleBetween(this.vec, other.vec));
  }
  
  /*
   * Check whether current_pos is above pos.
   * @return true if above, false otherwise.
   */
  boolean isAbove(Coord pos) {
    return (this.vec.y > pos.vec.y) ? true : false;
  }
  
  /*
   * Check whether current_pos is left of pos.
   * @return true if left, false otherwise.
   */   
  boolean isLeft(Coord pos) {
    return (this.vec.x < pos.vec.x) ? true : false;
  }
}


/*
 * Timer class for adding delay.
 */
class Timer  {
  int savedTime;  //  When Timer started
  int totalTime;  //  How long Timer should last
  
  Timer (int tempTotalTime)  {
    totalTime = tempTotalTime;
  }
  
  //  Starting the timer
  void start ()  {
    savedTime = millis();  //  When the Timer starts it stores the current time in milliseconds
  }
  
  boolean isFinished ()  {
    //  Check how much time has passed
    int passedTime = millis() - savedTime;
    if (passedTime > totalTime) return true;
    return false;
  }
}

/*
 * Select input key and motor conversion rate.
 */
void moveArm(float angle, int dir, int motor) {
  switch (motor) {
    case BASE_MOTOR:
      if (dir == GO_LEFT) _write_to_arduino(angle, UP_CONV_RATE, BASE_LEFT);
      else if (dir == GO_RIGHT) _write_to_arduino(angle, DOWN_CONV_RATE, BASE_RIGHT);
      break;
    case SHOULDER_MOTOR:
      if (dir == GO_UP) _write_to_arduino(angle, UP_CONV_RATE, SHOULDER_UP);
      else if (dir == GO_DOWN) _write_to_arduino(angle, DOWN_CONV_RATE, SHOULDER_DOWN);
      break;
    case ELBOW_MOTOR:
      if (dir == GO_UP) _write_to_arduino(angle, UP_CONV_RATE, ELBOW_UP);
      else if (dir == GO_DOWN) _write_to_arduino(angle, DOWN_CONV_RATE, ELBOW_DOWN);
      break;
    case WRIST_MOTOR:
      if (dir == GO_UP) _write_to_arduino(angle, UP_CONV_RATE, WRIST_UP);
      else if (dir == GO_DOWN) _write_to_arduino(angle, DOWN_CONV_RATE, WRIST_DOWN);
      break;
    case HAND_MOTOR:
      if (dir == GO_OPEN) _write_to_arduino(angle, UP_CONV_RATE, HAND_OPEN);
      else if (dir == GO_CLOSE) _write_to_arduino(angle, DOWN_CONV_RATE, HAND_CLOSE);
      break;
  }
}

/*
 * Write character input to Arduino.
 */
void _write_to_arduino(float angle, float rate, char input) {
  int counter = floor(angle*rate);
  while (counter-- > 0) {
    _port.write(input);
  }
  _port.write(STOP_ALL);
}

/*
 * UDP handler
 */
void receive(byte[] data, String ip, int port ) {
  String message = new String(data);
  // println( "receive: \""+message+"\" from "+ip+" on port "+port );
  deserialize(message);
  println("--- AFTER DESERiALIZE: ");
  println(_old_coords);
  println(_new_coords);
}

/*
 * Deserialize message strings from Kinect.
 * (TODO) NEED BETTER WAY OF CLEANUP when END... right now it is a race condition
 */
void deserialize (String message) {
  String[] data = split(message, '=');
  if (data[0].equals("START")) {
    String[] tokens = split(data[1], ';');
    _parse_coords(_old_coords, tokens);
    _run = true;
  } else if (data[0].equals("DATA")) {
    String[] tokens = split(data[1], ';');
    _parse_coords(_new_coords, tokens);
  } else if (data[0].equals("END")) {
    _old_coords.clear();
    _new_coords.clear();
    _run = false;
  } else {
    println("--- FAIL. SHOULDN'T GET HERE. ---");
  }
}

/*
 * Parse skeleton coords from Kinect
 * (TODO) add check to see if key is valid tracking point
 * (TODO) add check to see if 3 values provided for coords
 */
void _parse_coords (HashMap<String, Coord> _hm, String[] tokens) {
  for (int i=0; i<tokens.length; i++) {
    String[] metadata = split(tokens[i], ':');
    String key = metadata[0];
    float[] values = float(split(metadata[1], ','));
    
    // z values are always negative
    if (values[2] > 0) values[2] = 0;
    
    // if Elbow, set x coord to 0 to only track yz movement
    if (key.equals(ELBOW)) values[0] = 0;
    
    Coord tmp = new Coord(values[0], values[1], values[2]);
    _hm.put(key, tmp);
  }
}

