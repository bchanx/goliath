/*
 * Copyright © 2011-2012. All Rights Reserved.
 * Author: Brian Chan
 * Contact: bchanx@gmail.com
 */

  import hypermedia.net.*;
  import processing.net.*;
  import processing.serial.*;

  /*
   * Global variables.
   */
  Serial _port;
  UDP _udp;
  Timer _timer;
  boolean _run = false;
  boolean _reset = false;
  boolean _arduino = false;
  ArrayList<Character> _motors = new ArrayList<Character>();
  HashMap<String, Float> _motor_movement = new HashMap<String, Float>();
  HashMap<String, Coord> _old_coords = new HashMap<String, Coord>();
  HashMap<String, Coord> _new_coords = new HashMap<String, Coord>();

  // Motors
  final int BASE_MOTOR = 1;
  final int SHOULDER_MOTOR = 2;
  final int ELBOW_MOTOR = 3;
  final int WRIST_MOTOR = 4;
  final int HAND_MOTOR = 5;
  
  // Directional input keys
  final char BASE_LEFT = 'W';
  final char BASE_RIGHT = 'Q';
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
  final String BASE = "BASE";
  final String SHOULDER = "SHOULDER";
  final String ELBOW = "ELBOW";
  final String WRIST = "WRIST";
  final String HAND = "HAND";

  // Motor-degree Conversion Rates
  final float CONV_RATE = 55.0;
  final float UP_CONV_RATE = 60.0;
  final float DOWN_CONV_RATE = 50.0;
  
  // Angle treshold
  final float ANGLE_THRESHOLD = 3.0;

/*
 * Setup function.
 */
void setup () {
  if (Serial.list().length != 0) {
    _arduino = true;
    _port = new Serial(this, Serial.list()[0], 9600);
  }
  _udp = new UDP(this, 6000);
  _udp.listen(true);
  _motor_movement.put(BASE, 0.0);
  _motor_movement.put(SHOULDER, 0.0);
  _motor_movement.put(ELBOW, 0.0);
  _motor_movement.put(WRIST, 0.0);
}

/*
 * Continuous draw function.
 */
void draw () {
  if (_run && _valid()) {
    // Get Current Data vectors
    Coord new_elbow = _new_coords.get(ELBOW);
    Coord old_elbow = _old_coords.get(ELBOW);
    Coord new_wrist = _new_coords.get(WRIST);
    Coord old_wrist = _old_coords.get(WRIST);
    
    // Transform vector to keep track of movement
    Coord transform = new Coord(old_wrist);
    
    // check ELBOW up/down position
    float angle = new_elbow.angleBetween(old_elbow);
    int dir = (new_elbow.isAbove(old_elbow)) ? GO_UP : GO_DOWN;
    if(angle > ANGLE_THRESHOLD) {
      _addMotor(dir, SHOULDER_MOTOR);
      old_elbow.rotateVector(ANGLE_THRESHOLD, dir);
      old_wrist.rotateVector(ANGLE_THRESHOLD, dir);
    }
    transform.rotateVector(angle, dir);

    // check WRIST left/right
    angle = abs(new_wrist.angle_xz - old_wrist.angle_xz);
    dir = (new_wrist.isLeft(old_wrist)) ? GO_LEFT : GO_RIGHT;  
    if (angle > ANGLE_THRESHOLD) {
      _addMotor(dir, BASE_MOTOR);
      old_wrist.rotateVector(ANGLE_THRESHOLD, dir);
    }
    transform.rotateVector(angle, dir);

    // check WRIST up/down    
    angle = new_wrist.angleBetween(transform);
    dir = (new_wrist.isAbove(transform)) ? GO_UP : GO_DOWN;
    if (angle > ANGLE_THRESHOLD) {
      _addMotor(dir, ELBOW_MOTOR);
      old_wrist.rotateVector(ANGLE_THRESHOLD, dir);
    }

    // check HAND position
    
    // move motors
    _write_to_arduino(ANGLE_THRESHOLD, CONV_RATE);
  }
  
  // do cleanup
  if (_reset) {
    _reset = false;
    _reset_motor(BASE, BASE_LEFT, BASE_RIGHT);
    _reset_motor(SHOULDER, SHOULDER_UP, SHOULDER_DOWN);
    _reset_motor(ELBOW, ELBOW_UP, ELBOW_DOWN);
    _reset_motor(WRIST, WRIST_UP, WRIST_DOWN);
    _old_coords.clear();
    _new_coords.clear();
    println ("--- DONE RESET! ");
  }
}


/*
 * Helper function to check whether hashmaps are empty.
 */
boolean _valid () {
  if (_old_coords.isEmpty()) return false;
  if (_new_coords.isEmpty()) return false;
  return true;
}

/*
 * Reset motor based on movement up until now.
 */
void _reset_motor(String motor, char pos, char neg) {
  float angle = _motor_movement.get(motor);
  if (abs(angle) > 0) {
    if (angle > 0) _motors.add(neg);
    else _motors.add(pos);
    _write_to_arduino(angle, CONV_RATE);
    _motor_movement.put(motor, 0.0);
  }
}

/*
 * Add choose which motor we are moving.
 */
void _addMotor(int dir, int motor) {
  switch (motor) {
    case BASE_MOTOR:
      if (dir == GO_LEFT) _add_m(BASE_LEFT, BASE, ANGLE_THRESHOLD);
      else if (dir == GO_RIGHT) _add_m(BASE_RIGHT, BASE, -ANGLE_THRESHOLD);
      break;
    case SHOULDER_MOTOR:
      if (dir == GO_UP) _add_m(SHOULDER_UP, SHOULDER, ANGLE_THRESHOLD);
      else if (dir == GO_DOWN) _add_m(SHOULDER_DOWN, SHOULDER, -ANGLE_THRESHOLD);
      break;
    case ELBOW_MOTOR:
      if (dir == GO_UP) _add_m(ELBOW_UP, ELBOW, ANGLE_THRESHOLD);
      else if (dir == GO_DOWN) _add_m(ELBOW_DOWN, ELBOW, -ANGLE_THRESHOLD);
      break;
    case WRIST_MOTOR:
      if (dir == GO_UP) _add_m(WRIST_UP, WRIST, ANGLE_THRESHOLD);
      else if (dir == GO_DOWN) _add_m(WRIST_DOWN, WRIST, -ANGLE_THRESHOLD);
      break;
    case HAND_MOTOR:
      if (dir == GO_OPEN) _motors.add(HAND_OPEN);
      else if (dir == GO_CLOSE) _motors.add(HAND_CLOSE);
      break;
  }
}

/*
 * Add motor to _motor list and increment _motor_movement.
 */
void _add_m(char input, String motor, float angle) {
  _motors.add(input);
  float tmp = _motor_movement.get(motor);
  tmp += angle;
  _motor_movement.put(motor, tmp);  
}

/*
 * Write character input to Arduino.
 */
void _write_to_arduino(float angle, float rate) {
  if (_arduino && !_motors.isEmpty()) {
    int i, counter = floor(angle*rate);
    for (i=0; i<_motors.size()-1; i++) {
      _port.write(_motors.get(i));
    }
    while (counter-- > 0) {
      _port.write(_motors.get(i));
    }
    _port.write(STOP_ALL);
    _motors.clear();
  }
}

/*
 * UDP handler
 */
void receive(byte[] data, String ip, int port ) {
  String message = new String(data);
  // println( "receive: \""+message+"\" from "+ip+" on port "+port );
  _deserialize(message);
  println("--- AFTER DESERiALIZE: ");
  println(" old_coords: ");
  hm_toString(_old_coords);
  println(" new_coords: ");
  hm_toString(_new_coords);
}

/*
 * Helper function to print hash map.
 */
void hm_toString(HashMap<String, Coord> hm) {
  Iterator i = hm.entrySet().iterator();
  while (i.hasNext()) {
    Map.Entry me = (Map.Entry)i.next();
    print("   " + me.getKey() + " is ");
    Coord tmp = (Coord) me.getValue();
    println(tmp.toStr());
  }
}

/*
 * Deserialize message strings from Kinect.
 */
void _deserialize (String message) {
  String[] data = split(message, '=');
  if (data[0].equals("START")) {
    String[] tokens = split(data[1], ';');
    _parse_coords(_old_coords, tokens);
    _run = true;
  } else if (data[0].equals("DATA")) {
    String[] tokens = split(data[1], ';');
    _parse_coords(_new_coords, tokens);
  } else if (data[0].equals("END")) {
    _run = false;
    _reset = true;
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
    // create coord
    Coord tmp = new Coord(values[0], values[1], values[2]);
    // if Elbow, set rotate vector such that x==0 to track yz movement
    if (key.equals(ELBOW)) {
      float theta = 90 - tmp.angle_xz;
      int dir = (theta > 0) ? GO_RIGHT : GO_LEFT;
      tmp.rotateVector(theta, dir);
    }
    // put into hashmap
    _hm.put(key, tmp);
  }
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
    if (abs(theta) > 1) theta += abs(theta)%1; // sometimes goes beyond -1 due to precision rounding
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
   * Get the cross product vector for up/down rotation.
   * Generates a vector above or below current vector, then
   * calculates and returns the cross product.
   * int dir - expects GO_UP or GO_DOWN only.
   */
   PVector _get_cross_p (int dir) {
      float tmp_x, tmp_y, tmp_z;
      if (this.vec.x == 0 && this.vec.z == 0) {
        tmp_x = 0;
        tmp_y = 0;
        tmp_z = -1;
      } else {
        tmp_x = this.vec.x;
        tmp_y = (dir == GO_UP) ? this.vec.y + 1 : this.vec.y - 1;
        tmp_z = this.vec.z;
      }
      PVector tmp = new PVector(tmp_x, tmp_y, tmp_z);
      PVector cross_p = this.vec.cross(tmp);
      cross_p.normalize();
      return cross_p;
    }
    
  /*
   * Rotate vector by theta degrees in the left/right/up/down direction
   * Silently sets vec to new coordinates once rotation is complete.
   * Automatically increment/decrements angle_xz field.
   */
  void rotateVector(float theta, int dir) {
    PVector cross_p;
    if (dir == GO_LEFT) cross_p = new PVector(0, 1, 0);
    else if (dir == GO_RIGHT) cross_p = new PVector(0, -1, 0);
    else cross_p = _get_cross_p(dir);
    _rotate(theta, this.vec, cross_p);
    if (dir == GO_LEFT) this.angle_xz -= theta;
    else if (dir == GO_RIGHT) this.angle_xz += theta;
  }   
  
  /*
   * Rotate vector by theta degrees towards new_vec.
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
    float new_x =
    u.x*(u.x*old_vec.x + u.y*old_vec.y + u.z*old_vec.z)*(1-cos(theta)) +
    old_vec.x*cos(theta) +
    (-u.z*old_vec.y + u.y*old_vec.z)*sin(theta);
    
    float new_y =
    u.y*(u.x*old_vec.x + u.y*old_vec.y + u.z*old_vec.z)*(1-cos(theta)) +
    old_vec.y*cos(theta) +
    (u.z*old_vec.x - u.x*old_vec.z)*sin(theta);
    
    float new_z =
    u.z*(u.x*old_vec.x + u.y*old_vec.y + u.z*old_vec.z)*(1-cos(theta)) +
    old_vec.z*cos(theta) +
    (-u.y*old_vec.x + u.x*old_vec.y)*sin(theta);
    
    // update vector coordinates
    old_vec.x = new_x;
    old_vec.y = new_y;
    old_vec.z = new_z;
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

