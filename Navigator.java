@@ -0,0 +1,405 @@
import lejos.nxt.*;

/*========================================================================================
 * Navigator Class takes care of robot's movements and driving to locations
 * Methods ordering the robot to do certain movements are located at the end of this class
 ========================================================================================== */

public class Navigator extends Thread {
  
  // motors and sensors initilization
  public final NXTRegulatedMotor leftMotor = Motor.A;
  public final NXTRegulatedMotor rightMotor = Motor.C;
  private UltrasonicSensor frontUS, leftUS, rightUS;
  
  // robot specifications
  private final double leftRadius = Odometer.wheelRadius;
  private final double rightRadius = Odometer.wheelRadius;
  private final double wheelDist = Odometer.wheelDistance;
  // driving speed (300 deg/sec)
  private final int motorStraight = 300;
  
  // Ultrasonic Sensor data polling
  public static int rightData;
  public static int leftData;
  public static int frontData;
  
  // navigator specifications
  private boolean navigating = false;
  private static double NAVIGATOR_TOLERANCE_DIST = 2; //tolerance of arriving to destinate (in cm)
  
  // avoid obstacle specifications
  private final double reorientationThreshold = Math.toRadians(10); //orientation when in avoidance to exit avoidance
  private double dangerZone = 18;// threshold before avoiding an obstacle //in cm
  private double frontDangerZone = 23;// threshold before avoiding an obstacle //in cm
  private boolean avoid = false;
  private final int bandCenter = 17; //safe distance from wall during avoidance
  private final Odometer odometer;
  
//constructor
  public Navigator(Odometer odometer, UltrasonicSensor frontUS,
                   UltrasonicSensor leftUS, UltrasonicSensor rightUS) {
    this.odometer = odometer;
    this.frontUS = frontUS;
    this.leftUS = leftUS;
    this.rightUS = rightUS;
    
    frontUS.off();
    leftUS.off();
    rightUS.off();
  }
  
  
  public void run() {
    leftMotor.setAcceleration(9999);
    rightMotor.setAcceleration(9999);
    try {
      Thread.sleep(3000);
    } catch (InterruptedException ex) {
      // no error is expected here
    }
  }
//travel to (x,y) coordinate based on odometer reading
// enable/disable avoidance
  public void travelTo(double x, double y, boolean avoidance) {
    navigating = true;
    driveTo(x, y);
    
    // rotate to desired angle, and move to way-point
    setSpeed(motorStraight);
    
    double differenceX = x - odometer.getX();
    double differenceY = y - odometer.getY();
    
    // main loop that checks for obstacles and destination arrival
    
    while (navigating) {
      
      setSpeed(motorStraight);
      
      //retrieve current robot heading
      //and how close it is to destination
      double currentTheta = odometer.getTheta();
      differenceX = x - odometer.getX();
      differenceY = y - odometer.getY();
      
      leftMotor.forward();
      rightMotor.forward();
      // conditions for the presence of an obstacle
      boolean condition = true;
      //disable avoidance if located inside shooting area
      if (odometer.getX() > 8.5 * 30.48 && odometer.getY() > 8.5 * 30.48) {
        condition = false;
      }
      
      //left, right and front distance polling
      //always checks for incoming obstacles
      
      //left obstacle approaching
      //turn right and avoid using PController Wall-Follower
      if (leftDist() < 20 && avoidance == true && condition == true) {
        // driveDist(-2);
        turnTo(wrapAngle(currentTheta - Math.PI / 4));
        NAVIGATOR_TOLERANCE_DIST = 4;
        doPAvoidanceLeft(x, y);
      } else {
        leftMotor.setSpeed(motorStraight);
        rightMotor.setSpeed(motorStraight);
      }
      //right obstacle approaching
      //turn left and avoid using PController Wall-Follower
      if (rightDist() < 20 && avoidance == true && condition == true) {
        // driveDist(-2);
        turnTo(wrapAngle(currentTheta + Math.PI / 4));
        NAVIGATOR_TOLERANCE_DIST = 4;
        doPAvoidanceRight(x, y);
      } else {
        leftMotor.setSpeed(motorStraight);
        rightMotor.setSpeed(motorStraight);
      }
      //turns left or right depending on user's decision when initiating the
      //program (left or right button) --> See Execution.java for more details
      if (frontDist() < frontDangerZone && avoid == false
            && avoidance == true && condition == true) {
        //turn right
        if (Execution.right) {
          turnTo(wrapAngle(currentTheta - Math.PI / 4));
          NAVIGATOR_TOLERANCE_DIST = 4;
          // avoid mode enabled
          doPAvoidanceLeft(x, y);
        } 
        //turn left
        else if (Execution.right == false) {
          turnTo(wrapAngle(currentTheta + Math.PI / 4));
          NAVIGATOR_TOLERANCE_DIST = 4;
          // avoid mode enabled
          doPAvoidanceRight(x, y);
        }
      }
      
      // check if arrived at destination
      differenceX = x - odometer.getX();
      differenceY = y - odometer.getY();
      //uses pythagorean-theorem
      if (Math.sqrt(differenceX * differenceX + differenceY * differenceY) < NAVIGATOR_TOLERANCE_DIST) {
        setSpeed(0);
        NAVIGATOR_TOLERANCE_DIST = 2;
        navigating = false;
      }
    }
  }
  
  public void setSpeed(int speed) {
    leftMotor.setSpeed(speed);
    rightMotor.setSpeed(speed);
    return;
  }
  
  // this method adjusts the robot's orientation
  // the robot will turn to its minimal angle
  // theta is in radians
  // turns to ABSOLUTE ANGLE based on odometer reading
  public void turnTo(double theta) {
    
    setSpeed(motorStraight);
    double currentTheta = odometer.getTheta();
    double dTheta = theta - currentTheta;
    // conditions for rotating with minimal angle
    double dAngle = dTheta * 180 / Math.PI;
    if (dAngle < -180) {
      dAngle += 360;
    } else if (dAngle > 180) {
      dAngle -= 360;
    }
    
    // turn to angle
    leftMotor.rotate(-convertAngle(leftRadius, wheelDist, dAngle), true);
    rightMotor.rotate(convertAngle(rightRadius, wheelDist, dAngle), false);
    sleep(200);
  }
  //turn to relative angle
  
  public void turn(double angle) { // turn to relative angle
    
    if (angle < -180) {
      angle += 360;
    } else if (angle > 180) {
      angle -= 360;
    }
    
    // turn to angle
    leftMotor.setSpeed(200);
    rightMotor.setSpeed(200);
    leftMotor.rotate(-convertAngle(leftRadius, wheelDist, angle), true);
    rightMotor.rotate(convertAngle(rightRadius, wheelDist, angle), false);
  }
  
  // turns to way-point and drive forward
  // travelTo method will take care of checking if it arrives to destination
  public void driveTo(double x, double y) {
    setSpeed(motorStraight);
    double differenceX = x - odometer.getX();
    double differenceY = y - odometer.getY();
    turnTo(Math.atan2(differenceY, differenceX));
    leftMotor.forward();
    rightMotor.forward();
  }
  
  // drive a desired distance in cm
  public void driveDist(double distance) {
    leftMotor.rotate(convertDistance(leftRadius, distance), true);
    rightMotor.rotate(convertDistance(rightRadius, distance), false);
  }
  
  public boolean isNavigating() {
    return this.navigating;
  }
  
  //motorWheels conversion to angle and distances
  // turn to desired angle
  public static int convertAngle(double radius, double width, double angle) {
    return (int) ((180.0 * Math.PI * width * angle / 360.0) / (Math.PI * radius));
  }
  
  // travel desired distance
  private static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }
  
  // wrap the angle to simplify reading and avoid unnecessary turns
  private double wrapAngle(double rads) {
    return ((rads % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI);
  }
  
  // sleeps robot given in milliseconds
  public void sleep(int time) {
    try {
      Thread.sleep(time);
    } catch (Exception e) {
      // no error expected
    }
  }
  
  /*Data Polling from sensors
   *  uses the Ultrasonic Sensor Port 1,2 and 3
   * delay of 10ms to allow sonic waves to travel back
   */
  //front sensor
  public int frontDist() {
    int distance;
    frontUS.ping();
    sleep(10);
    distance = frontUS.getDistance();
    frontData = distance;
    return distance;
  }
  //left sensor
  public int leftDist() {
    int distance;
    leftUS.ping();
    sleep(10);
    distance = leftUS.getDistance();
    leftData = distance;
    return distance;
  }
  //right sensor
  public int rightDist() {
    int distance;
    rightUS.ping();
    sleep(10);
    distance = rightUS.getDistance();
    rightData = distance;
    return distance;
  }
  
  //do obstacle avoidance by following the wall using PController using left sensor
  public void doPAvoidanceLeft(double x, double y) {
    avoid = true;
    
    while (avoid) {
      double currentTheta = odometer.getTheta();
      int distance = leftDist();
      leftMotor.setSpeed(this.motorStraight);
      rightMotor.setSpeed(this.motorStraight);
      leftMotor.forward();
      rightMotor.forward();
      // reset the motors to regular speed if distance == 20
      if (odometer.getX() > 9.2 * 30.48 && odometer.getY() > 9.2 * 30.48) {
        driveTo(x, y);
        avoid = false;
      }
      if (distance > this.bandCenter) { // robot is too far away from the
        // wall
        if (distance > 30) {
          distance = 30;
          // to avoid the robot getting 255 distance and turning too
          // fast
        }
        leftMotor.setSpeed(this.motorStraight
                             - (distance - this.bandCenter) * 9);
        leftMotor.forward();
        // equation for left motor
        
        rightMotor
          .setSpeed((9 * (distance - this.bandCenter) + this.motorStraight));
        rightMotor.forward();
        // equation for right motor
      } else if (distance < this.bandCenter) { // robot is too close to
        // the wall
        leftMotor.setSpeed((this.bandCenter - distance) * 30
                             + this.motorStraight);
        leftMotor.forward();
        // equation for left motor
        
        rightMotor.setSpeed(30 * (distance - this.bandCenter)
                              + this.motorStraight);
        rightMotor.forward();
        // equation for right motor
      }
      //if detects obstacle in avoidance, turn 90degrees
      if (frontDist() < dangerZone) {
        setSpeed(motorStraight);
        
        turnTo(wrapAngle(currentTheta - Math.PI / 2));
      }
      
      double differenceX = x - odometer.getX();
      double differenceY = y - odometer.getY();
      // relocate if orientation matches way-point
      if ((wrapAngle(Math.abs(odometer.getTheta())
                       - wrapAngle(Math.atan2(differenceY, differenceX)))) < reorientationThreshold) {
        setSpeed(0);
        
        avoid = false;
      }
    }
    driveTo(x, y);
    
  }
  //do obstacle avoidance by following the wall using PController using right sensor
  public void doPAvoidanceRight(double x, double y) {
    avoid = true;
    
    while (avoid) {
      double currentTheta = odometer.getTheta();
      int distance = rightDist();
      leftMotor.setSpeed(this.motorStraight);
      rightMotor.setSpeed(this.motorStraight);
      leftMotor.forward();
      rightMotor.forward();
      // reset the motors to regular speed if distance == 20
      if (odometer.getX() > 9.2 * 30.48 && odometer.getY() > 9.2 * 30.48) {
        driveTo(x, y);
        avoid = false;
      }
      if (distance > this.bandCenter) { // robot is too far away from the
        // wall
        if (distance > 27) {
          distance = 27;
          // to avoid the robot getting 255 distance and turning too
          // fast
        }
        rightMotor.setSpeed(this.motorStraight
                              - (distance - this.bandCenter) * 9);
        rightMotor.forward();
        // equation for left motor
        leftMotor
          .setSpeed((9 * (distance - this.bandCenter) + this.motorStraight));
        leftMotor.forward();
        // equation for right motor
        
      } else if (distance < this.bandCenter) { // robot is too close to
        // the wall
        
        rightMotor.setSpeed((this.bandCenter - distance) * 30
                              + this.motorStraight);
        rightMotor.forward();
        // equation for left motor
        leftMotor.setSpeed(30 * (distance - this.bandCenter)
                             + this.motorStraight);
        leftMotor.forward();
        // equation for right motor
        
      }
      //if detect obstacle while in avoidance turn 90 degrees
      if (frontDist() < dangerZone) {
        setSpeed(motorStraight);
        
        turnTo(wrapAngle(currentTheta + Math.PI / 2));
      }
      
      double differenceX = x - odometer.getX();
      double differenceY = y - odometer.getY();
      // relocate if orientation matches way-point
      if ((wrapAngle(Math.abs(odometer.getTheta())
                       - wrapAngle(Math.atan2(differenceY, differenceX)))) < reorientationThreshold) {
        setSpeed(0);
        
        avoid = false;
      }
    }
    driveTo(x, y);
    
  }
  
}
