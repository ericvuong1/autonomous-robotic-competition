import lejos.nxt.*;

public class Shoot extends Thread {
  private final NXTRegulatedMotor motor = Motor.B;
  
  // to get maximum speed and acceleration
  private final int speed = 9999;
  private final int acceleration = 9999;
  
  // constructor
  public Shoot() {
  }
  
  // method to shoot a ping-pong ball
  public void launch() {
    try {
      Thread.sleep(200);
    } catch (Exception e) {
    }
    motor.setAcceleration(acceleration);
    motor.setSpeed(speed);
    motor.rotate(360); // motor makes a full rotation
  }
}