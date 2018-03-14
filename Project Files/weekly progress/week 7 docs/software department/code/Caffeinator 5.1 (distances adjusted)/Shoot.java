import lejos.nxt.*;

public class Shoot extends Thread {
	private final NXTRegulatedMotor motor = Motor.B;

	// to get maximum speed and acceleration
	private final int speed = 9999;
	private final int acceleration = 9999;

	// constructor
	public Shoot() {
	}

	// method to shoot a ball
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
/*
 * navigator.travelTo(9.2, 10.6); navigator.turnTo(53.13); Shoot.launch();
 * navigator.turnTo(12.09); Shoot.launch();
 */