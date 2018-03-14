/*
 * class for the ball shooting mechanism
 */
import lejos.nxt.*;

public class Shooter {

	private final NXTRegulatedMotor motor = Motor.B;
	
	// to get maximum speed and acceleration
	private final int speed = 9999;
	private final int acceleration = 9999;

	// constructor
	public Shooter() {
	}

	// method to shoot a ball
	public void launch() {

		while (Button.waitForAnyPress() != Button.ID_ESCAPE) {
			motor.setAcceleration(acceleration);
			motor.setSpeed(speed);
			motor.rotate(360); // motor makes a full rotation
		}

	}

}
