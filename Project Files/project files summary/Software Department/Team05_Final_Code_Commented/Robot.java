import lejos.nxt.UltrasonicSensor;
/*
 * This is an inherited class of navigator, 
 * which allows the robot to do perform the 
 * same movements as navigator and more. 
 */
public class Robot extends Navigator {
	//attributes
	private double forwardSpeed, rotationSpeed;
	//robot specification
	private final double leftRadius = Odometer.wheelRadius;
	private final double rightRadius = Odometer.wheelRadius;
	private final double wheelDist = Odometer.wheelDistance;

	public Robot(Odometer odometer, UltrasonicSensor frontUS,
			UltrasonicSensor leftUS, UltrasonicSensor rightUS) {
		super(odometer, frontUS, leftUS, rightUS);
	}
//set rotation speed
	public void setRotationSpeed(double speed) { // set the rotation speed of
													// robot
		rotationSpeed = speed;
		setSpeeds(forwardSpeed, rotationSpeed);
	}
//set rotation motor speed and rotation speed

	public void setSpeeds(double forwardSpeed, double rotationalSpeed) {

		double leftSpeed, rightSpeed;

		this.forwardSpeed = forwardSpeed;
		this.rotationSpeed = rotationalSpeed;

		leftSpeed = (forwardSpeed + rotationalSpeed * wheelDist * Math.PI
				/ 360.0)
				* 180.0 / (leftRadius * Math.PI);
		rightSpeed = (forwardSpeed - rotationalSpeed * wheelDist * Math.PI
				/ 360.0)
				* 180.0 / (rightRadius * Math.PI);

		// set motor directions
		if (leftSpeed > 0.0)
			leftMotor.forward();
		else {
			leftMotor.backward();
			leftSpeed = -leftSpeed;
		}

		if (rightSpeed > 0.0)
			rightMotor.forward();
		else {
			rightMotor.backward();
			rightSpeed = -rightSpeed;
		}

		// set motor speeds
		if (leftSpeed > 900.0)
			leftMotor.setSpeed(900);
		else
			leftMotor.setSpeed((int) leftSpeed);

		if (rightSpeed > 900.0)
			rightMotor.setSpeed(900);
		else
			rightMotor.setSpeed((int) rightSpeed);
	}

}
