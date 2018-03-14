import lejos.nxt.UltrasonicSensor;

public class Robot extends Navigator{
	private double forwardSpeed, rotationSpeed;
	private final double leftRadius = Odometer.wheelRadius;
	private final double rightRadius = Odometer.wheelRadius;
	private final double wheelDist = Odometer.wheelDistance;
	private Odometer odometer;
	private UltrasonicSensor frontUS, sideUS;

	 public Robot(Odometer odometer, UltrasonicSensor frontUS, UltrasonicSensor sideUS) {
	        super(odometer, frontUS, sideUS);
	    }
	public void setRotationSpeed(double speed) {
		rotationSpeed = speed;
		setSpeeds(forwardSpeed, rotationSpeed);
	}

	public void setSpeeds(double forwardSpeed, double rotationalSpeed) {

		double leftSpeed, rightSpeed;

		this.forwardSpeed = forwardSpeed;
		this.rotationSpeed = rotationalSpeed; 

		leftSpeed = (forwardSpeed + rotationalSpeed * wheelDist * Math.PI / 360.0) *
				180.0 / (leftRadius * Math.PI);
		rightSpeed = (forwardSpeed - rotationalSpeed * wheelDist * Math.PI / 360.0) *
				180.0 / (rightRadius * Math.PI);

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
			leftMotor.setSpeed((int)leftSpeed);
		
		if (rightSpeed > 900.0)
			rightMotor.setSpeed(900);
		else
			rightMotor.setSpeed((int)rightSpeed);
	}


}
