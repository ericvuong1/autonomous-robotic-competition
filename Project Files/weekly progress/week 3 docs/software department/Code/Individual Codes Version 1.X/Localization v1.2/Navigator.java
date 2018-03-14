import lejos.nxt.*;

public class Navigator extends Thread {
	private final NXTRegulatedMotor leftMotor = Motor.A;
	private final NXTRegulatedMotor rightMotor = Motor.C;
	// robot specifications
	private final double leftRadius = Odometer.wheelRadius;
	private final double rightRadius = Odometer.wheelRadius;
	private final double wheelDist = Odometer.wheelDistance;
	private final int motorStraight = 200;
	private double forwardSpeed, rotationSpeed;

	private final UltrasonicPoller usPoller = new UltrasonicPoller();
	private Odometer odometer;

	// navigator specifications
	private double[] path = new double[0];
	private boolean navigating = false;
	private static final double NAVIGATOR_TOLERANCE_DIST = 0.5;

	// avoid obstacle specifications
	private double dangerZone = 15;// threshold before avoiding an obstacle
	private boolean avoid = false;
	private final int safeDist = 100;
	
	public Navigator(Odometer odo) {
		this.odometer = odo;
	}

	public void run() {
		odometer.start();
		usPoller.start();
		// wait
		try {
			Thread.sleep(3000);
		} catch (InterruptedException ex) {
			// no error is expected here
		}
		// path order
		for (int i = 0; i < path.length; i += 2) {
			travelTo(path[i], path[i + 1]);
		}
	}

	public void travelTo(double x, double y) {
		navigating = true;

		// rotate to desired angle, and move to way-point
		leftMotor.setSpeed(motorStraight);
		rightMotor.setSpeed(motorStraight);
		driveTo(x, y);

		// main loop that checks for obstacle and destination arrival
		while (navigating) {
			driveTo(x, y);
			// conditions for the presence of an obstacle
			if (UltrasonicPoller.frontData < dangerZone && avoid == false) {
				// rotate until obstacle free in front
				boolean rotating = true;
				while (rotating) {
					leftMotor.forward();
					rightMotor.backward();
					if (UltrasonicPoller.frontData > safeDist) {
						sleep(1000);
						rotating = false;
					}
				}

				// avoid mode enabled
				avoid = true;
				// main loop for avoiding an obstacle
				while (avoid) {
					leftMotor.forward();
					rightMotor.forward();
					if (UltrasonicPoller.sideData > safeDist) {
						sleep(2000);
						avoid = false;
					}
					if (UltrasonicPoller.frontData < dangerZone) {
						rotating = true;
						while (rotating) {
							leftMotor.forward();
							rightMotor.backward();
							if (UltrasonicPoller.frontData > safeDist) {
								sleep(1000);
								rotating = false;
							}
						}
					}
					// re-orient itself and turn to target way-point once again
					driveTo(x, y);
				}
				// check if arrived at destination
				double differenceX = x - odometer.getX();
				double differenceY = y - odometer.getY();
				if (differenceX * differenceX + differenceY * differenceY < NAVIGATOR_TOLERANCE_DIST) {
					leftMotor.stop(false);
					rightMotor.stop(false);
					Sound.beep();
					navigating = false;
				}
			}
		}
	}

	// this method adjusts the robot's orientation
	// the robot will turn to its minimal angle
	// theta is in radians
	public void turnTo(double theta) { // turn to absolute angle
		// calculate min angle
		double currentTheta = odometer.getTheta()*180/Math.PI;
		double dTheta = theta - currentTheta;
		// conditions for rotating with minimal angle
		if (dTheta < Math.PI * -1) {
			dTheta += 2 * Math.PI;
		}
		if (dTheta > Math.PI * 1) {
			dTheta -= 2 * Math.PI;
		}
		// turn clockwise
		if (dTheta < 0) {
			leftMotor.rotate(
					convertAngle(leftRadius, wheelDist, Math.abs(dTheta) * 180
							/ Math.PI), true);
			rightMotor.rotate(
					-convertAngle(rightRadius, wheelDist, Math.abs(dTheta)
							* 180 / Math.PI), false);
		}
		// turn counter-clockwise
		if (dTheta > 0) {
			leftMotor.rotate(
					-convertAngle(leftRadius, wheelDist, Math.abs(dTheta) * 180
							/ Math.PI), true);
			rightMotor.rotate(
					convertAngle(rightRadius, wheelDist, Math.abs(dTheta) * 180
							/ Math.PI), false);
		}

	}
	
	public void turn(double angle){ //turn to relative angle
		
		if (angle <= 180 && angle >= -180) {
			angle = angle;
		} else if (angle < -180) {
			angle += 360;
		} else if (angle > 180) {
			angle -= 360;
		}
		Sound.beep();
		
		// turn to angle
		leftMotor.setSpeed(200);
		rightMotor.setSpeed(200);
		leftMotor.rotate(
				-convertAngle(leftRadius,
						wheelDist, angle), true);
		rightMotor.rotate(
				convertAngle(rightRadius,
						wheelDist, angle), false);
		Sound.beep();
	}

	// turns to way-point and drive forward
	public void driveTo(double x, double y) {
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

	// creates a path with an arbitrary size that will be determined at run-time
	// to set a path, write in this form "setPath(x1,y1,x2,y2,x3,y3,...)";
	public void setPath(double... path) {
		this.path = path;
	}

	// turn to desired angle
	public static int convertAngle(double radius, double width, double angle) {
		return (int) ((180.0 * Math.PI * width * angle / 360.0) / (Math.PI * radius));
	}

	// travel desired distance
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	// wrap the angle to simplify reading and avoid unecessary turns
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
	
	public double getLeftRadius(){
		return leftRadius;
	}
	
	public double getRightRadius(){
		return rightRadius;
	}
	
	public double getWheelDist(){
		return wheelDist;
	}
}
