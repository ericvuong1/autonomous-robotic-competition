import lejos.nxt.*;

public class Navigator extends Thread {
	public final NXTRegulatedMotor leftMotor = Motor.A;
	public final NXTRegulatedMotor rightMotor = Motor.C;
	private UltrasonicSensor frontUS, leftUS, rightUS;

	// robot specifications
	private final double leftRadius = Odometer.wheelRadius;
	private final double rightRadius = Odometer.wheelRadius;
	private final double wheelDist = Odometer.wheelDistance;
	private final int motorStraight = 200;
	private final int motorLowLow = 100;
	private final int motorLow = 280;
	private final int motorHigh = 320;
	private final int motorHighHigh = 420;
	public static int rightData;
	public static int leftData;
	public static int frontData;

	private final Odometer odometer;

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

	// navigator specifications
	private double[] path = new double[0];
	private boolean navigating = false;
	private final double THETA_THRESHOLD = 0.1;
	private static double NAVIGATOR_TOLERANCE_DIST = 0.8;
	private final double reorientationThresh = 0.0523598776;

	// avoid obstacle specifications
	private double dangerZone = 15;// threshold before avoiding an obstacle
	private double initiateObsDist = 23;// threshold before avoiding an obstacle
	private boolean avoid = false;
	private final int safeDist = 50;
	private final int wallFollowerClose = 15;
	private final int bandCenter = 20;
	private final int wallFollowerFar = 19;

	public void run() {
		// wait
		// leftMotor.setAcceleration(400);
		// rightMotor.setAcceleration(400);
		try {
			Thread.sleep(3000);
		} catch (InterruptedException ex) {
			// no error is expected here
		}
		// path order
		// for (int i = 0; i < path.length; i += 2) {
		// travelTo(path[i], path[i + 1], true);
		// }
	}

	public void travelTo(double x, double y, boolean avoidance) {
		navigating = true;
		driveTo(x, y);
		// rotate to desired angle, and move to way-point
		leftMotor.setSpeed(motorStraight);
		rightMotor.setSpeed(motorStraight);

		double differenceX = x - odometer.getX();
		double differenceY = y - odometer.getY();

		// main loop that checks for obstacle and destination arrival
		while (navigating) {
			setSpeed(motorStraight);
			// correct angle if deviated
			double currentTheta = odometer.getTheta();
			differenceX = x - odometer.getX();
			differenceY = y - odometer.getY();
			double angle = Math.atan2(differenceY, differenceX) * 180 / Math.PI;
			double theta = Math.atan2(differenceY, differenceX);
			double AbsDTheta = Math.abs(angle - currentTheta);

			leftMotor.forward();
			rightMotor.forward();
			// conditions for the presence of an obstacle
			boolean condition = true;
			if(odometer.getX() > 5.5*30.48|| odometer.getY()>3.5*30.48)
			{
				condition = false;
			}
			if (leftDist() < 20 && avoidance == true && condition == true) {
				//driveDist(-2);
				turnTo(wrapAngle(currentTheta - Math.PI / 4));
				NAVIGATOR_TOLERANCE_DIST = 4;
				doPAvoidanceLeft(x, y);
			} else {
				leftMotor.setSpeed(motorStraight);
				rightMotor.setSpeed(motorStraight);
			}

			if (rightDist() < 20 && avoidance == true && condition == true) {
				//driveDist(-2);
				turnTo(wrapAngle(currentTheta + Math.PI / 4));
				NAVIGATOR_TOLERANCE_DIST = 4;
				doPAvoidanceRight(x, y);
			} else {
				leftMotor.setSpeed(motorStraight);
				rightMotor.setSpeed(motorStraight);
			}
			if (frontDist() < initiateObsDist && avoid == false
					&& avoidance == true && condition == true) {
				//driveDist(-6);
				// rotate until obstacle free in front
				turnTo(wrapAngle(currentTheta + Math.PI / 2));
				NAVIGATOR_TOLERANCE_DIST = 4;
				// avoid mode enabled
				doPAvoidanceRight(x, y);
			}
			// check if arrived at destination
			differenceX = x - odometer.getX();
			differenceY = y - odometer.getY();
			if (Math.sqrt(differenceX * differenceX + differenceY * differenceY) < NAVIGATOR_TOLERANCE_DIST) {
				setSpeed(0);
				NAVIGATOR_TOLERANCE_DIST = 0.8;
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

	// turns to way-point and drive forward
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

	// creates a path with an arbitrary size that will be determined at run-time
	// to set a path, write in this form "setPath(x1,y1,x2,y2,x3,y3,...)";
	public void setPath(double... path) {
		this.path = path;
	}

	// turn to desired angle
	public static int convertAngle(double radius, double width, double angle) {
		return (int) ((180.0 * Math.PI * width * angle / 360.0) / (Math.PI * radius));
		// return convertDistance(radius,(angle*Math.PI/180)*(width/2));
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

	public int frontDist() {
		int distance;
		frontUS.ping();
		sleep(10);
		distance = frontUS.getDistance();
		frontData = distance;
		return distance;
	}

	public int leftDist() {
		int distance;
		leftUS.ping();
		sleep(10);
		distance = leftUS.getDistance();
		leftData = distance;
		return distance;
	}

	public int rightDist() {
		int distance;
		rightUS.ping();
		sleep(10);
		distance = rightUS.getDistance();
		rightData = distance;
		return distance;
	}

	public void doAvoidance(double x, double y, int side) {
		avoid = true;
		// main loop for avoiding an obstacle
		while (avoid) {
			double currentTheta = odometer.getTheta();
			leftMotor.forward();
			rightMotor.forward();
			// wallfollower -->bangbangcontroller
			if (leftDist() > wallFollowerFar) {
				leftMotor.setSpeed(motorLowLow + 20);
				rightMotor.setSpeed(motorHigh);
			} else if (leftDist() < wallFollowerClose) {
				leftMotor.setSpeed(motorHighHigh);
				rightMotor.setSpeed(motorLowLow);
			} else {
				leftMotor.setSpeed(motorStraight);
				rightMotor.setSpeed(motorStraight);

			}

			if (frontDist() < dangerZone) {
				setSpeed(motorStraight);

				turnTo(wrapAngle(currentTheta - Math.PI / 2));
			}
			double differenceX = x - odometer.getX();
			double differenceY = y - odometer.getY();
			// relocate if orientation matches way-point
			if ((wrapAngle(Math.abs(odometer.getTheta())
					- wrapAngle(Math.atan2(differenceY, differenceX)))) < reorientationThresh) {
				setSpeed(0);

				avoid = false;
			}
		}
		// re-orient itself and turn to target way-point once again
		driveTo(x, y);
	}

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

			if (distance > this.bandCenter) { // robot is too far away from the
												// wall
				if (distance > 55) {
					distance = 33;
					// to avoid the robot getting 255 distance and turning too
					// fast
				}
				leftMotor.setSpeed(this.motorStraight
						- (distance - this.bandCenter) * 6);
				leftMotor.forward();
				// equation for left motor

				rightMotor
						.setSpeed((6 * (distance - this.bandCenter) + this.motorStraight));
				rightMotor.forward();
				// equation for right motor
			} else if (distance < this.bandCenter) { // robot is too close to
														// the wall
				leftMotor.setSpeed((this.bandCenter - distance) * 20
						+ this.motorStraight);
				leftMotor.forward();
				// equation for left motor

				rightMotor.setSpeed(20 * (distance - this.bandCenter)
						+ this.motorStraight);
				rightMotor.forward();
				// equation for right motor
			}
			if (frontDist() < dangerZone) {
				setSpeed(motorStraight);

				turnTo(wrapAngle(currentTheta - Math.PI / 2));
			}
			double differenceX = x - odometer.getX();
			double differenceY = y - odometer.getY();
			// relocate if orientation matches way-point
			if ((wrapAngle(Math.abs(odometer.getTheta())
					- wrapAngle(Math.atan2(differenceY, differenceX)))) < reorientationThresh) {
				setSpeed(0);

				avoid = false;
			}
		}
		driveTo(x, y);

	}

	public void doPAvoidanceRight(double x, double y) {
		avoid = true;
		int counter = 0;

		while (avoid) {
			double currentTheta = odometer.getTheta();
			int distance = rightDist();
			leftMotor.setSpeed(this.motorStraight);
			rightMotor.setSpeed(this.motorStraight);
			leftMotor.forward();
			rightMotor.forward();
			// reset the motors to regular speed if distance == 20

			if (distance > this.bandCenter) { // robot is too far away from the
												// wall
				if (distance > 55) {
					distance = 33;
					// to avoid the robot getting 255 distance and turning too
					// fast
				}
				rightMotor.setSpeed(this.motorStraight
						- (distance - this.bandCenter) * 6);
				rightMotor.forward();
				// equation for left motor
				leftMotor
						.setSpeed((6 * (distance - this.bandCenter) + this.motorStraight));
				leftMotor.forward();
				// equation for right motor

			} else if (distance < this.bandCenter) { // robot is too close to
														// the wall

				rightMotor.setSpeed((this.bandCenter - distance) * 20
						+ this.motorStraight);
				rightMotor.forward();
				// equation for left motor
				leftMotor.setSpeed(20 * (distance - this.bandCenter)
						+ this.motorStraight);
				leftMotor.forward();
				// equation for right motor

			}
			if (frontDist() < dangerZone) {
				setSpeed(motorStraight);

				turnTo(wrapAngle(currentTheta - Math.PI / 2));
				counter++;
			}
			if(counter > 1)
			{
				doPAvoidanceLeft(x,y);
				avoid= false;
			}
			double differenceX = x - odometer.getX();
			double differenceY = y - odometer.getY();
			// relocate if orientation matches way-point
			if ((wrapAngle(Math.abs(odometer.getTheta())
					- wrapAngle(Math.atan2(differenceY, differenceX)))) < reorientationThresh) {
				setSpeed(0);

				avoid = false;
			}
		}
		driveTo(x, y);

	}

}
