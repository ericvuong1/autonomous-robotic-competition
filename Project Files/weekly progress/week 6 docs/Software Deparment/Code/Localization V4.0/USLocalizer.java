import lejos.nxt.Sound;
import lejos.nxt.UltrasonicSensor;

public class USLocalizer {

	private Odometer odo;
	private UltrasonicSensor us;
	private Robot robot;

	private final double centertoUS = 4.5; // center of robot to US
	private final int filterValue = 63; // to filter out large value for US
	private final static double ROTATE_SPEED = 40; // rotate speed of motors

	// for doAdvancedLocalization method
	private int sleep = 50; // thread sleep time in ms
	private final double angleAdjustment = 0; // based on test results from
													// Localization Test V2.0
	private final double AdjustmentX = 0;
	private final double AdjustmentY = 0;//1.638;

	// for other localization method
	private int flicker; // to switch between falling edge or rising edge
	private boolean WALLPRESENT;
	private double dtheta;
	private double angleA = 0, angleB = 0;
	// based on test results for rising edge localization
	private final double xAdjustment = 0.84;
	private final double yAdjustment = 2.41;

	public USLocalizer(Odometer odo, UltrasonicSensor us, Robot robot) {
		this.odo = odo;
		this.us = us;
		this.robot = robot;
		this.flicker = 0; // 1 means falling edge, 0 means rising edge

		// switch off the ultrasonic sensor
		us.off();
	}

	public void doLocalization() {
		int count = 0; // counter for the number of minimums the robot gets
		double currentDist; // current distance
		double nextDist; // next distance
		double[] minimum = new double[2]; // storing the minimum
		double[] angle = new double[2]; // storing the angle when min is
										// detected
		boolean negativeSlope;

		robot.setRotationSpeed(-ROTATE_SPEED);
		currentDist = getFilteredData();
		robot.sleep(sleep);
		nextDist = getFilteredData();

		if (currentDist - nextDist > 0) { // to detect if there is a negative
											// slope in
			// US distance at the start
			negativeSlope = true;
		} else
			negativeSlope = false;

		while (count < 2) { // getting two minimums

			while (negativeSlope && count < 2) {

				robot.sleep(sleep);
				nextDist = getFilteredData();

				if (nextDist - currentDist > 0) {
					minimum[count] = currentDist;
					angle[count] = odo.getAngle();
					negativeSlope = false;
					count++;
				}

				currentDist = nextDist;
			}

			while (!negativeSlope && count < 2) {

				robot.sleep(sleep);
				nextDist = getFilteredData();

				if (currentDist - nextDist > 0) {
					negativeSlope = true;
				}
				currentDist = nextDist;
			}
		}

		robot.setSpeed(0);

		// to determine which min corresponds to which axis
		double angleDifference = (360 + (angle[1] - angle[0])) % 360;

		if (angleDifference < 180) {
			double adjustedAngle = odo.getAngle() + 270 - angle[1]
					+ angleAdjustment;
			odo.setX(minimum[0] - 30 + centertoUS + AdjustmentX);
			odo.setY(minimum[1] - 30 + centertoUS + AdjustmentY);
			odo.setTheta(adjustedAngle * Math.PI / 180);
		}
		if (angleDifference > 180) {
			double adjustedAngle = odo.getAngle() + 180 - angle[1]
					+ angleAdjustment;
			odo.setX(minimum[1] - 30 + centertoUS + AdjustmentX);
			odo.setY(minimum[0] - 30 + centertoUS + AdjustmentY);
			odo.setTheta(adjustedAngle * Math.PI / 180);
		}
	}

	private void doOrientation() {
		double[] pos1 = new double[3];
		double[] pos2 = new double[3];
		if (flicker == 1) {
			// falling edge method
			if (getFilteredData() < filterValue) { // to determine if the robot
				// is initially facing a
				// wall or not
				WALLPRESENT = true;
			} else {
				WALLPRESENT = false;
			}

			// if robot is initially facing a wall, rotate clockwise the robot
			// until it sees no wall
			while (WALLPRESENT == true) {
				robot.setRotationSpeed(ROTATE_SPEED);
				if (getFilteredData() == filterValue) {
					WALLPRESENT = false;

					// to prevent skipping the next while loop
					try {
						Thread.sleep(150);
					} catch (InterruptedException e) {
						e.printStackTrace();
					}
				}
			}
			// keep rotating until the robot sees a wall, then latch the angle
			while (WALLPRESENT == false) {
				robot.setRotationSpeed(ROTATE_SPEED);
				if (getFilteredData() < filterValue) {
					WALLPRESENT = true;
					odo.getPosition(pos2);
					angleA = pos2[2]; // save this angle A

					// to prevent skipping the next while loop
					try {
						Thread.sleep(150);
					} catch (InterruptedException e) {
						e.printStackTrace();
					}
				}
			}

			// switch direction and wait until it sees no wall
			while (WALLPRESENT == true) {
				robot.setRotationSpeed(-ROTATE_SPEED);
				if (getFilteredData() == filterValue) {
					WALLPRESENT = false;

					// to prevent skipping the next while loop
					try {
						Thread.sleep(150);
					} catch (InterruptedException e) {
						e.printStackTrace();
					}
				}
			}
			// keep rotating until the robot sees a wall, then latch the angle
			while (WALLPRESENT == false) {
				robot.setRotationSpeed(-ROTATE_SPEED);
				if (getFilteredData() < filterValue) {
					WALLPRESENT = true;
					odo.getPosition(pos1);
					angleB = pos1[2]; // save this angle B

					// to prevent skipping
					try {
						Thread.sleep(150);
					} catch (InterruptedException e) {
						e.printStackTrace();
					}
				}
			}

			Sound.beep();
			// formulas to calculate the current orientation of the robot
			if (angleA < angleB) {
				dtheta = 45 - ((angleA + angleB) / 2);
			} else {
				dtheta = 225 - ((angleA + angleB) / 2);
			}
			// update the odometer position
			odo.setPosition(new double[] { 0.0, 0.0,
					(odo.getAngle() + dtheta) * Math.PI / 180 }, new boolean[] {
					true, true, true });
			robot.turn(-odo.getAngle());

			robot.sleep(100);
		} else {
			// Rising edge method

			if (getFilteredData() == filterValue) {
				WALLPRESENT = false;
			} else {
				WALLPRESENT = true;
			}

			// rotate the robot until it sees a wall
			while (WALLPRESENT == false) {
				robot.setRotationSpeed(ROTATE_SPEED);
				if (getFilteredData() < filterValue) {
					WALLPRESENT = true;

					// to prevent skipping
					try {
						Thread.sleep(150);
					} catch (InterruptedException e) {
						e.printStackTrace();
					}
				}
			}
			// keep rotating until the robot sees no wall, then latch the angle
			while (WALLPRESENT == true) {
				robot.setRotationSpeed(ROTATE_SPEED);
				if (getFilteredData() == filterValue) {
					WALLPRESENT = false;
					odo.getPosition(pos1);
					angleB = pos1[2]; // save angleB

					// to prevent skipping
					try {
						Thread.sleep(150);
					} catch (InterruptedException e) {
						e.printStackTrace();
					}
				}
			}

			// switch direction and wait until it sees a wall
			while (WALLPRESENT == false) {
				robot.setRotationSpeed(-ROTATE_SPEED);
				if (getFilteredData() < filterValue) {
					WALLPRESENT = true;

					// to prevent skipping
					try {
						Thread.sleep(150);
					} catch (InterruptedException e) {
						e.printStackTrace();
					}
				}
			}
			// keep rotating until the robot sees no wall, then latch the angle
			while (WALLPRESENT == true) {
				robot.setRotationSpeed(-ROTATE_SPEED);
				if (getFilteredData() == filterValue) {
					WALLPRESENT = false;
					odo.getPosition(pos2);
					angleA = pos2[2]; // save angleA

					// to prevent skipping
					try {
						Thread.sleep(150);
					} catch (InterruptedException e) {
						e.printStackTrace();
					}
				}
			}

			// formulas to calculate the current orientation of the robot
			if (angleA < angleB) {
				dtheta = 45 - ((angleA + angleB) / 2);
			} else {
				dtheta = 225 - ((angleA + angleB) / 2);
			}
			// update the odometer position
			odo.setPosition(new double[] { 0.0, 0.0,
					(odo.getAngle() + dtheta) * Math.PI / 180 }, new boolean[] {
					true, true, true });
			// robot.turn(-odo.getAngle());
			robot.turnTo(0);

			robot.sleep(100);
		}
	}

	private void doCoordinate() {

		// get Y distance
		robot.turnTo(3 * Math.PI / 2);
		robot.sleep(100);
		double y = getFilteredData() - 30 + centertoUS + yAdjustment;
		robot.sleep(100);

		// get X distance
		robot.turnTo(Math.PI);
		double x = getFilteredData() - 30 + centertoUS + xAdjustment;
		robot.sleep(100);

		odo.setX(x);
		odo.setY(y);
		/*
		 * double distY = (filterValue + centertoUS)*Math.cos((90 + angleA +
		 * dtheta) % 90) - 30; double distX = (filterValue +
		 * centertoUS)*Math.sin((90 + angleB + dtheta) % 90) - 30;
		 * odo.setX(distX); odo.setY(distY);
		 */

	}

	private int getFilteredData() {
		int distance;

		// do a ping
		us.ping();

		// wait for the ping to complete
		try {
			Thread.sleep(100);
		} catch (InterruptedException e) {
		}

		// to filter large distance
		if (us.getDistance() > filterValue) {
			distance = filterValue;
		} else {
			distance = us.getDistance();
		}
		return distance;
	}
}
