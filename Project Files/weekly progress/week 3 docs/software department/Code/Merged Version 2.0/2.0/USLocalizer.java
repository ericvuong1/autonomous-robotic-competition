import lejos.nxt.Sound;
import lejos.nxt.UltrasonicSensor;

public class USLocalizer {
	
	private Odometer odo;
	private UltrasonicSensor us;
	private Robot robot;

	private final double centertoUS = 4.5;
	private final int filterValue = 50;
	private final static double ROTATE_SPEED = 40;
	private boolean WALLPRESENT;
	private double dtheta;
	private double angleA = 0, angleB = 0;


	public USLocalizer(Odometer odo, UltrasonicSensor us, Robot robot) {
		this.odo = odo;
		this.us = us;
		this.robot = robot;

		// switch off the ultrasonic sensor
		us.off();
	}

	public void doLocalization() {
		double[] pos1 = new double[3];
		double[] pos2 = new double[3];
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

		Sound.beep();
		// formulas to calculate the current orientation of the robot
		if (angleA < angleB) {
			dtheta = 45 - ((angleA + angleB) / 2);
		} else {
			dtheta = 225 - ((angleA + angleB) / 2);
		}
		odo.setPosition(new double[] { 0.0, 0.0,
				(odo.getTheta() + dtheta) * Math.PI / 180 }, new boolean[] {
				true, true, true });
		robot.turn(-odo.getTheta());

		// update the odometer position
		odo.setPosition(new double[] { 0.0, 0.0, 0.0 }, new boolean[] { true,
				true, true });
		
		robot.sleep(100);

		double[] dist = getLocation();
		odo.setX(dist[0]);
		odo.setY(dist[1]);

	}

	private double[] getLocation() {
		double[] dist = new double[2];
		robot.turn(-90);
		robot.sleep(500);
		dist[1] = getFilteredData() - 30 + centertoUS;
		robot.sleep(500);
		robot.turn(-90);
		dist[0] = getFilteredData() - 30 + centertoUS;;
		robot.sleep(500);
		robot.turn(180);
		return dist;
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
