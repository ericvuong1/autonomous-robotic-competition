import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.Sound;
import lejos.nxt.UltrasonicSensor;

public class USLocalizer {

	private static double ROTATE_SPEED = 40;
	private Odometer odo;
	private UltrasonicSensor us;
	private boolean WALLPRESENT;
	private double dtheta;
	private int filterValue = 60;
	private double angleA = 0, angleB = 0;
	private int USAdjustment = 7;

	public USLocalizer(Odometer odo, UltrasonicSensor us) {
		this.odo = odo;
		this.us = us;

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
			odo.getNavigator().setRotationSpeed(ROTATE_SPEED);
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
			odo.getNavigator().setRotationSpeed(ROTATE_SPEED);
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
			odo.getNavigator().setRotationSpeed(-ROTATE_SPEED);
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
			odo.getNavigator().setRotationSpeed(-ROTATE_SPEED);
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
		odo.getNavigator().turn(-odo.getTheta());

		// update the odometer position
		odo.setPosition(new double[] { 0.0, 0.0, 0.0 }, new boolean[] { true,
				true, true });
		
		double[] dist = getLocation();
		odo.setX(dist[0]);
		odo.setY(dist[1]);

	}
	
	private double[] getLocation(){
		double[] dist = new double[2];
		odo.getNavigator().turn(-90);
		odo.getNavigator().sleep(200);
		dist[1] = getFilteredData();
		odo.getNavigator().sleep(200);
		odo.getNavigator().turn(-90);
		dist[0] = getFilteredData();
		odo.getNavigator().turn(180);
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
