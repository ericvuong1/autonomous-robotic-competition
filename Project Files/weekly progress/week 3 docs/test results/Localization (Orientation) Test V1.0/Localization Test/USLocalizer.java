import lejos.nxt.Sound;
import lejos.nxt.UltrasonicSensor;

public class USLocalizer {
	public enum LocalizationType {
		FALLING_EDGE, RISING_EDGE
	};

	private static double ROTATE_SPEED = 40;
	private Odometer odo;
	private TwoWheeledRobot robot;
	private UltrasonicSensor us;
	private LocalizationType locType;
	private boolean WALLPRESENT;
	private double dtheta;
	private int filterValue = 45;

	public USLocalizer(Odometer odo, UltrasonicSensor us,
			LocalizationType locType) {
		this.odo = odo;
		this.robot = odo.getTwoWheeledRobot();
		this.us = us;
		this.locType = locType;

		// switch off the ultrasonic sensor
		us.off();
	}

	public void doLocalization() {
		double[] pos1 = new double[3];
		double[] pos2 = new double[3];
		double angleA = 0, angleB = 0;

		// Falling edge method
		if (locType == LocalizationType.FALLING_EDGE) {
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
					Sound.beep();
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
					Sound.beep();
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
			if (angleA > angleB) {
				dtheta = 45 - ((angleA + angleB) / 2);
			} else {
				dtheta = 225 - ((angleA + angleB) / 2);
			}
			odo.setPosition(new double[] { 0.0, 0.0, odo.getTheta() + dtheta },
					new boolean[] { true, true, true });
			odo.getNavigation().turnTo(-odo.getTheta());

			// update the odometer position
			odo.setPosition(new double[] { 0.0, 0.0, 0.0 }, new boolean[] {
					true, true, true });
		} 
		// Rising edge method
		else {

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
			if (angleA > angleB) {
				dtheta = 45 - ((angleA + angleB) / 2);
			} else {
				dtheta = 225 - ((angleA + angleB) / 2);
			}
			odo.setPosition(new double[] { 0.0, 0.0, odo.getTheta() + dtheta },
					new boolean[] { true, true, true });
			odo.getNavigation().turnTo(-odo.getTheta());
			
			// update the odometer position
			odo.setPosition(new double[] { 0.0, 0.0, 0.0 }, new boolean[] {
					true, true, true });

		}

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
