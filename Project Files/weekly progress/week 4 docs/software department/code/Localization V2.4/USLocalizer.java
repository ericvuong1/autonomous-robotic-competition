import lejos.nxt.Sound;
import lejos.nxt.UltrasonicSensor;

public class USLocalizer {

	private Odometer odo;
	private UltrasonicSensor us;
	private Robot robot;

	private final double centertoUS = 4.5;
	private final int filterValue = 55;
	private final static double ROTATE_SPEED = 10;
	private boolean WALLPRESENT;
	private double dtheta;
	private double angleA = 0, angleB = 0;
	private int flicker;

	public USLocalizer(Odometer odo, UltrasonicSensor us, Robot robot) {
		this.odo = odo;
		this.us = us;
		this.robot = robot;
		this.flicker = 1; // 1 means falling edge, 0 means rising edge

		// switch off the ultrasonic sensor
		us.off();
	}

	public void doLocalization() {

		doAdvanceLocalization();
		/*doOrientation();
		doCoordinate();*/

	}
	
	private void doAdvanceLocalization(){
		int count = 0;
		double current;
		double next;
		double[] a = new double [2];
		double[] angle = new double[2];
		boolean negativeSlope;
		
		robot.setRotationSpeed(-ROTATE_SPEED);
		current = getFilteredData();
		robot.sleep(50);
		next = getFilteredData();
		
		if (current - next > 0){ // to detect if there is a negative slope in US distance
			negativeSlope = true;
		}
		else
			negativeSlope = false;
		
		while (negativeSlope){
			
			current = getFilteredData();
			robot.sleep(50);
			next = getFilteredData();
			
			if (current - next < 0){
				a[count] = next;
				angle[count] = odo.getAngle();
				negativeSlope = false;
				count ++;
			}
		}
		
		while (!negativeSlope){
			
			current = getFilteredData();
			robot.sleep(50);
			next = getFilteredData();
			
			if(current - next > 0){
				negativeSlope = true;
			}
		}
		
		while (negativeSlope){
			
			current = getFilteredData();
			robot.sleep(50);
			next = getFilteredData();
			
			if (current - next < 0){
				a[count] = next;
				angle[count] = odo.getAngle();
				negativeSlope = false;
				count ++;
			}
			
		}
		robot.stop();
		
		if (angle[0] - angle [1] < 180){
			odo.setX(a[0] - 30 + centertoUS);
			odo.setY(a[1] - 30 + centertoUS);
		}
		if (angle[0] - angle [1] > 180){
			odo.setX(a[1] - 30 + centertoUS);
			odo.setY(a[0] - 30 + centertoUS);
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
			//robot.turn(-odo.getAngle());
			robot.turnTo(0);

			robot.sleep(100);
		}
	}

	private void doCoordinate() {
		
		// get Y distance
		robot.turnTo(3*Math.PI/2);
		robot.sleep(100);
		double y = getFilteredData() - 30 + centertoUS; 
		robot.sleep(100);
		
		// get X distance
		robot.turnTo(Math.PI);
		double x = getFilteredData() - 30 + centertoUS; 
		robot.sleep(100);

		robot.turn(180);

		odo.setX(x);
		odo.setY(y);
		/*double distY = (filterValue + centertoUS)*Math.cos((90 + angleA + dtheta) % 90) - 30;
		double distX = (filterValue + centertoUS)*Math.sin((90 + angleB + dtheta) % 90) - 30;
		odo.setX(distX);
		odo.setY(distY);*/
		
	}

	private int getFilteredData() {
		int distance;

		// do a ping
		us.ping();

		// wait for the ping to complete
		try {
			Thread.sleep(10);
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
