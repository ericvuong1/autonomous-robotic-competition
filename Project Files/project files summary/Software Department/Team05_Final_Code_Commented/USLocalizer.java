import lejos.nxt.UltrasonicSensor;

public class USLocalizer {

	//initialize objects
	private Odometer odo;
	private UltrasonicSensor us;
	private Robot robot;

	private final double centertoUS = 4.5; // center of robot to US
	private int filterValue = 63; // to filter out large value for US
	private final static double ROTATE_SPEED = 40; // rotate speed of motors

	// for doAdvancedLocalization method
	private int sleep = 50; // thread sleep time in ms
	private final double angleAdjustment = 21.13; // based on test results from
	private final double AdjustmentX = -1.05;	  // see Localization Test V2.0 for more information
	private final double AdjustmentY = -0.07;

	//constructor
	public USLocalizer(Odometer odo, UltrasonicSensor us, Robot robot) {
		this.odo = odo;
		this.us = us;
		this.robot = robot;
		
		// switch off the ultrasonic sensor
		us.off();
	}
	//perform ultrasonic localization
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

		//Uses Differential Calculus to obtain the minimums of ultrasonic sensor readings
		//determine at which angle it detects the minimums
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

		// to determine which minimum corresponds to which axis
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
	//filters out big values to avoid detecting obstacles
	public void setFilterValue(int filterValue){
		this.filterValue = filterValue;
	}
	//obtain front sensor readings for localization
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
