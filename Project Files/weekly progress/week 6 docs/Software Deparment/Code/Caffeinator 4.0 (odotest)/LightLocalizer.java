import lejos.nxt.*;

public class LightLocalizer {
	private Odometer odo;
	private DifferentialFiltering df;
	private Robot robot;
	// setup array to store each line with count from 0
	private double lineAngle[];
	private final int ROTATE_SPEED = 40;
	private final double startingPosition = -3;
	private final double startingAngle = 45;
	private final double angleAdjustment = -3;

	private final int lsToCenter = 12; // light sensor to center of robot
										// distance

	public LightLocalizer(Odometer odo, DifferentialFiltering df, Robot robot) {
		this.odo = odo;
		this.df = df;
		this.robot = robot;
		lineAngle = new double[4];
	}

	public void doLocalization() {

		// to determine the coordinate of closest grid intersection
		double jX = Math.round(odo.getX() / 30);
		double jY = Math.round(odo.getY() / 30);
		double nearestIntersectionX = 30 * jX;
		double nearestIntersectionY = 30 * jY;

		// travel to starting coordinate and turn to the starting angle
		robot.travelTo(nearestIntersectionX + startingPosition,
				nearestIntersectionY + startingPosition, false);
		robot.turnTo(Math.toRadians(startingAngle));

		// start rotating and clock all 4 grid lines
		robot.setRotationSpeed(-ROTATE_SPEED);
		int count = 0;
		while (count < 4) { // clock all 4 grid lines
			if (df.lineDetection()) {
				lineAngle[count] = odo.getAngle();
				count++;
				robot.sleep(150);
			}
		}

		// do trig to compute current location
		double thetaY = lineAngle[2] - lineAngle[0];
		double thetaX = lineAngle[3] - lineAngle[1];
		double x = -lsToCenter * Math.cos(Math.toRadians(thetaY / 2))
				+ nearestIntersectionX;
		double y = -lsToCenter * Math.cos(Math.toRadians(thetaX / 2))
				+ nearestIntersectionY;
		double dTheta = 180 - (lineAngle[0] + lineAngle[2]) / 2;
		double theta = odo.getAngle() + dTheta + angleAdjustment;
		odo.setPosition(new double[] { x, y, (theta) * Math.PI / 180 },
				new boolean[] { true, true, true });

		Sound.beep();

		// when done, travel to (0,0) and turn to 0 degrees

		// robot.travelTo(nearestIntersectionX, nearestIntersectionY);
		// robot.turn(-odo.getAngle());

	}
}
