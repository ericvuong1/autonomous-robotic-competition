import lejos.nxt.*;

public class LightLocalizer {
	private Odometer odo;
	private ColorSensor ls;
	private DifferentialFiltering df;
	private Robot robot;
	// setup array to store each line with count from 0
	private double lineAngle[];
	private final int ROTATE_SPEED = 40;
	private final int startingPosition = 5;

	private final int lsToCenter = 12; // light sensor to center of robot distance

	public LightLocalizer(Odometer odo, ColorSensor ls, DifferentialFiltering df, Robot robot) {
		this.odo = odo;
		this.ls = ls;
		this.df = df;
		this.robot = robot;
		lineAngle = new double[4];

		// turn on the light
		ls.setFloodlight(true);
	}

	public void doLocalization() {
		
		// to determine the coordinate of closest grid intersection
		double jX = Math.round(odo.getX()/30);
		double jY = Math.round(odo.getY()/30);
		double nearestIntersectionX = 30*jX;
		double nearestIntersectionY = 30*jY;
		
		// travel to starting coordinate and turn to the starting angle
		robot.travelTo(nearestIntersectionX - startingPosition, nearestIntersectionY - startingPosition);
		robot.turnTo(Math.PI/2);
		
		// start rotating and clock all 4 grid lines
		int count = 0;
		robot.setRotationSpeed(-ROTATE_SPEED);
		while (count < 4) {
			if (df.lineDetection()) {
				lineAngle[count] = odo.getAngle();
				count++;
				Sound.beep();
				try {
					Thread.sleep(200);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
		}
		robot.setSpeed(0);

		// do trig to compute current location
		double thetaY = lineAngle[0] - lineAngle[2];
		double thetaX = lineAngle[3] - lineAngle[1];
		double x = -lsToCenter * Math.cos(Math.toRadians(thetaY / 2));
		double y = -lsToCenter * Math.cos(Math.toRadians(thetaX / 2));
		double dTheta = 270 - (lineAngle[1] + lineAngle[3]) / 2;
		double theta = odo.getAngle() + dTheta;
		odo.setPosition(new double[] { x, y, (theta)*Math.PI/180 },new boolean[] { true, true, true });

		// when done, travel to (0,0) and turn to 0 degrees

		robot.travelTo(0, 0);
		robot.turn(-odo.getAngle());

	}
}

