/*
 * Odometer.java
 */
import lejos.nxt.*;
import lejos.util.Timer;
import lejos.util.TimerListener;

public class Odometer extends Thread{
	private TwoWheeledRobot robot;
	private Timer odometerTimer;
	private Navigation nav;
	
	// robot position
	private double x, y, theta;
	// tachometer readings
	private double lastLeftTacho = 0.0, lastRightTacho = 0.0;
	// odometer update period, in ms
	private static final long ODOMETER_PERIOD = 25;
	// robot physical specifications
	private static final double wheelRadius = 2.1;
	private static final double wheelDistance = 15;

	// lock object for mutual exclusion
	private Object lock;

	// default constructor
	public Odometer(TwoWheeledRobot robot, boolean start) {
		// initialise variables
		this.robot = robot;
		this.nav = new Navigation(this);
		
		x = 0.0;
		y = 0.0;
		theta = Math.PI / 2;
		lock = new Object();
	}

	// run method (required for Thread)
	public void run() {
		long updateStart, updateEnd;

		while (true) {
			updateStart = System.currentTimeMillis();
			//display usReading
			LCD.drawString("D = " + UltrasonicPoller.data, 0, 4);
			// put (some of) your odometer code here

			// compute tachometer reading into radians
			double leftTacho = Math.toRadians(Motor.C.getTachoCount()); // Motor.B
																		// =
																		// leftWheel
			double rightTacho = Math.toRadians(Motor.A.getTachoCount());// Motor.A
																		// =
																		// rightWheel

			// compute change in angle
			double dLeftTacho = leftTacho - lastLeftTacho;
			double dRightTacho = rightTacho - lastRightTacho;

			// multiply dAngle * wheelsRadius
			// dleftArc or drightArc is arc length/distance made by each wheel
			double dleftArc = dLeftTacho * wheelRadius;
			double drightArc = dRightTacho * wheelRadius;

			// deltaC = arc length traveled by centre of robot
			double deltaC = (dleftArc + drightArc) / 2;

			// deltaTheta = angle made by arc length deltaC
			double deltaTheta = (dleftArc - drightArc) / wheelDistance;
			double halfDeltaTheta = deltaTheta / 2;

			// compute change in x and y (cartesian coordinates)
			double deltaX = deltaC * Math.cos(theta + halfDeltaTheta);
			double deltaY = deltaC * Math.sin(theta + halfDeltaTheta);

			// update last tachometer readings
			lastLeftTacho = leftTacho;
			lastRightTacho = rightTacho;

			synchronized (lock) {
				// don't use the variables x, y, or theta anywhere but here!

				// update current x/y/theta values
				x = x + deltaX;
				y = y + deltaY;
				theta = theta + deltaTheta;
				theta = stickToSameCircle(theta);
			}

			// this ensures that the odometer only runs once every period
			updateEnd = System.currentTimeMillis();
			if (updateEnd - updateStart < ODOMETER_PERIOD) {
				try {
					Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here because it is not
					// expected that the odometer will be interrupted by
					// another thread
				}
			}
		}
	}

	// this method prevents theta to become negative or over 2pi
	private double stickToSameCircle(double rads) {
		return ((rads % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI);
	}

	// accessors
	public void getPosition(double[] position, boolean[] update) {
		// ensure that the values don't change while the odometer is running
		synchronized (lock) {
			if (update[0])
				position[0] = x;
			if (update[1])
				position[1] = y;
			if (update[2])
				position[2] = theta * 180 / Math.PI; // display theta in terms
														// of degrees
		}
	}

	public double getX() {
		double result;

		synchronized (lock) {
			result = x;
		}

		return result;
	}

	public double getY() {
		double result;

		synchronized (lock) {
			result = y;
		}

		return result;
	}

	public double getTheta() {
		double result;

		synchronized (lock) {
			result = theta;
		}

		return result;
	}
	
	public void getPosition(double [] pos) {
		synchronized (lock) {
			pos[0] = x;
			pos[1] = y;
			pos[2] = theta;
		}
	}
	
	public Navigation getNavigation() {
		return this.nav;
	}

	// mutators
	public void setPosition(double[] position, boolean[] update) {
		// ensure that the values don't change while the odometer is running
		synchronized (lock) {
			if (update[0])
				x = position[0];
			if (update[1])
				y = position[1];
			if (update[2])
				theta = position[2];
		}
	}

	public void setX(double x) {
		synchronized (lock) {
			this.x = x;
		}
	}

	public void setY(double y) {
		synchronized (lock) {
			this.y = y;
		}
	}

	public void setTheta(double theta) {
		synchronized (lock) {
			this.theta = theta;
		}
	}
	
	public TwoWheeledRobot getTwoWheeledRobot() {
		return robot;
	}
}