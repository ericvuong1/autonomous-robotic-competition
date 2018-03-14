import lejos.nxt.*;
public class Navigation extends Thread
{
	private static final double NAVIGATOR_TOLERANCE_DIST = 0.5;
	private final NXTRegulatedMotor leftMotor = Motor.A;
	private final NXTRegulatedMotor rightMotor = Motor.C;
	private final NXTRegulatedMotor rotatingSensor = Motor.B;
	private Odometer odometer;
	private TwoWheeledRobot robot;
	//robot specifications
	private final double leftRadius = 2.1, rightRadius = 2.1;
	private final double wheelDist = 15.05;
	private final int motorStraight = 200;
	
	private final UltrasonicPoller usPoller = new UltrasonicPoller();
	//private final Odometer odometer = new Odometer();
	
	//navigator specifications
	private final double THETA_THRESHOLD = 0.05;
	private double[] path = new double[0];
	private boolean navigating = false;
	
	//avoid obstacle specifications
	private double dangerZone = 15;//threshold before avoiding an obstacle
	private boolean avoid = false;
	private final int avoidAngle = -90;
	private final int navigatingAngle = 0;
	private int safeDist = 20;
	private final int motorHigh = 400;
	private final int motorLow = 100;
	private final double reorientationThresh = 0.025;
	
	public Navigation(Odometer odo) {
		this.odometer = odo;
		this.robot = odo.getTwoWheeledRobot();
	}
	
	public void run()
	{
		odometer.start();
		usPoller.start();
		//wait
		try{
			Thread.sleep(3000);
		}
		catch (InterruptedException ex)
		{
			//no error is expected here
		}
		//path order
		for(int i=0; i<path.length; i+=2)
		{
			travelTo(path[i],path[i+1]);
		}
	}
	
	public void travelTo(double x, double y)
	{
		navigating = true;
		
		//rotate to desired angle, and move to way-point
		leftMotor.setSpeed(motorStraight);
		rightMotor.setSpeed(motorStraight);
		double differenceX = x-odometer.getX();
		double differenceY = y-odometer.getY();
		turnTo(Math.atan2(differenceY,differenceX));
		leftMotor.forward();
		rightMotor.forward();

		//main loop that checks for obstacle and destination arrival
		while(true)
		{
			//correct angle if deviated
			double currentTheta = odometer.getTheta();
			double theta = Math.atan2(differenceY,differenceX)*180/Math.PI;
			double AbsDTheta = Math.abs(theta - currentTheta);
			differenceX = x-odometer.getX();
			differenceY = y-odometer.getY();
			//correct if orientation is off from target way-point
			if(AbsDTheta<THETA_THRESHOLD)
			{
				turnTo(Math.atan2(differenceX,differenceY));
				leftMotor.forward();
				rightMotor.forward();
			}
			//conditions for the presence of an obstacle
			if (UltrasonicPoller.data < dangerZone && avoid==false)
			{
				//avoid mode enabled
				avoid = true;
				turnTo(wrapAngle(currentTheta+Math.PI/2));
				rotatingSensor.rotateTo(avoidAngle);
				//main loop for avoiding an obstacle
				while(avoid)
				{
					//wallfollower -->bangbangcontroller
					leftMotor.forward();
					rightMotor.forward();
					if(UltrasonicPoller.data>safeDist)
					{
						leftMotor.setSpeed(motorHigh);
						rightMotor.setSpeed(motorLow);
					}
					else if(UltrasonicPoller.data<dangerZone-3)
					{
						leftMotor.setSpeed(motorLow);
						rightMotor.setSpeed(motorHigh);
					}
					else
					{
						leftMotor.setSpeed(motorStraight);
						rightMotor.setSpeed(motorStraight);

					}
					differenceX = x-odometer.getX();
					differenceY = y-odometer.getY();
					//relocate if orientation matches way-point
					if((wrapAngle(Math.abs(odometer.getTheta())-wrapAngle(Math.atan2(differenceY,differenceX))))<reorientationThresh)
							{
								leftMotor.setSpeed(motorStraight);
								rightMotor.setSpeed(motorStraight);
								avoid = false;
								navigating = false;
							}
				}
				//relocate itself and turn to target way-point once again
				rotatingSensor.rotateTo(navigatingAngle);
				relocateTo(x,y);
			}
			//check if arrived at destination
			differenceX = x-odometer.getX();
			differenceY = y-odometer.getY();
			if(differenceX*differenceX +differenceY*differenceY<NAVIGATOR_TOLERANCE_DIST)
			{
				leftMotor.stop(false);
				rightMotor.stop(false);
				Sound.beep();
				navigating=false;
				return;
			}
		}
		
	}
	//this method adjusts the robot's orientation
	//theta is in radians
	public void turnTo(double theta){
		double currentTheta = odometer.getTheta();
		double dTheta = theta - currentTheta;
		//conditions for rotating with minimal angle
		if(dTheta<Math.PI*-1)
		{
			dTheta+=2*Math.PI;
		}
		if(dTheta>Math.PI*1)
		{
			dTheta-=2*Math.PI;
		}
		//turn clockwise
		if(dTheta<0){
		leftMotor.rotate(convertAngle(leftRadius, wheelDist, Math.abs(dTheta)*180/Math.PI), true);
		rightMotor.rotate(-convertAngle(rightRadius, wheelDist, Math.abs(dTheta)*180/Math.PI), false);
		}
		//turn counter-clockwise
		if(dTheta>0)
		{
			leftMotor.rotate(-convertAngle(leftRadius, wheelDist, Math.abs(dTheta)*180/Math.PI), true);
			rightMotor.rotate(convertAngle(rightRadius, wheelDist, Math.abs(dTheta)*180/Math.PI), false);
		}
	}
	//turns to way-point and drive forward
	public void relocateTo(double x, double y)
	{
		double differenceX = x-odometer.getX();
		double differenceY = y-odometer.getY();
		turnTo(Math.atan2(differenceY,differenceX));
		leftMotor.forward();
		rightMotor.forward();
	}
	
	//drive a desired distance in cm
	public void driveDist(double distance)
	{
		leftMotor.rotate(convertDistance(leftRadius, distance), true);
		rightMotor.rotate(convertDistance(rightRadius, distance), false);
	}
	
	public boolean isNavigating()
	{
		return this.navigating;
	}
	
	//creates a path with an arbitrary size that will be determined at run-time
	//to set a path, write in this form "setPath(x1,y1,x2,y2,x3,y3,...)";
	public void setPath(double... path)
	{
		this.path = path;
	}
	
	//turn to desired angle
	public static int convertAngle(double radius, double width, double angle)
	{
		return (int)((180.0*Math.PI*width*angle/360.0)/(Math.PI*radius));
	}
	
	//travel desired distance
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	//wrap the angle to simplify reading and avoid unecessary turns
	private double wrapAngle(double rads) {
		return ((rads % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI);
	}
}
