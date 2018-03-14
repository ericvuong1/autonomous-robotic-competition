import lejos.nxt.*;

public class Execution {
	public static void main(String[] args)
	{
		Odometer odometer = new Odometer();
		OdometryDisplay odometryDisplay = new OdometryDisplay(odometer);
		UltrasonicSensor frontUS = new UltrasonicSensor(SensorPort.S1);
		UltrasonicSensor sideUS = new UltrasonicSensor(SensorPort.S2);
		UltrasonicSensor oSideUS = new UltrasonicSensor(SensorPort.S4);
		ColorSensor ls = new ColorSensor(SensorPort.S3);
		DifferentialFiltering df = new DifferentialFiltering(ls);
		Navigator navigator = new Navigator(odometer, frontUS, sideUS, oSideUS);
		Robot robot = new Robot(odometer, frontUS, sideUS, oSideUS);
		USLocalizer usl = new USLocalizer(odometer, frontUS, robot);
		LightLocalizer lsl = new LightLocalizer(odometer, df, robot);
		Shoot shoot = new Shoot();
		OdometryCorrection odoCorrect = new OdometryCorrection(odometer,df,ls);
		int buttonChoice;
		do {
			// clear the display
			LCD.clear();

			// ask the user whether the motors should drive in a square or float
			LCD.drawString("< Left | Right >", 0, 0);
			LCD.drawString("       |        ", 0, 1);
			LCD.drawString(" Run   |  Run   ", 0, 2);
			LCD.drawString(" One   |  Two   ", 0, 3);
			
			buttonChoice = Button.waitForAnyPress();
		}
		while(buttonChoice !=Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);
		
		// options
		
		if(buttonChoice == Button.ID_LEFT)
		{
				//selects run 1 
			odometer.start();
			odometryDisplay.start();
			robot.start();
			usl.doLocalization();
			lsl.doLocalization();
			robot.travelTo(0,0,false);
			//Button.waitForAnyPress();
			robot.turnTo(Math.PI/2);
//			robot.travelTo(-15.24,30.48, false);
//			robot.travelTo(-15.24, 167.64,false );
//			robot.travelTo(45.72, 167.64,false );
//			robot.travelTo(45.72, 200,false );
//			lsl.doLocalization();
//			robot.turnTo(3*Math.PI/2);
//			shoot.launch();
//			robot.sleep(100);
//			shoot.launch();
//			robot.travelTo(45.72, 167.64,false );
//			robot.travelTo(-15.24, 167.64,false );
//			robot.travelTo(-15.24,30.48, false);
//			robot.travelTo(0,0,false);
//			lsl.doLocalization();
//			robot.travelTo(0,0,false);
//			robot.turnTo(Math.PI/2);
			
//			robot.turnTo(0);
//			robot.sleep(1000);
//			robot.turnTo(Math.PI);
//			robot.sleep(1000);
//			robot.turnTo(Math.PI/4);
//			robot.sleep(1000);
//			robot.turnTo(3*Math.PI/2);
//			robot.sleep(1000);
//			robot.turnTo(0);
//			robot.sleep(1000);
//			robot.turnTo(Math.PI/4);
//			robot.sleep(1000);
//			robot.turnTo(Math.PI/2);
			}
			
		else if(buttonChoice == Button.ID_RIGHT)
				{
			
			//selects run 2 
			odometer.start();
			odometryDisplay.start();
			robot.start();
			Button.waitForAnyPress();
			
	
				}
		while(Button.waitForAnyPress() != Button.ID_ESCAPE)
			;
		System.exit(0);
			
		}
}


