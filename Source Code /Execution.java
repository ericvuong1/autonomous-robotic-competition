@@ -0,0 +1,167 @@
import lejos.nxt.*;
/*===================================================================================
 * Main class where it tells robot what to do using the rest of the classes 
 * robot can perform various operations by simply calling other classes methods 
 ====================================================================================*/

public class Execution {
  public static boolean right = false;
  public static void main(String[] args) {
//Target SPECS
    //TARGET 1 
    double T1X = 7.9;
    double T1Y = 8.3;
    double T1T = Math.toRadians(-134.88);
    //TARGET 2
    double T2X = 28.8;
    double T2Y = 19.3;
    double T2T = Math.toRadians(149.24);
    
    Odometer odometer = new Odometer();
    OdometryDisplay odometryDisplay = new OdometryDisplay(odometer);
    UltrasonicSensor frontUS = new UltrasonicSensor(SensorPort.S1);
    UltrasonicSensor leftUS = new UltrasonicSensor(SensorPort.S2);
    UltrasonicSensor rightUS = new UltrasonicSensor(SensorPort.S4);
    ColorSensor ls = new ColorSensor(SensorPort.S3);
    DifferentialFiltering df = new DifferentialFiltering(ls);
    Navigator navigator = new Navigator(odometer, frontUS, leftUS, rightUS);
    Robot robot = new Robot(odometer, frontUS, leftUS, rightUS);
    USLocalizer usl = new USLocalizer(odometer, frontUS, robot);
    LightLocalizer lsl = new LightLocalizer(odometer, df, robot);
    Shoot shoot = new Shoot();
    int buttonChoice;
    do {
      // clear the display
      LCD.clear();
      
      // ask the user whether the motors should drive in a square or float
      LCD.drawString("< Left | Right >", 0, 0);
      LCD.drawString("       |        ", 0, 1);
      LCD.drawString(" Run   |  Run   ", 0, 2);
      LCD.drawString(" <--   |  -->   ", 0, 3);
      
      buttonChoice = Button.waitForAnyPress();
    } while (buttonChoice != Button.ID_LEFT
               && buttonChoice != Button.ID_RIGHT);
    
    // options
    
    if (buttonChoice == Button.ID_LEFT) {
      // selects run 1
      odometer.start();
      odometryDisplay.start();
      robot.start();
      
      
//Competition
      //Localize
      usl.doLocalization();
      lsl.doLocalization();
      usl.setFilterValue(90);
      
//travel to shooting
      
      robot.travelTo(9.5*30.48, 9.5*30.48, true);
      robot.turnTo(-Math.PI/4);
      
      usl.doLocalization();
      lsl.doLocalization();
//shooting targets
      //target 1
      robot.travelTo(T1X, T1Y, false);
      robot.turnTo(T1T);
      
      shoot.launch();
      robot.sleep(1000);
      shoot.launch();
      robot.sleep(1000);
      shoot.launch();
      robot.sleep(1000);
      
      
      robot.travelTo(T2X, T2Y, false);
      robot.turnTo(T2T);
      //target 2
      shoot.launch();
      robot.sleep(1000);
      shoot.launch();
      robot.sleep(1000);
      shoot.launch();
      robot.sleep(1000);
      
      robot.travelTo(0, 0, false);
      
//travel to 0,0
      
      lsl.doLocalization();
      robot.travelTo(9.5*30.48, 9.5*30.48, true);
      robot.turnTo(-Math.PI/4);
      usl.doLocalization();
      lsl.doLocalization();
      
      robot.travelTo(0,0,false);
      robot.turnTo(Math.PI/2);  
    }
    
    else if (buttonChoice == Button.ID_RIGHT) {
      
      // selects run 2
      odometer.start();
      odometryDisplay.start();
      right = true;
      robot.start();
      
//Competition
//Localize
      usl.doLocalization();
      lsl.doLocalization();
      usl.setFilterValue(90);
      
//travel to shooting
      robot.travelTo(9.5*30.48, 9.5*30.48, true);
      robot.turnTo(-Math.PI/4);
      usl.doLocalization();
      lsl.doLocalization();
      
//shooting targets
//target 1
      robot.travelTo(T1X, T1Y, false);
      robot.turnTo(T1T);
      
      shoot.launch();
      robot.sleep(1000);
      shoot.launch();
      robot.sleep(1000);
      shoot.launch();
      robot.sleep(1000);
      
//target 2
      robot.travelTo(T2X, T2Y, false);
      robot.turnTo(T2T);
      
      shoot.launch();
      robot.sleep(1000);
      shoot.launch();
      robot.sleep(1000);
      shoot.launch();
      robot.sleep(1000);
      
      robot.travelTo(0, 0, false);
      
//travel to 0,0
      
      lsl.doLocalization();
      robot.travelTo(9.5*30.48, 9.5*30.48, true);
      robot.turnTo(-Math.PI/4);
      usl.doLocalization();
      lsl.doLocalization();
      
      robot.travelTo(0,0,false);
      robot.turnTo(Math.PI/2);
      
    }
    while (Button.waitForAnyPress() != Button.ID_ESCAPE)
      ;
    System.exit(0);
  }
}
