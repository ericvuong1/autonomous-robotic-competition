import lejos.nxt.*;

public class Lab5 {

	public static void main(String[] args) {
		
		Button.waitForAnyPress();
		
		// initiate object
		Shooter robot = new Shooter();
		
		// call the shooting method
		robot.launch();
		
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}
}
