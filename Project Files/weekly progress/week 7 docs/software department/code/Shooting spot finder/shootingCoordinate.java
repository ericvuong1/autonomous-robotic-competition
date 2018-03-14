import java.util.*;

//===================================================================================
//coordinate finder
//find all possible shooting spots for the robot
//===================================================================================

public class shootingCoordinate {

	public static void main(String[] args) {
		// target specifications
		double actualTargetX = 13;
		double actualTargetY = 7;
		// after conversion
		double targetX = -(actualTargetX - 10) * 30;
		double targetY = -(actualTargetY - 10) * 30;

		// launcher specifications
		int shootingDistance1 = 138;
		int shootingDistance2 = 139;

		// shooting area specifications
		int shootingLimitX1 = -15;
		int shootingLimitY1 = -15;
		int shootingLimitX2 = 60;
		int shootingLimitY2 = 60;
		
		// finding all possible coordinates
		ArrayList<String> x = new ArrayList<String>();
		ArrayList<String> y = new ArrayList<String>();
		// scanning
		for (double i = shootingLimitX1; i < shootingLimitX2; i = i + 0.1) {
			for (double j = shootingLimitY1; j < shootingLimitY2; j = j + 0.1) {
				double distance = Math.sqrt((targetY - j) * (targetY - j)
						+ (targetX - i) * (targetX - i));
				// if found one, add to array list
				if (distance > shootingDistance1
						&& distance < shootingDistance2) {
					x.add(String.valueOf(i));
					y.add(String.valueOf(j));
				}
			}
		}
		
		// if there are possible coordinates
		if (x.size() > 0) {
			String xCoordinate = x.get((x.size() - 1) / 2);
			String yCoordinate = y.get((y.size() - 1) / 2);
			double CoordinateX = Double.parseDouble(xCoordinate);
			double CoordinateY = Double.parseDouble(yCoordinate);
			double angle = Math.toDegrees(Math.atan2(targetY - CoordinateY,
					targetX - CoordinateX));
			System.out.println(xCoordinate);
			System.out.println(yCoordinate);
			System.out.println(String.valueOf(angle));
		}
		// if there is no possible coordinate
		else{
			System.out.println("No possible coordinate");
		}
	}
}
