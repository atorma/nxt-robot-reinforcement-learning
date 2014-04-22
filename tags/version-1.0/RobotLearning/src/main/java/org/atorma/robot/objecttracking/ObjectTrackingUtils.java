package org.atorma.robot.objecttracking;

import static java.lang.Math.PI;

public class ObjectTrackingUtils {
	
	/** Normalizes an angle in radians to [0, 2*PI). */
	public static double normalizeRadians(double angleRad) {
		angleRad =  angleRad % (2*PI);
		return angleRad < 0 ? (2*PI) + angleRad : angleRad;
	}
	
	/** Normalizes an angle in degrees to [0, 360). */
	public static double normalizeDegrees(double angleDeg) {
		angleDeg =  angleDeg % 360;
		return angleDeg < 0 ? 360 + angleDeg : angleDeg;
	}

}
