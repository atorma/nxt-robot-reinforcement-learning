package org.atorma.robot.objecttracking;

import static java.lang.Math.*;

/**
 * Represents the location of an object with respect to the agent.
 * The agent is always facing degree zero in polar coordinates
 * and along the positive y-axis in Cartesian coordinates.
 */
public class TrackedObject {
		
	// Polar coordinate representation
	private double distance;
	private double angleRad;
	
	// Cartesian coordinate representation
	private double x;
	private double y;
	
	private TrackedObject() {
		
	}
	
	private TrackedObject(double distanceCm, double angleRad) {
		if (distanceCm < 0) {
			throw new IllegalArgumentException("Negative distance");
		}
		
		this.distance = distanceCm;
		this.angleRad = normalize(angleRad);
		
		
		this.x = distanceCm * cos(PI/2 - angleRad);
		this.y = distanceCm * sin(PI/2 - angleRad);
	}

	public double getDistance() {
		return distance;
	}
	
	public double getAngleRad() {
		return angleRad;
	}

	public double getAngleDeg() {
		return toDegrees(angleRad);
	}
	
	public double getX() {
		return x;
	}
	
	public double getY() {
		return y;
	}

	public TrackedObject afterObserverRotatesDeg(double observerRotationDeg) {
		return afterObserverRotatesRad(toRadians(observerRotationDeg));
	}
	
	public TrackedObject afterObserverRotatesRad(double observerRotationRad) {
		double newAngleRad = this.angleRad - observerRotationRad;
		TrackedObject o = new TrackedObject(this.distance, newAngleRad);
		return o;
	}

	public TrackedObject afterObserverMoves(double observerMove) {
		double angleBefore;
		boolean objectRight;
		if (this.angleRad <= PI) {
			angleBefore = this.angleRad;
			objectRight = true;
		} else {
			angleBefore = 2*PI - this.angleRad;
			objectRight = false;
		}
		
		double distanceAfter = sqrt(pow(observerMove, 2) + pow(distance, 2) -2*observerMove*distance*cos(angleBefore));
		
		double numerator = pow(distance, 2) + pow(distanceAfter, 2) - pow(observerMove, 2);
		double denominator = 2*distance*distanceAfter;
		double angleDelta = acos(numerator/denominator);
		double angleAfter = angleBefore + angleDelta; // between 0 and PI
		
		TrackedObject o;
		if (objectRight) {
			o = new TrackedObject(distanceAfter, angleAfter);
		} else {
			o = new TrackedObject(distanceAfter, 2*PI - angleAfter); 
		}
		return o;
		//return inCartesianCoordinates(this.x, this.y - observerMove);
	}


	@Override
	public String toString() {
		return "TrackedObject [distance=" + getDistance() + ", angleDeg=" + getAngleDeg() + "]";
	}

	/** Normalizes an angle in radians to [0, 2*PI). */
	private static double normalize(double angleRad) {
		angleRad =  angleRad % (2*PI);
		return angleRad < 0 ? (2*PI) + angleRad : angleRad;
	}
	
	public static TrackedObject inPolarDegreeCoordinates(double distance, double angleDeg) {
		return new TrackedObject(distance, toRadians(angleDeg));
	}
	
	public static TrackedObject inPolarRadianCoordinates(double distance, double angleRad) {
		return new TrackedObject(distance, angleRad);
	}

	public static TrackedObject inCartesianCoordinates(double x, double y) {
		TrackedObject o = new TrackedObject();
		o.x = x;
		o.y = y;
		o.distance = sqrt(pow(x, 2) + pow(y, 2));
		o.angleRad = normalize(atan2(x, y));
		return o;
	}

	
	
}
