package org.atorma.robot.objecttracking;

import static java.lang.Math.*;

/**
 * An immutable value object that represents the location 
 * of an object with respect to an observer.
 * <p>
 * The observer is always facing degree zero in polar coordinates
 * and along the positive y-axis in Cartesian coordinates.
 */
public class TrackedObject {
		
	// Polar coordinate representation
	private final double distance;
	private final double angleRad;
	
	// Cartesian coordinate representation
	private final double x;
	private final double y;
	

	private TrackedObject(double distance, double angleRad) {
		this(
			distance, 
			angleRad, 
			distance * cos(PI/2 - angleRad),
			distance * sin(PI/2 - angleRad)
		);
	}
	
	private TrackedObject(double distanceCm, double angleRad, double x, double y) {
		if (distanceCm < 0) {
			throw new IllegalArgumentException("Negative distance");
		}
		
		this.distance = distanceCm;
		this.angleRad = ObjectTrackingUtils.normalizeRadians(angleRad);

		this.x = x;
		this.y = y;
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
		return inCartesianCoordinates(this.x, this.y - observerMove);
	}


	@Override
	public String toString() {
		return "[dist~=" + Math.round(getDistance()) + ", deg~=" + Math.round(getAngleDeg()) + "]";
	}
	
	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		long temp;
		temp = Double.doubleToLongBits(angleRad);
		result = prime * result + (int) (temp ^ (temp >>> 32));
		temp = Double.doubleToLongBits(distance);
		result = prime * result + (int) (temp ^ (temp >>> 32));
		return result;
	}

	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		TrackedObject other = (TrackedObject) obj;
		if (Double.doubleToLongBits(angleRad) != Double
				.doubleToLongBits(other.angleRad))
			return false;
		if (Double.doubleToLongBits(distance) != Double
				.doubleToLongBits(other.distance))
			return false;
		return true;
	}

	
	public static TrackedObject inPolarDegreeCoordinates(double distance, double angleDeg) {
		return new TrackedObject(distance, toRadians(angleDeg));
	}
	
	public static TrackedObject inPolarRadianCoordinates(double distance, double angleRad) {
		return new TrackedObject(distance, angleRad);
	}

	public static TrackedObject inCartesianCoordinates(double x, double y) {
		TrackedObject o = new TrackedObject(
			sqrt(pow(x, 2) + pow(y, 2)),
			atan2(x, y),
			x,
			y
		);
		return o;
	}

	

	
	
}
