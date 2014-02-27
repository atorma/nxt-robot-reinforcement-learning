package org.atorma.robot.objecttracking;

import static java.lang.Math.*;

/**
 * Represents the location of an object with respect to the agent.
 * The agent is always facing degree zero in polar coordinates
 * and along the positive y-axis in Cartesian coordinates.
 */
public class TrackedObject implements Comparable<TrackedObject> {
		
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
		this.angleRad = ObjectTrackingUtils.normalizeRadians(angleRad);
		
		
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
		return inCartesianCoordinates(this.x, this.y - observerMove);
	}


	@Override
	public String toString() {
		return "TrackedObject [distance=" + getDistance() + ", angleDeg=" + getAngleDeg() + "]";
	}
	
	@Override
	public int compareTo(TrackedObject o) {
		int angleComparison = (int) signum(this.angleRad - o.angleRad);
		if (angleComparison != 0) {
			return angleComparison;
		} else {
			return (int) signum(this.distance - o.distance);
		}
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
		o.angleRad = ObjectTrackingUtils.normalizeRadians(atan2(x, y));
		return o;
	}

	

	
	
}