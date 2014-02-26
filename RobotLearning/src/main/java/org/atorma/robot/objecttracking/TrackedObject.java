package org.atorma.robot.objecttracking;

import static java.lang.Math.*;

/**
 * Represents the location of an object with respect to the agent.
 * The agent is always facing degree zero in polar coordinates 
 * and along positive y-axis in cartesian coordinates.
 */
public class TrackedObject {
	
	// Polar coordinate representation
	private double distanceCm;
	private double angleDeg;
	
	// Cartesian coordinate representation
	private double x;
	private double y;
	
	
	private TrackedObject() {
		
	}

	public TrackedObject(double distanceCm, double angleDeg) {
		if (distanceCm < 0) {
			throw new IllegalArgumentException("Negative distance");
		}
				
		this.distanceCm = distanceCm;
		this.angleDeg = normalize(angleDeg);
		
		this.x = distanceCm * cos(toRadians(90 - this.angleDeg));
		this.y = distanceCm * sin(toRadians(90 - this.angleDeg));
	}
		
	private double normalize(double angleDeg) {
		angleDeg = angleDeg % 360;
		return angleDeg < 0 ? 360 + angleDeg : angleDeg;
	}

	public double getDistanceCm() {
		return distanceCm;
	}

	public double getAngleDeg() {
		return angleDeg;
	}
	
	public double getXCm() {
		return x;
	}

	public double getYCm() {
		return y;
	}

	public TrackedObject afterObserverRotates(double observerRotationDegrees) {
		observerRotationDegrees = normalize(observerRotationDegrees);
		double newAngleDeg = this.angleDeg - observerRotationDegrees;
		return new TrackedObject(this.distanceCm, newAngleDeg);
	}

	public TrackedObject afterObserverMoves(double observerMoveCm) {
		return inCartesianCoordinates(this.x, this.y - observerMoveCm);
		
		/*
		double distanceAfter = sqrt(pow(observerMoveCm, 2) + pow(distanceCm, 2) 
				- 2*observerMoveCm*distanceCm*cos(toRadians(angleDeg))
			);
		
		double angleAfter = angleDeg + toDegrees(Math.acos(
					(pow(distanceCm, 2) + pow(distanceAfter, 2) - pow(observerMoveCm, 2)) /
					(2 * distanceCm * distanceAfter)
				));
				
		return new TrackedObject(distanceAfter, angleAfter);
		*/
	}

	@Override
	public String toString() {
		return "TrackedObject [distanceCm=" + distanceCm + ", angleDeg="
				+ angleDeg + "]";
	}
	
	public static TrackedObject inPolarCoordinates(double distanceCm, double angleDeg) {
		return new TrackedObject(distanceCm, angleDeg);
	}
	
	public static TrackedObject inCartesianCoordinates(double x, double y) {
		TrackedObject trackedObject = new TrackedObject();
		trackedObject.x = x;
		trackedObject.y = y;
		trackedObject.distanceCm = sqrt(pow(x, 2) + pow(y, 2));
		trackedObject.angleDeg = 90 - toDegrees(atan(y/x));
		return trackedObject;
	}
	
	

}
