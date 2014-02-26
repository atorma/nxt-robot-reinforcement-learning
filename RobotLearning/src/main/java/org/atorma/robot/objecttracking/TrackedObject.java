package org.atorma.robot.objecttracking;

import static org.apfloat.ApfloatMath.*;

import org.apfloat.Apfloat;

/**
 * Represents the location of an object with respect to the agent.
 * The agent is always facing degree zero in polar coordinates.
 */
public class TrackedObject {
	
	private static final long APFLOAT_PRECISION = 100;
	
	private static final Apfloat RAD_360 = pi(APFLOAT_PRECISION).multiply(new Apfloat(2));
	private static final Apfloat RAD_90 = pi(APFLOAT_PRECISION).divide(new Apfloat(2));
	private static final Apfloat PI = pi(APFLOAT_PRECISION);
	
	// Polar coordinate representation
	private Apfloat distanceCm;
	private Apfloat angleRad;
	
	// Cartesian coordinate representation
	private Apfloat x;
	private Apfloat y;
	
	private TrackedObject() {
		
	}
	
	public TrackedObject(double distanceCm, double angleDeg) {
		this(asApfloat(distanceCm), toRadians(asApfloat(angleDeg)));
	}
	
	private TrackedObject(Apfloat distanceCm, Apfloat angleRad) {
		if (distanceCm.signum() < 0) {
			throw new IllegalArgumentException("Negative distance");
		}
		
		this.distanceCm = distanceCm;
		this.angleRad = angleRad;
		
		if (this.angleRad.signum() == 0) {
			this.x = asApfloat(0);
			this.y = distanceCm;
		} else if (this.angleRad.equals(RAD_90)) {
			this.x = distanceCm;
			this.y = asApfloat(0);
		} else if (this.angleRad.equals(asApfloat(180))) {
			this.x = asApfloat(0);
			this.y = distanceCm.multiply(asApfloat(-1));
		} else if (this.angleRad.equals(asApfloat(270))) {
			this.x = distanceCm.multiply(asApfloat(-1));
			this.y = asApfloat(0);
		} else {
			this.x = distanceCm.multiply(cos(RAD_90.subtract(this.angleRad)));
			this.y = distanceCm.multiply(sin(RAD_90.subtract(this.angleRad)));
		}
		
		
	}

	public double getDistanceCm() {
		return distanceCm.doubleValue();
	}

	public double getAngleDeg() {
		return toDegrees(normalize(angleRad)).doubleValue();
	}
	
	public double getXCm() {
		return x.doubleValue();
	}
	
	public double getYCm() {
		return y.doubleValue();
	}

	public TrackedObject afterObserverRotates(double observerRotationDegrees) {
		Apfloat rotationRad = toRadians(asApfloat(observerRotationDegrees));
		Apfloat newAngleRad = this.angleRad.subtract(rotationRad);
		return new TrackedObject(this.distanceCm, newAngleRad);
	}

	public TrackedObject afterObserverMoves(double observerMoveCm) {
	
		Apfloat moveCm = asApfloat(observerMoveCm);
		/*

		Apfloat distanceAfter = sqrt(sum(
				pow(moveCm, 2), 
				pow(distanceCm, 2), 
				new Apfloat(-2).multiply(moveCm).multiply(distanceCm).multiply(cos(angleRad))
			));
		
		Apfloat numerator = pow(distanceCm, 2).add(pow(distanceAfter, 2)).subtract(pow(moveCm, 2));
		Apfloat denominator = new Apfloat(2).multiply(distanceCm).multiply(distanceAfter);
		Apfloat angleDelta = acos(numerator.divide(denominator));
		Apfloat angleAfter = angleRad.add(angleDelta);
		
		return new TrackedObject(distanceAfter, angleAfter);
		*/
		
		return inCartesianCoordinates(this.x, this.y.subtract(moveCm));
	}

	@Override
	public String toString() {
		return "TrackedObject [distanceCm=" + distanceCm + ", angleDeg="
				+ angleRad + "]";
	}
	
	public static TrackedObject inPolarCoordinates(double distanceCm, double angleDeg) {
		return new TrackedObject(distanceCm, angleDeg);
	}
	
	public static TrackedObject inCartesianCoordinates(double x, double y) {
		return inCartesianCoordinates(asApfloat(x), asApfloat(y));
	}

	private static Apfloat asApfloat(double d) {
		return new Apfloat(d, APFLOAT_PRECISION);
	}
	
	private static Apfloat normalize(Apfloat angleRad) {
		angleRad =  angleRad.mod(RAD_360);
		return angleRad.signum() < 0 ? RAD_360.add(angleRad) : angleRad;
	}
	
	private static Apfloat toRadians(Apfloat angleDeg) {
		return angleDeg.multiply(PI).divide(asApfloat(180));
	}
	
	private static Apfloat toDegrees(Apfloat angleRad) {
		return angleRad.divide(PI).multiply(asApfloat(180));
	}

	private static TrackedObject inCartesianCoordinates(Apfloat x, Apfloat y) {
		TrackedObject o = new TrackedObject();
		o.x = x;
		o.y = y;
		o.distanceCm = sqrt(pow(x, 2).add(pow(y, 2)));
		o.angleRad = normalize(atan2(x, y));
		return o;
	}

	
	
}
