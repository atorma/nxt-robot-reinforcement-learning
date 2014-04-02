package org.atorma.robot.objecttracking;

public class CircleSector {

	private final double fromAngleDeg;
	private final double toAngleDeg;

	public CircleSector(double fromAngleDeg, double toAngleDeg) {
		this.fromAngleDeg = ObjectTrackingUtils.normalizeDegrees(fromAngleDeg);
		this.toAngleDeg = ObjectTrackingUtils.normalizeDegrees(toAngleDeg);
	}

	public double getFromAngleDeg() {
		return fromAngleDeg;
	}

	public double getToAngleDeg() {
		return toAngleDeg;
	}
	
	public double getMidAngleDeg() {
		if (fromAngleDeg <= toAngleDeg) {
			return (fromAngleDeg + toAngleDeg)/2;
		} else {
			return (fromAngleDeg - 360 + toAngleDeg)/2;
		}
	}
	
	public boolean contains(double angleDeg) {
		angleDeg = ObjectTrackingUtils.normalizeDegrees(angleDeg);
		if (fromAngleDeg <= toAngleDeg) {
			return fromAngleDeg <= angleDeg && angleDeg < toAngleDeg;
		} else {
			return fromAngleDeg <= angleDeg || angleDeg < toAngleDeg;
		}
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		long temp;
		temp = Double.doubleToLongBits(fromAngleDeg);
		result = prime * result + (int) (temp ^ (temp >>> 32));
		temp = Double.doubleToLongBits(toAngleDeg);
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
		CircleSector other = (CircleSector) obj;
		if (Double.doubleToLongBits(fromAngleDeg) != Double
				.doubleToLongBits(other.fromAngleDeg))
			return false;
		if (Double.doubleToLongBits(toAngleDeg) != Double
				.doubleToLongBits(other.toAngleDeg))
			return false;
		return true;
	}

	@Override
	public String toString() {
		return "CircleSector [fromAngleDeg=" + fromAngleDeg + ", toAngleDeg=" + toAngleDeg + "]";
	}
	
	
}
