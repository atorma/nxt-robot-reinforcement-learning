package org.atorma.robot.objecttracking;

import static java.lang.Math.signum;

import java.util.Comparator;

public class TrackedObjectAngleComparator implements Comparator<TrackedObject> {

	@Override
	public int compare(TrackedObject o1, TrackedObject o2) {
		int angleComparison = (int) signum(o1.getAngleDeg() - o2.getAngleDeg());
		if (angleComparison != 0) {
			return angleComparison;
		} else {
			return (int) signum(o1.getDistance() - o2.getDistance());
		}
	}

}
