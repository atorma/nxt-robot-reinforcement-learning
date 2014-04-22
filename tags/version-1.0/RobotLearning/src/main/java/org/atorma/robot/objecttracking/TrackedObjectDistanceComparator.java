package org.atorma.robot.objecttracking;

import static java.lang.Math.signum;

import java.util.Comparator;

public class TrackedObjectDistanceComparator implements Comparator<TrackedObject> {

	@Override
	public int compare(TrackedObject o1, TrackedObject o2) {
		int distanceComparison = (int) signum(o1.getDistance() - o2.getDistance());
		if (distanceComparison != 0) {
			return distanceComparison;
		} else {
			return (int) signum(o1.getAngleDeg() - o2.getAngleDeg());
		}
	}

}
