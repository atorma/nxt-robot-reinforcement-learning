package org.atorma.robot.objecttracking;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class ObjectTrackingModel {
	
	private List<TrackedObject> objects = new ArrayList<>();

	public void addObservation(double distanceCm, double angleDegrees) {
		TrackedObject object = TrackedObject.inPolarDegreeCoordinates(distanceCm, angleDegrees);
		objects.add(object);
	}

	public List<TrackedObject> getObjectLocations() {
		return Collections.unmodifiableList(objects);
	}

}
