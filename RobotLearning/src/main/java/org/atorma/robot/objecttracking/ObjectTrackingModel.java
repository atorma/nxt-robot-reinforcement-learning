package org.atorma.robot.objecttracking;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.atorma.robot.discretization.Discretizer;

public class ObjectTrackingModel {
	
	private Map<Integer, TrackedObject> objectsBySector;
	private int numberOfSectors;
	private CircleSectorDiscretizer circleSectorDiscretizer;

	public ObjectTrackingModel(int numberOfSectors) {
		if (numberOfSectors <= 0) {
			throw new IllegalArgumentException();
		}
		this.numberOfSectors = numberOfSectors;
		objectsBySector = new HashMap<>(numberOfSectors);
		
		this.circleSectorDiscretizer = new CircleSectorDiscretizer(numberOfSectors);		
	}
	
	
	public void addObservation(TrackedObject obj) {
		objectsBySector.put(circleSectorDiscretizer.discretize(obj.getAngleDeg()), obj);
	}

	public List<TrackedObject> getObjectLocations() {
		return Collections.unmodifiableList(new ArrayList<TrackedObject>(objectsBySector.values()));
	}

	public void agentMoves(double agentMove) {
		Map<Integer, TrackedObject> oldObjects = objectsBySector;
		this.objectsBySector = new HashMap<>(numberOfSectors);
		for (TrackedObject o : oldObjects.values()) {
			addObservation(o.afterObserverMoves(agentMove));
		}
	}

	public void agentRotatesDeg(double agentTurnDeg) {
		Map<Integer, TrackedObject> oldObjects = objectsBySector;
		this.objectsBySector = new HashMap<>(numberOfSectors);
		for (TrackedObject o : oldObjects.values()) {
			addObservation(o.afterObserverRotatesDeg(agentTurnDeg));
		}
	}

	public TrackedObject getObjectInSectorDegree(double sectorDegree) {
		int sectorIndex = circleSectorDiscretizer.discretize(sectorDegree);
		return objectsBySector.get(sectorIndex);
	}


	public Set<Map.Entry<Integer, TrackedObject>> getObjectsBySectors() {
		return Collections.unmodifiableSet(objectsBySector.entrySet());
	}

	

}
