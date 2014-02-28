package org.atorma.robot.objecttracking;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * Tracks objects by dividing the circle around the agent into a number of sectors
 * and maintaining an estimate of object distance for each sector. A newer observation
 * or estimate replaces an older one within a sector. Max one estimate is maintained 
 * per sector. This way old estimates in sectors that have not been observed for a long
 * time (e.g. behind the robot) persist even when new observations come in other sectors. 
 */
public class ObjectTrackingModel {
	
	private static final int DEFAULT_NUMBER_OF_SECTORS = 36;
	
	private Map<Integer, TrackedObject> objectsBySector;
	private int internalNumberOfSectors;
	private CircleSectorDiscretizer circleSectorDiscretizer;
	
	public ObjectTrackingModel() {
		this(DEFAULT_NUMBER_OF_SECTORS);
	}

	public ObjectTrackingModel(int internalNumberOfSectors) {
		if (internalNumberOfSectors <= 0) {
			throw new IllegalArgumentException();
		}
		this.internalNumberOfSectors = internalNumberOfSectors;
		objectsBySector = new HashMap<>(internalNumberOfSectors);
		
		this.circleSectorDiscretizer = new CircleSectorDiscretizer(internalNumberOfSectors);		
	}
	
	
	public void addObservation(TrackedObject obj) {
		int sectorIndex = circleSectorDiscretizer.discretize(obj.getAngleDeg());
		objectsBySector.put(sectorIndex, obj);
	}

	public Collection<TrackedObject> getObjects() {
		return Collections.unmodifiableCollection(objectsBySector.values());
	}

	public void agentMoves(double agentMove) {
		Map<Integer, TrackedObject> oldObjects = objectsBySector;
		this.objectsBySector = new HashMap<>(internalNumberOfSectors);
		for (TrackedObject o : oldObjects.values()) {
			addObservation(o.afterObserverMoves(agentMove));
		}
	}

	public void agentRotatesDeg(double agentTurnDeg) {
		Map<Integer, TrackedObject> oldObjects = objectsBySector;
		this.objectsBySector = new HashMap<>(internalNumberOfSectors);
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

	public ObjectTrackingModel copy() {
		ObjectTrackingModel copy = new ObjectTrackingModel(this.internalNumberOfSectors);
		for (TrackedObject o : this.getObjects()) {
			copy.addObservation(o);
		}
		return copy;
	}

	public ObjectTrackingModel copyAndChangeNumberOfSectors(int numberOfSectors) {
		ObjectTrackingModel copy = new ObjectTrackingModel(numberOfSectors);
		for (TrackedObject o : this.getObjects()) {
			int sectorInCopy = copy.circleSectorDiscretizer.discretize(o.getAngleDeg());
			TrackedObject existing = copy.objectsBySector.get(sectorInCopy);
			if (existing == null || o.getDistance() < existing.getDistance()) {
				copy.objectsBySector.put(sectorInCopy, o);
			}
		}
		return copy;
	}

	
	@Override
	public String toString() {
		ArrayList<TrackedObject> objects = new ArrayList<>(getObjects());
		Collections.sort(objects);
		return "ObjectTrackingModel [objects=" + objects + "]";
	}

	
}
