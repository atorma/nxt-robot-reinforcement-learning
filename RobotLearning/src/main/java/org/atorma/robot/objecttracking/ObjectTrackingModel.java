package org.atorma.robot.objecttracking;

import java.util.*;

import org.atorma.robot.mdp.State;

/**
 * A model of objects' locations relative to an agent. Can also be
 * seen as an agent state.
 * <p>
 * Divides the circle around the agent to a number of sectors,
 * each of which holds max one object distance estimate or observation.
 * The effect of agent's moves can be simulated to get new, estimated
 * models.
 * 
 * @see TrackedObject
 * @see #afterAgentMoves(double)
 * @see #afterAgentRotatesDeg(double)
 */
public class ObjectTrackingModel implements State {
	
	public static final int DEFAULT_NUMBER_OF_SECTORS = 36;
	
	private Map<Integer, TrackedObject> objectsBySector;
	private int numberOfSectors;
	private CircleSectorDiscretizer circleSectorDiscretizer;
	
	public ObjectTrackingModel() {
		this(DEFAULT_NUMBER_OF_SECTORS);
	}

	public ObjectTrackingModel(int numberOfSectors) {
		if (numberOfSectors <= 0) {
			throw new IllegalArgumentException();
		}
		this.numberOfSectors = numberOfSectors;
		objectsBySector = new HashMap<>(numberOfSectors);
		
		this.circleSectorDiscretizer = new CircleSectorDiscretizer(numberOfSectors);		
	}
	
	/**
	 * Adds an observation into this model. The observation replaces the earlier estimate (if any)
	 * of an object in the same circle sector.
	 * <p>
	 * Note that adding an observation changes the state of the model.
	 */
	public void addObservation(TrackedObject obj) {
		int sectorIndex = circleSectorDiscretizer.discretize(obj.getAngleDeg());
		objectsBySector.put(sectorIndex, obj);
	}

	public Collection<TrackedObject> getObjects() {
		return Collections.unmodifiableCollection(objectsBySector.values());
	}
	
	/** 
	 * Initializes a new model instance for operations that return new, 
	 * updated model instances. Subclasses should override this method.
	 */
	protected ObjectTrackingModel initializeNew(int numberOfSectors) {
		return new ObjectTrackingModel(numberOfSectors);
	}


	/**
	 * Returns an estimated model after the agent has moved along a straight line.
	 * Within a circle sector, the estimate closest to the agent remains.
	 * 
	 * @param agentMove 
	 * 	agent's move distance, positive means forward, negative means backward
	 * @return
	 * 	estimated model of objects' locations relative to the agent
	 */
	public ObjectTrackingModel afterAgentMoves(double agentMove) {
		ObjectTrackingModel updatedModel = initializeNew(this.numberOfSectors);
		for (TrackedObject o : this.objectsBySector.values()) {
			updatedModel.addEstimate(o.afterObserverMoves(agentMove));
		}
		return updatedModel;
	}

	/**
	 * Returns an estimated model after the agent has rotated without moving.
	 * 
	 * @param agentTurnDeg 
	 * 	agent's rotation in degrees, positive means clockwise, negative means anti-clockwise
	 * @return
	 * 	estimated model of objects' locations relative to the agent
	 */
	public ObjectTrackingModel afterAgentRotatesDeg(double agentTurnDeg) {
		ObjectTrackingModel updatedModel = initializeNew(this.numberOfSectors);
		for (TrackedObject o : this.objectsBySector.values()) {
			updatedModel.addEstimate(o.afterObserverRotatesDeg(agentTurnDeg));
		}
		return updatedModel;
	}
	
	protected void addEstimate(TrackedObject obj) {
		int sectorIndex = circleSectorDiscretizer.discretize(obj.getAngleDeg());
		TrackedObject existing = objectsBySector.get(sectorIndex);
		if (existing == null || existing.getDistance() > obj.getDistance()) {
			objectsBySector.put(sectorIndex, obj);
		}
	}

	public TrackedObject getObjectInDirectionDegrees(double sectorDegree) {
		int sectorIndex = circleSectorDiscretizer.discretize(sectorDegree);
		return objectsBySector.get(sectorIndex);
	}
	
	public TrackedObject getNearestInSectorDegrees(CircleSector sector) {
		return getNearestInSectorDegrees(sector.getFromAngleDeg(), sector.getToAngleDeg());
	}
	
	public TrackedObject getNearestInSectorDegrees(double leftDegrees, double rightDegrees) {
		int fromSectorId = circleSectorDiscretizer.discretize(leftDegrees);
		int toSectorId = circleSectorDiscretizer.discretize(rightDegrees);
		
		TrackedObject nearest = null;
		if (fromSectorId > toSectorId) { // sector encompasses the 0 degree direction 
			
			for (int sectorId = fromSectorId; sectorId <= this.numberOfSectors; sectorId++) {
				TrackedObject obj = objectsBySector.get(sectorId);
				if ( obj != null && (nearest == null || nearest.getDistance() > obj.getDistance()) ) {
					nearest = obj;
				}
			}
			for (int sectorId = 0; sectorId <= toSectorId; sectorId++) {
				TrackedObject obj = objectsBySector.get(sectorId);
				if ( obj != null && (nearest == null || nearest.getDistance() > obj.getDistance()) ) {
					nearest = obj;
				}
			}
			
		} else {
			
			for (int sectorId = fromSectorId; sectorId <= toSectorId; sectorId++) {
				TrackedObject obj = objectsBySector.get(sectorId);
				if ( obj != null && (nearest == null || nearest.getDistance() > obj.getDistance()) ) {
					nearest = obj;
				}
			}
			
		}
		
		
		return nearest;
	}


	public Set<Map.Entry<Integer, TrackedObject>> getObjectsBySectors() {
		return Collections.unmodifiableSet(objectsBySector.entrySet());
	}


	public ObjectTrackingModel copy() {
		return copyAndChangeNumberOfSectors(this.numberOfSectors);
	}

	public ObjectTrackingModel copyAndChangeNumberOfSectors(int numberOfSectors) {
		ObjectTrackingModel copy = initializeNew(numberOfSectors);
		for (TrackedObject o : this.getObjects()) {
			copy.addEstimate(o);
		}
		return copy;
	}
	
	/**
	 * State vector representation of tracked object distances per sector. In the vector,
	 * element 0 is the front sector, element 1 is the next sector clockwise, etc.
	 * If there's no object in a sector, it's distance estimate is <tt>Double.MAX_VALUE</tt>.
	 */
	@Override
	public double[] getValues() {
		double[] distances = new double[this.numberOfSectors];
		for (int i = 0; i < this.numberOfSectors; i++) {
			TrackedObject obj = this.objectsBySector.get(i);
			distances[i] = obj != null ? obj.getDistance() : Double.MAX_VALUE;
		}
		return distances;
	}

	@Override
	public String toString() {
		ArrayList<TrackedObject> objects = new ArrayList<>(getObjects());
		Collections.sort(objects, new TrackedObjectAngleComparator());
		return "ObjectTrackingModel [objects=" + objects + "]";
	}

	

	
}
