package org.atorma.robot.objecttrackingbumper;

import org.atorma.robot.objecttracking.ObjectTrackingModel;
import org.atorma.robot.objecttracking.TrackedObject;
import org.atorma.robot.simplebumper.BumperAction;
import org.atorma.robot.simplebumper.BumperPercept;

public class ModeledBumperState extends ObjectTrackingModel {

	private boolean isCollided;
	
	
	public ModeledBumperState() {
		super();
	}

	public ModeledBumperState(int numberOfSectors) {
		super(numberOfSectors);
	}

	@Override
	public double[] getValues() {
		double[] distances = super.getValues();
		double[] distancesAndCollision = new double[distances.length + 1];
		for (int i = 0; i < distances.length; i++) {
			distancesAndCollision[i] = distances[i];
		}
		distancesAndCollision[distancesAndCollision.length - 1] = isCollided ? 1 : 0;
		return distancesAndCollision;
	}


	@Override
	protected ObjectTrackingModel initializeNew(int numberOfSectors) {
		return new ModeledBumperState(numberOfSectors);
	}
	
	public boolean isCollided() {
		return isCollided;
	}

	public void setCollided(boolean isCollided) {
		this.isCollided = isCollided;
	}
	
	public ModeledBumperState afterActionAndObservation(BumperAction previousAction, BumperPercept nextPercept) {
		ModeledBumperState updatedState;
		if (this.isCollided && nextPercept.isCollided()) {
			updatedState = (ModeledBumperState) this.copy();
		} else {
			updatedState = afterAction(previousAction);
		}
		updatedState.update(nextPercept);
		return updatedState;
	}
	
	private void update(BumperPercept percept) {
		this.setCollided(percept.isCollided());
		this.addObservation(TrackedObject.inPolarDegreeCoordinates(percept.getDistanceToObstacleInFrontCm(), 0));
	}
	
	public ModeledBumperState afterAction(BumperAction action) {
		switch(action) {
		case FORWARD: 
			return (ModeledBumperState) afterAgentMoves(BumperAction.DRIVE_DISTANCE_CM);
		case BACKWARD:
			return (ModeledBumperState) afterAgentMoves(-BumperAction.DRIVE_DISTANCE_CM);
		case LEFT:
			return (ModeledBumperState) afterAgentRotatesDeg(-BumperAction.TURN_DEGREES);
		case RIGHT:
			return (ModeledBumperState) afterAgentRotatesDeg(BumperAction.TURN_DEGREES);
		default:
			throw new IllegalArgumentException();
		}
	}

	@Override
	public ObjectTrackingModel copyAndChangeNumberOfSectors(int numberOfSectors) {
		ModeledBumperState state = (ModeledBumperState) super.copyAndChangeNumberOfSectors(numberOfSectors);
		state.setCollided(this.isCollided());
		return state;
	}
	
	
	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + (isCollided ? 1231 : 1237);
		result = prime * result + getObjects().hashCode();
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
		ModeledBumperState other = (ModeledBumperState) obj;
		if (isCollided != other.isCollided)
			return false;
		if (!getObjects().equals(other.getObjects()))
			return false;
		return true;
	}

	@Override
	public String toString() {
		return "ModeledBumperState [isCollided=" + isCollided + ", getObjects()=" + getObjects() + "]";
	}

	
	public static ModeledBumperState initialize(BumperPercept percept) {
		ModeledBumperState state = new ModeledBumperState();
		state.update(percept);
		return state;
	}
	
}
