package org.atorma.robot.objecttrackingbumper.prioritizedsweeping;

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
	
	public void addObservation(BumperPercept percept) {
		addObservation(TrackedObject.inPolarDegreeCoordinates(percept.getDistanceToObstacleInFrontCm(), 0));
		setCollided(percept.isCollided());
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
}
