package org.atorma.robot.objecttrackingbumper;

import java.util.Arrays;
import java.util.Map.Entry;

import org.atorma.robot.State;
import org.atorma.robot.objecttracking.ObjectTrackingModel;
import org.atorma.robot.objecttracking.TrackedObject;
import org.atorma.robot.simplebumper.BumperPercept;

public class ModeledBumperState implements State {
	
	// Not the same as ObjectTrackingModel's number of sectors.
	// We can track multiple objects but to prevent combinatorial 
	// state explosion in Q-learning we must limit the number of learnable states.
	public static int NUMBER_OF_SECTORS_FOR_OBJECT_TRACKING = 6;

	private double[] values;
	
	public ModeledBumperState(ObjectTrackingModel currentModel, boolean isCollided) {
		ObjectTrackingModel reducedModel = currentModel.copyAndChangeNumberOfSectors(NUMBER_OF_SECTORS_FOR_OBJECT_TRACKING);
		values = new double[NUMBER_OF_SECTORS_FOR_OBJECT_TRACKING + 1];
		
		for (int i = 0; i < NUMBER_OF_SECTORS_FOR_OBJECT_TRACKING; i++) {
			values[i] = BumperPercept.MAX_ULTRASONIC_DIST*50;
		}
		for (Entry<Integer, TrackedObject> e : reducedModel.getObjectsBySectors()) {
			values[e.getKey()] = e.getValue().getDistance();
		}
		values[NUMBER_OF_SECTORS_FOR_OBJECT_TRACKING] = isCollided ? 1 : 0;
	}
	
	@Override
	public double[] getValues() {
		return values;
	}
	
	public boolean isCollided() {
		return values[NUMBER_OF_SECTORS_FOR_OBJECT_TRACKING] == 1;
	}

	@Override
	public String toString() {
		return "ModeledBumperState [values=" + Arrays.toString(values) + "]";
	}
	
	
}
