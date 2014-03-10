package org.atorma.robot.objecttrackingbumper;

import org.atorma.robot.discretization.*;
import org.atorma.robot.mdp.State;
import org.atorma.robot.simplebumper.CollisionDiscretizer;
import org.atorma.robot.simplebumper.ObstacleDistanceDiscretizer;

public class BumperStateDiscretizer implements StateDiscretizer {
	
	public static final int NUMBER_OF_SECTORS = 6;
	private VectorDiscretizerImpl idFunction;

	public BumperStateDiscretizer() {
		Discretizer[] discretizers = new Discretizer[NUMBER_OF_SECTORS + 1];
		for (int i = 0; i < NUMBER_OF_SECTORS; i++) {
			discretizers[i] = new ObstacleDistanceDiscretizer();
		}
		discretizers[NUMBER_OF_SECTORS] = new CollisionDiscretizer();
		
		idFunction = new VectorDiscretizerImpl(discretizers);
	}
	

	@Override
	public int getId(State state) {
		ModeledBumperState modeledState = (ModeledBumperState) state;
		ModeledBumperState reducedState = (ModeledBumperState) modeledState.copyAndChangeNumberOfSectors(NUMBER_OF_SECTORS);		
		return idFunction.getId(reducedState.getValues());
	}
	
	public int getNumberOfStates() {
		return idFunction.getNumberOfValues();
	}
	
}
