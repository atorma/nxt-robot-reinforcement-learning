package org.atorma.robot.objecttrackingbumper;

import org.atorma.robot.discretization.*;
import org.atorma.robot.mdp.State;
import org.atorma.robot.simplebumper.CollisionDiscretizer;

public class BumperStateDiscretizer implements StateDiscretizer {
		
	private VectorDiscretizer idFunction;

	public BumperStateDiscretizer() {
		Discretizer[] discretizers = new Discretizer[ModeledBumperState.NUMBER_OF_SECTORS_FOR_OBJECT_TRACKING + 1];
		for (int i = 0; i < ModeledBumperState.NUMBER_OF_SECTORS_FOR_OBJECT_TRACKING; i++) {
			discretizers[i] = new ObstacleDistanceDiscretizer();
		}
		discretizers[ModeledBumperState.NUMBER_OF_SECTORS_FOR_OBJECT_TRACKING] = new CollisionDiscretizer();
		
		idFunction = new VectorDiscretizerImpl(discretizers);
	}


	@Override
	public int getId(State state) {
		ModeledBumperState modeledState = (ModeledBumperState) state;
		return idFunction.getId(modeledState.getValues());
	}
	
	

}
