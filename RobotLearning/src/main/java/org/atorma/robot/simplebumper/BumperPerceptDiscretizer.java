package org.atorma.robot.simplebumper;

import org.atorma.robot.discretization.*;
import org.atorma.robot.mdp.State;

public class BumperPerceptDiscretizer implements StateDiscretizer {
		
	private VectorDiscretizer idFunction;

	public BumperPerceptDiscretizer() {
		idFunction = new VectorDiscretizerImpl(new ObstacleDistanceDiscretizer(), new CollisionDiscretizer());
	}
	
	@Override
	public int getId(State state) {
		BumperPercept bumperState = (BumperPercept) state;
		return idFunction.getId(bumperState.getValues());
	}

	@Override
	public int getNumberOfStates() {
		return idFunction.getNumberOfValues();
	}

}
