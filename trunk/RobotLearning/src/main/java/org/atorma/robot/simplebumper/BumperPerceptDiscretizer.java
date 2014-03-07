package org.atorma.robot.simplebumper;

import org.atorma.robot.discretization.*;
import org.atorma.robot.mdp.State;

public class BumperPerceptDiscretizer implements StateDiscretizer {
		
	private VectorDiscretizer idFunction;

	public BumperPerceptDiscretizer() {
		idFunction = new VectorDiscretizerImpl(getUltrasonicDistanceDiscretizer(), new CollisionDiscretizer(), getLightValueDiscretizer());
	}

	private Discretizer getUltrasonicDistanceDiscretizer() {
		Discretizer discretizer = new EqualWidthDiscretizer(10, 50, 4);
		return discretizer;
	}
			
	private Discretizer getLightValueDiscretizer() {
		return new EqualWidthDiscretizer(0, 800, 5); // TODO sensible discretization
	}
	
	@Override
	public int getId(State state) {
		BumperPercept bumperState = (BumperPercept) state;
		return idFunction.getId(bumperState.getValues());
	}

}
