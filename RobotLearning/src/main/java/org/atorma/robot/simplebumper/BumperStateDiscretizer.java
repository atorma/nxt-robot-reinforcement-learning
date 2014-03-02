package org.atorma.robot.simplebumper;

import org.atorma.robot.discretization.*;

public class BumperStateDiscretizer implements VectorDiscretizer {
		
	private VectorDiscretizerImpl idFunction;

	public BumperStateDiscretizer() {
		idFunction = new VectorDiscretizerImpl(getUltrasonicDistanceDiscretizer(), getCollisionDiscretizer(), getLightValueDiscretizer());
	}

	private Discretizer getUltrasonicDistanceDiscretizer() {
		Discretizer discretizer = new EqualWidthDiscretizer(10, 50, 4);
		return discretizer;
	}
		
	private Discretizer getCollisionDiscretizer() {
		return new Discretizer() {
			
			@Override
			public int getNumberOfBins() {
				return 2;
			}
			
			@Override
			public int discretize(double value) {
				return value == 0 ? 0 : 1;
			}
		};
	}
	
	private Discretizer getLightValueDiscretizer() {
		return new EqualWidthDiscretizer(0, 800, 5); // TODO sensible discretization
	}
	
	@Override
	public int getId(double[] values) {
		return idFunction.getId(values);
	}

}
