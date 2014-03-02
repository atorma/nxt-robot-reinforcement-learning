package org.atorma.robot.simplebumper;

import org.atorma.robot.discretization.CustomBinsDiscretizer;
import org.atorma.robot.discretization.VectorDiscretizerImpl;
import org.atorma.robot.discretization.Discretizer;
import org.atorma.robot.discretization.EqualWidthDiscretizer;
import org.atorma.robot.discretization.VectorDiscretizer;

public class BumperStateIdFunction implements VectorDiscretizer {
		
	private VectorDiscretizerImpl idFunction;

	public BumperStateIdFunction() {
		idFunction = new VectorDiscretizerImpl(getUltrasonicDistanceDiscretizer(), getCollisionDiscretizer(), getLightValueDiscretizer());
	}

	private Discretizer getUltrasonicDistanceDiscretizer() {
		double[] bins = new double[] {7, 18, 35, 55};  
		Discretizer discretizer = new CustomBinsDiscretizer(bins);
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
