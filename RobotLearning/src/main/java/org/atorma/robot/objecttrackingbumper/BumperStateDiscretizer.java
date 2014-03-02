package org.atorma.robot.objecttrackingbumper;

import org.atorma.robot.discretization.CustomBinsDiscretizer;
import org.atorma.robot.discretization.VectorDiscretizerImpl;
import org.atorma.robot.discretization.Discretizer;
import org.atorma.robot.discretization.EqualWidthDiscretizer;
import org.atorma.robot.discretization.VectorDiscretizer;

public class BumperStateDiscretizer implements VectorDiscretizer {
		
	private VectorDiscretizerImpl idFunction;

	public BumperStateDiscretizer() {
		Discretizer[] discretizers = new Discretizer[ModeledBumperState.NUMBER_OF_SECTORS_FOR_OBJECT_TRACKING];
		for (int i = 0; i < ModeledBumperState.NUMBER_OF_SECTORS_FOR_OBJECT_TRACKING; i++) {
			discretizers[i] = createObjectDistanceDiscretizer();
		}
		
		idFunction = new VectorDiscretizerImpl(discretizers);
	}

	private Discretizer createObjectDistanceDiscretizer() {
		double[] bins = new double[] {10, 40};  
		Discretizer discretizer = new CustomBinsDiscretizer(bins);
		return discretizer;
	}
			
	private Discretizer getLightValueDiscretizer() {
		return new EqualWidthDiscretizer(0, 800, 5); // TODO sensible discretization
	}
	
	@Override
	public int getId(double[] values) {
		return idFunction.getId(values);
	}

}
