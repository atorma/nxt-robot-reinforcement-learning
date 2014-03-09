package org.atorma.robot.simplebumper;

import org.atorma.robot.discretization.EqualWidthDiscretizer;

public class ObstacleDistanceDiscretizer extends EqualWidthDiscretizer {

	public ObstacleDistanceDiscretizer() {
		super(10, 50, 4);
	}

}
