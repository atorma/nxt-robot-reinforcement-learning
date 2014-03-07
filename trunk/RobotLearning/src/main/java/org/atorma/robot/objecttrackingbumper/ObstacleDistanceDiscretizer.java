package org.atorma.robot.objecttrackingbumper;

import org.atorma.robot.discretization.EqualWidthDiscretizer;

public class ObstacleDistanceDiscretizer extends EqualWidthDiscretizer {

	public ObstacleDistanceDiscretizer() {
		super(10, 50, 4);
	}

}
