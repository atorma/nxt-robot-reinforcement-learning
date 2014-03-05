package org.atorma.robot.learning.cliffworld;

import static org.atorma.robot.learning.cliffworld.CliffWorldAction.*;

import java.util.*;

public class CliffWorldEnvironment {

	public static final List<CliffWorldAction> OPTIMAL_PATH;
	static {
		List<CliffWorldAction> path = new ArrayList<>();
		path.add(UP);
		for (int i = 0; i < 11; i++) {
			path.add(RIGHT);
		}
		path.add(DOWN);
		OPTIMAL_PATH = Collections.unmodifiableList(path);
	}

}
