package org.atorma.robot.objecttrackingbumper.prioritizedsweeping;

import org.atorma.robot.objecttracking.ObjectTrackingModel;

public class ModeledBumperState extends ObjectTrackingModel {

	private boolean isCollided;
	
	@Override
	public double[] getValues() {
		double[] distances = super.getValues();
		double[] distancesAndCollision = new double[distances.length + 1];
		for (int i = 0; i < distances.length; i++) {
			distancesAndCollision[i] = distances[i];
		}
		distancesAndCollision[distancesAndCollision.length - 1] = isCollided ? 1 : 0;
		return distancesAndCollision;
	}

	public boolean isCollided() {
		return isCollided;
	}

	public void setCollided(boolean isCollided) {
		this.isCollided = isCollided;
	}

	
	
}
