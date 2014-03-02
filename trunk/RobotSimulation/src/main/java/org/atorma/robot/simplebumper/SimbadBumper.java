package org.atorma.robot.simplebumper;

import javax.vecmath.Vector3d;

import org.atorma.robot.*;

import simbad.sim.RangeSensorBelt;

public class SimbadBumper extends SimbadRobot {
	
	private RangeSensorBelt ultrasonicSensor;

	public SimbadBumper(DiscreteRobotController controller) {
		super(controller, new Vector3d(0, 0, 0), "Toveri");
		
		this.height = 6f/100; // 6 cm height
		this.radius = 9f/100; // 9 cm radius

		ultrasonicSensor = new RangeSensorBelt(this.radius, 7f/100, 255.0f/100, 1, RangeSensorBelt.TYPE_SONAR, RangeSensorBelt.FLAG_SHOW_FULL_SENSOR_RAY);
		ultrasonicSensor.setUpdatePerSecond(60);
		this.addSensorDevice(ultrasonicSensor, new Vector3d(0, 0, 0), 0);
	}

	@Override
	public State getCurrentState() {
		BumperPercept state = new BumperPercept((int) (ultrasonicSensor.getMeasurement(0)*100), this.collisionDetected());
		return state;
	}
	
	@Override
	public SimbadAction getAction(int actionId) {
		BumperAction action = BumperAction.getAction(actionId);
		if (action == BumperAction.FORWARD) {
			return new Drive(BumperAction.FORWARD);
		} else if (action == BumperAction.BACKWARD) {
			return new Drive(BumperAction.BACKWARD);
		} else if (action == BumperAction.LEFT) {
			return new Turn(BumperAction.LEFT);
		} else if (action == BumperAction.RIGHT) {
			return new Turn(BumperAction.RIGHT);
		} else {
			throw new IllegalArgumentException();
		}
	}
	
	private class Drive implements SimbadAction {
		
		private static final double VELOCITY_CM_PER_SEC = 20.0;
		
		private double distanceTraveledCm = -1;
		private BumperAction action;
		
		public Drive(BumperAction action) {
			this.action = action;
		}
		
		@Override
		public void perform() {
			if (distanceTraveledCm < 0) {
				SimbadBumper.this.setRotationalVelocity(0);
				SimbadBumper.this.setTranslationalVelocity(action == BumperAction.FORWARD ? VELOCITY_CM_PER_SEC/100 : -VELOCITY_CM_PER_SEC/100);
				distanceTraveledCm = 0;
			} else {
				distanceTraveledCm += SimbadBumper.this.getOdometer()*100;
			}
		}

		@Override
		public boolean isCompleted() {
			return distanceTraveledCm/BumperAction.DRIVE_DISTANCE_CM >= 0.99  || SimbadBumper.this.collisionDetected();
		}

	}
		
	private class Turn implements SimbadAction {
		
		private static final double ROTATIONAL_VELOCITY_DEG_PER_SEC = 45;
		
		private double lifetimeWhenStarted;
		private double angleTurnedDeg = -1;
		private BumperAction action;
		
		public Turn(BumperAction action) {
			this.action = action;
			lifetimeWhenStarted = SimbadBumper.this.getLifeTime();
		}

		@Override
		public void perform() {
			if (angleTurnedDeg < 0) {
				SimbadBumper.this.setTranslationalVelocity(0);
				SimbadBumper.this.setRotationalVelocity(action == BumperAction.LEFT ? Math.toRadians(ROTATIONAL_VELOCITY_DEG_PER_SEC) : -Math.toRadians(ROTATIONAL_VELOCITY_DEG_PER_SEC)); 
				angleTurnedDeg = 0;
			} else {
				angleTurnedDeg += (SimbadBumper.this.getLifeTime() - lifetimeWhenStarted) * ROTATIONAL_VELOCITY_DEG_PER_SEC;
			}
		}

		@Override
		public boolean isCompleted() {
			return angleTurnedDeg/BumperAction.TURN_DEGREES >= 0.99;
		}

	}
	
}
