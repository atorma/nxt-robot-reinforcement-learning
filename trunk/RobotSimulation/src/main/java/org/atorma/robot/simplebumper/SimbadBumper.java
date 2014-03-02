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
		BumperAction bumperAction = BumperAction.getAction(actionId);
		if (bumperAction == BumperAction.FORWARD) {
			return new Drive(true);
		} else if (bumperAction == BumperAction.BACKWARD) {
			return new Drive(false);
		} else if (bumperAction == BumperAction.LEFT) {
			return new TurnLeft();
		} else if (bumperAction == BumperAction.RIGHT) {
			return new TurnRight();
		} else {
			throw new IllegalArgumentException();
		}
	}


	private class Drive implements SimbadAction {
		
		private static final double VELOCITY_CM_PER_SEC = 20.0;
		
		private double distanceTraveledCm = -1;
		private boolean isForward;
		
		public Drive(boolean isForward) {
			this.isForward = isForward;
		}
		
		@Override
		public void perform() {
			if (distanceTraveledCm < 0) {
				SimbadBumper.this.setRotationalVelocity(0);
				SimbadBumper.this.setTranslationalVelocity(isForward ? VELOCITY_CM_PER_SEC/100 : -VELOCITY_CM_PER_SEC/100);
				distanceTraveledCm = 0;
			} else {
				distanceTraveledCm += SimbadBumper.this.getOdometer()*100;
			}
		}

		@Override
		public boolean isCompleted() {
			return distanceTraveledCm >= BumperAction.DRIVE_DISTANCE_CM;
		}

	}
		
	private class TurnLeft implements SimbadAction {
		
		private boolean isCompleted = false;

		@Override
		public void perform() {
			SimbadBumper.this.setTranslationalVelocity(0);
			SimbadBumper.this.rotateY(0.262); // rad, 15 deg
			isCompleted = true;
		}

		@Override
		public boolean isCompleted() {
			return isCompleted;
		}

	}
	
	private class TurnRight implements SimbadAction {
		
		private boolean isCompleted = false;

		@Override
		public void perform() {
			SimbadBumper.this.setTranslationalVelocity(0);
			SimbadBumper.this.rotateY(-0.262); // rad, 15 deg
			isCompleted = true;
		}

		@Override
		public boolean isCompleted() {
			return isCompleted;
		}

	}

}
