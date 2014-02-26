package org.atorma.robot.simplebumper;

import javax.vecmath.Vector3d;

import org.atorma.robot.SimbadAction;
import org.atorma.robot.SimbadRobot;
import org.atorma.robot.State;
import org.atorma.robot.communications.ActionIdProvider;

import simbad.sim.RangeSensorBelt;

public class SimbadBumper extends SimbadRobot {
	
	private RangeSensorBelt ultrasonicSensor;
		
	private SimbadAction[] actions = new SimbadAction[] {new DriveForward(), new DriveBackward(), new TurnLeft(), new TurnRight()};

	public SimbadBumper(ActionIdProvider actionIdProvider) {
		super(actionIdProvider, new Vector3d(0, 0, 0), "Toveri");
		
		this.height = 6f/100; // 6 cm height
		this.radius = 9f/100; // 9 cm radius

		ultrasonicSensor = new RangeSensorBelt(this.radius, 7f/100, 255.0f/100, 1, RangeSensorBelt.TYPE_SONAR, RangeSensorBelt.FLAG_SHOW_FULL_SENSOR_RAY);
		ultrasonicSensor.setUpdatePerSecond(60);
		this.addSensorDevice(ultrasonicSensor, new Vector3d(0, 0, 0), 0);
		//ultrasonicSensor = RobotFactory.addSonarBeltSensor(this);
	}

	@Override
	public State getCurrentState() {
		BumperState state = new BumperState((int) (ultrasonicSensor.getMeasurement(0)*100), this.collisionDetected());
		return state;
	}
	
	@Override
	public SimbadAction getAction(int actionId) {
		return actions[actionId];
	}


	private class DriveForward implements SimbadAction {
		
		@Override
		public void perform() {
			SimbadBumper.this.setRotationalVelocity(0);
			SimbadBumper.this.setTranslationalVelocity(0.5);
		}

		@Override
		public int getId() {
			return BumperAction.FORWARD.getId();  
		}

	}
	
	private class DriveBackward implements SimbadAction {

		@Override
		public void perform() {
			SimbadBumper.this.setRotationalVelocity(0);
			SimbadBumper.this.setTranslationalVelocity(-0.5);
		}

		@Override
		public int getId() {
			return BumperAction.BACKWARD.getId();  
		}

	}
	
	private class TurnLeft implements SimbadAction {

		@Override
		public void perform() {
			SimbadBumper.this.setTranslationalVelocity(0);
			SimbadBumper.this.rotateY(0.262); // rad, 15 deg
		}

		@Override
		public int getId() {
			return BumperAction.LEFT.getId();  
		}
		
	}
	
	private class TurnRight implements SimbadAction {

		@Override
		public void perform() {
			SimbadBumper.this.setTranslationalVelocity(0);
			SimbadBumper.this.rotateY(-0.263); // rad, 15 deg
		}

		@Override
		public int getId() {
			return BumperAction.RIGHT.getId();
		}

	}

}
