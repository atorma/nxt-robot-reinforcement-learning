package org.atorma.robot.simplebumper;

import javax.vecmath.Vector3d;

import org.atorma.robot.RobotTestEnvironment;

import simbad.gui.Simbad;

public class BumperSimulation {

	public static void main(String[] args) {
		SimbadBumper robot = new SimbadBumper(new Vector3d(0, 0, 0), "Toveri");

		// request antialising
        System.setProperty("j3d.implicitAntialiasing", "true");
        // create Simbad instance with given environment
        Simbad frame = new Simbad(new RobotTestEnvironment(robot), false);
        
        QLearningBumper qLearning = new QLearningBumper(robot.getComms());
		Thread qLearningThread = new Thread(qLearning);
		qLearningThread.start();
	}
}
