package org.atorma.robot.simplebumper;

import java.io.File;

import org.atorma.robot.logging.CsvLogWriter;

public class BumperLogWriter {

	private CsvLogWriter logWriter;

	public BumperLogWriter(String fileName) {
		logWriter = new CsvLogWriter(new File(fileName), "Accumulated reward", "Accumulated collisions", "Collision", "Action"); 
	}
	
	public void log(double accumulatedReward, int accumulatedCollisions, boolean isCollided, BumperAction action) {
		logWriter.addRow(accumulatedReward, accumulatedCollisions, isCollided ? "TRUE" : "FALSE" , action);
	}
}
