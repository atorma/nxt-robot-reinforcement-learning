package org.atorma.robot.simplebumper;

import java.io.File;

import org.atorma.robot.logging.CsvLogWriter;

public class BumperLogWriter {

	private CsvLogWriter logWriter;
	private long currentTime = -1;

	public BumperLogWriter(String fileName) {
		logWriter = new CsvLogWriter(new File(fileName), "Time ms", "Accumulated reward", "Accumulated collisions", "Collision", "Action"); 
	}
	
	public void log(double accumulatedReward, int accumulatedCollisions, boolean isCollided, BumperAction action) {
		long time = currentTime < 0 ? 0 : System.currentTimeMillis() - currentTime;
		logWriter.addRow(time, accumulatedReward, accumulatedCollisions, isCollided ? "TRUE" : "FALSE" , action);
		currentTime = System.currentTimeMillis();
	}
}
