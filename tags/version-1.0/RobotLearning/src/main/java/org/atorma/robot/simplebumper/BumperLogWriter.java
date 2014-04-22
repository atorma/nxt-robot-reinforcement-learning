package org.atorma.robot.simplebumper;

import java.io.File;

import org.atorma.robot.logging.CsvLogWriter;

public class BumperLogWriter {

	private CsvLogWriter logWriter;
	private long previousLogTime = -1;
	private double timeMin = 0;

	public BumperLogWriter(String fileName) {
		logWriter = new CsvLogWriter(new File(fileName), "Time min", "Accumulated reward", "Accumulated collisions", "Collision", "Action"); 
	}
	
	public void log(double accumulatedReward, int accumulatedCollisions, boolean isCollided, BumperAction action) {
		timeMin += previousLogTime < 0 ? 0 : (System.currentTimeMillis() - previousLogTime)/(1000.0*60) ;
		logWriter.addRow(timeMin, accumulatedReward, accumulatedCollisions, isCollided ? "TRUE" : "FALSE" , action);
		previousLogTime = System.currentTimeMillis();
	}
	
}
