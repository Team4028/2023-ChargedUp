// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utilities.subsystem.BeakLimelight;

public class Limelight extends BeakLimelight {
	private static Limelight m_instance;
	/** Creates a new Limelight. */
	public Limelight() {
		super();
		
		super.setTargetHeight(32.5);
		super.setMountAngle(35.45);
		super.setMountHeight(21.);
	}
	
	public double getTargetDistance() {
		return getDistance() + (20.8 + 26.) / 12.;
	}

	public static Limelight getInstance() {
		if (m_instance == null) {
			m_instance = new Limelight();
		}
		return m_instance;
	}
	
	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		SmartDashboard.putNumber("Limelight Distance", getDistance());
		SmartDashboard.putNumber("Limelight Target X", getRoundedXOffset());
		SmartDashboard.putNumber("Limelight target Y", getY());
	}
}