
/**
 * Phoenix Software License Agreement
 *
 * Copyright (C) Cross The Road Electronics.  All rights
 * reserved.
 * 
 * Cross The Road Electronics (CTRE) licenses to you the right to 
 * use, publish, and distribute copies of CRF (Cross The Road) firmware files (*.crf) and 
 * Phoenix Software API Libraries ONLY when in use with CTR Electronics hardware products
 * as well as the FRC roboRIO when in use in FRC Competition.
 * 
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * CROSS THE ROAD ELECTRONICS BE LIABLE FOR ANY INCIDENTAL, SPECIAL, 
 * INDIRECT OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE
 */

/**
 * Description:
 * The MotionMagic example demonstrates the motion magic control mode.
 * Tested with Logitech F710 USB Gamepad inserted into Driver Station.
 * 
 * Be sure to select the correct feedback sensor using configSelectedFeedbackSensor() below.
 *
 * After deploying/debugging this to your RIO, first use the left Y-stick 
 * to throttle the Talon manually. This will confirm your hardware setup/sensors
 * and will allow you to take initial measurements.
 * 
 * Be sure to confirm that when the Talon is driving forward (green) the 
 * position sensor is moving in a positive direction. If this is not the 
 * cause, flip the boolean input to the setSensorPhase() call below.
 *
 * Ensure your feedback device is in-phase with the motor,
 * and you have followed the walk-through in the Talon SRX Software Reference Manual.
 * 
 * Controls:
 * Button 1(Button A): When held, put Talon in Motion Magic mode and allow Talon to drive [-10, 10] 
 * 	rotations.
 * Button 2(Button B): When pushed, the selected feedback sensor gets zero'd
 * POV 180(Dpad Down): When pushed, will decrement the smoothing of the motion magic down to 0
 * POV 0(Dpad Up): When pushed, will increment the smoothing of the motion magic up to 8
 * Left Joystick Y-Axis:
 * 	+ Percent Output: Throttle Talon SRX forward and reverse, use to confirm hardware setup.
 * Right Joystick Y-Axis:
 * 	+ Motion Magic: Servo Talon SRX forward and reverse, [-10, 10] rotations.
 * 
 * Gains for Motion Magic may need to be adjusted in Constants.java
 * 
 * Supported Version:
 * - Talon SRX: 4.00
 * - Victor SPX: 4.00
 * - Pigeon IMU: 4.00
 * - CANifier: 4.00
 */
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.*;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

public class Robot extends TimedRobot {

	// setpoints in encoder ticks
	double masterTargetMin = 50;
	double masterTargetMax = 500;

	double followerTargetMin = 50;
	double followerTargetMax = 500;

	/* Hardware */

	WPI_TalonSRX followTalon = new WPI_TalonSRX(12);
	WPI_TalonSRX masterTalon = new WPI_TalonSRX(11);

	XboxController logitech = new XboxController(3);

	/* create some followers */
	// BaseMotorController _follower1 = new WPI_TalonSRX(0);
	// BaseMotorController _follower2 = new WPI_VictorSPX(0);
	// BaseMotorController _follower3 = new WPI_VictorSPX(1);

	/* Used to build string throughout loop */
	StringBuilder _sb = new StringBuilder();

	/** How much smoothing [0,8] to use during MotionMagic */
	int _smoothing = 0;

	/** save the last Point Of View / D-pad value */
	int _pov = -1;

	public void robotInit() {
		/* Zero the sensor once on robot boot up */
		followTalon.setSelectedSensorPosition(0, Constants.kPIDLoopIdx,
				Constants.kTimeoutMs);
		/* Zero the sensor once on robot boot up */
		masterTalon.setSelectedSensorPosition(0, Constants.kPIDLoopIdx,
				Constants.kTimeoutMs);

		/* Factory default hardware to prevent unexpected behavior */
		masterTalon.configFactoryDefault();
		followTalon.configFactoryDefault();

		// followTalon.follow(masterTalon, FollowerType.AuxOutput1);
		// _talon.setNeutralMode(NeutralMode.Brake);
		// _talon.setSelectedSensorPosition(0);

		/* Configure Sensor Source for Pirmary PID */
		masterTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.kPIDLoopIdx,
				Constants.kTimeoutMs);

		masterTalon.configNeutralDeadband(0.001, Constants.kTimeoutMs);

		// talon 12 settings
		followTalon.setSensorPhase(false);
		followTalon.setInverted(false);

		// talon 11 settings
		masterTalon.setSensorPhase(true);
		masterTalon.setInverted(false);

		followTalon.setNeutralMode(NeutralMode.Brake);
		masterTalon.setNeutralMode(NeutralMode.Brake);

		/* Set relevant frame periods to be at least as fast as periodic rate */
		masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
		masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);

		/* Set the peak and nominal outputs */
		masterTalon.configNominalOutputForward(0, Constants.kTimeoutMs);
		masterTalon.configNominalOutputReverse(0, Constants.kTimeoutMs);
		masterTalon.configPeakOutputForward(1, Constants.kTimeoutMs);
		masterTalon.configPeakOutputReverse(-1, Constants.kTimeoutMs);

		/* Set Motion Magic gains in slot0 - see documentation */
		masterTalon.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
		masterTalon.config_kF(Constants.kSlotIdx, Constants.masterGains.kF, Constants.kTimeoutMs);
		masterTalon.config_kP(Constants.kSlotIdx, Constants.masterGains.kP, Constants.kTimeoutMs);
		masterTalon.config_kI(Constants.kSlotIdx, Constants.masterGains.kI, Constants.kTimeoutMs);
		masterTalon.config_kD(Constants.kSlotIdx, Constants.masterGains.kD, Constants.kTimeoutMs);

		/* Set acceleration and vcruise velocity - see documentation */
		masterTalon.configMotionCruiseVelocity(148, Constants.kTimeoutMs);
		masterTalon.configMotionAcceleration(147.6, Constants.kTimeoutMs);

		masterTalon.configFeedbackNotContinuous(true, Constants.kTimeoutMs);

		/* Zero the sensor once on robot boot up */
		masterTalon.setSelectedSensorPosition(0, Constants.kPIDLoopIdx,
				Constants.kTimeoutMs);

		// Configure current limits
		masterTalon.configPeakCurrentLimit(30);
		masterTalon.configPeakCurrentDuration(150);

		// takes in AMPS
		masterTalon.configContinuousCurrentLimit(20);

		// integral zone
		masterTalon.config_IntegralZone(Constants.kSlotIdx, 3);

		/* Set relevant frame periods to be at least as fast as periodic rate */
		followTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
		followTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);

		/* Set the peak and nominal outputs */
		followTalon.configNominalOutputForward(0, Constants.kTimeoutMs);
		followTalon.configNominalOutputReverse(0, Constants.kTimeoutMs);
		followTalon.configPeakOutputForward(1, Constants.kTimeoutMs);
		followTalon.configPeakOutputReverse(-1, Constants.kTimeoutMs);

		/* Set Motion Magic gains in slot0 - see documentation */
		followTalon.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
		followTalon.config_kF(Constants.kSlotIdx, Constants.followerGains.kF, Constants.kTimeoutMs);
		followTalon.config_kP(Constants.kSlotIdx, Constants.followerGains.kP, Constants.kTimeoutMs);
		followTalon.config_kI(Constants.kSlotIdx, Constants.followerGains.kI, Constants.kTimeoutMs);
		followTalon.config_kD(Constants.kSlotIdx, Constants.followerGains.kD, Constants.kTimeoutMs);

		/* Set acceleration and vcruise velocity - see documentation */
		followTalon.configMotionCruiseVelocity(148, Constants.kTimeoutMs);
		followTalon.configMotionAcceleration(147.6, Constants.kTimeoutMs);

		followTalon.configFeedbackNotContinuous(true, Constants.kTimeoutMs);

		// Configure current limits
		followTalon.configPeakCurrentLimit(30);
		followTalon.configPeakCurrentDuration(150);

		// takes in AMPS
		followTalon.configContinuousCurrentLimit(20);

		// integral zone
		followTalon.config_IntegralZone(Constants.kSlotIdx, 3);

	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		SmartDashboard.putString("talon mode", masterTalon.getControlMode().toString());
		/* Get gamepad axis - forward stick is positive */
		double leftYstick = -1.0 * logitech.getLeftY(); /* left-side Y for Xbox360Gamepad */
		double rightYstick = -1.0 * logitech.getRightY(); /* right-side Y for Xbox360Gamepad */
		if (Math.abs(leftYstick) < 0.10) {
			leftYstick = 0;
		} /* deadband 10% */
		if (Math.abs(rightYstick) < 0.10) {
			rightYstick = 0;
		} /* deadband 10% */

		/* Get current Talon SRX motor output */
		double motorOutput = masterTalon.getMotorOutputPercent();

		/* Prepare line to print */
		_sb.append("\tOut%:");
		_sb.append(motorOutput);
		_sb.append("\tVel:");
		_sb.append(masterTalon.getSelectedSensorVelocity(Constants.kPIDLoopIdx));

		_sb.append("\t Position:");
		_sb.append(masterTalon.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("Lift position", masterTalon.getSelectedSensorPosition());

		/**
		 * Perform Motion Magic when Button 1 is held, else run Percent Output, which
		 * can
		 * be used to confirm hardware setup.
		 */

		// A

		if (logitech.getRawButton(1)) {
			/* Motion Magic */

			/* 4096 ticks/rev * 10 Rotations in either direction */
			// double targetPos = targetMin;
			// * 4096 * 10.0;
			masterTalon.set(ControlMode.MotionMagic, masterTargetMin);
			followTalon.set(ControlMode.MotionMagic, followerTargetMin);

			/* Append more signals to print when in speed mode */
			_sb.append("\terr:");
			_sb.append(masterTalon.getClosedLoopError(Constants.kPIDLoopIdx));
			_sb.append("\ttrg:");
			_sb.append(masterTargetMin);

			// Y
		} else if (logitech.getRawButton(4)) {
			/* Motion Magic */

			/* 4096 ticks/rev * 10 Rotations in either direction */
			// double targetPos = targetMax; //* 4096 * -10.0;
			masterTalon.set(ControlMode.MotionMagic, masterTargetMax);
			followTalon.set(ControlMode.MotionMagic, followerTargetMax);

			/* Append more signals to print when in speed mode */
			_sb.append("\terr:");
			_sb.append(masterTalon.getClosedLoopError(Constants.kPIDLoopIdx));
			_sb.append("\ttrg:");
			_sb.append(masterTargetMin);

		} else {
			/* Percent Output */

			masterTalon.set(ControlMode.PercentOutput, leftYstick * 0.25);
			followTalon.set(ControlMode.PercentOutput, rightYstick * 0.25);

			masterTalon.setNeutralMode(NeutralMode.Brake);
			followTalon.setNeutralMode(NeutralMode.Brake);
		}
		// if (logitech.getRawButton(2)) {
		// /* Zero sensor positions */
		// _talon.setSelectedSensorPosition(0);
		// }

		int pov = logitech.getPOV();
		if (_pov == pov) {
			/* no change */
		} else if (_pov == 180) { // D-Pad down
			/* Decrease smoothing */
			_smoothing--;
			if (_smoothing < 0)
				_smoothing = 0;
			masterTalon.configMotionSCurveStrength(_smoothing);

			System.out.println("Smoothing is set to: " + _smoothing);
		} else if (_pov == 0) { // D-Pad up
			/* Increase smoothing */
			_smoothing++;
			if (_smoothing > 8)
				_smoothing = 8;
			masterTalon.configMotionSCurveStrength(_smoothing);

			System.out.println("Smoothing is set to: " + _smoothing);
		}
		_pov = pov; /* save the pov value for next time */

		/* Instrumentation */
		Instrum.Process(masterTalon, _sb);
	}

}