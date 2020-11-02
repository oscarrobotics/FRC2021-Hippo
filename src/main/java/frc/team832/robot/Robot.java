/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team832.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team832.lib.CANDevice;
import frc.team832.lib.control.PCM;
import frc.team832.lib.motorcontrol.NeutralMode;

public class Robot extends TimedRobot {

  public final RobotContainer robotContainer = new RobotContainer();

  private final Compressor pcm = robotContainer.pcm;
    private final Drive

  @Override
  public void robotInit() {
//    pcm.setClosedLoopControl(true);

    if (drivetrain.initSuccessful) {
      System.out.println("Drivetrain - init OK");
    } else {
      System.out.println("Drivetrain - init FAILED");
    }
    drivetrainTelemetryNotifier.startPeriodic(0.02);

    if (intake.initSuccessful) {
      System.out.println("Intake - init OK");
      intakeTelemetryNotifier.startPeriodic(0.02);
    } else {
      System.out.println("Intake - init FAILED");
    }

    if (vision.initSuccessful) {
      System.out.println("Vision - init OK");
    } else {
      System.out.println("Vision - init FAILED");
    }
    visionTelemetryNotifier.startPeriodic(0.02);

    if (shooter.initSuccessful) {
      System.out.println("Shooter - init OK");
    } else {
      System.out.println("Shooter - init FAILED");
    }
    shooterTelemetryNotifier.startPeriodic(0.02);

    if (turret.initSuccessful) {
      System.out.println("Turret - init OK");
    } else {
      System.out.println("Turret - init FAILED");
    }
    turretTelemetryNotifier.startPeriodic(0.02);

    if (spindexer.initSuccessful) {
      System.out.println("Spindexer - init OK");
    } else {
      System.out.println("Spindexer - init FAILED");
    }
    spindexerTelemetryNotifier.startPeriodic(0.02);

    if (climber.initSuccessful) {
      System.out.println("Climber - init OK");
    } else {
      System.out.println("Climber - init FAILED");
    }
    climberTelemetryNotifier.startPeriodic(0.02);

    if (wheelOfFortune.initSuccessful) {
      System.out.println("WheelOfFortune - init OK");
    } else {
      System.out.println("WheelOfFortune - init FAILED");
    }

    superStructureTelemetryNotifier.startPeriodic(0.02);

    CANDevice.printMissingDevices();
//        autoCommand = new BasicAutonomous(superStructure, drivetrain);
    autoCommand = new DumbPathAuto(drivetrain);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    NeutralMode mode = NeutralMode.kBrake;
    drivetrain.setNeutralMode(mode);
    shooter.setFlyheelNeutralMode(mode);
    shooter.setFeederNeutralMode(mode);
    turret.holdTurretPosition();
    spindexer.setNeutralMode(mode);
    turret.setNeutralMode(mode);
    shooter.setHood(2.7);
    climber.zeroDeploy();

    autoCommand.schedule();
  }

  @Override
  public void disabledInit() {
    NeutralMode mode = NeutralMode.kCoast;
    drivetrain.setNeutralMode(mode);
    shooter.setFlyheelNeutralMode(mode);
    shooter.setFeederNeutralMode(mode);
    spindexer.setNeutralMode(mode);
    turret.setNeutralMode(mode);
    climber.lockClimb();
  }

  @Override
  public void disabledPeriodic() {

  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    autoCommand.cancel();
    NeutralMode mode = NeutralMode.kBrake;
    drivetrain.setNeutralMode(mode);
    shooter.setFlyheelNeutralMode(mode);
    shooter.setFeederNeutralMode(mode);
    turret.holdTurretPosition();
    spindexer.setNeutralMode(mode);
    turret.setNeutralMode(mode);
    shooter.setHood(2.7);
    climber.zeroDeploy();
  }

  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void testInit() {
    drivetrain.driveMusic.play();
  }


}
