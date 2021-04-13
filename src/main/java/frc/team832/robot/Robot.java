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
import frc.team832.lib.motorcontrol.NeutralMode;
import frc.team832.robot.subsystems.*;

public class Robot extends TimedRobot {

  public final RobotContainer robotContainer = new RobotContainer();

  private final Compressor pcm = robotContainer.pcm;
  private final DrivetrainSubsystem drivetrain = robotContainer.drivetrainSubsystem;
  private final IntakeSubsystem intake = robotContainer.intake;
  private final TurretSubsystem turret = robotContainer.turret;
  private final ShooterSubsystem shooter = robotContainer.shooter;
  private final VisionSubsystem vision = robotContainer.vision;
  private final SpindexerSubsystem spindexer = robotContainer.spindexer;

  @Override
  public void robotInit() {
    if (drivetrain.initSuccessful) {
      System.out.println("Drivetrain - init OK");
      addPeriodic(drivetrain::updateControlLoops, 0.05);
    } else {
      System.out.println("Drivetrain - init FAILED");
    }

    if (intake.initSuccessful) {
      System.out.println("Intake - init OK");
    } else {
      System.out.println("Intake - init FAILED");
    }

    if (vision.initSuccessful) {
      System.out.println("Vision - init OK");
    } else {
      System.out.println("Vision - init FAILED");
    }

    if (shooter.initSuccessful) {
      System.out.println("Shooter - init OK");
      addPeriodic(shooter::updateControlLoops, 0.05);
    } else {
      System.out.println("Shooter - init FAILED");
    }

    if (turret.initSuccessful) {
      System.out.println("Turret - init OK");
    } else {
      System.out.println("Turret - init FAILED");
    }

    if (spindexer.initSuccessful) {
      System.out.println("Spindexer - init OK");
    } else {
      System.out.println("Spindexer - init FAILED");
    }

//    if (climber.initSuccessful) {
//      System.out.println("Climber - init OK");
//    } else {
//      System.out.println("Climber - init FAILED");
//    }
//
//    if (wheelOfFortune.initSuccessful) {
//      System.out.println("WheelOfFortune - init OK");
//    } else {
//      System.out.println("WheelOfFortune - init FAILED");
//    }

    CANDevice.printMissingDevices();
//    autoCommand = new BasicAutonomous(superStructure, drivetrain);
//    autoCommand = new DumbPathAuto(drivetrain);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    NeutralMode mode = NeutralMode.kBrake;
    drivetrain.setNeutralMode(mode);
    shooter.setFlyheelNeutralMode(NeutralMode.kCoast);
    shooter.setFeederNeutralMode(mode);
//    turret.holdTurretPosition();
//    spindexer.setNeutralMode(mode);
//    turret.setNeutralMode(mode);
//    shooter.setHood(2.7);
//    climber.zeroDeploy();
//
//    autoCommand.schedule();
  }

  @Override
  public void disabledInit() {
    NeutralMode mode = NeutralMode.kCoast;
    drivetrain.setNeutralMode(mode);
    shooter.setFlyheelNeutralMode(mode);
    shooter.setFeederNeutralMode(mode);
//    spindexer.setNeutralMode(mode);
//    turret.setNeutralMode(mode);
//    climber.lockClimb();
  }

  @Override
  public void disabledPeriodic() {

  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
//    autoCommand.cancel();
    NeutralMode mode = NeutralMode.kBrake;
    drivetrain.setNeutralMode(mode);
    shooter.setFlyheelNeutralMode(mode);
    shooter.setFeederNeutralMode(mode);
//    turret.holdTurretPosition();
//    spindexer.setNeutralMode(mode);
//    turret.setNeutralMode(mode);
//    shooter.setHood(2.7);
//    climber.zeroDeploy();
  }

  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void testInit() {
//    drivetrainSubsystem.driveMusic.play();
  }


}
