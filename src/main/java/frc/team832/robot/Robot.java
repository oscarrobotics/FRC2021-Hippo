/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team832.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team832.lib.CANDevice;
import frc.team832.lib.motorcontrol.NeutralMode;
import frc.team832.robot.commands.DumbAutoCommand;
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
  private final ClimbSubsystem climber = robotContainer.climber;

  private final Command autoCommand = new DumbAutoCommand(drivetrain, robotContainer.superStructure);

  @Override
  public void robotInit() {
    if (drivetrain.initSuccessful) {
      System.out.println("Drivetrain - init OK");
      addPeriodic(drivetrain::updateControlLoops, Constants.DrivetrainValues.ControlLoopPeriod);
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
      addPeriodic(shooter::updateControlLoops, Constants.ShooterValues.ControlLoopPeriod);
    } else {
      System.out.println("Shooter - init FAILED");
    }

    if (turret.initSuccessful) {
      addPeriodic(turret::updateControlLoops, Constants.TurretValues.ControlLoopPeriod);
      System.out.println("Turret - init OK");
    } else {
      System.out.println("Turret - init FAILED");
    }

    if (spindexer.initSuccessful) {
      System.out.println("Spindexer - init OK");
      addPeriodic(spindexer::updateControlLoops, Constants.SpindexerValues.ControlLoopPeriod);
    } else {
      System.out.println("Spindexer - init FAILED");
    }

    if (climber.initSuccessful) {
      System.out.println("Climber - init OK");
    } else {
      System.out.println("Climber - init FAILED");
    }
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
    shooter.setHoodNeutralMode(mode);
    spindexer.setNeutralMode(mode);
    turret.setNeutralMode(mode);
    shooter.zeroHood();
    shooter.setHoodAngle(Constants.ShooterValues.HoodMinAngle);
    intake.setNeutralMode(NeutralMode.kCoast);
    climber.zeroDeploy();
//
    autoCommand.schedule();
  }

  @Override
  public void disabledInit() {
    NeutralMode coast = NeutralMode.kCoast;
    drivetrain.setNeutralMode(coast);
    shooter.setFlyheelNeutralMode(coast);
    shooter.setFeederNeutralMode(coast);
    shooter.setHoodNeutralMode(coast);
    spindexer.setNeutralMode(coast);
    turret.setNeutralMode(coast);
    intake.setNeutralMode(coast);
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
    NeutralMode brake = NeutralMode.kBrake;
    drivetrain.setNeutralMode(brake);
    shooter.setFlyheelNeutralMode(NeutralMode.kCoast);
    shooter.setFeederNeutralMode(brake);
    shooter.setHoodNeutralMode(brake);
    spindexer.setNeutralMode(brake);
    turret.setNeutralMode(brake);
    shooter.setHoodAngle(Constants.ShooterValues.HoodMinAngle);
  }

  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void testInit() {
//    drivetrainSubsystem.driveMusic.play();
  }


}
