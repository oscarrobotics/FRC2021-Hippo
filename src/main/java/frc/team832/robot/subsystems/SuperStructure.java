package frc.team832.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.driverstation.dashboard.DashboardManager;

public class SuperStructure extends SubsystemBase {

    private final IntakeSubsystem intake;
    private final ShooterSubsystem shooter;
    private final SpindexerSubsystem spindexer;
    private final TurretSubsystem turret;
    private final VisionSubsystem vision;

    public final IdleCommand idleCommand;
    public final TargetingCommand targetingCommand;
    public final ShootCommand shootCommand;
    public final IntakeCommand intakeCommand;

    private double spindexerIntakeRpm = 15;

    public SuperStructure(IntakeSubsystem intake, ShooterSubsystem shooter, SpindexerSubsystem spindexer, TurretSubsystem turret, VisionSubsystem vision) {
        this.intake = intake;
        this.shooter = shooter;
        this.spindexer = spindexer;
        this.turret = turret;
        this.vision = vision;

        idleCommand = new IdleCommand();
        targetingCommand = new TargetingCommand();
        shootCommand = new ShootCommand();
        intakeCommand = new IntakeCommand();

        DashboardManager.addTab(this);
    }

    private class IdleCommand extends InstantCommand {
        IdleCommand() {
            addRequirements(shooter, intake, spindexer, turret, SuperStructure.this);
        }

        @Override
        public void initialize() {
            idleAll();
            turret.setTurretTargetDegrees(0, false);
        }
    }

    private class IntakeCommand extends CommandBase {
        IntakeCommand() {
            addRequirements(shooter, intake, spindexer, turret, SuperStructure.this);
        }

        @Override
        public void initialize() {
            shooter.setFeedRPM(0);
            turret.setIntake();
        }

        @Override
        public void execute() {
            if (Math.signum(spindexerIntakeRpm) == -1) {
                intake(0.7, -spindexerIntakeRpm, SpindexerSubsystem.SpinnerDirection.CounterClockwise);
            } else {
                intake(0.7, spindexerIntakeRpm, SpindexerSubsystem.SpinnerDirection.Clockwise);
            }
        }

        @Override
        public void end(boolean interrupted) {
            idleIntake();
        }

    }

    public void intake(double power, double spinRPM, SpindexerSubsystem.SpinnerDirection direction) {
        intake.intake(power);
        spindexer.setSpinRPM(spinRPM, direction);
        intake.extendIntake();
    }

    private class ShootCommand extends CommandBase {

    }

    private class TargetingCommand extends CommandBase {

    }

    public void RunIdleCommand() {
        idleCommand.schedule();
    }

    public void RunIntakeCommand() {
        intakeCommand.schedule();
    }

    public void idleAll() {
        idleShooter();
        idleIntake();
        idleSpindexer();
    }

    public void idleIntake() {
        intake.stop();
        intake.retractIntake();
    }

    public void idleShooter() {

    }

    public void idleSpindexer() {

    }
}
