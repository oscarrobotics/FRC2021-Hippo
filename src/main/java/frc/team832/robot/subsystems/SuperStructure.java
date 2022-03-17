package frc.team832.robot.subsystems;

import edu.wpi.first.wpilibj2.command.*;
import frc.team832.lib.driverinput.controllers.StratComInterface;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.robot.Constants;

public class SuperStructure extends SubsystemBase {

    private final IntakeSubsystem intake;
    private final ShooterSubsystem shooter;
    private final SpindexerSubsystem spindexer;
    private final TurretSubsystem turret;
    private final VisionSubsystem vision;

    private final double spindexerIntakeRpm = 15;

    public SuperStructure(IntakeSubsystem intake, ShooterSubsystem shooter, SpindexerSubsystem spindexer, TurretSubsystem turret, VisionSubsystem vision) {
        this.intake = intake;
        this.shooter = shooter;
        this.spindexer = spindexer;
        this.turret = turret;
        this.vision = vision;


        DashboardManager.addTab(this);
    }

    public IdleCommand getIdleCommand() {
        return new IdleCommand();
    }

    public TargetingCommand getTargetingCommand() {
        return new TargetingCommand();
    }


    public ShootCommandGroup getShootCommand(int spindexerRPM) {
        return new ShootCommandGroup(spindexerRPM);
    }

    public IntakeCommand getIntakeCommand() {
        return new IntakeCommand();
    }

    public ExtendIntakeCommand getExtendIntakeCommand() {
        return new ExtendIntakeCommand();
    }

    public ExtendOuttakeCommand getExtendOuttakeCommand() {
        return new ExtendOuttakeCommand();
    }

    public RetractIntakeCommand getRetractIntakeCommand() {
        return new RetractIntakeCommand();
    }

    public void setSoindexerRPM(double rpm, SpindexerSubsystem.SpinnerDirection direction) {
        spindexer.setSpinRPM(rpm, direction);
    }

    public void setFeedRpm(double rpm){
        shooter.setFeedRPM(rpm);
    }

    //COMMANDS:
    private class IdleCommand extends InstantCommand {
        IdleCommand() {
            addRequirements(shooter, intake, spindexer, turret, SuperStructure.this);
        }

        @Override
        public void initialize() {
            idleAll();
            turret.setTurretTargetDegrees(0);
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
                intake(0.6, -spindexerIntakeRpm, SpindexerSubsystem.SpinnerDirection.CounterClockwise);
            } else {
                intake(0.6, spindexerIntakeRpm, SpindexerSubsystem.SpinnerDirection.Clockwise);
            }
        }

        @Override
        public void end(boolean interrupted) {
            idleIntake();
        }

    }

    public class ShootCommandGroup extends ParallelCommandGroup {
        ShootCommandGroup(int spindexerRPM) {
            addRequirements(shooter, intake, spindexer, turret, SuperStructure.this);
            addCommands(
                    // tracking target
                    new FunctionalCommand(turret::setForward, SuperStructure.this::trackTarget, (interrupted) -> {}, () -> false),
                    // wait then shoot
                    new SequentialCommandGroup(
                            new WaitCommand(0.5),
                            new FunctionalCommand(
                                    () -> spindexer.setSpinRPM(spindexerRPM, SpindexerSubsystem.SpinnerDirection.Clockwise),
                                    SuperStructure.this::shootAtTarget,
                                    (interrupted) -> { idleSpindexer(); idleShooter(); },
                                    () -> false
                            )
                    )
            );
        }
    }

    public class TargetingCommand extends CommandBase {
        TargetingCommand() {
            addRequirements(shooter, intake, spindexer, turret, SuperStructure.this);
        }

        @Override
        public void initialize() {
            idleSpindexer();
            turret.setForward();
        }

        @Override
        public void execute() {
            trackTarget();
//			spindexer.setTargetRotation(getNearestSafeRotationRelativeToFeeder());might be unnecessary
        }

        @Override
        public void end(boolean interrupted) {
            idleAll();
        }
    }


    public class ExtendIntakeCommand extends SequentialCommandGroup {
        ExtendIntakeCommand() {
            addRequirements(intake, shooter, spindexer, turret, SuperStructure.this);
            addCommands(
                    new InstantCommand(intake::extendIntake),
                    new InstantCommand(() -> spindexer.setSpinRPM(30, SpindexerSubsystem.SpinnerDirection.Clockwise)),
                    new WaitCommand(0.5),
                    new InstantCommand(() -> intake.intake(0.5))
//                    spindexer.getAntiJamSpinCommand(15, 1.0)
            );
        }
    }

    public class ExtendOuttakeCommand extends SequentialCommandGroup {
        ExtendOuttakeCommand() {
            addRequirements(intake, shooter, spindexer, turret, SuperStructure.this);
            addCommands(
                    new InstantCommand(intake::extendIntake),
                    new InstantCommand(() -> spindexer.setSpinRPM(25, SpindexerSubsystem.SpinnerDirection.CounterClockwise)),
                    new WaitCommand(0.25),
                    new InstantCommand(() -> intake.outtake(0.4))
            );
        }
    }

    public class RetractIntakeCommand extends SequentialCommandGroup {
        RetractIntakeCommand() {
            addRequirements(intake, shooter, spindexer, turret, SuperStructure.this);
            addCommands(
                    new InstantCommand(() -> spindexer.setSpinRPM(20, SpindexerSubsystem.SpinnerDirection.Clockwise)),
                    new InstantCommand(intake::retractIntake),
                    new WaitCommand(1.0),
                    new InstantCommand(intake::stop),
                    new InstantCommand(spindexer::idle)
            );
        }
    }

    //HELPER METHODS
    @SuppressWarnings("SameParameterValue")
    private void intake(double power, double spinRPM, SpindexerSubsystem.SpinnerDirection direction) {
        intake.intake(power);
        spindexer.setSpinRPM(spinRPM, direction);
        intake.extendIntake();
    }

    private void trackTarget() {
        if (vision.hasTarget()) {
            turret.trackTarget();
            shooter.trackTarget();
        } else {
            turret.setTurretTargetDegrees(0.0);
        }
    }

    private void shootAtTarget() {
        if (vision.hasTarget()) {
            turret.trackTarget();
            shooter.trackTarget();
            shooter.setFeedPow(1);
        } else {
            turret.setTurretTargetDegrees(0.0);
        }
    }

    private void idleAll() {
        idleShooter();
        idleIntake();
        idleSpindexer();
    }

    private void idleIntake() {
        intake.stop();
        intake.retractIntake();
    }

    private void idleShooter() {
        shooter.setFlywheelRPM(0);
        shooter.setFeedRPM(0);
        shooter.setFeedPow(0);
    }

    private void idleSpindexer() {
        spindexer.idle();
    }
}
