package frc.team832.robot.subsystems;

import edu.wpi.first.wpilibj2.command.*;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.robot.Constants;

public class SuperStructure extends SubsystemBase {

    private final IntakeSubsystem intake;
    private final ShooterSubsystem shooter;
    private final SpindexerSubsystem spindexer;
    private final TurretSubsystem turret;
    private final VisionSubsystem vision;

    public final IdleCommand idleCommand;
    public final TargetingCommand targetingCommand;
    public final ShootCommandGroup shootCommand;
    public final IntakeCommand intakeCommand;
    public final ExtendIntakeCommand extendIntakeCommand;
    public final ExtendOuttakeCommand extendOuttakeCommand;
    public final RetractIntakeCommand retractIntakeCommand;

    private final double spindexerIntakeRpm = 15;

    public SuperStructure(IntakeSubsystem intake, ShooterSubsystem shooter, SpindexerSubsystem spindexer, TurretSubsystem turret, VisionSubsystem vision) {
        this.intake = intake;
        this.shooter = shooter;
        this.spindexer = spindexer;
        this.turret = turret;
        this.vision = vision;

        idleCommand = new IdleCommand();
        targetingCommand = new TargetingCommand();
        shootCommand = new ShootCommandGroup();
        intakeCommand = new IntakeCommand();
        extendIntakeCommand = new ExtendIntakeCommand();
        extendOuttakeCommand = new ExtendOuttakeCommand();
        retractIntakeCommand = new RetractIntakeCommand();

        DashboardManager.addTab(this);
    }

    //COMMANDS:
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

    public class ShootCommandGroup extends ParallelCommandGroup {
        public ShootCommandGroup() {
            addRequirements(shooter, intake, spindexer, turret, SuperStructure.this);
            addCommands(
                    // tracking target
                    new FunctionalCommand(() -> turret.setForward(true), SuperStructure.this::trackTarget, (interrupted) -> {}, () -> false),
                    // wait then shoot
                    new SequentialCommandGroup(
                            new WaitCommand(0.5),
                            new FunctionalCommand(
                                    () -> spindexer.setSpinRPM(90, SpindexerSubsystem.SpinnerDirection.Clockwise),
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
            turret.setForward(true);
        }

        @Override
        public void execute() {
            trackTarget();
//			spindexer.setTargetRotation(getNearestSafeRotationRelativeToFeeder());might be unnecessary
        }
    }

    public void RunIdleCommand() {
        idleCommand.schedule();
    }

    public void RunIntakeCommand() {
        intakeCommand.schedule();
    }

    public void shootAtTarget() {
        if (vision.hasTarget()) {
            turret.trackTarget();
            shooter.trackTarget();
            shooter.setFeedRPM(Constants.ShooterValues.FeedRpm);
        } else {
            turret.setTurretTargetDegrees(0.0, true);
        }
    }


    public class ExtendIntakeCommand extends SequentialCommandGroup {
        public ExtendIntakeCommand() {
            addRequirements(intake, shooter, spindexer, turret, SuperStructure.this);
            addCommands(
                    new InstantCommand(intake::extendIntake),
                    new InstantCommand(() -> spindexer.setSpinRPM(30, SpindexerSubsystem.SpinnerDirection.Clockwise)),
                    new WaitCommand(0.5),
                    new InstantCommand(() -> intake.intake(0.5)),
                    spindexer.getAntiJamSpinCommand(15, 1.0)
            );
        }
    }

    public class ExtendOuttakeCommand extends SequentialCommandGroup {
        public ExtendOuttakeCommand() {
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
        public RetractIntakeCommand() {
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
    public void intake(double power, double spinRPM, SpindexerSubsystem.SpinnerDirection direction) {
        intake.intake(power);
        spindexer.setSpinRPM(spinRPM, direction);
        intake.extendIntake();
    }

    public void trackTarget() {
        if (vision.hasTarget()) {
            turret.trackTarget();
            shooter.trackTarget();
        } else {
            turret.setTurretTargetDegrees(0.0, true);
        }
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
        spindexer.idle();
    }
}
