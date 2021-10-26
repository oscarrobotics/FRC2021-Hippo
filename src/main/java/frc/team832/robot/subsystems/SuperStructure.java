package frc.team832.robot.subsystems;

import edu.wpi.first.wpilibj2.command.*;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.robot.Constants;
import frc.team832.robot.utilities.state.ShooterCalculations;

public class SuperStructure extends SubsystemBase {

    private final IntakeSubsystem intake;
    private final ShooterSubsystem shooter;
    private final SpindexerSubsystem spindexer;
    private final TurretSubsystem turret;
    private final VisionSubsystem vision;

    public final IdleCommand idleCommand;
    public final TargetingCommand targetingCommand;
    public final ShootCommandGroup shootOnTarget;
    public final CloseRangeShootCommandGroup closeShoot;
    public final TargetingTestCommand testTargeting;
    public final ExtendIntakeCommand extendIntake;
    public final ExtendOuttakeCommand extendOuttake;
    public final RetractIntakeCommand retractIntake;

    private double spindexerIntakeRpm = 15;

    public SuperStructure(IntakeSubsystem intake, ShooterSubsystem shooter, SpindexerSubsystem spindexer, TurretSubsystem turret, VisionSubsystem vision) {
        this.intake = intake;
        this.shooter = shooter;
        this.spindexer = spindexer;
        this.turret = turret;
        this.vision = vision;

        idleCommand = new IdleCommand();
        targetingCommand = new TargetingCommand();
        shootOnTarget = new ShootCommandGroup();
        closeShoot = new CloseRangeShootCommandGroup();
        testTargeting = new TargetingTestCommand();
        extendIntake = new ExtendIntakeCommand();
        extendOuttake = new ExtendOuttakeCommand();
        retractIntake = new RetractIntakeCommand();

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

    public class TargetingCommand extends CommandBase {
        TargetingCommand() {
            addRequirements(shooter, intake, spindexer, turret, SuperStructure.this);
        }

        @Override
        public void initialize() {
            spindexer.idle();
            turret.setForward();
        }

        @Override
        public void execute() {
            trackTarget();
        }
    }

    public void RunIdleCommand() {
        idleCommand.schedule();
    }

    public void shootAtTarget() {
        if (vision.hasTarget()) {
            turret.trackTarget(spindexer.getVelocity());
            shooter.trackTarget();
            shooter.setFeedRPM(Constants.ShooterValues.FeedRpm);
        } else {
            turret.setTurretTargetDegrees(0.0, true);
        }
    }

    public boolean readyToShoot() {
        return shooter.atShootingRpm() && shooter.atHoodAngle() && turret.atTargetAngle();
    }


    public class ShootCommandGroup extends ParallelCommandGroup {
        public ShootCommandGroup() {
            addRequirements(shooter, intake, spindexer, turret, SuperStructure.this);
            addCommands(
                    new RunEndCommand(
                            SuperStructure.this::trackTarget,
                            () -> {
                                shooter.idleAll();
                                spindexer.idle();
                            }
                    ),
                    new SequentialCommandGroup(
                            new WaitUntilCommand(SuperStructure.this::readyToShoot),
                            new InstantCommand(() -> shooter.setFeedRPM(3000)),
                            new WaitUntilCommand(shooter::atFeedRpm),
                            new RunEndCommand(
                                    SuperStructure.this::shootAtTarget,
                                    () -> {
                                        shooter.idleAll();
                                        spindexer.idle();
                                    }
                            )
                    )
            );
        }
    }

    public class CloseRangeShootCommandGroup extends ParallelCommandGroup {
        public CloseRangeShootCommandGroup() {
            addRequirements(shooter, intake, spindexer, turret, SuperStructure.this);
            addCommands(
                    new StartEndCommand(
                            () -> {
                                shooter.setFlywheelRPM(4500);
                                shooter.setHoodAngle(19);
                                turret.setForward();
                            },
                            () -> {
                                shooter.idleAll();
                                spindexer.idle();
                            }
                    ),
                    new SequentialCommandGroup(
                            new WaitUntilCommand(SuperStructure.this::readyToShoot),
                            new InstantCommand(() -> shooter.setFeedRPM(2500)),
                            new WaitUntilCommand(shooter::atFeedRpm),
                            new StartEndCommand(
                                    () -> spindexer.setSpinRPM(100, SpindexerSubsystem.SpinnerDirection.Clockwise),
                                    () -> {
                                        shooter.idleAll();
                                        spindexer.idle();
                                    }
                            )
                    )
            );
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

    public class TargetingTestCommand extends CommandBase {
        TargetingTestCommand() {
            addRequirements(shooter, intake, spindexer, turret, SuperStructure.this);
        }

        @Override
        public void initialize() {
            spindexer.idle();
            turret.setForward();
            spindexer.setSpinRPM(60, SpindexerSubsystem.SpinnerDirection.Clockwise);
        }

        @Override
        public void execute() {
            testTrackTarget();
        }

        @Override
        public void end(boolean interrupted) {
            shooter.idleAll();
            spindexer.idle();
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
            turret.trackTarget(spindexer.getVelocity());
            shooter.trackTarget();
        } else {
            turret.setTurretTargetDegrees(0.0, true);
        }
    }

    public void testTrackTarget() {
        if (vision.hasTarget()) {
            turret.trackTarget(spindexer.getRPM());
            shooter.setHoodToVisionDistance();
        } else {
            turret.setTurretTargetDegrees(0.0, true);
        }
    }

    public void idleAll() {
        shooter.idleAll();
        intake.idleAll();
        spindexer.idleAll();
    }
}
