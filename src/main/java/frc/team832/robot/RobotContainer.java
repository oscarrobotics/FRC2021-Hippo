package frc.team832.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunEndCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.team832.lib.driverinput.controllers.Attack3;
import frc.team832.lib.driverinput.controllers.Extreme3DPro;
import frc.team832.lib.driverinput.controllers.StratComInterface;
import frc.team832.lib.driverinput.oi.DriverOI;
import frc.team832.lib.driverinput.oi.OperatorInterface;
import frc.team832.lib.driverinput.oi.SticksDriverOI;
import frc.team832.lib.driverinput.oi.XboxDriverOI;
import frc.team832.lib.power.GrouchPDP;
import frc.team832.robot.subsystems.Drivetrain;

public class RobotContainer {

    public final GrouchPDP pdp = new GrouchPDP(0);
    public final Compressor pcm = new Compressor(0);

    public final DriverOI driverOI;
    public static final boolean isSticks = RobotBase.isReal();
    public final StratComInterface stratComInterface = new StratComInterface(isSticks ? 2 : 1);

    public Attack3 leftStick;
    public Extreme3DPro rightStick;

    // Subsystems
    public final Drivetrain drivetrain;
    public final Vision vision;
    public final Intake intake = new Intake(pdp);
    public final Shooter shooter = new Shooter(pdp);
    public final Spindexer spindexer = new Spindexer(pdp);
    public final Turret turret = new Turret(pdp);
    public final Climber climber = new Climber(pdp);
    public final WheelOfFortune wheelOfFortune = new WheelOfFortune();
    public final SuperStructure superStructure = new SuperStructure(intake, shooter, spindexer, turret, vision);

    public static final Notifier drivetrainTelemetryNotifier = new Notifier(drivetrain::updateDashboardData);
    public static final Notifier shooterTelemetryNotifier = new Notifier(shooter::updateDashboardData);
    public static final Notifier intakeTelemetryNotifier = new Notifier(intake::updateDashboardData);
    public static final Notifier turretTelemetryNotifier = new Notifier(turret::updateDashboardData);
    public static final Notifier visionTelemetryNotifier = new Notifier(vision::updateDashboardData);
    public static final Notifier climberTelemetryNotifier = new Notifier(climber::updateDashboardData);
    public static final Notifier spindexerTelemetryNotifier = new Notifier(spindexer::updateDashboardData);
    public static final Notifier superStructureTelemetryNotifier = new Notifier(superStructure::updateDashboardData);


    public RobotContainer() {
        if (isSticks) {
            driverOI = new SticksDriverOI();
            leftStick = ((SticksDriverOI)driverOI).leftStick;
            rightStick = ((SticksDriverOI)driverOI).rightStick;
        } else {
            driverOI = new XboxDriverOI();
        }

        drivetrain = new Drivetrain(pdp, driverOI);
        vision = new Vision(drivetrain);

        if (OperatorInterface.getConnectedControllerCount() > 1) {
            configTestingCommands();
        }
    }

    private void configureBrandonLayout() {

    }

    private void configTestingCommands() {

    }
}