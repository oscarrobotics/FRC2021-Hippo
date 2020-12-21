package frc.team832.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.RobotBase;
import frc.team832.lib.driverinput.controllers.Attack3;
import frc.team832.lib.driverinput.controllers.Extreme3DPro;
import frc.team832.lib.driverinput.controllers.StratComInterface;
import frc.team832.lib.driverinput.oi.DriverOI;
import frc.team832.lib.driverinput.oi.OperatorInterface;
import frc.team832.lib.driverinput.oi.SticksDriverOI;
import frc.team832.lib.driverinput.oi.XboxDriverOI;
import frc.team832.lib.power.GrouchPDP;
import frc.team832.robot.subsystems.*;

public class RobotContainer {

    public final GrouchPDP pdp = new GrouchPDP(0);
    public final Compressor pcm = new Compressor(0);

    public final DriverOI driverOI;
    public static final boolean isSticks = RobotBase.isReal();
    public final StratComInterface stratComInterface = new StratComInterface(isSticks ? 2 : 1);

    public Attack3 leftStick;
    public Extreme3DPro rightStick;

    // Subsystems
    public final DrivetrainSubsystem drivetrainSubsystem;
    public final VisionSubsystem vision;
    public final IntakeSubsystem intake = new IntakeSubsystem(pdp);
    public final ShooterSubsystem shooter = new ShooterSubsystem(pdp);
    public final SpindexerSubsystem spindexer = new SpindexerSubsystem(pdp);
    public final TurretSubsystem turret = new TurretSubsystem(pdp);
//    public final Climber climber = new Climber(pdp);
//    public final WheelOfFortune wheelOfFortune = new WheelOfFortune();
//    public final SuperStructure superStructure = new SuperStructure(intake, shooter, spindexer, turret, vision);

    public RobotContainer() {
        if (isSticks) {
            driverOI = new SticksDriverOI();
            leftStick = ((SticksDriverOI)driverOI).leftStick;
            rightStick = ((SticksDriverOI)driverOI).rightStick;
        } else {
            driverOI = new XboxDriverOI();
        }

        drivetrainSubsystem = new DrivetrainSubsystem(pdp, driverOI);
        vision = new VisionSubsystem(drivetrainSubsystem);

        if (OperatorInterface.getConnectedControllerCount() > 1) {
            configTestingCommands();
        }
    }

    private void configTestingCommands() {

    }
}