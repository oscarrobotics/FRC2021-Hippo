package frc.team832.robot;

import edu.wpi.first.wpilibj.Compressor;
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

import static frc.team832.robot.Robot.turret;

public class RobotContainer {

    public final GrouchPDP pdp = new GrouchPDP(0);
    public final Compressor pcm = new Compressor(0);

    // Sub
    public final Drivetrain drivetrain = new Drivetrain(pdp);
    public final Vision vision = new Vision(drivetrain);
    public final Intake intake = new Intake(pdp);
    public final Shooter shooter = new Shooter(pdp);
    public final Spindexer spindexer = new Spindexer(pdp);
    public final Turret turret = new Turret(pdp);
    public final Climber climber = new Climber(pdp);
    public final WheelOfFortune wheelOfFortune = new WheelOfFortune();
    public final SuperStructure superStructure = new SuperStructure(intake, shooter, spindexer, turret, vision);

    public final DriverOI driverOI;
    public static final boolean isSticks = RobotBase.isReal();
    public final StratComInterface stratComInterface = new StratComInterface(isSticks ? 2 : 1);


    public Attack3 leftStick;
    public Extreme3DPro rightStick;


    public RobotContainer() {
        templateSubsystem = new TemplateSubsystem();

        if (templateSubsystem.initializedSuccessfully) {
            System.out.println("Template Subsys - INIT OK");
        } else {
            System.out.println("Template Subsys - INIT FAIL");
        }

        if (isSticks) {
            driverOI = new SticksDriverOI();
            leftStick = ((SticksDriverOI)driverOI).leftStick;
            rightStick = ((SticksDriverOI)driverOI).rightStick;
        } else {
            driverOI = new XboxDriverOI();
        }

        if (OperatorInterface.getConnectedControllerCount() > 1) {
            configTestingCommands();
        }
    }

    private void configureBrandonLayout() {

        stratComInterface.getArcadeBlackRight().whenHeld(new StartEndCommand(() -> superStructure.setState(SuperStructure.SuperstructureState.IDLE),
                () -> superStructure.setState(SuperStructure.SuperstructureState.IDLE)));

        stratComInterface.getArcadeBlackLeft().whenHeld(new StartEndCommand(() -> superStructure.setState(SuperStructure.SuperstructureState.INTAKE),
                () -> superStructure.setState(SuperStructure.SuperstructureState.IDLE)));
        stratComInterface.getArcadeBlackLeft().whenHeld(new RunEndCommand(() -> superStructure.configureSpindexerRPMSlider(stratComInterface.getRightSlider()),
                superStructure::setSpindexerIntakeRpmDefault));

        stratComInterface.getArcadeWhiteLeft().whenHeld(new StartEndCommand(() -> superStructure.setState(SuperStructure.SuperstructureState.TARGETING),
                () -> superStructure.setState(SuperStructure.SuperstructureState.IDLE)));

        stratComInterface.getArcadeWhiteRight().whenHeld(new StartEndCommand(() -> superStructure.setState(SuperStructure.SuperstructureState.SHOOTING),
                () -> superStructure.setState(SuperStructure.SuperstructureState.IDLE)));


        stratComInterface.getSingleToggle().whenHeld(new RunEndCommand(() -> climber.adjustHook(stratComInterface.getLeftSlider()), climber::stopExtend));
        stratComInterface.getSingleToggle().whenReleased(new InstantCommand(climber::retractHook));

        stratComInterface.getSCPlus().whileHeld(new StartClimbGroup(climber, true));
        stratComInterface.getSCPlus().whenReleased(new InstantCommand(climber::lockClimb));

        stratComInterface.getSCMinus().whileHeld(new StartClimbGroup(climber, false));
        stratComInterface.getSCMinus().whenReleased(new InstantCommand(climber::lockClimb));

//		stratComInterface.getSCSideTop().whenHeld(new StartEndCommand(wheelOfFortune::extendWOFManipulator, wheelOfFortune::retractWOFManipulator));
//		stratComInterface.getSC1().whenHeld(new StartEndCommand(wheelOfFortune::spinCounterclockwise, wheelOfFortune::stopSpin, wheelOfFortune));//torque vector up
//		stratComInterface.getSC3().whenHeld(new StartEndCommand(wheelOfFortune::spinClockwise, wheelOfFortune::stopSpin, wheelOfFortune));//torque vector down
//		stratComInterface.getSC2().whenHeld(new RunEndCommand(wheelOfFortune::spinThreeTimes, wheelOfFortune::stopSpin, wheelOfFortune));
    }

    private void configTestingCommands() {
        stratComInterface.getArcadeBlackRight().whenPressed(new InstantCommand(intake::extendIntake));
        stratComInterface.getArcadeBlackRight().whenReleased(new InstantCommand(intake::retractIntake));

//		stratComInterface.getSingleToggle().whenHeld(new RunEndCommand(() -> climber.adjustHook(stratComInterface.getLeftSlider()), climber::stopExtend));
//		stratComInterface.getSingleToggle().whenReleased(new InstantCommand(climber::retractHook));

        stratComInterface.getSCPlus().whileHeld(new StartClimbGroup(climber, true));
        stratComInterface.getSCPlus().whenReleased(new StopClimbGroup(climber));

        stratComInterface.getSCMinus().whileHeld(new StartClimbGroup(climber, false));
        stratComInterface.getSCMinus().whenReleased(new StopClimbGroup(climber));

        stratComInterface.getDoubleToggleUp().whenHeld(new RunEndCommand(() -> turret.setHeadingSlider(stratComInterface.getRightSlider()), () -> turret.setHeadingSlider(0)));
    }
}