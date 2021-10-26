package frc.team832.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team832.robot.subsystems.ClimbSubsystem;

public class StartClimbGroup extends SequentialCommandGroup {

    public StartClimbGroup(ClimbSubsystem climber, boolean climbUp){
        addCommands(
                new InstantCommand(climber::unlockClimb, climber),
                new WaitCommand(0.2),
                new InstantCommand(climbUp ? climber::windWinch : climber::unwindWinch, climber)
        );
    }
}
