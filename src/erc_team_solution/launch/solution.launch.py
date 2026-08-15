"""Launch the complete ERC 2026 team solution."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Create the evaluator-compatible solution launch description."""
    shelf_column_number = LaunchConfiguration(
        "shelf_column_number"
    )

    book_colour = LaunchConfiguration(
        "book_colour"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "shelf_column_number",
                description=(
                    "Requested shelf marker number from 1 to 5."
                ),
            ),
            DeclareLaunchArgument(
                "book_colour",
                description=(
                    "Requested book colour: "
                    "red, green, blue, or yellow."
                ),
            ),
            Node(
                package="erc_team_solution",
                executable="orchestrator",
                name="erc_orchestrator",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "shelf_column_number": ParameterValue(
                            shelf_column_number,
                            value_type=int,
                        ),
                        "book_colour": ParameterValue(
                            book_colour,
                            value_type=str,
                        ),
                        "use_sim_time": True,
                    }
                ],
            ),
        ]
    )
