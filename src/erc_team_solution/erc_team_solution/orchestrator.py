"""Top-level state-machine entry point for the ERC 2026 solution."""

from __future__ import annotations

from typing import Final

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


VALID_BOOK_COLOURS: Final = frozenset(
    {
        "red",
        "green",
        "blue",
        "yellow",
    }
)


class Orchestrator(Node):
    """Validate trial arguments and coordinate all solution subsystems."""

    def __init__(self) -> None:
        """Initialize the top-level competition coordinator."""
        super().__init__("erc_orchestrator")

        self.declare_parameter("shelf_column_number", 1)
        self.declare_parameter("book_colour", "red")

        shelf_parameter = self.get_parameter(
            "shelf_column_number"
        ).get_parameter_value()

        colour_parameter = self.get_parameter(
            "book_colour"
        ).get_parameter_value()

        self.shelf_column_number = int(
            shelf_parameter.integer_value
        )

        self.book_colour = (
            colour_parameter.string_value.strip().lower()
        )

        self._validate_trial_arguments()

        self.shelf_column_publisher = self.create_publisher(
            Int32,
            "/erc/shelf_column_identification",
            10,
        )

        self.shelf_row_publisher = self.create_publisher(
            Int32,
            "/erc/shelf_row_identification",
            10,
        )

        self.get_logger().info(
            "Validated trial arguments: "
            f"shelf_column_number={self.shelf_column_number}, "
            f"book_colour={self.book_colour}"
        )

        self.get_logger().warning(
            "Day 1 skeleton only: no robot command will be issued."
        )

    def _validate_trial_arguments(self) -> None:
        """Reject malformed evaluator inputs before commanding the robot."""
        if self.shelf_column_number not in range(1, 6):
            raise ValueError(
                "shelf_column_number must be an integer from 1 to 5; "
                f"received {self.shelf_column_number}"
            )

        if self.book_colour not in VALID_BOOK_COLOURS:
            valid_values = ", ".join(
                sorted(VALID_BOOK_COLOURS)
            )
            raise ValueError(
                "book_colour must be one of "
                f"[{valid_values}]; received {self.book_colour!r}"
            )


def main(args: list[str] | None = None) -> None:
    """Run the orchestrator until ROS shuts down."""
    rclpy.init(args=args)
    node: Orchestrator | None = None

    try:
        node = Orchestrator()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
