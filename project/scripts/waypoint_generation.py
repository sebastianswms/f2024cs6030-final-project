
FIRST_CORNER = (0, 0)
SECOND_CORNER = (100, 200)


def generate_waypoints(
    first_corner: tuple[float, float],
    second_corner: tuple[float, float],
) -> list[tuple[float, float]]:
    """Gemerates waypoints based on a rectangle defined by its corners."""

if __name__ == "__main__":
    waypoints = generate_waypoints(
        first_corner=FIRST_CORNER,
        second_corner=SECOND_CORNER
    )
    print("\n".join(waypoints))