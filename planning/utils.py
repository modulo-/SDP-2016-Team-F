def ball_is_static(world):
    # TODO find real threshold value
    STATIC_THRESHOLD = 0.1
    return world.ball.velocity == 0.1
