from gymnasium.envs.registration import register

register(
    id='DeepRacer-v0',
    entry_point='deepracer_env.environments.deepracer_env:DeepRacerEnv',
    # reward_fn must be supplied as a keyword argument to gymnasium.make()
    # or by constructing DeepRacerEnv directly.
)
