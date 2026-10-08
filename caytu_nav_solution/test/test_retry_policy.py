"""Décisions après un échec de Nav2 : tests sans ROS."""
from caytu_nav_solution.retry_policy import (
    CAUSE_NONE, CLEAR_ALL, CLEAR_FAR, CLEAR_NEAR, FALLBACK, RetryPolicy, count_causes,
    describe_error, resolve_error_names)


def test_default_error_table_matches_nav2_jazzy():
    table = resolve_error_names()
    assert table[208] == ('planificateur', 'NO_VALID_PATH')
    assert table[205] == ('planificateur', 'START_OCCUPIED')
    assert table[206] == ('planificateur', 'GOAL_OCCUPIED')
    assert table[106] == ('contrôleur', 'NO_VALID_CONTROL')
    assert table[105] == ('contrôleur', 'FAILED_TO_MAKE_PROGRESS')
    assert table[703] == ('pivot', 'COLLISION_AHEAD')
    assert table[714] == ('recul', 'COLLISION_AHEAD')


def test_error_numbers_are_read_from_the_installed_package():
    class OtherVersion:                 # une version de Nav2 numérotée autrement
        UNKNOWN = 201
        NO_VALID_PATH = 209
        GOAL_OCCUPIED = 207
        START_OCCUPIED = True           # valeur aberrante : ignorée
    table = resolve_error_names({'planificateur': OtherVersion})
    assert table[209] == ('planificateur', 'NO_VALID_PATH')
    assert table[207] == ('planificateur', 'GOAL_OCCUPIED')
    assert table[205] == ('planificateur', 'START_OCCUPIED')     # numéro par défaut gardé
    assert table[106][1] == 'NO_VALID_CONTROL'                   # autres familles intactes


def test_describe_error_gives_a_cause_and_a_sentence():
    table = resolve_error_names()
    cause, text = describe_error(208, table)
    assert cause == 'NO_VALID_PATH' and 'aucun chemin' in text and '208' in text
    assert describe_error(0, table)[0] == CAUSE_NONE
    assert describe_error(None, table)[0] == CAUSE_NONE
    cause, text = describe_error(4242, table)
    assert cause == 'CODE_4242' and '4242' in text


def test_count_causes_is_stable():
    assert count_causes([]) == ''
    assert count_causes(['B', 'A', 'B']) == 'A x1; B x2'


def test_progress_keeps_the_gentle_cleanup():
    policy = RetryPolicy(progress_min=0.25, fallback_after=2)
    policy.reset(10.0)
    # Le robot avance entre deux échecs : jamais de repli ni d'effacement total.
    for distance in (8.0, 6.5, 6.2, 3.0):
        assert policy.on_failure(distance, 'NO_VALID_CONTROL') == CLEAR_FAR
        assert policy.stalls == 0


def test_ladder_without_progress():
    policy = RetryPolicy(progress_min=0.25, fallback_after=2)
    policy.reset(5.0)
    actions = [policy.on_failure(5.0, 'NO_VALID_PATH') for _ in range(4)]
    assert actions == [CLEAR_FAR, FALLBACK, CLEAR_FAR, FALLBACK]
    # Un vrai progrès remet l'échelle à zéro.
    assert policy.on_failure(4.0, 'NO_VALID_PATH') == CLEAR_FAR and policy.stalls == 0
    assert policy.on_failure(3.9, 'NO_VALID_PATH') == CLEAR_FAR and policy.stalls == 1


def test_small_moves_are_not_progress():
    policy = RetryPolicy(progress_min=0.25, fallback_after=2)
    policy.reset(5.0)
    assert policy.on_failure(4.9, CAUSE_NONE) == CLEAR_FAR
    assert policy.on_failure(4.8, CAUSE_NONE) == FALLBACK      # 0,2 m en deux échecs


def test_goal_occupied_goes_straight_to_fallback():
    policy = RetryPolicy()
    policy.reset(3.0)
    assert policy.on_failure(3.0, 'GOAL_OCCUPIED') == FALLBACK


def test_blocked_goal_seen_in_the_costmap_goes_straight_to_fallback():
    # Cas réel : Smac répond NO_VALID_PATH, c'est la costmap qui dit « but occupé ».
    policy = RetryPolicy()
    policy.reset(3.0)
    assert policy.on_failure(3.0, 'NO_VALID_PATH', goal_blocked=True) == FALLBACK
    off = RetryPolicy(fallback_enabled=False)
    off.reset(3.0)
    assert off.on_failure(3.0, 'NO_VALID_PATH', goal_blocked=True) == CLEAR_FAR


def test_start_occupied_clears_around_the_robot_then_escalates():
    policy = RetryPolicy(fallback_after=2)
    policy.reset(3.0)
    actions = [policy.on_failure(3.0, 'START_OCCUPIED') for _ in range(4)]
    # Une seule fois autour du robot ; si cela ne suffit pas, l'échelle normale.
    assert actions == [CLEAR_NEAR, FALLBACK, CLEAR_FAR, FALLBACK]
    # Après un vrai progrès, un nouveau « départ occupé » repart du nettoyage local.
    assert policy.on_failure(2.0, 'NO_VALID_PATH') == CLEAR_FAR
    assert policy.on_failure(2.0, 'START_OCCUPIED') == CLEAR_NEAR


def test_without_fallback_the_last_resort_is_a_full_clear():
    policy = RetryPolicy(fallback_after=2, fallback_enabled=False)
    policy.reset(3.0)
    assert policy.on_failure(3.0, 'GOAL_OCCUPIED') == CLEAR_FAR
    assert policy.on_failure(3.0, 'GOAL_OCCUPIED') == CLEAR_ALL


def test_unknown_pose_never_counts_as_progress():
    policy = RetryPolicy(fallback_after=2)
    policy.reset(None)
    assert policy.on_failure(None, CAUSE_NONE) == CLEAR_FAR
    assert policy.on_failure(None, CAUSE_NONE) == FALLBACK
    # La première distance connue sert de référence, sans être un progrès.
    assert policy.on_failure(2.0, CAUSE_NONE) == CLEAR_FAR and policy.stalls == 3


def test_tf_error_is_resent_without_moving_up_the_ladder():
    from caytu_nav_solution.retry_policy import RESEND
    policy = RetryPolicy(resend_limit=2)
    policy.reset(10.0)
    assert policy.on_failure(10.0, 'TF_ERROR') == RESEND
    assert policy.on_failure(10.0, 'TF_ERROR') == RESEND
    assert policy.stalls == 0
    # Au-delà de la limite, c'est un échec comme un autre.
    assert policy.on_failure(10.0, 'TF_ERROR') == CLEAR_FAR
    assert policy.stalls == 1
    # Un autre échec remet le compteur de renvois à zéro.
    assert policy.on_failure(10.0, 'TF_ERROR') == RESEND


def test_an_immediate_failure_without_cause_is_resent():
    from caytu_nav_solution.retry_policy import CAUSE_NONE, RESEND
    policy = RetryPolicy()
    policy.reset(10.0)
    assert policy.on_failure(10.0, CAUSE_NONE, immediate=True) == RESEND
    assert policy.on_failure(10.0, CAUSE_NONE, immediate=False) == CLEAR_FAR
